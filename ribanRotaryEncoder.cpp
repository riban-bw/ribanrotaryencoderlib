/**	Rotary encoder class for Raspberry Pi
	Copyright Brian Walton (brian@riban.co.uk) 2019
*/
#include "ribanRotaryEncoder.h"
#include <unistd.h> //provide usleep
#include <poll.h> //provides poll
#include <fcntl.h> //provides file open constants
//#include <cstdio> //provides printf (enable for debug)
#include <pthread.h> //Provides threading
#include <time.h> //provides clock
#include <sys/mman.h> //provides mmap

#define MAX_GPI     54
#define BLOCK_SIZE  (1024 * 4)
#define GPSET0      7
#define GPCLR0      10
#define GPLEV0      13
#define GPPUD       37
#define GPPUDCLK0   38

static const int8_t anValid[16] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0}; //Table of valid encoder states
static const int8_t anUnavailableGpi[MAX_GPI] = {1,1,0,0,0,0,0,0,0,0,
                                                0,0,0,0,0,0,0,0,0,0,
                                                0,0,0,0,0,0,0,0,0,0,
                                                0,0,1,1,1,1,1,1,1,1,
                                                1,1,1,1,1,1,1,1,1,1,
                                                1,1,1,1}; //Table of unavailable GPI pins

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER; //Initialise mutex for thread safety

ribanRotaryEncoder::ribanRotaryEncoder(uint8_t clk, uint8_t data, uint8_t button)
{
    m_bUnInit = true;
    //Configure GPIO
    m_nClk = clk;
    m_nData = data;
    m_nButton = button;
    m_nLastButtonPress = GetMillis();
    m_nDebounce = 50;
    //Initialise encoder registers
    SetValue(0);
    SetMin(-100);
    SetMax(100);
    SetThreshold(0);
    SetScale(1);
    SetMultiplier(1);
    if(initgpi())
    {
        ConfigureGpi(clk, GPI_INPUT_PULLUP);
        ConfigureGpi(data, GPI_INPUT_PULLUP);
        ConfigureGpi(button, GPI_INPUT_PULLUP);
        m_bPoll = true;
        pthread_t threadPoll;
        pthread_create(&threadPoll, NULL, &ribanRotaryEncoder::getPoll, this);
        m_bButton = !GetGpi(m_nButton); //Invert button because button is pulled up
    }
}

ribanRotaryEncoder::~ribanRotaryEncoder()
{
    if(!m_bUnInit)
    {
        m_bPoll = false;
        usleep(1000); //Wait for last iteration of encoder (if running)
        uninitgpi();
    }
}

void ribanRotaryEncoder::SetThreshold(int8_t threshold)
{
    m_nThreshold = threshold;
}

int8_t ribanRotaryEncoder::GetThreshold()
{
    return m_nThreshold;
}

void ribanRotaryEncoder::SetScale(int32_t scale)
{
    m_nScale = scale;
}

int32_t ribanRotaryEncoder::GetScale()
{
    return m_nScale;
}

void ribanRotaryEncoder::SetValue(int32_t value)
{
    pthread_mutex_lock(&mutex1);
    m_lValue = value;
    pthread_mutex_unlock(&mutex1);
}

int32_t ribanRotaryEncoder::GetValue(bool reset)
{
    int32_t lValue = m_lValue;
    if(reset)
        SetValue(0);
    return lValue;
}

void ribanRotaryEncoder::SetMin(int32_t min)
{
    m_lMin = min;
    if(GetValue() < min)
        SetValue(min);
}

int32_t ribanRotaryEncoder::GetMin()
{
    return m_lMin;
}

void ribanRotaryEncoder::SetMax(int32_t max)
{
    m_lMax = max;
    if(GetValue() > max)
       SetValue(max);
}

int32_t ribanRotaryEncoder::GetMax()
{
    return m_lMax;
}

void ribanRotaryEncoder::SetMultiplier(int32_t multiplier)
{
    m_lMultiplier = multiplier;
}

int32_t ribanRotaryEncoder::GetMultiplier()
{
    return m_lMultiplier;
}

bool ribanRotaryEncoder::IsButtonPressed()
{
    if(GetMillis() > m_nLastButtonPress + m_nDebounce)
    {
        m_nLastButtonPress = GetMillis();
        m_bButton = !GetGpi(m_nButton); //Invert value because button is pulled up
    }
    return m_bButton;
}

void ribanRotaryEncoder::SetDebounce(int debounce)
{
    m_nDebounce = debounce;;
}

void *ribanRotaryEncoder::getPoll(void *context)
{
    return ((ribanRotaryEncoder*)context)->pollEnc();
}

void *ribanRotaryEncoder::pollEnc()
{
    /* 	State of CLK and DATA are stored as 2-bit words
        The previous and current states are combined to a 4-bit words
        The combined 4-bit word is validated by a look-up table where 1 represents a valid transition
    */
    uint8_t nEncCode = 0; //2-bit word (CLK DATA)
    uint8_t nEncHist = 0; //4-bit history of last two 2-bit words
    int8_t nDir = 0; //Direction of rotation [-1, 0, +1]
    uint32_t lTime = GetMillis(); //Time of last encoder pulse used for fast scroll detection

    while(m_bPoll)
    {
        while(!GetGpi(m_nClk)) // Wait for clock signal avoiding 100% CPU (check every 1ms)
            usleep(1000); //!@todo Can we use edge detection?
        nDir = 0;
        while(nDir == 0)
        {
            // Create 4-bit word representing previous and current CLK and DATA states
            nEncCode <<= 2;
            if(GetGpi(m_nData))
                nEncCode |= 0x02;
            if(GetGpi(m_nClk))
                nEncCode |= 0x01;
            nEncCode &= 0x0f;

            // If valid then add to 8-bit history validate rotation codes and process
            if(anValid[nEncCode])
            {
                nEncHist <<= 4;
                nEncHist |= nEncCode;
                if(nEncHist==0x2b)
                    --nDir;
                else if(nEncHist==0x17)
                    ++nDir;
                if(nDir)
                {
                    uint32_t lNow = GetMillis();
                    pthread_mutex_lock(&mutex1);
                    if(lNow >= lTime + m_nThreshold)
                        m_lValue += nDir * m_lMultiplier; //Slow rotation
                    else
                        m_lValue += nDir * m_lMultiplier * m_nScale; //Fast rotation
                    lTime = lNow;
                    if(m_lValue > m_lMax)
                        m_lValue = m_lMax;
                    if(m_lValue < m_lMin)
                        m_lValue = m_lMin;
                    pthread_mutex_unlock(&mutex1);
                }
            }
            usleep(1000); //Avoid 100% CPU and debounce encoder wipers
        }
    }
}

bool ribanRotaryEncoder::GetGpi(uint8_t gpi)
{
    if(m_bUnInit || gpi > MAX_GPI || anUnavailableGpi[gpi])
        return false;
    return(((*(m_pGpiMap + GPLEV0 + gpi / 32)) & (1 << (gpi % 32))) != 0);
}

void ribanRotaryEncoder::SetGpi(uint8_t gpi, bool value)
{
    if(m_bUnInit || gpi > MAX_GPI || anUnavailableGpi[gpi])
        return;
    if(value)
        *(m_pGpiMap + GPSET0) = 1 << gpi;
    else
        *(m_pGpiMap + GPCLR0) = 1 << gpi;
}

bool ribanRotaryEncoder::ConfigureGpi(uint8_t gpi, uint8_t flags)
{
    if(m_bUnInit || gpi > MAX_GPI || anUnavailableGpi[gpi])
        return false;
    /*  There are 10 GPI configurations per register. Registers start at GPIO_BASE
        Each configuration consists of three bits
        Bits 1&2 are zero for GPI and bit 0 indicates direction
    */
    //Configure GPI as input
    *(m_pGpiMap + (gpi / 10)) &= ~(7 << ((gpi % 10) * 3)); //reset 3 flags for this gpi

    if(flags & GPI_OUTPUT)
    {
        *(m_pGpiMap + (gpi / 10)) |= (1 << ((gpi % 10) * 3)); //set first bit of flags for this gpi
        return true;
    }
    //Set pull-up/down flags then clock into selected pin
    *(m_pGpiMap + GPPUD) = flags & 0x03;
    usleep(1); //Need to wait 150 cycles which is 0.6us on the slowest RPi so let's wait 1us
    *(m_pGpiMap + GPPUDCLK0 + (gpi / 32)) = 1 << (gpi % 32);
    usleep(1); //Need to wait 150 cycles which is 0.6us on the slowest RPi so let's wait 1us
    *(m_pGpiMap + GPPUD) = 0;
    *(m_pGpiMap + GPPUDCLK0 + (gpi /32)) = 0;
    return true;
}

bool ribanRotaryEncoder::IsInit()
{
    return !m_bUnInit;
}

uint32_t ribanRotaryEncoder::GetMillis()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

bool ribanRotaryEncoder::initgpi()
{
    int fd;
    if((fd = open("/dev/gpiomem", O_RDWR|O_SYNC) ) < 0)
        return false;
    m_pMap = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd); //Don't need the file open after memory map
    if(m_pMap != MAP_FAILED)
    {
        m_pGpiMap = (volatile uint32_t *)m_pMap;
        m_bUnInit = false;
    }
    return m_bUnInit;
}

void ribanRotaryEncoder::uninitgpi()
{
    munmap(m_pMap, BLOCK_SIZE);
    m_bUnInit = true;
}
