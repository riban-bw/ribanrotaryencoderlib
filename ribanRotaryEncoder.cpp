/**	Rotary encoder class for Raspberry Pi
	Copyright Brian Walton (brian@riban.co.uk) 2019
*/
#include "ribanRotaryEncoder.h"
#include <unistd.h> //provide usleep
#include <poll.h> //provides poll
#include <fcntl.h> //provides file open constants
#include <cstdio>
#include <pthread.h> //Provides threading
#include <time.h>

static const int8_t anValid[16] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

ribanRotaryEncoder::ribanRotaryEncoder(uint8_t clk, uint8_t data, uint8_t button)
{
    //Configure GPIO
    m_nClk = clk;
    m_nData = data;
    m_nButton = button;
    m_nLastButtonPress = clock();
    for(int i = 0; i < MAX_GPI; ++i)
        m_fdGpi[i] = -1;
    m_nDebounce = 50;
    configureGpi(clk, GPI_INPUT | GPI_PULLUP);
    configureGpi(data, GPI_INPUT | GPI_PULLUP);
    configureGpi(button, GPI_INPUT | GPI_PULLUP);
    //Initialise encoder registers
    SetValue(0);
    SetMin(-100);
    SetMax(100);
    SetThreshold(0);
    SetScale(1);
    m_bPoll = true;
    pthread_t threadPoll;
    pthread_create(&threadPoll, NULL, &ribanRotaryEncoder::getPoll, this);
}

ribanRotaryEncoder::~ribanRotaryEncoder()
{
    m_bPoll = false;
    usleep(1000); //Wait for last iteration of encoder (if running)
    unconfigureGpi();
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
    m_lValue = value;
}

int32_t ribanRotaryEncoder::GetValue(bool reset)
{
    int32_t lValue = m_lValue;
    if(reset)
        m_lValue = 0;
    return lValue;
}

void ribanRotaryEncoder::SetMin(int32_t min)
{
    m_lMin = min;
    if(m_lValue < min)
        m_lValue = min;
}

int32_t ribanRotaryEncoder::GetMin()
{
    return m_lMin;
}

void ribanRotaryEncoder::SetMax(int32_t max)
{
    m_lMax = max;
    if(m_lValue > max)
        m_lMax = max;
}

int32_t ribanRotaryEncoder::GetMax()
{
    return m_lMax;
}

bool ribanRotaryEncoder::GetButton()
{
    if(clock() < m_nLastButtonPress + m_nDebounce)
        return false;
    m_nLastButtonPress = clock();
    return readGpi(m_nButton);
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
    uint32_t lTime = clock(); //Time of last encoder pulse used for fast scroll detection
    int fd = -1; //File descriptor of GPIO sysfs device to monitor for change
    char fPath[30]; //Holds path to GPIO device/value
    sprintf (fPath, "/sys/class/gpio/gpio%d/value", m_nClk);
    char c; //Sacrificial variable to dump GPIO values to
    //Configure poll file descriptor structure to look for interrupts
    struct pollfd polls;
    polls.events = POLLPRI | POLLERR;

    while(m_bPoll)
    {
        // Wait for clock signal
        if((fd = open(fPath, O_RDWR)) < 0) //Open the sysfs GPIO interface
        {
            //Can't open so wait a while an try again
            fprintf(stderr, "Failed to open %s\n", fPath);
            usleep(500000);
            continue;
        }
        polls.fd = fd;
        while(read (fd, &c, 1) > 0); //Clear the GPIO queue
        int x = poll(&polls, 1, -1);
        nDir = 0;
        while(nDir == 0)
        {
            // Create 4-bit word representing previous and current CLK and DATA states
            nEncCode <<= 2;
            if(readGpi(m_nData))
                nEncCode |= 0x02;
            if(readGpi(m_nClk))
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
                    uint32_t lNow = clock(); //wiringPi
                    pthread_mutex_lock(&mutex1);
                    if(lNow >= lTime + m_nThreshold)
                        m_lValue += nDir; //Slow rotation
                    else
                        m_lValue += nDir * m_nScale; //Fast rotation
                    lTime = lNow;
                    if(m_lValue > m_lMax)
                        m_lValue = m_lMax;
                    if(m_lValue < m_lMin)
                        m_lValue = m_lMin;
                    pthread_mutex_unlock(&mutex1);
                }
            }
            close(fd);
            usleep(1000);
        }
    }
}

bool ribanRotaryEncoder::configureGpi(uint8_t gpi, uint8_t flags)
{
    if(gpi >= MAX_GPI)
        return false;
    FILE *fd;
    char sFname[64];
    if((fd = fopen("/sys/class/gpio/export", "w")) == NULL)
        return false;
    fprintf(fd, "%d\n", gpi);
    fclose(fd);
    sprintf(sFname, "/sys/class/gpio/gpio%d/direction", gpi);
    if((fd = fopen(sFname, "w")) == NULL)
        return false;
    fprintf(fd, (flags & 1)?"out\n":"in\n");
    fprintf(fd, (flags & 2)?"high\n":"low\n"); //!@todo Validate that this actually does pull-up
    fclose(fd);
    sprintf(sFname, "/sys/class/gpio/gpio%d/value", gpi);
    if((m_fdGpi[gpi] = open(sFname, O_RDWR)) < 0)
        return false;
    return true;
}

void ribanRotaryEncoder::unconfigureGpi()
{
    //!@todo Allow unconfigure of each GPI separately
    for(int i = 0; i < MAX_GPI; ++i)
        if(m_fdGpi[i] != -1)
           close(m_fdGpi[i]);
    FILE *fd;
    char sFname[64];
    if((fd = fopen("/sys/class/gpio/unexport", "w")) == NULL)
        return;
    fprintf(fd, "%d\n", m_nClk);
    fprintf(fd, "%d\n", m_nData);
    if(m_nButton != -1)
        fprintf(fd, "%d\n", m_nButton);
    fclose(fd);
}

bool ribanRotaryEncoder::readGpi(uint8_t gpi)
{
    if(gpi > MAX_GPI || m_fdGpi[gpi] < 0)
        return false;
    char c;
    //lseek(fd, 0L, SEEK_SET);
    read(m_fdGpi[gpi], &c, 1);
    return(c == '1');
}
