/*  This is a test program demonstrating and validating the functionality of ribanRotaryEncoder
    Copyright riban 2019
    Author: Brian Walton brian@riban.co.uk
*/

#include "ribanRotaryEncoder.h"
#include <cstdio>
#include <unistd.h>

#define LED 1
#define CLK 17
#define DATA 27
#define BUTTON 4

int main(int argc, char* argv[])
{
    uint32_t lValue = -1;
    printf("riban Rotary Encoder test\n");
    ribanRotaryEncoder enc(CLK, DATA, BUTTON);
    printf("Encoder library %sinitialised\n", enc.IsInit()?"":"not ");
    printf("%s\n", enc.GetModel().c_str());
    printf("Button is %spressed\n", enc.IsButtonPressed()?"":"not ");
    printf("Showing linear rotation value [%d..%d]. Press button to move to next test\n", enc.GetMin(), enc.GetMax());
    while(enc.IsButtonPressed() == false)
    {
        if(lValue != enc.GetValue())
        {
            lValue = enc.GetValue();
            printf("Rotate: %d\n", lValue);
        }
        usleep(1000); //Avoid 100% CPU
    }
    while(enc.IsButtonPressed()); //Wait for button release
    enc.SetThreshold(5);
    enc.SetScale(10);
    printf("Showing rotation value with fast rotation scaling. Press button to move to next test\n");
    while(enc.IsButtonPressed() == false)
    {
        if(lValue != enc.GetValue())
        {
            lValue = enc.GetValue();
            printf("Rotate: %d\n", lValue);
        }
        usleep(1000); //Avoid 100% CPU
    }
    while(enc.IsButtonPressed()); //Wait for button release
    enc.SetThreshold(0);
    enc.SetScale(1);
    printf("Showing rotation direction. Press button to move to next test\n");
    while(enc.IsButtonPressed() == false)
    {
        lValue = enc.GetValue(true);
        if(lValue > 0)
            printf("Clockwise\n");
        if(lValue < 1)
            printf("Counter-clockwise\n");
    }
    while(enc.IsButtonPressed()); //Wait for button release
    printf("Flash LED. Press button to move to next test\n");
    while(enc.IsButtonPressed() == false)
    {
        enc.ConfigureGpi(LED, GPI_OUTPUT);
        enc.SetGpi(LED, true);
        usleep(500000);
        enc.SetGpi(LED, false);
        usleep(500000);
    }
    printf("No more tests. Press button to end\n");
    while(enc.IsButtonPressed() == false);
    return 0;
}
