/**	Rotary encoder class for Raspberry Pi
	Copyright Brian Walton (brian@riban.co.uk) 2019
	Credit to John Main (best-microcontroller-projects.com) for description of digital filter
	Credit to Gordan (projects@drogon.net) for GPI handling (wiringPi)
	Rotation faster than THRESHOLD change value at higher rate defined by SCALE
	Depends on wiringPi - todo: remove this dependency
	Depends on ptread for pulse detection
	Note: Waits for interrupt on clock pin then polls until filter completes hence quiescent state has low CPU usage but this increases during rotation
*/
#pragma once

#include <stdint.h> //Provides fixed length integer types

#define MAX_GPI         32
#define GPI_INPUT       0x00
#define GPI_OUTPUT      0x01
#define GPI_PULLUP      0x02
#define GPI_PULLDOWN    0x04

class ribanRotaryEncoder
{
    public:
        /** @brief  Instantiate a rotary encoder object
        *   @param  clk GPIO pin for CLK
        *   @param  data GPIO pin for DATA
        *   @param  button GPIO pin for BUTTON (optional)
        */
        ribanRotaryEncoder(uint8_t clk, uint8_t data, uint8_t button = -1);

        virtual ~ribanRotaryEncoder();

        /** @brief  Set the threshold at which rotation speed to increment size changes
        *   @param  threshold Quantity of milliseconds between clock pulses to set threshold
        */
        void SetThreshold(int8_t threshold);

        /** @brief  Get the threshold at which rotation speed to increment size changes
        *   @retval int8_t Quantity of milliseconds between clock pulses to set threshold
        */
        int8_t GetThreshold();

        /** @brief  Set the scaling factor for rotation value rotation above threshold
        *   @param  scale Multiplier by which rate of value change is scaled if rotation is above threshold rate
        */
        void SetScale(int32_t scale);

        /** @brief  Get the scaling factor for rotation value ration above threshold
        *   @retval int32_t Multiplier by which rate of value change is scaled
        */
        int32_t GetScale();

        /** @brief  Set the current value of the encoder
        *   @param  value New value [Default: 0]
        */
        void SetValue(int32_t value = 0);

        /** @brief  Get the current value of the encoder
        *   @param  reset True to reset the value, e.g. polling for relative position [Default: false]
        *   @retval int32_t Current value
        */
        int32_t GetValue(bool reset = false);

        /** @brief  Set the minimum value for the absolute reading
        *   @param  min Minimum value
        */
        void SetMin(int32_t min);

        /** @brief  Get the minimum value for the absolute reading
        *   @retval int32_t Minimum value
        */
        int32_t GetMin();

        /** @brief  Set the maximum value for the absolute reading
        *   @param  max Maximum value
        */
        void SetMax(int32_t max);

        /** @brief  Get the maximum value for the absolute reading
        *   @retval int32_t Maximum value
        */
        int32_t GetMax();

        /** @brief  Get button value
        *   @retval bool True if button pressed
        */
        bool GetButton();

        /** @brief Set the button debounce period
        *   @param  debounce Quantity of milliseconds to ignore button after last press
        */
        void SetDebounce(int debounce);

        static void *getPoll(void *context); //Get pointer to encoder poll function
        void *pollEnc(); // Polls encoder (run as separate thread)

    protected:

    private:
        bool configureGpi(uint8_t gpi, uint8_t flags); // Configure GPI pins returns true on success
        void unconfigureGpi(); // Unconfigure GPI pins
        bool readGpi(uint8_t gpi); // Read GPI pin
        bool m_bPoll; // True to enable polling
        uint8_t m_nClk; // GPIO pin connected to encoder clock
        uint8_t m_nData; // GPIO pin connected to encoder data
        uint8_t m_nButton; // GPIO pin connected to button
        int8_t m_nThreshold;
        int32_t m_nScale;
        int32_t m_lValue; // Current absolute value
        int32_t m_lMin; // Minimum permissible value
        int32_t m_lMax; // Maximum permissible value
        int m_fdGpi[MAX_GPI]; // Array of file descriptors for GPI values
        int m_nLastButtonPress; // Time of last button press (for debounce)
        int m_nDebounce; // Quantity of milliseconds to ignore button after last press
};
