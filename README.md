# ribanrotaryencoderlib
Rotary encoder library (for Raspberry Pi running Linux) written in C++.

This C++ library provides an interface to a quadrature encoding rotary encoder using GPI pins of a Raspberry Pi (or similar device) running Linux. The code should be simple to convert to other devices and operating systems.

A rotary encoder will have three pins for the rotary encoding and possibly (probably) a further two for a switch operated by pushing the shaft. The switch is also handled by the library with debounce code. The common (usually the centre) pin should be wired to a 0V / ground point. The other two pins should be wired to GPI input pins. The library asserts the GPI pull-up resistors so no external components are required. A digital filter algorithm removes the need for debounce circuitry or code.

The library provides a value that is increased and decreased by one each time the encoder passes from one indent to the next / previous. The value may be cleared to allow the encoder to be used as a simple up / down signal or the library's value may be used within the application code. The value can be range limited with minimum and maximum values. Incremental values can be scaled to allow larger value changes per step and negative scaling allows to reverse the rotation direction.

A threshold may be set to allow fast rotation of the encoder to change the value by larger increments.

This library works well with some of the lowest quality encoders with poor bounce response from mechanical wipers.

Credit and thanks to John Main (best-microcontroller-projects.com) for his inspiration and information that allowed development of this library.

To compile a program with source code called main.cpp to an executeable called test:

`g++ -o test main.cpp ribanRotaryEncoder.cpp -lpthread`

(This assumes libpthread.a (or libpthread.o) are in linker path and associated header files are in include path. Use -I to specifiy additional inclue path and -L to specify additional linker paths as required.) There is also a Makefile to allow the test to be compiled with GNU make.
