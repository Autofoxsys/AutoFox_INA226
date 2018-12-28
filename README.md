# AutoFox_INA226
This repository contains both C and C++ files for using the INA226 voltage, current and power measurement chip from Texas Instruments.

The only platform specific parts of this library are related to I2C communications.  On Arduino I use Wire for I2C comms with the INA226 while for STM32, which I build using CubeMX and TrueStudio I use the HAL I2C drivers.  If you want to use some other library for I2C on your platform then it should be relatively easy.

I started by developing a C++ library for Arduino.
When I wanted to re-use this library on an STM32 using ST's CubeMX development tools I found that I needed to convert it to C only (there's a C++ workaround but it didn't suit me).

Both the C++ and C source files are in the same folder - but you only need one or the other (header + associated source file).

About the INA226:

There are a number of low cost breakout boards for the INA226 (similar to the INA219) available from sites such as Aliexpress.  None of the libraries that I found were complete enough for my needs so I wrote this one.

Please consult the Wiki page on GitHub for explanations and examples.
