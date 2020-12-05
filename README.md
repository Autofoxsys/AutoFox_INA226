# INA226 driver

This repository is forked from [Autofoxsys](https://github.com/Autofoxsys/AutoFox_INA226). For more info check it.

There is some modification for much better code portability, its very easy to reuse with any kind of microcontroller (STM32F1,2,3,.., PIC, etc..)
How to use:
  - Provide own ```Transmit(..)``` and ```Receive(..)``` functions (```IsDeviceReady``` function is optional) in ```INA226_callback.c``` file.
  - Declare ```INA226 INA226_1``` struct.
  - Initialize ```INA226_Init(INA226* this, void* i2c_device, uint8_t aI2C_Address, double aShuntResistor_Ohms, double aMaxCurrent_Amps)``` where:
    - ```INA226* this``` is pointer to previosly declared struct.
    - ```void* i2c_device``` is pointer for I2C device handler (on STM32Fxx usually hi2c1, hi2c2, ..).
    - ```uint8_t aI2C_Address``` Adress is determined by electrical layout, see datasheet, or INA226.h.
    - ```double aShuntResistor_Ohms``` thre resistance of shunt in ohms.
    - ```double aMaxCurrent_Amps``` The maximum amperage, this is needed for proper set up external the external amplification, etc..
  - Read values or change operation mode with provided functions

### About the INA226: ###

There are a number of low cost breakout boards for the INA226 (similar to the INA219) available from sites such as Aliexpress.  None of the libraries that I found were complete enough for my needs so I wrote this one.

Please consult the Wiki page on GitHub for explanations and examples.
