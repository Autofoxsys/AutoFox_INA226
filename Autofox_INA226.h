/*
   Library to facilitate use of the INA226 Voltage, Current & Power monitor
   from Texas Intruments.
   See spec. here: http://www.ti.com/lit/ds/symlink/ina226.pdf.

   Copyright [2018] [AutoFox] autofoxsys@gmail.com

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   This library is for Arduino.

   In developing this library, besides reading the INA226 spec, I consulted
   the following code:

   * Linux drivers for the INA family (written in C)
   * https://github.com/SV-Zanshin/INA226 (most useful)

   PLATFORM DEPENDENCE
   Embodied in 3 functions related to I2C reading/writing

*/


#ifndef __AUTOFOX_INA226_H__
#define __AUTOFOX_INA226_H__

#include <inttypes.h>

//Heads up for the classes declared in this header
class AutoFox_INA226; //This is the one you will use directly

class INA226_Registers;
class INA226_HardCodedChipConst;
class INA226_DefaultSettings;

//Most functions will return an error status
typedef enum {OK=0, FAIL=-1,
    INA226_TI_ID_MISMATCH = -2,
    INA226_DIE_ID_MISMATCH =-3,
    CONFIG_ERROR = -4,
    I2C_TRANSMISSION_ERROR = -5,
    BAD_PARAMETER = -6,
    NOT_INITIALIZED = -7,
    INVALID_I2C_ADDRESS} status;


//=============================================================================
class AutoFox_INA226{
public:

    enum eOperatingMode {//Shutdown=0,
                        ShuntVoltageTriggered        = 1,
                        BusVoltageTriggered          = 2,
                        ShuntAndBusTriggered         = 3,
                        Shutdown                     = 4,
                        ShuntVoltageContinuous       = 5,
                        BusVoltageContinuous         = 6,
                        ShuntAndBusVoltageContinuous = 7}; //default

    enum eAlertTrigger {ClearTriggers                = 0x0000, //default
                        ShuntVoltageOverLimit        = 0x8000,
                        ShuntVoltageUnderLimit       = 0x4000,
                        BusVoltageOverLimit          = 0x2000,
                        BusVoltageUnderLimit         = 0x1000,
                        PowerOverLimit               = 0x0800,
                        ConversionReady              = 0x0400};

    enum eAlertTriggerCause{
                        Unknown=0,
                        AlertFunctionFlag            = 0x10,
                        ConversionReadyFlag          = 0x08,
                        MathOverflowFlag             = 0x04,
                        AlertPolarityBit             = 0x02};

    AutoFox_INA226();

    //Resets the INA226 and configures it according to the supplied parameters - should be called first.
    status Init(uint8_t aI2C_Address=0x40, double aShuntResistor_Ohms=0.1, double aMaxCurrent_Amps=3.2767);

    int32_t GetShuntVoltage_uV();
    int32_t GetBusVoltage_uV();
    int32_t GetCurrent_uA();
    int32_t GetPower_uW();

    status SetOperatingMode(enum eOperatingMode aOpMode);
    status Hibernate(); //Enters a very low power mode, no voltage measurements
    status Wakeup();    //Wake-up and enter the last operating mode

    //The trigger value is in microwatts or microvolts, depending on the trigger
    status ConfigureAlertPinTrigger(enum eAlertTrigger aAlertTrigger, int32_t aValue, bool aLatching=false);
    status ResetAlertPin();
    status ResetAlertPin(enum  eAlertTriggerCause& aAlertTriggerCause ); //provides feedback as to what caused the alert

    //The parameters for the two functions below are indicies into the tables defined in the INA226 spec
    //These tables are copied below for your information (caNumSamplesAveraged & caVoltageConvTimeMicroSecs)
    status ConfigureVoltageConversionTime(int aIndexToConversionTimeTable);
    status ConfigureNumSampleAveraging(int aIndexToSampleAverageTable);

    status Debug_GetConfigRegister(uint16_t& aConfigReg);

protected:
    bool     mInitialized;
    uint8_t  mI2C_Address;
    uint16_t mConfigRegister;        //local copy from the INA226
    uint16_t mCalibrationValue;        //local copy from the INA226
    int32_t  mCurrentMicroAmpsPerBit; //This is the Current_LSB, as defined in the INA266 spec
    int32_t  mPowerMicroWattPerBit;

protected:
    status CheckI2cAddress(uint8_t aI2C_Address);
    status WriteRegister(uint8_t aRegister, uint16_t aValue);
    status ReadRegister(uint8_t aRegister, uint16_t& aValue);
    status setupCalibration(double aShuntResistor_Ohms, double aMaxCurrent_Amps);

    static const uint16_t cResetCommand                = 0x8000;
    static const uint16_t cOperatingModeMask           = 0x0007;
    static const uint16_t cAlertPinModeMask            = 0xFC00;
    static const uint16_t cAlertCauseMask              = 0x001E;
	static const uint16_t cAlertLatchingMode           = 0x0001;
    static const uint16_t cSampleAvgMask               = 0x0E00;
    static const uint16_t cBusVoltageConvTimeMask      = 0x01C0;
    static const uint16_t cShuntVoltageConvTimeMask    = 0x0038;
    static const int      cSampleAvgIdxShift           = 9;
    static const int      cBusVoltConvTimeIdxShift     = 6;
    static const int      cShuntVoltConvTimeIdxShift   = 3;
    static const int      cMaxSampleAvgTblIdx          = 7;    //occupies 3 bit positions
    static const int      cMaxConvTimeTblIdx           = 7; //occupies 3 bit positions
    static const uint16_t caNumSamplesAveraged[8]      = {1, 4, 16, 64, 128, 256, 512, 1024};
    static const uint16_t caVoltageConvTimeMicroSecs[8] = {140, 204, 332, 588, 1100, 2116, 4156, 8244}; //microseconds

};
//=============================================================================
class INA226_Registers{
public:
    static const uint8_t    INA226_CONFIG              = 0x00;
    static const uint8_t    INA226_SHUNT_VOLTAGE       = 0x01; // readonly
    static const uint8_t    INA226_BUS_VOLTAGE         = 0x02; // readonly
    static const uint8_t    INA226_POWER               = 0x03; // readonly
    static const uint8_t    INA226_CURRENT             = 0x04; // readonly
    static const uint8_t    INA226_CALIBRATION         = 0x05;
    static const uint8_t    INA226_MASK_ENABLE         = 0x06;
    static const uint8_t    INA226_ALERT_LIMIT         = 0x07;
    static const uint8_t    INA226_MANUFACTURER_ID     = 0xFE; // readonly
    static const uint8_t    INA226_DIE_ID              = 0xFF; // readonly
};

//=============================================================================
class INA226_HardCodedChipConst{
public:
    static const int32_t    INA226_BUS_VOLTAGE_LSB     = 1250; //1250uV per bit
    static const double     INA226_SHUNT_VOLTAGE_LSB   = 2.5;    //2.5uV per bit
    static const int32_t    INA226_POWER_LSB_FACTOR    = 25;
    static const uint16_t   INA226_MANUFACTURER_ID     = 0x5449;
    static const uint16_t   INA226_DIE_ID              = 0x2260;
    static const uint16_t   INA226_CONFIG_RESET_VALUE  = 0x4127; // value of config reg after a reset
};
//=============================================================================
class INA226_DefaultSettings{
public:
    static const uint8_t    INA226_DEFAULT_I2C_ADDRESS  = 0x40;
    static const uint16_t   INA226_CONFIG_DEFAULT       = 0x4527; // Our default config reg settings
};

#endif __AUTOFOX_INA226_H__
