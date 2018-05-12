#include <Wire.h>
#include <Autofox_INA226.h>

const int ALERT_PIN = 8;
const uint8_t INA226_IC2_ADDRESS = 0x40;
const double SHUNT_RESISTOR_OHMS = 0.1;

AutoFox_INA226 ina226;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(ALERT_PIN, INPUT);

  status theErrorCode = ina226.Init(INA226_IC2_ADDRESS, SHUNT_RESISTOR_OHMS);

  if(theErrorCode==OK){
    Serial.println("Init succeeded...");
  }else{
    Serial.print("Init failed, error code: ");
    Serial.println(theErrorCode);
  }
  
}

void loop() {
  
    double theVoltage_V = ina226.GetBusVoltage_uV() / 1000000.0;
    double theCurrent_mA = ina226.GetCurrent_uA() / 1000.0;
    double thePower_mA = ina226.GetPower_uW() / 1000.0;

    Serial.print("Bus Voltage:   ");  Serial.print(theVoltage_V , 2); Serial.println(" V");
    Serial.print("Current:       ");  Serial.print(theCurrent_mA, 2); Serial.println(" mA");
    Serial.print("Power:         ");  Serial.print(thePower_mA, 2); Serial.println(" mW");
    Serial.println("");
    
    delay(1000);
}
