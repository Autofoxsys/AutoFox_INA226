#include <Wire.h>
#include <Autofox_INA226.h>

// In this example, we use the Alert feature of the INA226 to detect a low voltage.
// The INA226 alert pin goes LOW when the specified alert condition occurs.
// It seems that the voltage/power reading that generates the alert is before any averaging
// of the voltage is done on the chip so a noisy power supply could end up generating lots of alerts.
// The pin can be configured to latch (stay low) until reset (by calling ResetAlertPin) or it
// can reset itself as soon as the condition is resolved (in our case, the voltage goes above the
// threshold we specified).
// I've had some problems with flaky performance when using the latch more.  Often the interrupt
// wouldn't trigger, seemingly because it triggered again very fast after being reset?
//
// The Alert pin on the INA226 us open drain, so when it's not being driven LOW it's floating.
// For this reason we make the input pin on the Arduino INPUT_PULLUP to engage the internal pullup
// resistor.

const int ALERT_PIN = 3; //using INT1 external interrupt
const uint8_t INA226_IC2_ADDRESS = 0x40;
const double SHUNT_RESISTOR_OHMS = 0.1;
const uint32_t LOW_VOLTAGE_VALUE_ALERT = 4800000; //4.8V or 4800000uV

AutoFox_INA226 ina226;

volatile bool gAlertDetected = false;

void alert(void)
{
  gAlertDetected = true;
  //disable INT1
  EIMSK &= ~(1 << INT1); //stop this interrupt from being handled until we re-enable it.
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(ALERT_PIN, INPUT_PULLUP); //Alert pin is open drain, needs a pullup.
  attachInterrupt(digitalPinToInterrupt(ALERT_PIN), alert, LOW);

  status theErrorCode = ina226.Init(INA226_IC2_ADDRESS, SHUNT_RESISTOR_OHMS);

  if(theErrorCode==OK){
    Serial.println("Init succeeded...");
  }else{
    Serial.print("Init failed, error code: ");
    Serial.println(theErrorCode);   
  }
  // Here we configure the sensor to generate an alert if the bus voltage drops
  // below the specified value in microvolts.

  ina226.ConfigureAlertPinTrigger(AutoFox_INA226::eAlertTrigger::BusVoltageUnderLimit, LOW_VOLTAGE_VALUE_ALERT);  
}

void loop() {
  
    double theVoltage_V = ina226.GetBusVoltage_uV() / 1000000.0;
    double theCurrent_mA = ina226.GetCurrent_uA() / 1000.0;
    double thePower_mA = ina226.GetPower_uW() / 1000.0;

    if(gAlertDetected){
      AutoFox_INA226::eAlertTriggerCause theAlertCause;
      gAlertDetected = false;
      ina226.ResetAlertPin(theAlertCause); //reset the alert pin and check what triggered it
      if(theAlertCause & AutoFox_INA226::eAlertTriggerCause::AlertFunctionFlag){
        Serial.println("Low voltage alert triggered");
        Serial.print("Alert threshold = ");Serial.print((double)LOW_VOLTAGE_VALUE_ALERT/1000000, 2);
        Serial.print("V, current voltage (averaged) is ");Serial.print(theVoltage_V , 2); Serial.println(" V");
      }
      EIMSK |= (1 << INT1); //re-enable the interrupt handler.
    }

    Serial.print("Bus Voltage:   ");  Serial.print(theVoltage_V , 2); Serial.println(" V");
    Serial.print("Current:       ");  Serial.print(theCurrent_mA, 2); Serial.println(" mA");
    Serial.print("Power:         ");  Serial.print(thePower_mA, 2); Serial.println(" mW");
    Serial.println("");
    
    delay(1000);
}
