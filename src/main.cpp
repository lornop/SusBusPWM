#include <Arduino.h>
#include <INA226.h>


// put function declarations here:
int myFunction(int, int);

INA226 INA(0x40);

void setup() {

  Wire.begin();
  if (!INA.begin() )
    {
      Serial.println("could not connect. Fix and Reboot");
    }
  INA.setMaxCurrentShunt(1, 0.002);


}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

/* Example of functuions for the INA226
Serial.println("\nBUS\tSHUNT\tCURRENT\tPOWER");
  for (int i = 0; i < 20; i++)
  {
    Serial.print(INA.getBusVoltage(), 3);
    Serial.print("\t");
    Serial.print(INA.getShuntVoltage_mV(), 3);
    Serial.print("\t");
    Serial.print(INA.getCurrent_mA(), 3);
    Serial.print("\t");
    Serial.print(INA.getPower_mW(), 3);
    Serial.println();
    delay(1000);
*/

  