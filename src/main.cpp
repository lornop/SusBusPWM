/*********************************
 * Writing code for a ESP32 powered PWM Current Limter
 * Its using a INA226 to read the current. 
 * 
 * The INA 226 is conected with I2C on I/O 21 and 22
 * 
 * I/O 04 is used to control the MOSFET Driver an MCP1407
 * 
 * The PCB is designed to have a 10A capacity at 12V
 * 
 * 
 * 
 * 
 * 
 * *********************************/

#include <Arduino.h>
#include <INA226.h>
#include <Wire.h>


// put function declarations here:
int GetCurrent();         //Get the current reading from the INA226 in mA


/************************* POWER SETTING *************************/
const int max_power = 8;


/************************* HARDWARE PINS *************************/
const int pwm_out     = 04;


/************************* INA226 SETUP  *************************/
INA226 INA(0x40);


/************************* START SETUP ***************************/

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(pwm_out, OUTPUT);
  digitalWrite(pwm_out, LOW);

  if (!INA.begin() )
    {
      Serial.println("could not connect. Fix and Reboot");
    }
  INA.setMaxCurrentShunt(1, 0.002);


}


/************************* MAIN LOOP *************************/

void loop() {
  // put your main code here, to run repeatedly:
}



/************************* FUNCTIONS *************************/

//Return the current reading from IN226 in mA
int GetCurrent() {
  int current = 0;
  
  current = INA.getCurrent_mA();

  return(current);


}


  