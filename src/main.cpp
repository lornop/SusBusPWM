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
float GetCurrent();         //Get the current reading from the INA226 in mA
int   DoPWMTime(int);       //Figure out how long to have the MOSFET Open

void coreOne(void * pvParameters);
void coreTwo(void * pvParameters);


/************************* GLOBAL VARIABLES *************************/

const float_t max_power = 8000;             //Max Power in mA
float currentNow        = 0.0;              //The INA226 Reading
int   timeOn            = 0;                //How Long Our PWM pulse is

TaskHandle_t Core1;
TaskHandle_t Core2;


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

  // Launch FreeRTOS tasks on both cores
  xTaskCreatePinnedToCore(coreOne, "Core1", 6000, NULL, 1, &Core1, 0);
  xTaskCreatePinnedToCore(coreTwo, "Core2", 6000, NULL, 2, &Core2, 1);

}


/************************* MAIN LOOP (NOT USEED) *************************/
void loop() {
  // Not USED In MULTI-THREAD
}

/*********************************************************************
   CORE 1 TASK (Do PWM)
*********************************************************************/
void coreOne(void * pvParameters) {

  int onTime = timeOn;
  int offTime = 100 - onTime;

  for (int x = 0; x < onTime; x++){                   //Turn the PWM Pin HIGH
    digitalWrite(pwm_out, HIGH);
    vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay
  }

  for (int x = 0; x < offTime; x++){                  //Turn the PWM Pin LOW
    digitalWrite(pwm_out, LOW);
    vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay
  }


}


/*********************************************************************
   CORE 2 TASK (READ CURRENT AND ADJUST TIME)
*********************************************************************/
void coreTwo(void * pvParameters) {
  
  currentNow = GetCurrent();

  timeOn = DoPWMTime(timeOn);


}

/************************* FUNCTIONS *************************/

//Return the average of three samples current reading from IN226 in mA
float GetCurrent() {
  float current = 0;
  
  for (int x =0; x <= 3; x++){
    current += float(INA.getCurrent_mA());
    vTaskDelay(5 / portTICK_PERIOD_MS); // Small delay
  }
  current = current / 3.0;

  return(current);


}


//Take the current amount of current and either increase or decrease the amount of time to pulse the mosfet pin
int DoPWMTime(int time) {

  if ((currentNow < max_power) && (time < 100)){      //Crank it up
    time ++;
  }
  
  if ((currentNow > max_power) && (time > 0)){        //Turn it down
    time --;
  }

  if ((time > 100) || (time < 0)){                    // Something is wrong, just turn it off and start again
    time = 0;   
  }

  else {
    time = time;                                      //Just right
  }
  
  return(time);

}

  