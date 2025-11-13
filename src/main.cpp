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

//*ESP32 LEDC Setup and PWM */
// use 12 bit precision for LEDC timer
#define LEDC_TIMER_12_BIT 12

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// define starting duty, target duty and maximum fade time
#define LEDC_START_DUTY  (0)
#define LEDC_TARGET_DUTY (4095)



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
const int PWM_OUT     = 04;


/************************* INA226 SETUP  *************************/
INA226 INA(0x40);


/************************* START SETUP ***************************/


void setup() {
  Serial.begin(115200);
  Wire.begin();


  if (!INA.begin() )
    {
      Serial.println("could not connect. Fix and Reboot");
    }

  INA.setMaxCurrentShunt(10, 0.002); 
  INA.setAverage(INA226_4_SAMPLES); 



  // Launch FreeRTOS tasks on both cores
  xTaskCreatePinnedToCore(coreOne, "Core1", 6000, NULL, 1, &Core1, 0);
  xTaskCreatePinnedToCore(coreTwo, "Core2", 6000, NULL, 2, &Core2, 1);
  
  // Setup the PWM Functionality
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
  ledcAttachPin(PWM_OUT, 0);
}


/************************* MAIN LOOP (NOT USEED) *************************/
void loop() {
  // Not USED In MULTI-THREAD
}

/*********************************************************************
   CORE 1 TASK (Do PWM)
*********************************************************************/
void coreOne(void * pvParameters) {
  while(1){

    int PWMFreq = timeOn;
    ledcWrite(0, PWMFreq);

  }

}


/*********************************************************************
   CORE 2 TASK (READ CURRENT AND ADJUST TIME)
*********************************************************************/
void coreTwo(void * pvParameters) {

  while(1){
    
    currentNow = GetCurrent();

    timeOn = DoPWMTime(timeOn);
  }

}

/************************* FUNCTIONS *************************/

//Return the average of three samples current reading from IN226 in mA
float GetCurrent() {
  float current = 0;
  

  current = INA.getCurrent_mA();
  

  return(current);


}


//Take the current amount of current and either increase or decrease the amount of time to pulse the mosfet pin
int DoPWMTime(int time) {

  if ((currentNow < max_power) && (time < LEDC_TARGET_DUTY)){      //Crank it up
    time ++;
  }
  
  if ((currentNow > max_power) && (time > LEDC_START_DUTY)){        //Turn it down
    time --;
  }

  if ((time > LEDC_TARGET_DUTY) || (time < LEDC_START_DUTY)){       // Something is wrong, just turn it off and start again
    time = 0;   
  }

  else {
    time = time;                                                    //Just right
  }
  
  return(time);

}

  