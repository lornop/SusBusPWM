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
 * Uses:
 * INA226 by Rob Tillaart 0.6.4 Arduino library for INA226 power sensor
 * 
 * 
 * 
 * *********************************/

#include <Arduino.h>
#include <INA226.h>
#include <Wire.h>

/***************************************************************
 * Function Declarations
 ***************************************************************/
float GetCurrent();          // Get current reading from INA226 in mA
int   DoPWMTime(int value);  // Calculate how long to keep MOSFET open

void coreOne(void *pvParameters);
void coreTwo(void *pvParameters);

/***************************************************************
 * ESP32 LEDC Setup and PWM Configuration
***************************************************************/

// Use 12-bit precision for LEDC timer
#define LEDC_TIMER_12_BIT   12
// Use 5000 Hz as LEDC base frequency
#define LEDC_BASE_FREQ      5000
// Define Minimum and Maximum Duty Cycles
#define LEDC_START_DUTY     0
#define LEDC_TARGET_DUTY    4095

/***************************************************************
 * Global Variables
 ***************************************************************/
const float max_power = 8000.0;   // Max power in mA
float currentNow      = 0.0;      // Current INA226 reading
int   timeOn          = 0;        // PWM pulse duration
TaskHandle_t Core1;
TaskHandle_t Core2;


/***************************************************************
 * Hardware Pins
 ***************************************************************/
const int PWM_OUT = 4;


/***************************************************************
 * INA226 Configuration
 ***************************************************************/
INA226 INA(0x40);


/***************************************************************
 * Setup
 ***************************************************************/

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Start the INA226 Initialization
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
    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay
    
    currentNow = GetCurrent();

    timeOn = DoPWMTime(timeOn);
    
  }

}

/************************* FUNCTIONS *************************/

//Return the average of three samples current reading from IN226 in mA
float GetCurrent() {
  float current = 0;
  vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay
  
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

  