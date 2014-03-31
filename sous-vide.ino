// PID Library
#include "PID.h"

//Includes for the DS1820 Temp Sensor
#include "ds1820.h"
#include "OneWire.h"

// ***********************************
// Pin Definitions
// ***********************************
#define ONE_WIRE_PIN D6                 //Pin for DS1820 sensor
#define RELAY_PIN D0                    //Pin to control relay

// ***********************************
// Sensor Vairables and Constants
// plugged into D6
// ***********************************
static OneWire oneWire(ONE_WIRE_PIN);   // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
static DS1820Sensor sensor;

// ***********************************
// Global Definitions
// ***********************************
char cTemp[10];                         //Convert float to String for posting 


// ***********************************
// PID Variables and constants
// ***********************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

// pid tuning parameters
double Kp;                              //
double Ki;                              //
double Kd;                              //

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// This routine runs only once upon reset
void setup() {

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);       //Ensure the relay is turned off to begin with.
	Serial.begin(9600);
	delay(1000);

}
// This routine gets called repeatedly, like once every 5-15 milliseconds.
// Spark firmware interleaves background CPU activity associated with WiFi + Cloud activity with your code. 
// Make sure none of your code delays or blocks for too long (like more than 5 seconds), or weird things can happen.
void loop() {
    updateSensor();
    
	delay(1000);  // Wait for 1 second in off mode
    getTemp();
    
    delay(1000);
}           





int updateSensor() 
{
  uint8_t addr[8];
  unsigned long now = millis();

  // Are we already working on a sensor? service it, possibly writting value,
  if (sensor.Initialized() || sensor.Busy()) 
  {
    if (sensor.Update(now)) 
    {
            uint8_t *addr=sensor.m_addr;
            //sensor.Reset();
    } 
    else if (sensor.Busy())
    {
          // More cycles needed on this sensor
          return 1;
    } 
    else 
    {
        // finished or not started
    }
  }
  // First time, or finished with last sensor; clean up, and look more more devices.
  int more_search = oneWire.search(addr);
  
  if (!more_search) 
  {
    // Bus exhausted; start over
    oneWire.reset_search();
    return 0;
  }
  
  // New sensor. Initialize and start work.
  sensor.Initialize(&oneWire, addr);
  sensor.Update(now);
  return 1;
}

 void getTemp()
 {
            long tempd =sensor.GetTemp();
            sprintf(cTemp, "%ld", tempd);
            Serial.println(cTemp);
 }



































