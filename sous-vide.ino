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
volatile long onTime = 0;

// ***********************************
// PID Variables and constants
// ***********************************

//Define Variables we'll be connecting to
double Setpoint = 50;
double Input;
double Output;

// pid tuning parameters
double Kp = 0.1;                              //http://www.over-engineered.com/projects/sous-vide-pid-controller/
double Ki = 150;                              //
double Kd = 0.45;                             //

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;
unsigned long now;

//Specify the links and initial tuning parameters
//Input = Current Temp of the SousVide machine 
//Output = ?
//Setpoint = Target Temp
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// This routine runs only once upon reset
void setup() {
    
    windowStartTime = millis();

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);       //Ensure the relay is turned off to begin with.
    
	Serial.begin(9600);
	delay(1000);
	
	myPID.SetTunings(Kp, Ki, Kd);
	
	myPID.SetSampleTime(1000);
	myPID.SetOutputLimits(0, WindowSize);
	
	myPID.SetMode(AUTOMATIC);
}
// This routine gets called repeatedly, like once every 5-15 milliseconds.
// Spark firmware interleaves background CPU activity associated with WiFi + Cloud activity with your code. 
// Make sure none of your code delays or blocks for too long (like more than 5 seconds), or weird things can happen.
void loop() {
    
    //windowStartTime = millis();
    
    //windowStartTime = millis();
    updateSensor();
    Input = sensor.GetTemp()/1000000;
	delay(1000);  // Wait for 1 second in off mode
    //getTemp();
    myPID.Compute();
    now = millis();
    
    
     if(now - windowStartTime>WindowSize)
    { //time to shift the Relay Window
        windowStartTime += WindowSize;
        Serial.println("New WIndow");
    }

    //double test = (double)(now - windowStartTime);
     if(Output > now - windowStartTime)
     {
         digitalWrite(RELAY_PIN,HIGH);
     }
    else 
    {
        digitalWrite(RELAY_PIN,LOW);
        Serial.println(now - windowStartTime);
        Serial.println((String)Output + "O");
    }
    
    //Serial.println(windowStartTime);
    //Serial.println((String)Input + "I");
    //Serial.println((String)Output + "O");
    //Serial.println(now);
    delay(1000);
    //Serial.println((String)windowStartTime + "W");*/
    
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
