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
//char cTemp[10];                       // Convert float to String for posting 
bool PIDActive = false;                 // Is PID Active
bool RelayUsable = true;                // Allows the relay to be overidden so can completely shut everything down! #SAFETY
double cTemp;                           // Stores the current temp.

// ***********************************
// PID Variables and constants
// ***********************************

//Define Variables we'll be connecting to
double Setpoint;                        //Target temprature
double Input;                           //Current Temprature
double Output;                          //How much needs to be output to make Input match setpoint. Effectively this decides if the relay is on or off. 

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

// ***********************************
// Spark Functions
// ***********************************
    Spark.function("SetSetPt", SetSetPoint);
    Spark.function("TogglePID", TogglePID);
    Spark.function("ToggleRelayU", ToggleRelayU);
    Spark.function("SetTunings", SetTunings);
    
// ***********************************
// Spark Variables
// ***********************************
    Spark.variable("GetSetPt", &Setpoint, DOUBLE);
    Spark.variable("GetCTemp", &cTemp, DOUBLE);         //MAKE A FUNCTION. GetCurrentTemp does not work if PID is on.
    Spark.variable("GetRelay", &RelayUsable, INT);
    Spark.variable("GetPIDAct", &PIDActive, INT);
    
    Spark.variable("GetKp", &Kp, DOUBLE);
    Spark.variable("GetKi", &Ki, DOUBLE);
    Spark.variable("GetKd", &Kd, DOUBLE);
    
// ***********************************
// Relay setup
// ***********************************

    pinMode(RELAY_PIN, OUTPUT);
    if(RelayUsable)digitalWrite(RELAY_PIN, LOW);       //Ensure the relay is turned off to begin with.
	delay(1000);
    
// ***********************************
// PID Setup
// ***********************************
    
    windowStartTime = millis();

	myPID.SetTunings(Kp, Ki, Kd);
	
	myPID.SetSampleTime(1000);
	myPID.SetOutputLimits(0, WindowSize);
	
	myPID.SetMode(AUTOMATIC);
}


void loop() 
{
    
    GetCurrentTemp();                                //Allows the temprature to be read, even when PID/Relay isn't enabled. Doesn't "smell" right. I would prefer it to be called by a Spark Function.
    
    if(PIDActive)
    {
        DoWork();
    }
    delay(1000);
}


void DoWork()
{
    
    Input = cTemp;
	delay(1000);  // Wait for 1 second in off mode
    myPID.Compute();
    now = millis();
    
    
    if(now - windowStartTime>WindowSize)
    { 
        windowStartTime += WindowSize;
    }

    if(Output > now - windowStartTime)
    {
        if(RelayUsable)digitalWrite(RELAY_PIN,HIGH);
    }
    else 
    {
        if(RelayUsable)digitalWrite(RELAY_PIN,LOW);
    }
}

//Gets the current temprature of the probe. Again doesn't really smell right, not a get. More of an update the sensor, assign that to cTemp and then return cTemp. Not as catchy though.
double GetCurrentTemp()
{
    updateSensor();
    cTemp = sensor.GetTemp()/1000000;
    return cTemp;
}

int SetTunings(String command)                 //Aquired from https://github.com/plan44/messagetorch/blob/master/messagetorch.cpp
{                                              //Would quite like for JSON to be passed in and read, however seems a bit OTT. For another day perhaps.
     int p = 0;
    while (p<(int)command.length()) 
    {
        int i = command.indexOf(',',p);
        if (i<0) i = command.length();
        int j = command.indexOf('=',p);
        if (j<0) break;
        String key = command.substring(p,j);
        String value = command.substring(j+1,i);
        double val = atof(value.c_str());
        if(key=="Kp")
            Kp = val;
        if(key=="Ki")
            Ki = val;
        if(key=="Kd")
            Kd = val;
        p = i + 1;
    }
    return (int)(Kp + Ki + Kd);
}

//Sets the target temprature(in Degrees) also used as the Setpoint for the PID. Used for Spark.Function "SetSetPt";
//FUTURE - Allow the addition of time to be added, 0 = no time, 16:00 means go at 16:00 or whatever in UNIX time.
int SetSetPoint(String args)
{   
    //To-Do
    //Error checking - if Input = Args and Active = true then return 1 else return 2? Worthwhile?
    Setpoint = atof(args.c_str());
    TogglePID("ON");
    
    if(RelayUsable)
    {
        return 1;
    }
    else
    {
        return 2;
    }
    
}

//Allows PID to be turned on or off
int TogglePID(String args)
{
    if(args == "ON")
    {
        PIDActive = true;
        return 1;
    }
    else if(args == "OFF")
    {
        PIDActive = false;
        if(RelayUsable)digitalWrite(RELAY_PIN,LOW);
        return 2;
    }
    else
    {
        return -1;
    }
}

//Allows the relay not to be used. Kill switch. Refactor into TogglePID?
//Is it needed at all? Seemsed like a good idea, at the same not sure if it acutally does anything that TogglePID doesn't do apart from allow relay to be turned on without PID which I don't know is useful!
int ToggleRelayU(String args)
{
    if(args == "ON")
    {
        RelayUsable = true;
        digitalWrite(RELAY_PIN,HIGH);
        return 1;
    }
    else if(args == "OFF")
    {
        RelayUsable = false;
        digitalWrite(RELAY_PIN,LOW);
        return 2;
    }
    else
    {
        return -1;
    }
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





































