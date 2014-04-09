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
double Setpoint;
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

// ***********************************
// Spark Functions
// ***********************************
    Spark.function("SetSetPt", SetSetPoint);
    Spark.function("TogglePID", TogglePID);
    Spark.function("ToggleRelayU", ToggleRelayU);
    //SET Ki, Kp, Kd
    //Set RelayActive
    
// ***********************************
// Spark Variables
// ***********************************
    Spark.variable("GetSetPt", &Setpoint, DOUBLE);
    Spark.variable("GetCTemp", &cTemp, DOUBLE);         //MAKE A FUNCTION. GetCurrentTemp does not work if PID is on.
    Spark.variable("GetRelay", &RelayUsable, INT);
    Spark.variable("GetPIDAct", &PIDActive, INT);
    
    windowStartTime = millis();

    pinMode(RELAY_PIN, OUTPUT);
    if(RelayUsable)digitalWrite(RELAY_PIN, LOW);       //Ensure the relay is turned off to begin with.
    
	delay(1000);
	
	myPID.SetTunings(Kp, Ki, Kd);
	
	myPID.SetSampleTime(1000);
	myPID.SetOutputLimits(0, WindowSize);
	
	myPID.SetMode(AUTOMATIC);
}


void loop() 
{
    
    if(PIDActive)
    {
        DoWork();
    }
    delay(1000);
}


void DoWork()
{
    
    Input = GetCurrentTemp();
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

//Gets the current temprature of the probe. Used to set the input
double GetCurrentTemp()
{
    updateSensor();
    cTemp = sensor.GetTemp()/1000000;
    return cTemp;
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
//Is it needed at all? Seems like a good idea, at the same not sure if it acutally does anything that TogglePID doesn't do apart from allow relay to be turned on without PID which I don't know is useful!
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





































