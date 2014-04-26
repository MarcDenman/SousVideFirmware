// PID Library
#include "PID.h"

//Includes for the DS1820 Temp Sensor
#include "ds1820.h"
#include "OneWire.h"

// ***********************************
// Pin Definitions
// ***********************************
#define ONE_WIRE_PIN D6                             //Pin for DS1820 sensor
#define RELAY_PIN D0                                //Pin to control relay

// ***********************************
// Defining Xively  ID's
//  ***********************************
#define FEED_ID "FAKE_SHORT_NO"
#define XIVELY_API_KEY "FAKE_LONG_NO"

// ***********************************
// Sensor Vairables and Constants
// plugged into D6
// ***********************************
static OneWire oneWire(ONE_WIRE_PIN);               // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
static DS1820Sensor sensor;

// ***********************************
// Global Definitions
// ***********************************
bool PIDActive = false;                             // Is PID Controller Active?
bool RelayUsable = false;                           // Allows the relay to be overridden so can completely shut everything down! #SAFETY
double cTemp =0 ;                                   // Stores the current temp. Refactor cTemp into Input? Seems completely pointless having exactly the same data.

TCPClient client;                                   //Used for sending posting details, temp etc.
unsigned long LastSyncTime = 0;                     //Keeps count of the last time the temp was sent to the cloud (syncs every 30 secs).

// ***********************************
// PID Variables and constants
// ***********************************

double Setpoint = 0;                                //Target temperature - Set to zero so GetSetPt can be called from the cloud before it is set.
double Input = 0;                                   //Current Temperature
double Output;                                      //How much needs to be output to make Input match SetPoint. Effectively this decides if the relay is on or off. 

// PID Tuning parameters
double Kp = 0.1;                                    //http://www.over-engineered.com/projects/sous-vide-pid-controller/
double Ki = 150;                                    //
double Kd = 0.45;                                   //

// 10 second Time Proportional Output window. Checks what the PID should do every 10 seconds.
int WindowSize = 10000; 
unsigned long windowStartTime;
unsigned long now;


//Specify the links and initial tuning parameters
//Input = Current Temp of the SousVide machine 
//Output = On or off the relay. Is actually a double and if under 1000 then relay turned off.
//Setpoint = Target Temp
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {

// ***********************************
// Spark Functions
// ***********************************
    Spark.function("SetSetPt", SetSetPoint);
    Spark.function("TogglePID", TogglePID);
    Spark.function("SetTunings", SetTunings);
    
// ***********************************
// Spark Variables
// ***********************************
    Spark.variable("GetSetPt", &Setpoint, DOUBLE);  
    Spark.variable("GetCTemp", &cTemp, DOUBLE);         
    Spark.variable("GetRelay", &RelayUsable, INT);
    Spark.variable("GetPIDAct", &PIDActive, INT);
    
    Spark.variable("GetKp", &Kp, DOUBLE);           //This could probably be made into one variable returning Kp, Ki, Kd as a one nice big string
    Spark.variable("GetKi", &Ki, DOUBLE);
    Spark.variable("GetKd", &Kd, DOUBLE);
    
// ***********************************
// Relay setup
// ***********************************

    pinMode(RELAY_PIN, OUTPUT);
    if(RelayUsable)digitalWrite(RELAY_PIN, LOW);    //Ensure the relay is turned off to begin with.
	delay(1000);
    
// ***********************************
// PID Setup
// ***********************************
    
    windowStartTime = millis();

	myPID.SetTunings(Kp, Ki, Kd);
	
	myPID.SetSampleTime(1000);                      //95% certain I don't need this line. Don't want to check that theory though as cooker is con.
	myPID.SetOutputLimits(0, WindowSize);
	
	myPID.SetMode(AUTOMATIC);
}


void loop() 
{
                                                     //REMEMBER - ALL DigitalWrites to relay MUST go through RelayActive first.
    GetCurrentTemp();                                //Allows the temperature to be read, even when PID/Relay isn't enabled. Doesn't "smell" right. I would prefer it to be called by a Spark Function otherwise it is updating every second when no-one really cares.
    
    if(PIDActive)                                    //If PID has been turned on then allow do work and to control relay.
    {
        DoWork();
    }
    
    delay(1000);

    if (millis()-LastSyncTime>1000*30)               //Sync the current temp with Xively.
    {
        xivelyTemp();
        LastSyncTime = millis();
     }

}

void xivelyTemp() {

    if (client.connect("api.xively.com", 8081)) 
    {
        // Connection succesful, update datastreams
        client.print("{");
        client.print("  \"method\" : \"put\",");
        client.print("  \"resource\" : \"/feeds/");
        client.print(FEED_ID);
        client.print("\",");
        client.print("  \"params\" : {},");
        client.print("  \"headers\" : {\"X-ApiKey\":\"");
        client.print(XIVELY_API_KEY);
        client.print("\"},");
        client.print("  \"body\" :");
        client.print("    {");
        client.print("      \"version\" : \"1.0.0\",");
        client.print("      \"datastreams\" : [");
        client.print("        {");
        client.print("          \"id\" : \"CurrentTemp\",");    //Feed or channel to be updated by put command.
        client.print("          \"current_value\" : \"");
        client.print(cTemp);                                    //Value to be put on the Xively server
        client.print("\"");
        client.print("        }");
        client.print("      ]");
        client.print("    },");
        client.print("  \"token\" : \"0x123abc\"");
        client.print("}");
        client.println();
    } 
    else 
    {
    }
    
    client.flush();
    delay(2000);                                     //Stop connection being closed before everything has been sent. Stupid crappy router.
    client.stop();
}


void DoWork()
{
    
    Input = cTemp;
	delay(1000); 
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

//Gets the current temperature of the probe. Again doesn't really smell right, not a get. More of an update the sensor, assign that to cTemp and then return cTemp. Not as catchy though. Refactor Input instead of Ctemp
double GetCurrentTemp()
{
    updateSensor();
    cTemp = sensor.GetTemp()/1000000.0;
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

int SetSetPoint(String args)                //Sets the target temperature(in Degrees) also used as the Setpoint for the PID. Used for Spark.Function "SetSetPt";
{   
    Setpoint = atof(args.c_str());          //Error checking - if Input = Args and Active = true then return 1 else return 2? Worthwhile?
    TogglePID("ON");
    
    if(RelayUsable)                         //Doesn't do anything, TogglePID makes relay usable anyway. May be removed - Return TogglePID("ON"); instead.
    {
        return 1;
    }
    else
    {
        return 2;
    }
    
}

                                            //Allows PID to be turned on or off
int TogglePID(String args)                  //Can't say I particularly like using "ON" or "OFF", string comparison feels flaky.
{
    if(args == "ON")
    {
        PIDActive = true;
        RelayUsable = true;
        return 1;
    }
    else if(args == "OFF")
    {
        PIDActive = false;
        if(RelayUsable)digitalWrite(RELAY_PIN,LOW);
        RelayUsable = false;
        return 2;
    }
    else
    {
        return -1;
    }
}

int updateSensor()                          //I wish I could remember where I stole this from.
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