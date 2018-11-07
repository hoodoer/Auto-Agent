// This program is the
// interface between FlightGear
// and GVP, and also contains the
// autonomous code to fly the aircraft
//
// Written by Drew Kirkpatrick, drew.kirkpatrick@gmail.com



#include <iostream>
using namespace std;
#include <pthread.h>
#include <signal.h>


// These defines are
// needed by gvpApiClient
// to correctly find
// the TAO corba headers
#define LINUX
#define ACE_5_4_6


#include "ucav.h"
#include "ucavStates.h"
#include "fgFdmReceiver.h"
#include "fgCtrlsTransmitter.h"
#include "joystick.h"
#include "net_ctrls.hxx"
#include "gvpApiClient.hpp"
#include "gvpTimer.hpp"
#include "gvpEntityStore.hpp"
#include "gvpEllipsoid.hpp"
#include "gvpDeadReckonEnum.hpp"
#include "gvpLegend.hpp"
#include "gvpPlugin.hpp"
#include "speech.h"






// Global variables

// Whether or not the program
// should end. Set to TRUE 
// to exit the program
bool DONE        = false;

// Use joystick, or autonomous
// mode. Defaults to autonomous
bool USEJOYSTICK = false;

// Default host that GVP
// is running on. Override with
// command line option
string GVPHOST = "localhost";


// Whether or not gvp is being used
// controls whether or not the gvp
// update thread is spawned.
bool USEGVP = false;



// Position datastruct from fgFdmReceiver
// Allowed to be used by both processes
// i.e. gvpUpdateThread writes it,
// main process reads it
localDataStruct posData;
float airSpeed  = 0.0;


bool SIMSTARTED = false;








// Handle signals sent to us by HOST
void signalHandler(int signal)
{
  switch(signal)
    {
    case SIGINT:  // Time to exit
      cout<<"Caught a SIGINT!"<<endl;
      DONE = TRUE;
      break;
    default:
      // Some other signal...
      break;
    }
}







// This thread handles receiving 
// position updates from the
// flight model in FlightGear
// and sending them to GVP
void *gvpUpdateThread(void*)
{
  // Used for briefly suspending the thread
  struct timespec ts;
  ts.tv_sec  = 0;
  ts.tv_nsec = 1000;

  // listen for position updates 
  // from FlightGear on port 5060
  FgFdmReceiver fdmInput(5060);


  // The GVP api client, so I can 
  // send position updates to the 
  // GVP viewpoint
  gvpApiClient gvp(GVPHOST.c_str(), 5050);



  // This is the viewpoint entity
  unsigned long view = gvp.getViewEntity();

  



  while(!DONE)
    {
      // Get the position from flightgear
      fdmInput.update();

      if (!SIMSTARTED)
	{
	  sayOutloud(false, "simulation has begun");
	  sleep(1);

	  sayOutloud(false, "starting autonomous agent in 3");
	  sayOutloud(false, "2");
	  sayOutloud(false, "1");
	  sayOutloud(true, "autonomous agent active");

	  SIMSTARTED = true;
	}

      posData  = fdmInput.getPositionGeo();
      airSpeed = fdmInput.getAirspeed();


      // Put that position into GVP
      gvp.setPositionGeo(currentTime(), view,
			 posData.latitude, posData.longitude,
			 posData.altitude * FEET2MET, posData.heading,
			 posData.pitch, posData.roll);

      // Suspend execution briefly
      nanosleep(&ts, NULL);
    }


  cout<<"Exiting gvpUpdateThread"<<endl;
}





// This thread handles receiving 
// position updates from the
// flight model in FlightGear
void *positionUpdateThread(void*)
{
  // Used for briefly suspending the thread
  struct timespec ts;
  ts.tv_sec  = 0;
  ts.tv_nsec = 1000;

  // listen for position updates 
  // from FlightGear on port 5060
  FgFdmReceiver fdmInput(5060);




  while(!DONE)
    {
      // Get the position from flightgear
      fdmInput.update();

      if (!SIMSTARTED)
	{
	  sayOutloud(false, "simulation has begun");
	  sleep(1);

	  sayOutloud(false, "starting autonomous agent in 3");
	  sayOutloud(false, "2");
	  sayOutloud(false, "1");
	  sayOutloud(true, "autonomous agent active");

	  SIMSTARTED = true;
	}

      posData  = fdmInput.getPositionGeo();
      airSpeed = fdmInput.getAirspeed();

      // Suspend execution briefly
      nanosleep(&ts, NULL);
    }


  cout<<"Exiting positionUpdateThread"<<endl;
}









// Main loop sends control to FlightGear to control the
// flight model, or FDM
int main(int argc, char **argv)
{
  double newElevator;
  double newAileron;
  double newRudder;
  double newThrottle;

  bool simStartInitialized = false;
  bool gearRaised          = false;


  Ucav uav(1);


  // Command line options...
  switch(argc)
    {
    case 1:
      USEGVP = false;
      break;

    case 2:
      USEGVP  = true;

      GVPHOST = argv[1];
      break;

    case 3: 
      USEGVP  = true;

      GVPHOST = argv[1];

      if (!strcmp(argv[2], "joy"))
	{
	  cout<<"Using joystick mode"<<endl;
	  sleep(1);
	  USEJOYSTICK = TRUE;
	}
      break;

    default:
      cout<<"Usage: autoAgent [gvphost] [joy]\n"<<endl;
    }


  
  // Used for briefly suspending the thread
  struct timespec ts;
  ts.tv_sec  = 0;
  ts.tv_nsec = 1000;


  // Network control of FlightGear
  FgCtrlsTransmitter ctrlsOutput("localhost", 5070);


  // Get the first joystick on the system,
  // For use when not autonomously flying
  Joystick *js;
  joystickValues jsVals;
  
  

  // Thread to pull data from the FDM of
  // Flightgear, and send it to GVP
  pthread_t gvpUpdate;



  // If GVP isn't being used,
  // this thread will be spawned
  // to grab FDM data we need
  pthread_t positionUpdate;



  // Creat the thread that will update GVP
  if (USEGVP)
    {
      pthread_create(&gvpUpdate, NULL, gvpUpdateThread, NULL);
    }
  else
    {
      pthread_create(&positionUpdate, NULL, 
		     positionUpdateThread, NULL);
    }

  // Catch sigint to shutdown cleanly
  signal(SIGINT,  signalHandler);




  
  if (USEJOYSTICK)
    {
      // Tell the uav we're in joystick mode
      uav.setAutoMode(false);
      js = new Joystick(0);

      sayOutloud(false, "Starting autoagent in manual mode");
    }
  else
    {
      // Tell the uav to fly autonomously
      uav.setAutoMode(true);

      sayOutloud(false, "Starting autoagent in autonomous mode");
   }



  while(!DONE)
    {
      if (USEJOYSTICK)
	{
	  // Retrieve joystick values
	  jsVals = js->getJoystickValues();
      
	  // Set values for sending to FlightGear
	  ctrlsOutput.setElevator(jsVals.elevator);
	  ctrlsOutput.setAileron(jsVals.aileron);
	  ctrlsOutput.setRudder(jsVals.rudder);
	  ctrlsOutput.setThrottle(0, jsVals.throttle);
	  ctrlsOutput.setThrottle(1, jsVals.throttle);
	  ctrlsOutput.setThrottle(2, jsVals.throttle);
	  ctrlsOutput.setThrottle(3, jsVals.throttle);
	}
      else
	{
	  // Wait until we receive some data from FlightGear
	  while(!SIMSTARTED)
	    {
	      if (!simStartInitialized)
		{
		  sayOutloud(true, "Waiting for FlightGear to start");
		  simStartInitialized = true;
		}
	      cout<<"Waiting for FlightGear packets..."<<endl;
	      nanosleep(&ts, NULL);
	    }
	  

	  // Run Finite State Machine...
	  uav.setPositionGeo(posData.latitude, 
			     posData.longitude,
			     posData.altitude * FEET2MET,
			     posData.agl      * FEET2MET,
			     posData.heading, 
			     posData.pitch, 
			     posData.roll);

	  uav.setAirspeed(airSpeed);
	  
	  if ((!gearRaised) && (!uav.getGearDown()))
	    {
	      sayOutloud(true, "raising landing gear");
	      ctrlsOutput.setGear(0);
	      gearRaised = true;
	    }

	  uav.update();
	  
	  uav.getControlPositions(newElevator, newAileron,
				  newRudder, newThrottle);

	  ctrlsOutput.setElevator(newElevator);
	  ctrlsOutput.setAileron(newAileron);
	  ctrlsOutput.setRudder(newRudder);
	  ctrlsOutput.setThrottle(0, newThrottle);
	  ctrlsOutput.setThrottle(1, newThrottle);
	  ctrlsOutput.setThrottle(2, newThrottle);
	  ctrlsOutput.setThrottle(3, newThrottle);
	}

      // Send data to FlightGear
      ctrlsOutput.sendData();

      // Suspend execution briefly
      nanosleep(&ts, NULL);
    }
  cout<<"Main loop exited..."<<endl;


  if (USEJOYSTICK)
    {
      delete js;
    }



  if (USEGVP)
    {
      pthread_join(gvpUpdate, NULL);
    }
  else
    {
      pthread_join(positionUpdate, NULL);
    }

  sayOutloud(true, "autoagent program stopped");
  cout<<"Done."<<endl;

  return 0;
}
