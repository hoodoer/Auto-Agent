#include <iostream>
using namespace std;
#include <fstream>



#ifndef FALSE
#define FALSE 0
#define TRUE !FALSE
#endif


#include "ucav.h"
#include "ucavStates.h"



// Ucav constructor, initializes 
// variables, and also checks to 
// see if it's supposed to do 
// an emergency start. 
Ucav::Ucav(int newId):BaseEntity(newId)
{
  stateMachine = new StateMachine<Ucav>(this);
  stateMachine->setGlobalState(GlobalState::Instance());
  pilot = new Pilot;

  position.lat            = 0.0;       
  position.lon            = 0.0;       
  position.alt            = 0.0;
  position.radAlt         = 0.0;
  position.heading        = 0.0; 
  position.pitch          = 0.0;   
  position.roll           = 0.0;    
  
  data.systemTime         = 0;
  data.autoMode           = TRUE;
  data.stalled            = FALSE;
  data.errorState         = FALSE;
  data.detectThreat       = FALSE;
  data.haveNavCourse      = FALSE;
  data.gearDown           = TRUE;
  data.numberOfWaypoints  = 0;
  data.currentWaypoint    = 0;

  flightControls.elevator = 0.0;
  flightControls.aileron  = 0.0;
  flightControls.rudder   = 0.0;
  flightControls.throttle = 0.0;

  readInWaypoints();

  if(!emergencyStart())
    {
      stateMachine->setCurrentState(TakeOffState::Instance());

      // Call this so the entry function is called...
      stateMachine->changeState(TakeOffState::Instance());

      data.recoveryBoot = FALSE;
    }
  else
    {
      // It's an emergency boot
      // global state will handle
      // it from here...
      data.recoveryBoot = TRUE;
    }
}





// Ucav destructor, checks to see
// if the Ucav is in an error state
// on shutdown, and sets up a 
// file with some last state 
// information. The constructor
// looks for this file to help 
// it setup a faster recovery on
// "reboot" of the Ucav.
Ucav::~Ucav()
{
  cout<<"In Ucav destructor..."<<endl;

  if (data.errorState || 
      stateMachine->isInState(RecoveryState::Instance()))
    {
      cout<<"Ucav software exiting in an bad state, "
	  <<"save information to memory for use "
	  <<"in next boot"<<endl;
      emergencyDataDump();
    }

  if (stateMachine)
    {
      delete stateMachine;
    }

  if ((data.haveNavCourse) && (waypoints != NULL))
    {
      delete waypoints;
    }
  
  if (pilot)
    {
      delete pilot;
    }
}





// This is the primary execution function, 
// must be called repeatedly for the 
// entity and Finite State Machine to
// function. 
void Ucav::update()
{
  // To make it easier to track each tick:
  cout<<endl<<"*************************************************"
      <<endl<<endl;
 
  pollSensors();

  if(data.autoMode)
    {
      stateMachine->update();
      pilot->setPositionGeo(position.lat, position.lon,
			    position.alt, position.radAlt, 
			    position.heading, position.pitch, 
			    position.roll);

      pilot->update();

      pilot->getControlPositions(flightControls.elevator, 
				 flightControls.aileron,
				 flightControls.rudder,
				 flightControls.throttle);
    }
  else
    {
      cout<<"Manual mode, need a human pilot!"<<endl;
    }


  // For now, systemTime is just
  // number of "ticks". Handy for
  // demonstrating state switching
  // by using systemTime to drive the
  // pollSensors() function
  data.systemTime++;
}






void Ucav::setAutoMode(bool newMode)
{
  data.autoMode = newMode;
}




bool Ucav::getAutoMode() const
{
  return data.autoMode;
}


double Ucav::getBaroAltitude() const
{
  return position.alt;
}


// This would be handled by a flight model
// in a real simulation
void Ucav::setBaroAltitude(double newAltitude)
{
  position.alt = newAltitude;
}



void Ucav::getPositionGeo(double &lat, double &lon, 
			  double &alt, double &radAlt,
			  double &heading, double &pitch, 
			  double &roll)
{
  lat     = position.lat;
  lon     = position.lon;
  alt     = position.alt;
  radAlt  = position.radAlt;
  pitch   = position.pitch;
  roll    = position.roll;
  heading = position.heading;
}


void Ucav::setPositionGeo(double lat, double lon, 
			  double alt, double radAlt,
			  double heading, double pitch, 
			  double roll)
{
  position.lat     = lat;
  position.lon     = lon;
  position.alt     = alt;
  position.radAlt  = radAlt;
  position.pitch   = pitch;
  position.roll    = roll;
  position.heading = heading;
}




void Ucav::getControlPositions(double &elevator, double &aileron, 
			 double &rudder, double &throttle)
{
  elevator = flightControls.elevator;
  aileron  = flightControls.aileron;
  rudder   = flightControls.rudder;
  throttle = flightControls.throttle;
}


void Ucav::setControlPositions(double elevator, double aileron, 
			 double rudder, double throttle)
{
  flightControls.elevator = elevator;
  flightControls.aileron  = aileron;
  flightControls.rudder   = rudder; 
  flightControls.throttle = throttle;
}




void Ucav::getLatLon(double &lat, double &lon)
{
  lat = position.lat;
  lon = position.lon;
}



void Ucav::setLatLon(double newLat, double newLon)
{
  position.lat = newLat;
  position.lon = newLon;
}



float Ucav::getAirspeed() const
{
  return data.airSpeed;
}




void Ucav::setAirspeed(double newAirspeed)
{
  data.airSpeed = newAirspeed;
  pilot->setAirspeed(newAirspeed);
}



bool Ucav::getGearDown() const
{
  return data.gearDown;
}


void Ucav::setGearDown(bool newState)
{
  data.gearDown = newState;
}



int Ucav::getNumberOfWaypoints() const
{
  return data.numberOfWaypoints;
}


int Ucav::getNextWaypoint() const
{
  return data.currentWaypoint;
}



void Ucav::getPositionOfNextWaypoint(double &lat, double &lon, 
				     double &alt)
{
  if (data.haveNavCourse)
    {
      waypointData wyptData = waypoints->at(data.currentWaypoint-1); 
      lat = wyptData.lat;
      lon = wyptData.lon;
      alt = wyptData.alt;
    }
  else
    {
      cout<<"Warning, trying to retrieve a waypoint "
	  <<"position when there aren't any!"<<endl;

      lat = 0.0;
      lon = 0.0;
      alt = 0.0;
    }
}




void Ucav::setNextWaypoint(int newNextWypt)
{
  if (newNextWypt <= data.numberOfWaypoints)
    {
      data.currentWaypoint = newNextWypt;
    }
  else
    {
      cout<<"Ucav::setNextWypt error, bigger than last wypt!"<<endl;
    }
}





void Ucav::setTargetSeekerSideBoundary(float newSideAngleBoundary)
{
  pilot->setTargetSeekerSideBoundary(newSideAngleBoundary);
}


float Ucav::getTargetSeekerSideBoundary() const
{
  return pilot->getTargetSeekerSideBoundary();
}


void Ucav::setTargetSeekerVertBoundary(float newVertAngleBoundary)
{
  pilot->setTargetSeekerVertBoundary(newVertAngleBoundary);
}


float Ucav::getTargetSeekerVertBoundary() const
{
  return pilot->getTargetSeekerVertBoundary();
}



void Ucav::setTargetSeekerTargetPos(double lat, double lon, double alt)
{
  pilot->setTargetSeekerTargetPos(lat, lon, alt);
  pilot->setPilotMode(TargetSeekMode);
}




float Ucav::getTargetDistance() const
{
  return pilot->getTargetDistance();
}



bool Ucav::getStallState() const
{
  return data.stalled;
}


void Ucav::notStalledAnymore()
{
  data.stalled = FALSE;
}


bool Ucav::getErrorState() const
{
  return data.errorState;
}

bool Ucav::getThreatDetection() const
{
  return data.detectThreat;
}


void Ucav::threatKilled()
{
  data.detectThreat = FALSE;
}


bool Ucav::getRecoveryBoot() const
{
  return data.recoveryBoot;
}


void Ucav::recoveryBootFinished()
{
  data.recoveryBoot = FALSE;
}


void Ucav::setPilotMode(FlyingMode newMode)
{
  pilot->setPilotMode(newMode);
}



/**********************************************/
// Private member functions


// This function is simulating some
// simple sensors, all triggered 
// by number of cycles
void Ucav::pollSensors()
{
  //  cout<<"Updating the sensor data for the Ucav..."<<endl<<endl;

  // No sensors now... 

  //
  // This would be an excellent place to have an 
  // entity store sync'ed with the global
  // GVP entity manager for networked sims...

}



// This function simply stores
// whether or not the Ucav was in 
// a stalled state on exit, and if
// there was a threat when it last
// exited. This only happens if
// the Ucav had an error state on
// exit. 
void Ucav::emergencyDataDump()
{
  cout<<"In emergencyDataDump()"<<endl;
  ofstream emergencyFile("./emergencyFile", ios::out);
  
  if (!emergencyFile)
    {
      cout<<"Failed to open emergencyFile"<<endl;
    }
  else
    {
      // Simply store wether we were 
      // stalled when we died, or
      // if there was a threat when 
      // we died...

      if (data.stalled)
	emergencyFile<<"1 ";
      else
	emergencyFile<<"0 ";

      if (data.detectThreat)
	emergencyFile<<"1 ";
      else
	emergencyFile<<"0 ";

      emergencyFile<<endl;
	
      emergencyFile.close();
    }
}



// This file will read in an 
// emergencyFile to see if the 
// Ucav was stalled, and if there
// was a threat last time it exited. 
bool  Ucav::emergencyStart()
{
  int readInteger = 0;

  cout<<"In emergencyStart()"<<endl;

  ifstream emergencyFile("./emergencyFile", ios::in);

  if (!emergencyFile)
    {
      cout<<"There isn't an emergency file, "
	  <<"assume it's a normal startup..."<<endl;
      return FALSE;
    }
  else
    {
      cout<<"There is an emergency file, "
	  <<"jump start the initialization!"<<endl;

      emergencyFile>>readInteger;

      if (readInteger == 1)
	data.stalled = TRUE;
      else
	data.stalled = FALSE;

      emergencyFile>>readInteger;

       if (readInteger == 1)
	data.detectThreat = TRUE;
      else
	data.detectThreat = FALSE;
       
     return TRUE;
    }
}



// This function checks for a waypoint
// file, and if it exists reads it in
// to a vector, and sets the waypoints
// vector pointer for the Ucav class. 
void Ucav::readInWaypoints()
{
  int waypointCounter = 0;
  waypointData waypointStruct;

  cout<<"In readInWaypoints()"<<endl;
  
  ifstream waypointFile("./waypointsFile", ios::in);

  if (!waypointFile)
    {
      cout<<"There isn't a waypointsFile, not "
	  <<"loading a course."<<endl;
      data.haveNavCourse = FALSE;
    }
  else
    {
      waypoints = new vector<waypointData>;
      data.haveNavCourse = TRUE;

      cout<<"Loading a navigation course from waypointsFile"<<endl;

      while(!waypointFile.fail())
      {
	waypointFile>>waypointStruct.lat;
	waypointFile>>waypointStruct.lon;
	waypointFile>>waypointStruct.alt;

	waypoints->push_back(waypointStruct);
	waypointCounter++;
      }

      // Since eof is always behind by one
      waypointCounter--;
      waypoints->pop_back();
      cout<<"Done reading in waypoints..."<<endl;
      data.numberOfWaypoints = waypointCounter;
      data.currentWaypoint   = 1;
      waypointFile.close();
    }
}
