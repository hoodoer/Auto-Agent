// This file contains the
// code need to actually control
// the aircraft. It uses a neural
// network to manipulate the 
// aircraft controls. The neural network
// class was taken from:
// AI for Game Developers, Orielly press
// and is located in the neural subdirectory.
// Some modifications were added to the file 
// handling of that neural network class. 
// 
// Drew Kirkpatrick, drew.kirkpatrick@gmail.com

#include <iostream>
using namespace std;


#include "neuralNet.h"
#include "pilot.h"
#include "speech.h"


#define ROLLCUTOFF 5.0

#ifndef RAD2DEG
#define RAD2DEG  57.29578
#endif





Pilot::Pilot()
{
  // Assume we're starting off on the ground
  // and stopped
  currentMode = SitStillMode;

  positionData.lat        = 0;
  positionData.lon        = 0;
  positionData.alt        = 0;
  positionData.radAlt     = 0;
  positionData.heading    = 0;
  positionData.pitch      = 0;
  positionData.roll       = 0;  
  positionData.airSpeed   = 0;

  goalData.targetLat      = 0;
  goalData.targetLon 	  = 0;
  goalData.targetAlt 	  = 0;
  goalData.holdHeading 	  = 0;
  goalData.holdPitch 	  = 0;
  goalData.holdRoll 	  = 0;
  goalData.holdAirSpeed   = 0;

  flightControls.elevator = 0;
  flightControls.aileron  = 0;
  flightControls.rudder   = 0;
  flightControls.throttle = 0;

  targetSeekerBrain = "./targetSeekerNeuralNet";
  trgtSeekNet = new TargetSeeker(5.0, 5.0, targetSeekerBrain);
}




Pilot::~Pilot()
{
  delete trgtSeekNet;
}



void Pilot::setPositionGeo(double lat, double lon, double alt, 
			   double radAlt, double heading, 
			   double pitch, double roll)
{
  positionData.lat     = lat;
  positionData.lon     = lon;
  positionData.alt     = alt;
  positionData.radAlt  = radAlt;
  positionData.heading = heading;
  positionData.pitch   = pitch;
  positionData.roll    = roll;  
}




void Pilot::setAirspeed(double newAirspeed)
{
  positionData.airSpeed = newAirspeed;
}




void Pilot::setTargetSeekerTargetPos(double lat, double lon, double alt)
{
  trgtSeekNet->setTargetPosition(lat, lon, alt);
}


float Pilot::getTargetDistance() const
{
  return trgtSeekNet->getTargetDistance();
}




void Pilot::setHoldAttitude(double heading, double pitch, 
			    double roll)
{
  goalData.holdHeading = heading;
  goalData.holdPitch   = pitch;
  goalData.holdRoll    = roll;
}



void Pilot::setHoldAirspeed(double airspeed)
{
  goalData.holdAirSpeed = airspeed;
}



void Pilot::getControlPositions(double &elevator, double &aileron, 
				double &rudder, double &throttle)
{
  elevator = flightControls.elevator;
  aileron  = flightControls.aileron;
  rudder   = flightControls.rudder;
  throttle = flightControls.throttle;
}



void Pilot::setControlPositions(double elevator, double aileron, 
			 double rudder, double throttle)
{
  flightControls.elevator = elevator;
  flightControls.aileron  = aileron;
  flightControls.rudder   = rudder;
  flightControls.throttle = throttle;
}




void Pilot::setSmoothedControlPositions(double elevator, double aileron, 
					double rudder, double throttle)
{
  static const int meanPositions = 15;
  static bool      initialized   = false;
  static int       headIndex = 0;

  static double elevators[meanPositions];
  static double ailerons [meanPositions];
  static double rudders  [meanPositions];
  static double throttles[meanPositions];
  
  static double smoothedElevator = 0.0;
  static double smoothedAileron  = 0.0;
  static double smoothedRudder   = 0.0;
  static double smoothedThrottle = 0.0;

  static double summedElevator;
  static double summedAileron;
  static double summedRudder;
  static double summedThrottle;

  int counter = 0;


  summedElevator = 0.0;
  summedAileron  = 0.0;
  summedRudder   = 0.0;
  summedThrottle = 0.0;

  if (!initialized)
    {
      for (int i = 0; i < meanPositions; i++)
	{
	  elevators[i] = 0.0;
	  ailerons [i] = 0.0;
	  rudders  [i] = 0.0;
	  throttles[i] = 0.0;
	}

      initialized = true;
    }

  elevators[headIndex] = elevator;
  ailerons [headIndex] = aileron; 
  rudders  [headIndex] = rudder;
  throttles[headIndex] = throttle;
  
  counter = headIndex;

  for (int i = 0; i < meanPositions; i++)
    {
      summedElevator += elevators[counter];
      summedAileron  += ailerons [counter];
      summedRudder   += rudders  [counter];
      summedThrottle += throttles[counter];

      counter--;
      
      if (counter < 0)
	{
	  counter = meanPositions;
	}
    }

  smoothedElevator = summedElevator/(double)meanPositions;
  smoothedAileron  = summedAileron /(double)meanPositions;
  smoothedRudder   = summedRudder  /(double)meanPositions;
  smoothedThrottle = summedThrottle/(double)meanPositions;

  flightControls.elevator = smoothedElevator;
  flightControls.aileron  = smoothedAileron;
  flightControls.rudder   = smoothedRudder;
  flightControls.throttle = smoothedThrottle;
  
  headIndex++;

  if (headIndex > meanPositions)
    {
      headIndex = 0;
    }
}




void Pilot::setTargetSeekerSideBoundary(float newSideAngleBoundary)
{
  trgtSeekNet->setSideAngleBoundary(newSideAngleBoundary);
}


float Pilot::getTargetSeekerSideBoundary() const
{
  return trgtSeekNet->getSideAngleBoundary();
}



void Pilot::setTargetSeekerVertBoundary(float newVertAngleBoundary)
{
  trgtSeekNet->setVertAngleBoundary(newVertAngleBoundary);
}


float Pilot::getTargetSeekerVertBoundary() const
{
  return trgtSeekNet->getVertAngleBoundary();
}



void Pilot::setPilotMode(FlyingMode newMode)
{
  if ((newMode < 1 ) || (newMode >= endList))
    {
      cout<<"Error, trying to Pilot::setPilotMode to "
	  <<"to an invalid mode ("<<newMode<<")"<<endl;
      exit(1);
    }
  else
    {
      currentMode = newMode;
    }
}





void Pilot::update()
{
  static float  targetBearing      = 0.0;
  static float  targetElevation    = 0.0;
  static float  sideAngleBoundary  = 0.0;
  static float  vertAngleBoundary  = 0.0;

  static double tempElevator       = 0.0;
  static double tempAileron        = 0.0;
  static double tempRudder         = 0.0;

  switch(currentMode)
    {
    case TargetSeekMode:
      trgtSeekNet->setAircraftPosition(positionData.lat, positionData.lon,
				       positionData.alt, positionData.heading,
				       positionData.pitch, positionData.roll);
      trgtSeekNet->calculateNewTargetBearingElevationDist();
      trgtSeekNet->getAircraftControls(tempElevator, 
				       tempAileron,
				       tempRudder);
      setSmoothedControlPositions(tempElevator, tempAileron,
				  tempRudder, 1.0);
      break;

    case AttitudeHoldMode:
      cout<<"In Pilot::update, in AttitudeHoldMode"<<endl;
      holdAttitude();
      
      break;

    case TakeOffMode:
      cout<<"In Pilot::update, in TakeOffMode"<<endl;
      takeoffAircraft();
      break;

    case SitStillMode:
      
      break;
      
    default:
      cout<<"Pilot::update(), invalid currentMode: "<<currentMode<<endl;
    }
}




/*********************************************************

Private functions

*********************************************************/


void Pilot::takeoffAircraft()
{
  // Constants to strive for...
  static const float takeOffSpeed =  175.0;
  static const float wowCutoff    =    3.0;
  static const float targetPitch  =   30.0;
  static const float targetRoll   =    0.0;


  static double targetHeading  = 0.0;
  static bool   initialized    = false;

  // This should be a valid initial heading down the runway
  // Need the heading down the runway so we don't go 
  // off the side of the runway
  if ((!initialized) && (positionData.heading != 0.0) && 
      (positionData.airSpeed != 0.0))
    {
      initialized   = true;
      targetHeading = positionData.heading;
    }

  // Steer rudder left and right to 
  // keep us straight
  if ((targetHeading - positionData.heading) > 0.0)
    {
      flightControls.rudder = 0.05;
    }
  else if ((targetHeading - positionData.heading) < 0.0)
    {
      flightControls.rudder = -0.05;
    }
  else
    {
      flightControls.rudder = 0.0;
    }


  // Time to lift off, set an attitude to hold, and let the 
  // attitudeHoldMode handle it...
  if (positionData.airSpeed >= takeOffSpeed)
    {
      // Time to liftoff...
      setHoldAttitude(targetHeading, targetPitch, targetRoll);
      setPilotMode(AttitudeHoldMode);
      sayOutloud(true, "liftoff, aircraft is airborne");
    }

  // Peg the throttles for takeoff!
  flightControls.throttle = 1.0;
}







void Pilot::holdAttitude()
{
//   static const double pitchDiffScaling = 0.1;         // 10 degrees..
//   static const double rollDiffScaling  = 0.066666667; // 15 degrees...  

  static const double pitchDiffScaling = 0.2;         // 5 degrees..
  static const double rollDiffScaling  = 0.066666667; // 15 degrees...  

  static double pitchDiff = 0.0;
  static double rollDiff  = 0.0;
  
  pitchDiff = (positionData.pitch - goalData.holdPitch) 
    * pitchDiffScaling;

  rollDiff  = (positionData.roll - goalData.holdRoll) 
    * rollDiffScaling;


  if (pitchDiff > 1.0)
    {
      pitchDiff = 1.0;
    }
  else if (pitchDiff < -1.0)
    {
      pitchDiff = -1.0;
    }


  // If the roll is this extreme,
  // ignore pitch for the moment,
  // level the wings first. This 
  // way the heading doesn't
  // get too out of whack
  if (rollDiff > 1.0)
    {
      pitchDiff = 0.0;
      rollDiff = 1.0;
    }
  else if (rollDiff < -1.0)
    {
      pitchDiff = 0.0;
      rollDiff = -1.0;
    }
  setControlPositions(pitchDiff, -rollDiff, 0.0, 1.0);
}














/******************************************************/
// Target seeker class, for utilizing
// a neural network
/******************************************************/


TargetSeeker::TargetSeeker(float sideAngleBoundary,
			   float verticalAngleBoundary,
			   string netFileName)
{
  sideBoundary = sideAngleBoundary;
  vertBoundary = verticalAngleBoundary;
  
  newElevator = 0.0;
  newAileron  = 0.0;
  newRudder   = 0.0;

  targetLatitude  = 0.0;
  targetLongitude = 0.0;
  targetAltitude  = 0.0;

  trgtBearing   = 0.0;
  trgtElevation = 0.0;

  entityStore = new gvpEntityStore;

  target  = (gvpEntity*)entityStore->add("trgt");
  ownship = (gvpEntity*)entityStore->add("ownship");

  ellipsoid = new gvpEllipsoid;
  ellipsoid->set(ellipsoid->WGS84);
  
  targetNet = new NeuralNetwork;
  targetNet->ReadData(netFileName);
}



TargetSeeker::~TargetSeeker()
{
  delete ellipsoid;
  delete entityStore;
  delete targetNet;
}




void TargetSeeker::setTargetPosition(double lat, double lon, double alt)
{
  targetLatitude  = lat;
  targetLongitude = lon;
  targetAltitude  = alt;
}


void TargetSeeker::getTargetPosition(double &lat, double &lon, double &alt)
{
  lat = targetLatitude;
  lon = targetLongitude;
  alt = targetAltitude;
}


void TargetSeeker::setAircraftPosition(double newLat, double newLon, 
				       double newAlt, double newHeading, 
				       double newPitch, double newRoll)
{
  latitude  = newLat;
  longitude = newLon;
  altitude  = newAlt;
  heading   = newHeading;
  pitch     = newPitch;
  roll      = newRoll;
}



void TargetSeeker::setSideAngleBoundary(float newSideAngleBoundary)
{
  sideBoundary = newSideAngleBoundary;
}


float TargetSeeker::getSideAngleBoundary() const
{
  return sideBoundary;
}


void TargetSeeker::setVertAngleBoundary(float newVertAngleBoundary)
{
  vertBoundary = newVertAngleBoundary;
}


float TargetSeeker::getVertAngleBoundary() const
{
  return vertBoundary;
}


float TargetSeeker::getTargetBearing() const
{
  return trgtBearing;
}


float TargetSeeker::getTargetElevation() const
{
  return trgtElevation;
}


float TargetSeeker::getTargetDistance() const
{
  return targetDistance;
}


void TargetSeeker::calculateNewTargetBearingElevationDist()
{
  static double qx = 0.0, qy = 0.0,qz = 0.0, qw = 0.0;;
  static double trgtH = 0.0, trgtP = 0.0, trgtR = 0.0;

  static gvpVector<> horizTrgtVector;
  static gvpVector<> vertTrgtVector;
  static gvpVector<> straightAhead(0.0, 1.0, 0.0);
  static gvpVector<> toTarget;
  static gvpVector<> targetPosition;
  static gvpVector<> shipPosition;
  static gvpQuaternion<> throwAway(qx, qy, qz, qw);


  static double time  = 0.0;



  time = currentTime();

  target->setPositionGeo(time, ellipsoid,
			 targetLatitude, targetLongitude,
			 targetAltitude, 0.0, 0.0, 0.0);
  

  ownship->setPositionGeo(time, ellipsoid,
			  latitude, longitude, altitude,
			  heading, pitch, roll);

  target->getRelativePosition(currentTime(), ownship,
			      relativeTrgtX, relativeTrgtY, 
			      relativeTrgtZ,
			      trgtH, trgtP, trgtR); 

  horizTrgtVector.set(relativeTrgtX, relativeTrgtY, 0.0);
  vertTrgtVector.set(0.0, relativeTrgtY, relativeTrgtZ);

  target->getAbsolutePosition(time, targetPosition, throwAway);
  ownship->getAbsolutePosition(time, shipPosition, throwAway);

  toTarget = targetPosition - shipPosition;
  targetDistance = toTarget.magnitude();
 
  horizTrgtVector.normalize();
  vertTrgtVector.normalize();
  straightAhead.normalize();
  
  trgtBearing    = acos(straightAhead.dot(horizTrgtVector));
  trgtBearing   *= RAD2DEG;
  trgtElevation  = acos(straightAhead.dot(vertTrgtVector));
  trgtElevation *= RAD2DEG;
}





// This is where the magic happens...
void TargetSeeker::getAircraftControls(double &elevator, double &aileron, 
				       double &rudder)
{
  static double elevatorMultiplier = 0.0;
  static double aileronMultiplier  = 0.0;

  onSide   = false;
  onLeft   = false;
  onRight  = false;
  onVert   = false;
  onTop    = false;
  onBottom = false;
 
  
  // Must call this explicitely prior to 
  // calling getAircraftcontrols...
  // calculateNewTargetBearingElevationDist();


  if ((fabs(trgtBearing)) >= sideBoundary)
    {
      onSide = true;

      if (relativeTrgtX <= 0.0)
	{
	  onLeft = true;
	}
      else
	{
	  onRight = true;
	}
    }
  else
    {
      onSide = false;
    }

  if ((fabs(trgtElevation)) >= vertBoundary)
    {
      onVert = true;

      if (relativeTrgtZ <= 0.0)
	{
	  onBottom = true;
	}
      else
	{
	  onTop = true;
	}
    }
  else
    {
      onVert = false;
    }
  
  // Initialize values...
  centerEye     = -1.0;
  upperEye      = -1.0;
  upperRightEye = -1.0;
  rightEye      = -1.0;
  lowerRightEye = -1.0;
  lowerEye      = -1.0;
  lowerLeftEye  = -1.0;
  leftEye       = -1.0;
  upperLeftEye  = -1.0;

  rolledRight   = -1.0;
  rolledLeft    = -1.0;


  cout<<endl<<"*************************************************"<<endl;
  // Set "eye" activation levels
  if(onLeft && onTop)
    {
      cout<<"*XX"<<endl<<"XXX"<<endl<<"XXX"<<endl<<endl;
      upperLeftEye = 1.0;
    }
  else
    if(!onSide && onTop)
      {
	cout<<"X*X"<<endl<<"XXX"<<endl<<"XXX"<<endl<<endl;
	upperEye = 1.0;
      }
    else
      if(onRight && onTop)
	{
	  cout<<"XX*"<<endl<<"XXX"<<endl<<"XXX"<<endl<<endl;
	  upperRightEye = 1.0;
	}
      else
	if(!onVert && onLeft)
	  {
	    cout<<"XXX"<<endl<<"*XX"<<endl<<"XXX"<<endl<<endl;
	    leftEye = 1.0;
	  }
	else
	  if(!onVert && !onSide)
	    {
	      cout<<"XXX"<<endl<<"X*X"<<endl<<"XXX"<<endl<<endl;
	      centerEye = 1.0;
	    }
	  else
	    if (!onVert && onRight)
	      {
		cout<<"XXX"<<endl<<"XX*"<<endl<<"XXX"<<endl<<endl;
		rightEye = 1.0;
	      }
	    else
	      if (onBottom && onLeft)
		{
		  cout<<"XXX"<<endl<<"XXX"<<endl<<"*XX"<<endl<<endl;
		  lowerLeftEye = 1.0;
		}
	      else
		if (!onSide && onBottom)
		  {
		    cout<<"XXX"<<endl<<"XXX"<<endl<<"X*X"<<endl<<endl;
		    lowerEye = 1.0;
		  }
		else
		  if (onRight && onBottom)
		    {
		      cout<<"XXX"<<endl<<"XXX"<<endl<<"XX*"<<endl;
		      lowerRightEye = 1.0;
		    }
		  else
		    {
		      cout<<"Error, invalid eye activation!"<<endl;
		      exit(1);
		    }

  if (roll >= ROLLCUTOFF)
    {
      rolledRight = 1.0;
    }
  else if (roll <= -ROLLCUTOFF)
    {
      rolledLeft = 1.0;
    }
  
  cout<<"*************************************************"<<endl<<endl;


  // Data is ready for input
  // run neural net here....
  targetNet->SetInput(0,  centerEye);
  targetNet->SetInput(1,  upperEye);
  targetNet->SetInput(2,  upperRightEye);
  targetNet->SetInput(3,  rightEye);
  targetNet->SetInput(4,  lowerRightEye);
  targetNet->SetInput(5,  lowerEye);
  targetNet->SetInput(6,  lowerLeftEye);
  targetNet->SetInput(7,  leftEye);
  targetNet->SetInput(8,  upperLeftEye);

  targetNet->SetInput(9,  rolledRight);
  targetNet->SetInput(10, rolledLeft);
  
  targetNet->FeedForward();
  
  pullBack    = targetNet->GetOutput(0);
  pushForward = targetNet->GetOutput(1);
  rollRight   = targetNet->GetOutput(2);
  rollLeft    = targetNet->GetOutput(3);
 

  // Set the multipliers for smoother flight...
  if (fabs(trgtElevation) >= 30.0)
    {
      elevatorMultiplier = 1.0;
    }
  else if (fabs(trgtElevation) >= 15.0)
    {
      elevatorMultiplier = 0.6;
    }
  else
    {
      elevatorMultiplier = 0.3;
    }

  if (fabs(trgtBearing) >= 30.0)
    {
      aileronMultiplier = 1.0;
    }
  else if (fabs(trgtBearing) >= 15.0)
    {
      aileronMultiplier = 0.6;
    }
  else
    {
      aileronMultiplier = 0.3;
    }
  

  if (pullBack > pushForward)
    {
      newElevator = -elevatorMultiplier * pullBack;
    }
  else
    {
      newElevator = elevatorMultiplier * pushForward;
    }

  if (rollRight > rollLeft)
    {
      newAileron = aileronMultiplier * rollRight;
    }
  else
    {
      newAileron = aileronMultiplier * (-1.0 * rollLeft);
    }

  elevator = newElevator;
  aileron  = newAileron;
  rudder   = 0.0;
}
