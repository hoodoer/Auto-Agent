#ifndef FLIER_H
#define FLIER_H

// These flier classes use neural networks
// to pilot the aircraft given certain inputs
// They are meant to be instantiated by the 
// Ucav class, and called from the Ucav's 
// Finite State Machine. The require
// a functional neural network. See the 
// aiTrainer program for tools to create
// and train a neural network.
//
// Drew Kirkpatrick, drew.kirkpatrick@gmail.com




// These defines are
// needed by gvpApiClient
// to correctly find
// the TAO corba headers
#define LINUX
#define ACE_5_4_6



#include "neuralNet.h"

#include "gvpApiClient.hpp"
#include "gvpTimer.hpp"
#include "gvpEntityStore.hpp"
#include "gvpEllipsoid.hpp"
#include "gvpDeadReckonEnum.hpp"
#include "gvpLegend.hpp"
#include "gvpPlugin.hpp"




enum FlyingMode
  {
    TargetSeekMode = 1,
    AttitudeHoldMode,
    TakeOffMode,
    SitStillMode,
    endList
  };




// basic position information,
// attitude, and speed information
struct posStruct
{
  double lat;
  double lon;
  double alt;
  double radAlt;
  double heading;
  double pitch;
  double roll;  
  double airSpeed;
};


struct desiredState
{
  // For target seeking mode
  double targetLat;
  double targetLon;
  double targetAlt;
  
  // For attitude hold mode
  double holdHeading;
  double holdPitch;
  double holdRoll;
  double holdAirSpeed;
};




// basic flight control info
struct controlsStruct
{
  double elevator;
  double aileron;
  double rudder;
  double throttle;
};






// Defined further down
class TargetSeeker;






// This class handles flying
// the aircraft using a number
// of techniques
class Pilot
{
 public:
  Pilot();
  ~Pilot();

  // Tell the Pilot where the aircraft is
  void setPositionGeo(double lat, double lon, double alt, 
		      double radAlt, double heading, 
		      double pitch, double roll);
  void setAirspeed(double newAirspeed);

  // This position is used for the targetSeeker Neural Net
  void setTargetSeekerTargetPos(double lat, double lon, double alt);

  // Get the distance in meters to the target..
  float getTargetDistance() const;

  // This attitude is to be held during AttitudeHoldMode
  void setHoldAttitude(double heading, double pitch, double roll);

  // The desired airspeed
  void setHoldAirspeed(double airspeed);

  // Needed for Ucav to control the flightmodel
  void getControlPositions(double &elevator, double &aileron, 
			   double &rudder, double &throttle);
  void setControlPositions(double elevator, double aileron, 
			   double rudder, double throttle);
  // Averages the positions over time to "smooth" the
  // control of the aircraft
  void setSmoothedControlPositions(double elevator, double aileron, 
				   double rudder, double throttle);

  // Interface functions for accessing the 
  // Pilot's targetSeeker neural net system
  void  setTargetSeekerSideBoundary(float newSideAngleBoundary);
  float getTargetSeekerSideBoundary() const;
  void  setTargetSeekerVertBoundary(float newVertAngleBoundary);
  float getTargetSeekerVertBoundary() const;

  
  // Set the mode of the pilot object
  void setPilotMode(FlyingMode newMode);


  // Update the pilot, execute the current mode
  // and calculate new control positions
  void update();


  
  

 private:
  FlyingMode     currentMode;
  TargetSeeker  *trgtSeekNet;
  posStruct      positionData;
  desiredState   goalData;
  controlsStruct flightControls;
  string         targetSeekerBrain;

  void takeoffAircraft();
  void holdAttitude();
};











// This class contains a neural net 
// that will change the course of
// the aircraft to fly towards a
// given target. 
class TargetSeeker
{
 public:
  // The angles given to the constructor
  // are the boundary's where the target seeker
  // will consider a target to be to the side, up.
  // down, etc. For example, if the sideAngleBoundry
  // is 10 degrees, then a target will be not 
  // be considered to be to the side until it's
  // relative bearing from course is at least 10
  // degrees. verticlAngleBoundry is the elevation 
  // boundary. The netFileName is the text file
  // that contains the neural network
  TargetSeeker(float sideAngleBoundary,
	       float verticalAngleBoundary,
	       string netFileName);
  ~TargetSeeker();


  // The location to fly to. Altitude is in meters.
  void setTargetPosition(double lat, double lon, double alt);
  void getTargetPosition(double &lat, double &lon, double &alt);
  
  // The aircraft position and orientation
  void setAircraftPosition(double newLat, double newLon, 
			   double newAlt, double newHeading, 
			   double newPitch, double newRoll);
  
  
  // Call this after setting the ownship position, and target position
  // will feed the data into the neural net, and return an answer
  void  getAircraftControls(double &elevator, double &aileron, double &rudder);

  void  setSideAngleBoundary(float newSideAngleBoundary);
  float getSideAngleBoundary() const;

  void  setVertAngleBoundary(float newVertAngleBoundary);
  float getVertAngleBoundary() const;


  // The calculated bearing and elevation of the 
  // current target. These return the last 
  // calculated bearing, elevation, or distance...
  float getTargetBearing()   const;
  float getTargetElevation() const;
  float getTargetDistance()  const;

  // ...and this version forces a new
  // bearing, elevation, and distance calculation,
  // and retrieves both...
  void  calculateNewTargetBearingElevationDist();

  

 private:
  // The description of these
  // is provided by the constructor
  float sideBoundary;
  float vertBoundary;

  // These will be the outputs of the 
  // target seeker
  float newElevator;
  float newAileron;
  float newRudder;
  
  // Where the target seeker
  // should fly to. 
  double targetLatitude;
  double targetLongitude;
  double targetAltitude;
  double targetDistance;

  double relativeTrgtX;
  double relativeTrgtY;
  double relativeTrgtZ;

  float trgtBearing;
  float trgtElevation;


  // Where I am
  double latitude;
  double longitude;
  double altitude;
  double heading;
  double pitch;
  double roll;

  bool onSide;
  bool onLeft;
  bool onRight;
  bool onVert;
  bool onTop;
  bool onBottom;
  
  // Neural Net inputs,
  // a 9 sensor "eye"
  float centerEye;
  float upperEye;
  float upperRightEye;
  float rightEye;
  float lowerRightEye;
  float lowerEye;
  float lowerLeftEye;
  float leftEye;
  float upperLeftEye;

  // And additional inputs
  // to the input layer
  // of the neural net
  float rolledRight;
  float rolledLeft;

  // Outputs from the neural net
  float pullBack;
  float pushForward;
  float rollRight;
  float rollLeft; 

  // Used for calculations
  gvpEntityStore *entityStore;
  gvpEntity      *ownship;
  gvpEntity      *target;
  gvpEllipsoid   *ellipsoid;

  // And of course, the
  // neural network
  NeuralNetwork  *targetNet;
};







#endif // FLIER_H
