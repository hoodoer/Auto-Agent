// This program is for training the 
// artificial intelligence used in the 
// autoAgent program. It's primary 
// purpose is to provide a framework
// for training the neural network
// used in flying the F-16 flight model
// in FlightGear
//
// Written by Drew Kirkpatrick, drew.kirkpatrick@gmail.com


#include <iostream>
using namespace std;
#include <fstream>

#include <signal.h>
#include <pthread.h>


// These defines are
// needed by gvpApiClient
// to correctly find
// the TAO corba headers
#define LINUX
#define ACE_5_4_6


#include "fgFdmReceiver.h"
#include "fgCtrlsTransmitter.h"
#include "joystick.h"
#include "net_ctrls.hxx"
#include "neuralNet.h"

#include "gvpApiClient.hpp"
#include "gvpTimer.hpp"
#include "gvpEntityStore.hpp"
#include "gvpEllipsoid.hpp"
#include "gvpDeadReckonEnum.hpp"
#include "gvpLegend.hpp"
#include "gvpPlugin.hpp"




#define ROLLCUTOFF 5.0


enum Mode
  {
    TRAININGMODE,
    RECORDINGMODE,
    TESTINGMODE
  };


Mode currentMode;
bool activateNeuralNet = false;


// Whether or not the program is done,
// only works for some modes...
bool DONE = false;


// Position datastruct from fgFdmReceiver
localDataStruct posData;


// Target data, position of target used
// the XYZ will be set by the program, 
// user must fill in the latitude and
// longitude


// Right over the helipad at North Island...
// Used for collecting training data
double trgtLatitude  =  32.709672;
double trgtLongitude = -117.213235;
double trgtAltitude  =  1000.0;

// This is the relative XYZ to the ownship
double trgtX         = 0.0;
double trgtY         = 0.0;
double trgtZ         = 0.0;

double trgtH         = 0.0;
double trgtP         = 0.0;
double trgtR         = 0.0;

string dataFileName;
string brainFile;
string gvpHost;

gvpApiClient   *gvp = NULL;
Joystick       *js  = NULL;
joystickValues jsVals;



// Structure of neural net
#define INPUTNEURONS  11
#define OUTPUTNEURONS 4

// input on commandline
int HIDDENNEURONS  = 12;



// The angular distance in degrees
// for example, if the trgt is 
// has a bearing SIDEBOUNTRY 
// degrees off course, its
// to the side, not ahead
#define SIDEBOUNDRY 8.0
#define VERTBOUNDRY 8.0

bool onSide   = false;
bool onLeft   = false;
bool onRight  = false;
bool onVert   = false;
bool onTop    = false;
bool onBottom = false;



// All booleans
struct brainInputs
{
  float centerEye;
  float upperEye;
  float upperRightEye;
  float rightEye;
  float lowerRightEye;
  float lowerEye;
  float lowerLeftEye;
  float leftEye;
  float upperLeftEye;

  float rolledRight;
  float rolledLeft;
};


// All booleans
struct brainOutputs
{
  float pullBack;
  float pushForward;
  float rollRight;
  float rollLeft;
};




brainInputs  neuralInputData;
brainOutputs neuralOutputData;


float newElevator = 0.0;
float newAileron  = 0.0;
float newRudder   = 0.0;


// Handle signals sent to us by HOST
void signalHandler(int signal)
{
  switch(signal)
    {
    case SIGINT:  // Time to exit
      DONE = TRUE;
      break;
    default:
      // Some other signal...
      break;
    }
}






// Switch to flip from joystick 
// to neural net aircraft control
void watchForNeuralActivation()
{
  // Used for briefly suspending the thread
  struct timespec ts;
  ts.tv_sec  = 0;
  ts.tv_nsec = 10000;

  string temp;
  cout<<"Enter something to Activate Neural Net:";
  cin>>temp;

  activateNeuralNet = true;
}






// Run calcs to drive neural net, and
// either record, or input to neural net
void runNeuralNetData()
{
  // Used for briefly suspending the thread
  struct timespec ts;
  ts.tv_sec  = 0;
  ts.tv_nsec = 1000;

  // Toss the first loop of recording
  bool initialized = false;

  gvpEntityStore *entityStore;


  entityStore = new gvpEntityStore;

  gvpEntity *target;
  gvpEntity *ownship;

  gvpEllipsoid *ellipsoid;
  ellipsoid = new gvpEllipsoid;
  ellipsoid->set(ellipsoid->WGS84);

  NeuralNetwork testingBrain;

  while(gvp == NULL)
    {
      nanosleep (&ts, NULL);
    }
  
  

  // This is the viewpoint entity
  unsigned long view     = gvp->getViewEntity();
  unsigned long targetID = 0;
  
  ofstream trainingDataFile;


  gvpVector<> horizTrgtVector;
  gvpVector<> vertTrgtVector;
  gvpVector<> straightAhead(0.0, 1.0, 0.0);

  float trgtBearing;
  float trgtElevation;



  if (currentMode == TESTINGMODE)
    {
      cout<<"Opening brainFile ("<<brainFile<<") for testing..."<<endl;
      testingBrain.ReadData(brainFile);
    }





  if (currentMode == RECORDINGMODE)
    {
      trainingDataFile.open(dataFileName.c_str(), ios::out);
      if(!trainingDataFile)
	{
	  cout<<"Failed to open "<<dataFileName<<endl;
	  exit(1);
	}
    }
  

  target  = (gvpEntity*)entityStore->add("trgt");
  ownship = (gvpEntity*)entityStore->add("ownship");
	  
  target->setPositionGeo(currentTime(), ellipsoid,
			 trgtLatitude, trgtLongitude, trgtAltitude,
			 0.0, 0.0, 0.0);
  ownship->setPositionGeo(currentTime(), ellipsoid,
			  posData.latitude, 
			  posData.longitude,
			  posData.altitude * FEET2MET, 
			  posData.heading,
			  posData.pitch, 
			  posData.roll);

  target->getRelativePosition(currentTime(), ownship, 
			      trgtX, trgtY, trgtZ,
			      trgtH, trgtP, trgtR);
  targetID = gvp->add("target", "aircraft", "f-18");

  gvp->setPositionGeo(currentTime(), targetID,
		      trgtLatitude, trgtLongitude, trgtAltitude,
		      0.0, 0.0, 0.0);
  
  if (currentMode == RECORDINGMODE)
    {
      string temp;
      cout<<"Enter something to start recording:";
      cin>>temp;
      cout<<endl<<"Starting recording..."<<endl;
    }

  while (!DONE)
    {
      onSide   = false;
      onLeft   = false;
      onRight  = false;
      onVert   = false;
      onTop    = false;
      onBottom = false;


      ownship->setPositionGeo(currentTime(), ellipsoid,
			      posData.latitude, posData.longitude,
			      posData.altitude * FEET2MET, posData.heading,
			      posData.pitch, posData.roll);
      target->getRelativePosition(currentTime(), ownship, 
				  trgtX, trgtY, trgtZ,
				  trgtH, trgtP, trgtR);
      
      horizTrgtVector.set(trgtX, trgtY, 0.0);
      vertTrgtVector.set(0.0, trgtY, trgtZ);

      horizTrgtVector.normalize();
      vertTrgtVector.normalize();
      straightAhead.normalize();

      trgtBearing    = acos(straightAhead.dot(horizTrgtVector));
      trgtBearing   *= RAD2DEG;
      trgtElevation  = acos(straightAhead.dot(vertTrgtVector));
      trgtElevation *= RAD2DEG;


      if ((fabs(trgtBearing)) >= SIDEBOUNDRY)
	{
	  onSide = true;

	  if (trgtX <= 0.0)
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

      if ((fabs(trgtElevation)) >= VERTBOUNDRY)
	{
	  onVert = true;

	  if (trgtZ <= 0.0)
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
      neuralInputData.centerEye     = -1.0;
      neuralInputData.upperEye      = -1.0;
      neuralInputData.upperRightEye = -1.0;
      neuralInputData.rightEye      = -1.0;
      neuralInputData.lowerRightEye = -1.0;
      neuralInputData.lowerEye      = -1.0;
      neuralInputData.lowerLeftEye  = -1.0;
      neuralInputData.leftEye       = -1.0;
      neuralInputData.upperLeftEye  = -1.0;

      neuralInputData.rolledRight   = -1.0;
      neuralInputData.rolledLeft    = -1.0;
      
      cout<<endl<<"*************************************************"<<endl;
      // Set "eye" activation levels
      if(onLeft && onTop)
	{
	  cout<<"*XX"<<endl<<"XXX"<<endl<<"XXX"<<endl<<endl;
	  neuralInputData.upperLeftEye = 1.0;
	}
      else
	if(!onSide && onTop)
	  {
	    cout<<"X*X"<<endl<<"XXX"<<endl<<"XXX"<<endl<<endl;
	    neuralInputData.upperEye = 1.0;
	  }
	else
	  if(onRight && onTop)
	    {
	      cout<<"XX*"<<endl<<"XXX"<<endl<<"XXX"<<endl<<endl;
	      neuralInputData.upperRightEye = 1.0;
	    }
	  else
	    if(!onVert && onLeft)
	      {
		cout<<"XXX"<<endl<<"*XX"<<endl<<"XXX"<<endl<<endl;
		neuralInputData.leftEye = 1.0;
	      }
	    else
	      if(!onVert && !onSide)
		{
		  cout<<"XXX"<<endl<<"X*X"<<endl<<"XXX"<<endl<<endl;
		  neuralInputData.centerEye = 1.0;
		}
	      else
		if (!onVert && onRight)
		  {
		    cout<<"XXX"<<endl<<"XX*"<<endl<<"XXX"<<endl<<endl;
		    neuralInputData.rightEye = 1.0;
		  }
		else
		  if (onBottom && onLeft)
		    {
		      cout<<"XXX"<<endl<<"XXX"<<endl<<"*XX"<<endl<<endl;
		      neuralInputData.lowerLeftEye = 1.0;
		    }
		  else
		    if (!onSide && onBottom)
		      {
			cout<<"XXX"<<endl<<"XXX"<<endl<<"X*X"<<endl<<endl;
			neuralInputData.lowerEye = 1.0;
		      }
		    else
		      if (onRight && onBottom)
			{
			  cout<<"XXX"<<endl<<"XXX"<<endl<<"XX*"<<endl<<endl;
			  neuralInputData.lowerRightEye = 1.0;
			}
		      else
			{
			  cout<<"Error, invalid eye activation!"<<endl;
			  exit(1);
			}


      if (posData.roll >= ROLLCUTOFF)
	{
	  neuralInputData.rolledRight = 1.0;
	  cout<<"ROLLED RIGHT!!!"<<endl;
	}
      else
	if (posData.roll <= -ROLLCUTOFF)
	  {
	    neuralInputData.rolledLeft = 1.0;
	    cout<<"ROLLED LEFT!!!"<<endl;
	  }

      cout<<"*************************************************"<<endl<<endl;

      if (activateNeuralNet)
	{
	  // run neural net here....
	  testingBrain.SetInput(0, neuralInputData.centerEye);
	  testingBrain.SetInput(1, neuralInputData.upperEye);
	  testingBrain.SetInput(2, neuralInputData.upperRightEye);
	  testingBrain.SetInput(3, neuralInputData.rightEye);
	  testingBrain.SetInput(4, neuralInputData.lowerRightEye);
	  testingBrain.SetInput(5, neuralInputData.lowerEye);
	  testingBrain.SetInput(6, neuralInputData.lowerLeftEye);
	  testingBrain.SetInput(7, neuralInputData.leftEye);
	  testingBrain.SetInput(8, neuralInputData.upperLeftEye);
	  
	  testingBrain.FeedForward();

	  neuralOutputData.pullBack    = testingBrain.GetOutput(0);
	  neuralOutputData.pushForward = testingBrain.GetOutput(1);
	  neuralOutputData.rollRight   = testingBrain.GetOutput(2);
	  neuralOutputData.rollLeft    = testingBrain.GetOutput(3);
	  

	  cout<<"Output from NN:"<<endl;
// 	  cout<<"PullBack: "<<neuralOutputData.pullBack<<endl;
// 	  cout<<"PushForward: "<<neuralOutputData.pushForward<<endl;
// 	  cout<<"RollRight: "<<neuralOutputData.rollRight<<endl;
// 	  cout<<"RollLeft: "<<neuralOutputData.rollLeft<<endl;

	  if (neuralOutputData.pullBack > neuralOutputData.pushForward)
	    {
	      newElevator = -1.0 * neuralOutputData.pullBack;
	      if (fabs(newElevator) >= 0.4)
		cout<<"PULLBACK!"<<endl;
	    }
	  else
	    {
	      newElevator = neuralOutputData.pushForward;
	      if (fabs(newElevator) >= 0.4)
		cout<<"PUSHFORWARD!"<<endl;
	    }

	  if (neuralOutputData.rollRight > neuralOutputData.rollLeft)
	    {
	      newAileron = 0.7 * neuralOutputData.rollRight;
	      if (fabs(newAileron) >= 0.4)
		cout<<"ROLLRIGHT!"<<endl;
	    }
	  else
	    {
	      newAileron = 0.7 * (-1.0 * neuralOutputData.rollLeft);
	      if (fabs(newAileron) >= 0.4)
		cout<<"ROLLLEFT!"<<endl;
	    }
	}
      else
	{
	  newElevator = jsVals.elevator;
	  newAileron  = jsVals.aileron;
	  newRudder   = jsVals.rudder;
	}


      // don't record the first cycle to trainingDataFile
      if (!initialized)
	{
	  initialized = true;
	  continue;
	}

      if (currentMode == RECORDINGMODE)
	{
	  cout<<"RECORDING MODE BROKEN!!!!!!!"<<endl;
	  cout<<"Training files must be hand written!"<<endl;

	  // 	  trainingDataFile<< neuralInputData.centerEye     <<" "
	  // 			  << neuralInputData.upperEye      <<" "
	  // 			  << neuralInputData.upperRightEye <<" "
	  // 			  << neuralInputData.rightEye      <<" "
	  // 			  << neuralInputData.lowerRightEye <<" "
	  // 			  << neuralInputData.lowerEye      <<" "
	  // 			  << neuralInputData.lowerLeftEye  <<" "
	  // 			  << neuralInputData.leftEye       <<" "
	  // 			  << neuralInputData.upperLeftEye  <<" "
	  // 			  << neuralInputData.inverted      <<" "
	  // 			  << neuralInputData.roll          <<" "
	  // 			  << neuralOutputData.newElevator  <<" "
	  // 			  << neuralOutputData.newAileron   <<" "
	  // 			  << neuralOutputData.newRudder    <<endl;
	}
      
      nanosleep(&ts, NULL);
    }

if (currentMode == RECORDINGMODE)
    {
      trainingDataFile.close();
    }

  delete entityStore;
  delete ellipsoid;
}






// Use training data sets created using the
// record option to create a neural net.
void trainBrain()
{
  bool existingBrain = false;
  int i;
  double error    = 1;
  int counter     = 0;
  int lineCounter = 0;


  // Neural network used for training
  NeuralNetwork trainerBrain;

  ifstream trainingDataFile(dataFileName.c_str(), ios::in);
  ifstream testBrainFile;

  testBrainFile.open(brainFile.c_str(), ios::in);
  testBrainFile.close();

  if(testBrainFile.fail())
    {
      cout<<"Starting a new neural net"<<endl;
      existingBrain = false;
    }
  else
    {
      cout<<"Modifying existing neural net"<<endl;
      existingBrain = true;
    }
  

  if(!trainingDataFile)
    {
      cout<<"Failed to open "<<dataFileName<<endl;
      exit(1);
    }

  brainInputs  neuralInputData;
  brainOutputs neuralOutputData;

  
  if (existingBrain)
    {
      // Read in the existing neural net
      trainerBrain.ReadData(brainFile);
    }
  else
    {
      // Initialize a new neural net
      trainerBrain.Initialize(INPUTNEURONS, 
			      HIDDENNEURONS, 
			      OUTPUTNEURONS);
    }
  
  trainerBrain.SetLearningRate(0.2);
  trainerBrain.SetMomentum(true,0.9);


  while (!trainingDataFile.eof())
    {
      // Read training data
      trainingDataFile>>neuralInputData.centerEye;
      trainingDataFile>>neuralInputData.upperEye;
      trainingDataFile>>neuralInputData.upperRightEye;
      trainingDataFile>>neuralInputData.rightEye;
      trainingDataFile>>neuralInputData.lowerRightEye;
      trainingDataFile>>neuralInputData.lowerEye;
      trainingDataFile>>neuralInputData.lowerLeftEye;
      trainingDataFile>>neuralInputData.leftEye;
      trainingDataFile>>neuralInputData.upperLeftEye;
      trainingDataFile>>neuralInputData.rolledRight;
      trainingDataFile>>neuralInputData.rolledLeft;
      trainingDataFile>>neuralOutputData.pullBack;
      trainingDataFile>>neuralOutputData.pushForward;
      trainingDataFile>>neuralOutputData.rollRight;
      trainingDataFile>>neuralOutputData.rollLeft;    


      while ((error > 0.05) && (counter < 50000))
	{
// 	  cout<<"Training loop counter at: "<<counter<<endl;
	  error = 0;
	  counter++;

	  // Insert training data
	  trainerBrain.SetInput(0,  neuralInputData.centerEye);   
	  trainerBrain.SetInput(1,  neuralInputData.upperEye);    
	  trainerBrain.SetInput(2,  neuralInputData.upperRightEye);
	  trainerBrain.SetInput(3,  neuralInputData.rightEye);    
	  trainerBrain.SetInput(4,  neuralInputData.lowerRightEye);
	  trainerBrain.SetInput(5,  neuralInputData.lowerEye);    
	  trainerBrain.SetInput(6,  neuralInputData.lowerLeftEye);
	  trainerBrain.SetInput(7,  neuralInputData.leftEye);     
	  trainerBrain.SetInput(8,  neuralInputData.upperLeftEye);
	  trainerBrain.SetInput(9,  neuralInputData.rolledRight);
	  trainerBrain.SetInput(10, neuralInputData.rolledLeft);

	  trainerBrain.SetDesiredOutput(0, neuralOutputData.pullBack);
	  trainerBrain.SetDesiredOutput(1, neuralOutputData.pushForward);
	  trainerBrain.SetDesiredOutput(2, neuralOutputData.rollRight);
	  trainerBrain.SetDesiredOutput(3, neuralOutputData.rollLeft);    

	  // Learn...
	  trainerBrain.FeedForward();
	  error += trainerBrain.CalculateError();
	  trainerBrain.BackPropagate();
	  lineCounter++;
	}
    }

  trainerBrain.DumpData(brainFile);
  trainingDataFile.close();
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

  // Network control of FlightGear
  FgCtrlsTransmitter ctrlsOutput("localhost", 5070);

  // The GVP api client, so I can 
  // send position updates to the 
  // GVP viewpoint
  gvp = new gvpApiClient(gvpHost.c_str(), 5050);


  // Get the first joystick on the system,
  // For use when not autonomously flying

  js = new Joystick(0);


  // This is the viewpoint entity
  unsigned long view = gvp->getViewEntity();



  



  while(!DONE)
    {
      // Get the position from flightgear
      fdmInput.update();
      posData = fdmInput.getPositionGeo();
      
      if (activateNeuralNet)
	{
	  ctrlsOutput.setElevator(newElevator);
	  ctrlsOutput.setAileron(newAileron);
	  ctrlsOutput.setRudder(0.0);
	  ctrlsOutput.setThrottle(0, 1.0);
	  ctrlsOutput.setThrottle(1, 1.0);
	  ctrlsOutput.setThrottle(2, 1.0);
	  ctrlsOutput.setThrottle(3, 1.0);
	}
      else
	{
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
	  // Send data to FlightGear
      ctrlsOutput.sendData();
	  



      // Put that position into GVP
      gvp->setPositionGeo(currentTime(), view,
			 posData.latitude, posData.longitude,
			 posData.altitude * FEET2MET, posData.heading,
			 posData.pitch, posData.roll);

      // Suspend execution briefly
      nanosleep(&ts, NULL);
    }


  delete gvp;
  delete js;
  cout<<"Exiting gvpUpdateThread"<<endl;
}









void printUsageInfo()
{
  cout<<"Usage: "<<endl<<endl;
  cout<<"For training:  aiTrainer train [Data set filename] [numHiddenNodes]"
      <<"[Neural Net file]"<<endl<<endl;
  cout<<"For recording: aiTrainer record [GVPHOSTNAME] "
      <<"[Data Set filename to save]"<<endl<<endl;
  cout<<"For test: aiTrainer test [GVPHOSTNAME] "
      <<"[Neural Net file]"<<endl<<endl;
}







int main(int argc, char **argv)
{
  // Thread to pull data from the FDM of
  // Flightgear, and send it to GVP
  // Only needed if recording data
  // in the simulation
  pthread_t gvpUpdate;

  if (argc < 3)
    {
      printUsageInfo();
      return 0;
    }

  

  if (!strcmp(argv[1], "train"))
    {
      cout<<"Starting training session"<<endl;
      cout<<"Using brainName: "<<argv[3]<<endl;
      currentMode   = TRAININGMODE;
      dataFileName  = argv[2];
      brainFile     = argv[3];
      HIDDENNEURONS = atoi(argv[4]);
    }
  else
    if (!strcmp(argv[1], "record"))
      {
	cout<<"Starting recording session"<<endl;
	currentMode  = RECORDINGMODE;
	gvpHost      = argv[2];
	dataFileName = argv[3]; 
      }
    else
      if (!strcmp(argv[1], "test"))
	{
	  cout<<"Starting testing session"<<endl;
	  currentMode = TESTINGMODE;
	  gvpHost     = argv[2];
	  brainFile   = argv[3]; 
	}
      else
	{
	  printUsageInfo();
	  return 0;
	}
  
  
  switch(currentMode)
    {
    case TRAININGMODE:
      // run trainBrain completely through the 
      // training set
      trainBrain();
      break;

    case RECORDINGMODE:
      // We're not training, we're collecting 
      // data to use in training...
      // requires the full simulation,
      // so split off a thread for GVP
      // updates, and watch for a sigint signal
      // Catch sigint to shutdown cleanly
      signal(SIGINT,  signalHandler);

      // Run net code needed for simulation and
      // data collection (flightgear and GVP)
      pthread_create(&gvpUpdate, NULL, gvpUpdateThread, NULL);


      // run recordData until sigint received...
      runNeuralNetData();

      // Wait for the GVP thread to complete it's
      // memory cleanup...
      pthread_join(gvpUpdate, NULL);
      break;

    case TESTINGMODE:
      // Allows the user to fly the aircraft to 
      // whatever location they like around the
      // target, and then activate the neural net
      // to see if it successfully flies the aircraft
      signal(SIGINT,  signalHandler);

      // Run net code needed for simulation and
      // data collection (flightgear and GVP)
      pthread_create(&gvpUpdate, NULL, gvpUpdateThread, NULL);

      // gvpUpdate thread will run as normal until this
      // function sets boolean activateNeuralNet
      watchForNeuralActivation();
      runNeuralNetData();
      pthread_join(gvpUpdate, NULL);
      break;

    default:
      cout<<"Invalid mode: "<<currentMode<<endl;
      exit(1);
    }

  return 0;
}
