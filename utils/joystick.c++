// This class simplifies using a joystick
// Drew Kirkpatrick, drew.kirkpatrick@gmail.com



#include <iostream>
using namespace std;

#include "joystick.h"




/******************************************
 Public Functions
*******************************************/

// Constructor, takes the number of the the joystick,
// starting with 0. This is in case there are a number
// of joysticks on the computer.
Joystick::Joystick(int jsNumber)
{
  joystickNumber = jsNumber;

  js = new jsJoystick(joystickNumber);
  
  if (!js)
    {
      cout<<"Error instantiating a jsJoystick in Joystick::Joystick()"
	  <<endl;
      exit(1);
    }

  jsInit();
  
  cout<<"Joystick: "<<js->getName()<<endl;

  if (js->notWorking())
    {
      cout<<"Joystick "<<joystickNumber<<" isn't responding!"<<endl;
      exit(1);
    }
  
  jsVals.numberOfAxes = js->getNumAxes();

  axis = new float[jsVals.numberOfAxes];
}



// Destructor
Joystick::~Joystick()
{
  delete js;
  delete axis;
}



joystickValues Joystick::getJoystickValues()
{
  static int buttons;
  static int i;
  static int counter;
  static int value;

  js->read(&buttons, axis);

  jsVals.aileron      =  axis[0];
  jsVals.elevator     = -axis[1];
  jsVals.rudder       =  axis[2];
  jsVals.throttle     = -axis[3];
  jsVals.hatLeftRight =  axis[4];
  jsVals.hatUpDown    =  axis[5];

  
  for (int i = 0; i < MAXBUTTONS; i++)
    {
      jsVals.buttons[i] = 0;
    }
  
  // If true, some button is pushed
  if (buttons)
    {
      counter = 0;
      value   = buttons;

      while (value != 1)
	{
	  value /= 2;
	  counter++;
	}
      jsVals.buttons[counter] = 1;
    }
      
  return jsVals;
}



