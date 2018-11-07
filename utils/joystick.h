// This class simplifies using a joystick
// Drew Kirkpatrick, drew.kirkpatrick@gmail.com
// Make sure when you compile this you link in
// the proper libraries (-lplibjs -lplibul)


#ifndef JOYSTICK_H
#define JOYSTICK_H


#include <plib/js.h>



#define MAXAXIS    8
#define MAXBUTTONS 32



struct joystickValues
{
  float elevator;
  float aileron;
  float rudder;
  float throttle;
  float hatLeftRight;
  float hatUpDown;

  int   numberOfAxes;

  int   buttons[MAXBUTTONS];
};





class Joystick
{
 public:
  // Constructor, take a number corresponding
  // to which joystick on the system to use,
  // starting at 0 (If multiple joysticks)
  Joystick(int jsNumber); 
  ~Joystick();

  joystickValues getJoystickValues();

 private:
  jsJoystick *js;
  float      *axis;
  joystickValues jsVals;
  

  int joystickNumber;


};





#endif // JOYSTICK_H
