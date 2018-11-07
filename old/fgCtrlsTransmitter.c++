// This class simplifies using sockets to 
// control the aircraft in Flightgear. 
// To get FlightGear to listen for UDP 
// data, start Flightgear like this:
// fgfs --native-ctrls='socket,in,UPDATERATE,,PORT,udp'
// Where UPDATERATE is the HZ that udp updates will
// be sent out, and the PORT where we'll be listening
//
// Written by Drew Kirkpatrick, drew.kirkpatrick@gmail.com




#include <iostream>
using namespace std;
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>


#include "fgCtrlsTransmitter.h"





/******************************************
 Public Functions
*******************************************/


// FlightGear Controls transmitter constructor
// Needs the hostname of the computer that
// Flightgear is running on, and the port
// Flightgear is listening on.
FgCtrlsTransmitter::FgCtrlsTransmitter(string hostname,
				       int port)
{
  cout<<"FgCtrlsTransmitter constructor..."<<endl;
  struct sockaddr_in my_addr;	// my address information 
  their_addr = new struct sockaddr_in;
  
  struct hostent *he;

  if ((port <= 0) || (port > 65535))
    {
      perror("FgCtrlsTransmitter::constructor: port range");
      exit(1);
    }
  
  sendToHostname = hostname;
  sendToPort     = port;


  if ((he=gethostbyname(sendToHostname.c_str())) == NULL) 
    {  // get the host info
      perror("FgCtrlsTransmitter::constructor:gethostbyname");
      exit(1);
    }
  
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
    {
      perror("FgCtrlsTransmitter::constructor: socket");
      exit(1);
    }


  their_addr->sin_family = AF_INET;	   // host byte order
  their_addr->sin_port = htons(sendToPort); // short, network byte order
  their_addr->sin_addr = *((struct in_addr *)he->h_addr);
  memset(&(their_addr->sin_zero), '\0', 8);  // zero the rest of the struct


  controls.aileron = 0.0;
  controls.elevator = 0.0;
  controls.rudder = 0.0;
  controls.aileron_trim = 0.0;
  controls.elevator_trim = 0.0;
  controls.rudder_trim = 0.0;
  controls.flaps = 0.0;

  for(int i = 0; i < controls.FG_MAX_ENGINES; i++)
    {
      controls.throttle[i] = 0.0;
    }
  
  controls.freeze = false;
}





// Flightgear control transmitter destructor
FgCtrlsTransmitter::~FgCtrlsTransmitter()
{
  cout<<"FgCtrlsTransmitter destructor..."<<endl;

  delete their_addr;
  close(sockfd);
}





// Gets and Sets...
double FgCtrlsTransmitter::getAileron() const
{
  return controls.aileron;
}



void FgCtrlsTransmitter::setAileron(double newAileron)
{
  if ((newAileron < -1.0) || (newAileron > 1.0))
    {
      perror("FgCtrlsTransmitter::setAileron: aileron range");
      exit(1);
    }

  controls.aileron = newAileron;
}







double FgCtrlsTransmitter::getElevator() const
{
  return controls.elevator;
}



void FgCtrlsTransmitter::setElevator(double newElevator)
{
  if ((newElevator < -1.0) || (newElevator > 1.0))
    {
      perror("FgCtrlsTransmitter::setElevator: elevator range");
      exit(1);
    }

  controls.elevator = newElevator;
}





double FgCtrlsTransmitter::getRudder() const
{
  return controls.rudder;
}



void FgCtrlsTransmitter::setRudder(double newRudder)
{
  if ((newRudder < -1.0) || (newRudder > 1.0))
    {
      perror("FgCtrlsTransmitter::setRudder:  Rudder range");
      exit(1);
    }

  controls.rudder = newRudder;
}





double FgCtrlsTransmitter::getAileronTrim() const
{
  return controls.aileron_trim;
}



void FgCtrlsTransmitter::setAileronTrim(double newAileronTrim)
{
  if ((newAileronTrim < -1.0) || (newAileronTrim > 1.0))
    {
      perror("FgCtrlsTransmitter::setAileronTrim: AileronTrim range");
      exit(1);
    }

  controls.aileron_trim = newAileronTrim;
}





double FgCtrlsTransmitter::getElevatorTrim() const
{
  return controls.elevator_trim;
}



void FgCtrlsTransmitter::setElevatorTrim(double newElevatorTrim)
{
  if ((newElevatorTrim < -1.0) || (newElevatorTrim > 1.0))
    {
      perror("FgCtrlsTransmitter::setElevatorTrim: ElevatorTrim range");
      exit(1);
    }

  controls.elevator_trim = newElevatorTrim;
}




double FgCtrlsTransmitter::getRudderTrim() const
{
  return controls.rudder_trim;
}



void FgCtrlsTransmitter::setRudderTrim(double newRudderTrim)
{
  if ((newRudderTrim < -1.0) || (newRudderTrim > 1.0))
    {
      perror("FgCtrlsTransmitter::setRudderTrim: RudderTrim range");
      exit(1);
    }

  controls.rudder_trim = newRudderTrim;
}




double FgCtrlsTransmitter::getFlaps() const
{
  return controls.flaps;
}



void FgCtrlsTransmitter::setFlaps(double newFlaps)
{
  if ((newFlaps < 0.0) || (newFlaps > 1.0))
    {
      perror("FgCtrlsTransmitter::setFlaps: flaps range");
      exit(1);
    }

  controls.flaps = newFlaps;
}





double FgCtrlsTransmitter::getThrottle(int engineNumber) const
{
  if ((engineNumber < 0) || 
      (engineNumber > controls.FG_MAX_ENGINES))
    {
      perror("FgCtrlsTransmitter::getThrottle: engineNum range");
      exit(1);
    }

  return controls.throttle[engineNumber];
}


void FgCtrlsTransmitter::setThrottle(int engineNumber, 
				     double newThrottle)
{
  if ((engineNumber < 0) || 
      (engineNumber > controls.FG_MAX_ENGINES))
    {
      perror("FgCtrlsTransmitter::getThrottle: engineNum range");
      exit(1);
    }

  if ((newThrottle < 0.0) || (newThrottle > 1.0))
    {
      perror("FgCtrlsTransmitter::setThrottle: newThrottle range");
      exit(1);
    }

  controls.throttle[engineNumber] = newThrottle;
}






int FgCtrlsTransmitter::getFreeze() const
{
  return controls.freeze;
}



void FgCtrlsTransmitter::setFreeze(int newFreeze)
{
  controls.freeze = newFreeze;
}




// Send data to Flightgear
void FgCtrlsTransmitter::sendData()
{
  if ((numbytes = sendto(sockfd, (char*)(&controls), 
			sizeof(controls),
			0, (struct sockaddr *)&their_addr, 
			sizeof(sockaddr))) == -1)
    {
      perror("FgCtrlsTransmitter::sendData: sendto");
    }
}






/******************************************
 Private Functions
*******************************************/




