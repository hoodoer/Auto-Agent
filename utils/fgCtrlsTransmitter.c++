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
  int so_reuseaddr = 1;
  struct protoent *pte;

  struct hostent *he;


  if ((port <= 0) || (port > 65535))
    {
      perror("FgCtrlsTransmitter::constructor: port range");
      exit(1);
    }

  if ((he = gethostbyname(hostname.c_str())) == NULL)
    {
      perror("FgCtrlsTransmitter::constructor: gethostbyname");
      exit(1);
    }
  
  sendToHostname = hostname;
  sendToIP       = inet_ntoa(*((struct in_addr *)he->h_addr));
  sendToPort     = port;
  

  pte = getprotobyname("UDP");

  if ((sockfd = socket(PF_INET, SOCK_DGRAM, pte->p_proto)) == -1) 
    {
      perror("FgCtrlsTransmitter::constructor: socket");
      exit(1);
    }
  
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, 
 	     &so_reuseaddr, sizeof(so_reuseaddr));

  their_addr->sin_family = PF_INET;	   // host byte order
  their_addr->sin_port = htons((uint16_t)sendToPort); // short, network byte order
  their_addr->sin_addr.s_addr = inet_addr(sendToIP.c_str());
  

  // Initialize variables to something reasonable
  controls.version = FG_NET_CTRLS_VERSION;
  controls.aileron = 0.;
  controls.elevator = 0.;
  controls.rudder = 0.;
  controls.aileron_trim = 0.;
  controls.elevator_trim = 0.;
  controls.rudder_trim = 0.;
  controls.flaps = 0.;
  controls.spoilers = 0.;
  controls.speedbrake = 0.;
  
  controls.flaps_power = 1;
  controls.flap_motor_ok = 1;

  controls.num_engines = 2;

  for (int i=0; i< 2; i++) 
    {
      controls.master_bat[i] = 1;
      controls.master_alt[i] = 1;
      controls.magnetos[i] = 1;
      controls.starter_power[i] = 0;
      controls.throttle[i] = 0.;
      controls.mixture[i] = 0.;
      controls.condition[i] = 0.;
      controls.fuel_pump_power[i] = 1;
      controls.prop_advance[i] = 1.;
      controls.feed_tank_to[i] = 1;
      controls.reverse[i] = 0;
      controls.engine_ok[i] = 1;
      controls.mag_left_ok[i] = 1;
      controls.mag_right_ok[i] = 1;
      controls.spark_plugs_ok[i] = 1;
      controls.oil_press_status[i] = 0;
      controls.fuel_pump_ok[i] = 1;
    }

  controls.num_tanks = 1;
  controls.fuel_selector[0] = 1;

  controls.xfer_pump[0] = 1;
  controls.cross_feed = 0;

  controls.brake_left = 0.;
  controls.brake_right = 0.;
  controls.copilot_brake_left = 0.;
  controls.copilot_brake_right = 0.;
  controls.brake_parking = 0.;

  controls.gear_handle = 1;
  controls.master_avionics = 0;

  controls.comm_1 = 123.4;
  controls.comm_2 = 123.4;
  controls.nav_1 = 123.4;
  controls.nav_2 = 123.4;

  controls.wind_speed_kt = 0.;
  controls.wind_dir_deg = 0.;
  controls.turbulence_norm = 0.;

  controls.temp_c = 25.;
  controls.press_inhg = 25.;

  controls.hground = 25.;
  controls.magvar = 0.;

  controls.speedup = 1;
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
      cout<<"Tried to set: "<<newAileron<<endl;
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

  if ((newThrottle < -1.0) || (newThrottle > 1.0))
    {
      cout<<"Error in FgCtrlsTransmitter::setThrottle, "
	  <<"trying to set throttle to "
	  <<newThrottle<<endl;
      //      perror("FgCtrlsTransmitter::setThrottle: newThrottle range");
      exit(1);
    }

  controls.throttle[engineNumber] = newThrottle;
}



int  FgCtrlsTransmitter::getGear() const
{
  return controls.gear_handle;
}


void FgCtrlsTransmitter::setGear(int newGearPosition)
{
  controls.gear_handle = newGearPosition;
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
  //  FGProps2NetCtrls(&controls);

  if ((numbytes = sendto(sockfd, (char*)&controls, 
			sizeof(FGNetCtrls),0,
			(struct sockaddr *)their_addr, 
			sizeof(sockaddr))) == -1)
    {
      perror("FgCtrlsTransmitter::sendData: sendto");
    }
}






/******************************************
 Private Functions
*******************************************/



// For changing byte ordering of network messages
void FgCtrlsTransmitter::FGProps2NetCtrls(FGNetCtrls *net)
{
  int i;


  // convert to network byte order
  net->version = htonl(net->version);
  htond(net->aileron);
  htond(net->elevator);
  htond(net->rudder);
  htond(net->aileron_trim);
  htond(net->elevator_trim);
  htond(net->rudder_trim);
  htond(net->flaps);
  net->flaps_power = htonl(net->flaps_power);
  net->flap_motor_ok = htonl(net->flap_motor_ok);

  net->num_engines = htonl(net->num_engines);

  for (i = 0; i < FGNetCtrls::FG_MAX_ENGINES; ++i) 
    {
      net->master_bat[i] = htonl(net->master_bat[i]);
      net->master_alt[i] = htonl(net->master_alt[i]);
      net->magnetos[i] = htonl(net->magnetos[i]);
      net->starter_power[i] = htonl(net->starter_power[i]);
      htond(net->throttle[i]);
      htond(net->mixture[i]);
      net->fuel_pump_power[i] = htonl(net->fuel_pump_power[i]);
      htond(net->prop_advance[i]);
      htond(net->condition[i]);
      net->engine_ok[i] = htonl(net->engine_ok[i]);
      net->mag_left_ok[i] = htonl(net->mag_left_ok[i]);
      net->mag_right_ok[i] = htonl(net->mag_right_ok[i]);
      net->spark_plugs_ok[i] = htonl(net->spark_plugs_ok[i]);
      net->oil_press_status[i] = htonl(net->oil_press_status[i]);
      net->fuel_pump_ok[i] = htonl(net->fuel_pump_ok[i]);
    }

  net->num_tanks = htonl(net->num_tanks);
  for (i = 0; i < FGNetCtrls::FG_MAX_TANKS; ++i) 
    {
      net->fuel_selector[i] = htonl(net->fuel_selector[i]);
    }

  net->cross_feed = htonl(net->cross_feed);
  htond(net->brake_left);
  htond(net->brake_right);
  htond(net->copilot_brake_left);
  htond(net->copilot_brake_right);
  htond(net->brake_parking);
  net->gear_handle = htonl(net->gear_handle);
  net->master_avionics = htonl(net->master_avionics);
  htond(net->wind_speed_kt);
  htond(net->wind_dir_deg);
  htond(net->turbulence_norm);
  htond(net->temp_c);
  htond(net->press_inhg);
  htond(net->hground);
  htond(net->magvar);
  net->icing = htonl(net->icing);
  net->speedup = htonl(net->speedup);
  net->freeze = htonl(net->freeze);
}




void FgCtrlsTransmitter::htond(double &x)
{
  if ( sgIsLittleEndian() ) 
    {
      int    *Double_Overlay;
      int     Holding_Buffer;
    
      Double_Overlay = (int *) &x;
      Holding_Buffer = Double_Overlay [0];
    
      Double_Overlay [0] = htonl (Double_Overlay [1]);
      Double_Overlay [1] = htonl (Holding_Buffer);
    } 
  else 
    {
      return;
    }
}
