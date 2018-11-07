// This class simplifies using sockets to 
// pull FDM data from FlightGear
// To get FlightGear to output UDP 
// Data to feed this class's sockets
// interface, start Flightgear like this:
// fgfs --native-fdm='socket,out,UPDATERATE,HOST,PORT,udp'
// Where UPDATERATE is the HZ that udp updates will
// be sent out, HOST is the hostname/ip of the 
// computer this class will be running on, and the
// PORT where we'll be listening
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


#include "fgFdmReceiver.h"





/******************************************
 Public Functions
*******************************************/




// Flightgear FDM receiver constructor
// Needs the port number to listen on 
// passed in as an argument
FgFdmReceiver::FgFdmReceiver(int port)
{
  cout<<"FgFdmReceiver constructor..."<<endl;

  struct sockaddr_in my_addr;	// my address information 

  if ((port <= 0) || (port > 65535))
    {
      perror("FgFdmReceiver::constructor: port range");
      exit(1);
    }
  
  listenPort     = port;
  data.longitude = 0.0;
  data.latitude  = 0.0;
  data.altitude  = 0.0;
  data.agl       = 0.0;
  data.roll      = 0.0;
  data.pitch     = 0.0;
  data.heading   = 0.0;
  data.aoa       = 0.0;
  data.sideSlip  = 0.0;


  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) 
    {
      perror("FgFdmReceiver::constructor: socket");
      exit(1);
    }

  my_addr.sin_family = AF_INET;		// host byte order
  my_addr.sin_port = htons(listenPort);	// short, network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
  memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct

  if (bind(sockfd, (struct sockaddr *)&my_addr,
	   sizeof(struct sockaddr)) == -1) 
    {
      perror("FgFdmReceiver::constructor: bind");
      exit(1);
    }

  addr_len = sizeof(struct sockaddr);
}






// Flightgear FDM receiver destructor
FgFdmReceiver::~FgFdmReceiver()
{
  cout<<"FgFdmReceiver::destructor"<<endl;
  close(sockfd);
}






// This function needs to be 
// called repeatedly, it reads
// data from the network
// Should be called faster than
// the network update rate passed to 
// Flightgear when started
void FgFdmReceiver::update()
{
  struct sockaddr_in their_addr; // connector's address information
  if ((numbytes = recvfrom(sockfd, (char*)(&fdm), sizeof(fdm), 0,
			   (struct sockaddr *)&their_addr, &addr_len)) == -1) 
    {
      perror("FgFdmReceive::update:recvfrom");
      exit(1);
    }

  // Fix all the byte ordering stuff...
  FGNetFDM2Props(&fdm);
 
  data.longitude = fdm.longitude * RAD2DEG;
  data.latitude  = fdm.latitude  * RAD2DEG;
  data.altitude  = fdm.altitude  * MET2FEET;
  data.agl       = fdm.agl       * MET2FEET;
  data.roll      = fdm.phi       * RAD2DEG;
  data.pitch     = fdm.theta     * RAD2DEG;
  data.heading   = fdm.psi       * RAD2DEG;
  data.aoa       = fdm.alpha     * RAD2DEG;
  data.sideSlip  = fdm.beta      * RAD2DEG;
  data.airspeed  = fdm.vcas;
}


// Prints out basic position information
void FgFdmReceiver::printBasicData()
{
      cout<<"Pos ("<<data.latitude<<", "<<data.longitude<<")"<<endl;
      cout<<"altitude of: "<<data.altitude<<endl;
      cout<<"agl of: "<<data.agl<<endl<<endl;
      cout<<"Heading: "<<data.heading<<endl;
      cout<<"Pitch: "<<data.pitch<<", roll: "<<data.roll<<endl;
//       cout<<"rpm of engine 1: "<<fdm.rpm[0]<<endl;
//       cout<<"Fuel of tank 1: "<<fdm.fuel_quantity[0]<<endl;
}



// Returns the current latitude and longitude
void FgFdmReceiver::getLatLong(double &latitude, 
			       double &longitude) const
{
  latitude  = data.latitude;
  longitude = data.longitude;
}



// Returns the current altitudes
void FgFdmReceiver::getAltitudes(double &altitude, 
				 float &agl) const
{
  altitude = data.altitude;
  agl      = data.agl;
}

// Returns the attitude data for the aircraft
void FgFdmReceiver::getAttitude(float &pitch, float & roll, float &heading,
		   float &aoa, float &sideSlip) const
{
  pitch    = data.pitch;
  roll     = data.roll;
  heading  = data.heading;
  aoa      = data.aoa;
  sideSlip = data.sideSlip;
}


// Returns the calibrated airspeed
float FgFdmReceiver::getAirspeed() const
{
  return data.airspeed;
}



// Returns the whole data structure for position
localDataStruct  FgFdmReceiver::getPositionGeo() const
{
  return data;
}





/******************************************
 Private Functions
*******************************************/



void FgFdmReceiver::FGNetFDM2Props(FGNetFDM *net)
{
  unsigned int i;

  // Convert to the net buffer from network format
  net->version = ntohl(net->version);

  htond(net->longitude);
  htond(net->latitude);
  htond(net->altitude);
  htonf(net->agl);
  htonf(net->phi);
  htonf(net->theta);
  htonf(net->psi);
  htonf(net->alpha);
  htonf(net->beta);

  htonf(net->phidot);
  htonf(net->thetadot);
  htonf(net->psidot);
  htonf(net->vcas);
  htonf(net->climb_rate);
  htonf(net->v_north);
  htonf(net->v_east);
  htonf(net->v_down);
  htonf(net->v_wind_body_north);
  htonf(net->v_wind_body_east);
  htonf(net->v_wind_body_down);

  htonf(net->A_X_pilot);
  htonf(net->A_Y_pilot);
  htonf(net->A_Z_pilot);

  htonf(net->stall_warning);
  htonf(net->slip_deg);

  net->num_engines = htonl(net->num_engines);
  for ( i = 0; i < net->num_engines; ++i ) 
    {
      net->eng_state[i] = htonl(net->eng_state[i]);
      htonf(net->rpm[i]);
      htonf(net->fuel_flow[i]);
      htonf(net->egt[i]);
      htonf(net->cht[i]);
      htonf(net->mp_osi[i]);
      htonf(net->tit[i]);
      htonf(net->oil_temp[i]);
      htonf(net->oil_px[i]);
    }

  net->num_tanks = htonl(net->num_tanks);
  for ( i = 0; i < net->num_tanks; ++i ) 
    {
      htonf(net->fuel_quantity[i]);
    }

  net->num_wheels = htonl(net->num_wheels);
  for ( i = 0; i < net->num_wheels; ++i ) 
    {
      net->wow[i] = htonl(net->wow[i]);
      htonf(net->gear_pos[i]);
      htonf(net->gear_steer[i]);
      htonf(net->gear_compression[i]);
    }

  net->cur_time = htonl(net->cur_time);
  net->warp = ntohl(net->warp);
  htonf(net->visibility);

  htonf(net->elevator);
  htonf(net->elevator_trim_tab);
  htonf(net->left_flap);
  htonf(net->right_flap);
  htonf(net->left_aileron);
  htonf(net->right_aileron);
  htonf(net->rudder);
  htonf(net->nose_wheel);
  htonf(net->speedbrake);
  htonf(net->spoilers);
}



void FgFdmReceiver::htond (double &x)	
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




// Float version
void FgFdmReceiver::htonf (float &x)	
{
  if ( sgIsLittleEndian() ) 
    {
      int    *Float_Overlay;
      int     Holding_Buffer;
    
      Float_Overlay = (int *) &x;
      Holding_Buffer = Float_Overlay [0];
    
      Float_Overlay [0] = htonl (Holding_Buffer);
    } 
  else 
    {
      return;
    }
}
