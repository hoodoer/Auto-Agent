
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
using namespace std;


#include "net_fdm.hxx"



#define MYPORT 5050	// the port users will be connecting to

#define RAD2DEG  57.29578
#define DEG2RAD  0.017453293
#define MET2FEET 3.2808399
#define FEET2MET 0.3048


// Local data struct with units fixed 
// to degrees/feet
struct aircraftData
{
  double longitude;
  double latitude; 
  double altitude; 
  float  agl;	     
  float  roll;     
  float  pitch;    
  float  heading;
  float  aoa;      
  float  sideSlip; 
} acData;


// The function htond is defined this way due to the way some
// processors and OSes treat floating point values.  Some will raise
// an exception whenever a "bad" floating point value is loaded into a
// floating point register.  Solaris is notorious for this, but then
// so is LynxOS on the PowerPC.  By translating the data in place,
// there is no need to load a FP register with the "corruped" floating
// point value.  By doing the BIG_ENDIAN test, I can optimize the
// routine for big-endian processors so it can be as efficient as
// possible
static void htond (double &x)	
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
static void htonf (float &x)	
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




void FGNetFDM2Props(FGNetFDM *net) 
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





int main(int argc, char **argv)
{
  int sockfd;
  struct sockaddr_in my_addr;	// my address information
  struct sockaddr_in their_addr; // connector's address information
  socklen_t addr_len;
  int numbytes;

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }

  my_addr.sin_family = AF_INET;		 // host byte order
  my_addr.sin_port = htons(MYPORT);	 // short, network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
  memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct

  if (bind(sockfd, (struct sockaddr *)&my_addr,
	   sizeof(struct sockaddr)) == -1) {
    perror("bind");
    exit(1);
  }

  addr_len = sizeof(struct sockaddr);


  // Output of the flight model in FlightGear
  // Read over a socket, provides position and
  // attitude, as well as other aircraft data
  FGNetFDM fdm;


  while(true)
    {
      if ((numbytes = recvfrom(sockfd, (char*)(&fdm), sizeof(fdm), 0,
			       (struct sockaddr *)&their_addr, &addr_len)) == -1) {
	perror("recvfrom");
	exit(1);
      }

      FGNetFDM2Props(&fdm);


      acData.longitude = fdm.longitude * RAD2DEG;
      acData.latitude  = fdm.latitude  * RAD2DEG;
      acData.altitude  = fdm.altitude  * MET2FEET;
      acData.agl       = fdm.agl       * MET2FEET;
      acData.roll      = fdm.phi       * RAD2DEG;
      acData.pitch     = fdm.theta     * RAD2DEG;
      acData.heading   = fdm.psi       * RAD2DEG;
      acData.aoa       = fdm.alpha     * RAD2DEG;
      acData.sideSlip  = fdm.beta      * RAD2DEG;

      cout<<"Pos ("<<acData.latitude<<", "<<acData.longitude<<")"<<endl;
      cout<<"altitude of: "<<acData.altitude<<endl;
      cout<<"Heading: "<<acData.heading<<endl;
      cout<<"Pitch: "<<acData.pitch<<", roll: "<<acData.roll<<endl;
      cout<<"agl of: "<<acData.agl<<endl<<endl;
      cout<<"rpm of engine 1: "<<fdm.rpm[0]<<endl;
      cout<<"Fuel of tank 1: "<<fdm.fuel_quantity[0]<<endl;
      cout<<endl<<endl;
    }

  close(sockfd);

  return 0;
}
