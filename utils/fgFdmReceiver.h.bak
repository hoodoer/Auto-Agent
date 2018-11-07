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



#ifndef FGFDMRECEIVER_H
#define FGFDMRECEIVER_H


// From the flightgear src/Network/ directory
// Defines the class object pass over the network
// socket, FGNetFDM
#include "net_fdm.hxx"  


#ifndef RAD2DEG
#define RAD2DEG  57.29578
#endif

#ifndef DEG2RAD 
#define DEG2RAD  0.017453293
#endif

#ifndef MET2FEET
#define MET2FEET 3.2808399
#endif

#ifndef FEET2MET
#define FEET2MET 0.3048
#endif




// Local copy of data received and
// converted to user friendly units
// some data doesn't need to be converted
// and isn't contained in this struct
struct localDataStruct
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
};



class FgFdmReceiver
{
 public:
  FgFdmReceiver(int port);
  ~FgFdmReceiver();


  // cycle, and update from UDP stream
  // must be called continuously in a loop,
  // or it's own thread
  void update();

  // Funcs to retrieve basic position information
  void getPositionGeo(double &latitude, double &longitude) const;
  void getAltitudes(double &altitude, float &agl) const;
  void getAttitude(float &pitch, float & roll, float &heading,
		   float &aoa, float &sideSlip) const;

  // Print out basic position information
  void printBasicData();


  // Program complete, close the socket
  void closeReceiver(); 
  
 private:
  localDataStruct data; // local data
  
  // Networking data
  int sockfd;
  int listenPort;  // port to listen on
  socklen_t addr_len;
  int numbytes;

  // Data received through UDP
  FGNetFDM fdm;


  // Function for fixing byte ordering
  void FGNetFDM2Props(FGNetFDM *net);
  void htond (double &x);
  void htonf (float &x);
};



#endif // FGFDMRECEIVER_H
