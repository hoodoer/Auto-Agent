// This class simplifies using sockets to 
// control the aircraft in Flightgear. 
// To get FlightGear to listen for UDP 
// data, start Flightgear like this:
// fgfs --native-ctrls='socket,in,UPDATERATE,,PORT,udp'
// Where UPDATERATE is the HZ that udp updates will
// be sent out, and the PORT where we'll be listening
//
// Written by Drew Kirkpatrick, drew.kirkpatrick@gmail.com



#ifndef FGCTRLSTRANSMITTER_H
#define FGCTRLSTRANSMITTER_H

#include "net_ctrls.hxx"



class FgCtrlsTransmitter
{
 public:
  FgCtrlsTransmitter(string hostname, int port);
  ~FgCtrlsTransmitter();
  

  // -1...1
  double getAileron() const;
  void   setAileron(double newAileron);

  // -1...1
  double getElevator() const;
  void   setElevator(double newElevator);

  // -1...1
  double getRudder() const;
  void   setRudder(double newRudder);

  // -1...1
  double getAileronTrim() const;
  void   setAileronTrim(double newAileronTrim);

  // -1...1
  double getElevatorTrim() const;
  void   setElevatorTrim(double newElevatorTrim);

  // -1...1
  double getRudderTrim() const;
  void   setRudderTrim(double newRudderTrim);

  // 0...1
  double getFlaps() const;
  void   setFlaps(double newFlaps);
  
  // 0...1
  double getThrottle(int engineNumber) const;
  void   setThrottle(int engineNumber, double newThrottle);


  // ?? Bool ina int???
  int    getFreeze() const;
  void   setFreeze(int newFreeze);

  // After setting values with the setXXX funcs, call 
  // this function to actually send the udp transmission
  void sendData(); 

 private:

  // Networking data
  string sendToHostname; // Host running FlightGear
  struct sockaddr_in *their_addr;
  socklen_t addr_len;
  int sendToPort;  // Port FlightGear is listening on
  int sockfd;
  int numbytes;

  // Datatype to sent through UDP
  FGNetCtrls controls;
};

#endif // FGCTRLSTRANSMITTER_H
