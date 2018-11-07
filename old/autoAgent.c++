#include <iostream>
using namespace std;

#include "fgFdmReceiver.h"
#include "fgCtrlsTransmitter.h"



int main()
{
  cout<<"Top of new autoAgent..."<<endl;
  FgFdmReceiver     fdmInput(5050);
  FgCtrlsTransmitter ctrlsOutput("127.0.0.1", 5060);


  while(true)
    {
      fdmInput.update();
      fdmInput.printBasicData();

      ctrlsOutput.setThrottle(0, 0.0);
      ctrlsOutput.sendData();
    }

  return 0;
}
