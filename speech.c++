// Simple file for saying things. Didn't have much luck
// linking against the development libraries,
// so a simple system call to the festival 
// program itself will do.
// Drew Kirkpatrick, drew.kirkpatrick@gmail.com



#include <iostream>
using namespace std;
#include <stdlib.h>


#include "speech.h"



// the speakScript simply throws the command
// to the background. Easier than coming up
// with an extra thread just for running
// speech commands
void sayOutloud(bool background, char* toSay)
{
  char command[100];

  if (background)
    {
      // Return immediate from call, speak in the background
      sprintf(command, "echo \"%s\" | festival --tts &", toSay);
    }
  else
    {
      // Wait until the speech is finished before returning
      sprintf(command, "echo \"%s\" | festival --tts", toSay);
    }
  
  system(command);
}




void sayOutloud(bool background, string toSay)
{
  sayOutloud(background, toSay.c_str());
}



