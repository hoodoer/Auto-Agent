#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <cassert>
#include <string>
#include <typeinfo>


#include "baseState.h"




// Template for a finite state machine,
// with switching logic embedded in 
// the states themselves
template <class entityType>
class StateMachine
{
 public:
  StateMachine(entityType* newAgent):
    agent(newAgent),
    currentState(NULL),
    previousState(NULL),
    globalState(NULL)
    {}


  // For directly setting the current state.
  // This is really for initialization, during
  // runtime the changeState function should
  // be used instead. 
  void setCurrentState(BaseState<entityType>* newState)
    {
      currentState = newState;
    }

  // For directly setting the previous state.
  // This is really for initialization, during
  // runtime the changeState function should
  // be allowed to automatically handle this
  void setPreviousState(BaseState<entityType>* newPrevState)
    {
      previousState = newPrevState; 
    }


  // set the global state, which is called
  // every update, just before the currentState is
  // executed. This is a good spot to put general
  // state switching logic. 
  void setGlobalState(BaseState<entityType>* newGlobalState)
    {
      globalState = newGlobalState; 
    }


  BaseState<entityType>* getCurrentState()  const 
    {
      return currentState;
    }


  BaseState<entityType>* getPreviousState() const 
    {
      return previousState;
    }
  

  // This function handles the switching of states
  // and the callign of entry and exit functions
  // for the new, and old states. 
  void changeState(BaseState<entityType>* newState)
    {
      cout<<"Top of changeState..."<<endl;
      assert(newState && "<StateMachine::ChangeState>:newState is NULL");

      if (currentState)
	{
	  previousState = currentState;
	  currentState->exit(agent);
	}

      currentState = newState;
      currentState->enter(agent);
    }




  // If there is a previous state, 
  // this function reverts the
  // state machine to it and returns
  // TRUE, if there isn't a previous
  // state, the function simply 
  // returns FALSE;
  bool revertState()
    {
      if (previousState)
	{
	  changeState(previousState);
	  return TRUE;
	}
      else
	{
	  return FALSE;
	}
    }

  
  // Returns TRUE if the arguement passed is the 
  // current state, and FALSE if it isn't. 
  bool isInState(const BaseState<entityType>* state) const
  {
    if(!strcmp((typeid(*currentState).name()), 
	       (typeid(*state).name())))
      {
	return TRUE;
      }
    else
      {
	return FALSE;
      }
  }


  // Update the State machine. Execute the
  // global state, and execute the current state.
  void update() const
    {
      if (globalState)
	{
	  globalState->execute(agent);
	}

      if (currentState)
	{
	  currentState->execute(agent);
	}
    }


  
 private:
  // This is the "owner" of the statemachine
  entityType* agent;

  // This state is always executed
  BaseState<entityType>*  globalState;

  // This state is executed after the global state
  BaseState<entityType>*  currentState;

  BaseState<entityType>*  previousState;
};

#endif
