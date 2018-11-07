#ifndef BASESTATE_H
#define BASESTATE_H



#ifndef FALSE
#define FALSE 0
#define TRUE !FALSE
#endif


// A template class for states
// to be used in conjunction with the
// state machine in stateMachine.h
// Needs an entity derived from
// baseEntity.h
template <class entityType>
class BaseState
{
 public:
  // State entry function, run once, upon entry of the state
  virtual void enter(entityType*) = 0;

  // Run the state, to be called from 
  // the stateMachine's update function
  // when the state is active
  virtual void execute(entityType*) = 0;

  // State exit function, run once, upon exit of the state
  virtual void exit(entityType*) = 0;
};



#endif
