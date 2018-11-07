#include "baseEntity.h"
#include <cassert>
#include <iostream>
using namespace std;


// Initialize the static nextValidId variable
int BaseEntity::nextValidId = 0;



int BaseEntity::getId() const
{
  return id;
}



// This function simply makes sure that 
// Each instance of a BaseEntity object has a 
// unique id. Handy for messaging.
void BaseEntity::setId(int newId)
{
  assert((newId >= nextValidId) && "<baseEntity::setId>: invalid ID");

  id = newId;
  nextValidId = id + 1;
  cout<<"Instatiated a base entity with id "<<id<<endl;
}
