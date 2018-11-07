#ifndef BASEENTITY_H
#define BASEENTITY_H


// Base class for an entity
// Needs to be inherited
// by a more specific entity
// class
class BaseEntity
{
 public:
  BaseEntity(int id)
    {
      setId(id);
    }
  
  // Entities must have an update
  // function. This needs to be
  // continually polled for the
  // entity to correctly
  // "animate" itself. 
  virtual void update() = 0;

  int getId() const;
  
 private:
  // Static so that you can 
  // ensure that multiple 
  // instances of a BaseEntity
  // will all have mutually
  // exclusive ID's. 
  static int nextValidId;

  // Identifying number
  int id;
  void setId(int id);
};


#endif
