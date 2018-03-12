#ifndef _UnitTimeCAT_h__
#define _UnitTimeCAT_h__

#include "ConflictAvoidanceTable.h"
#include <set>

template <typename BB, typename action>
class UnitTimeCAT : public ConflictAvoidanceTable<BB,action>{
public:

  UnitTimeCAT():ConflictAvoidanceTable<BB,action>(),cat(0) {}
  virtual void set(std::vector<std::vector<BB>*> const*const ref){cat=ref;}

  virtual void remove(std::vector<BB> const& thePath, SearchEnvironment<typename BB::State,action> const* env, unsigned agent){
    // Do nothing
  }
  virtual void insert(std::vector<BB> const& thePath, SearchEnvironment<typename BB::State,action> const* env, unsigned agent){
    // Do nothing
  }
  typename BB::State const& get(unsigned agent, unsigned t)const{
    return (*(*cat)[agent])[t];
  } 

  unsigned numAgents()const{return cat->size();}
  
private:
  std::vector<std::vector<typename BB::State>*> const* cat;
};

#endif
