#ifndef _UnitTimeCAT_h__
#define _UnitTimeCAT_h__

#include "ConflictAvoidanceTable.h"
#include <set>

template <typename state, typename environment>
class UnitTimeCAT : public ConflictAvoidanceTable<state,environment>{
public:

  UnitTimeCAT():ConflictAvoidanceTable<state,environment>(),cat(0) {}
  virtual void set(std::vector<std::vector<state> > const*const ref){cat=ref;}

  virtual void remove(std::vector<state> const& thePath, environment const* env, unsigned agent){
    // Do nothing
  }
  virtual void insert(std::vector<state> const& thePath, environment const* env, unsigned agent){
    // Do nothing
  }
  state const& get(unsigned agent, unsigned t)const{
    return (*cat)[agent][t];
  } 

  unsigned numAgents()const{return cat->size();}
  
private:
  std::vector<std::vector<state> > const* cat;
};

#endif
