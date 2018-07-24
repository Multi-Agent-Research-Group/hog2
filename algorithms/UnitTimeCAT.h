#ifndef _UnitTimeCAT_h__
#define _UnitTimeCAT_h__

#include "ConflictAvoidanceTable.h"
#include <set>

template <typename state, typename action>
class UnitTimeCAT : public ConflictAvoidanceTable<state,action>{
public:

  UnitTimeCAT():ConflictAvoidanceTable<state,action>(),cat(0) {}
  virtual void set(std::vector<std::vector<state> > const*const ref){cat=ref;}
  virtual void set(std::vector<std::vector<state>*> const*const ref){ptrCat=ref;}

  virtual void remove(std::vector<state> const& thePath, SearchEnvironment<state,action> const* env, unsigned agent){
    // Do nothing
  }
  virtual void insert(std::vector<state> const& thePath, SearchEnvironment<state,action> const* env, unsigned agent){
    // Do nothing
  }
  state const& get(unsigned agent, unsigned t)const{
    if(cat)
      return (*cat)[agent][t];
    else
      return *(*ptrCat)[agent][t];
  } 

  unsigned numAgents()const{return cat->size();}
  
private:
  std::vector<std::vector<state> > const* cat;
  std::vector<std::vector<state>*> const* ptrCat;
};

#endif
