#ifndef _ConflictAvoidanceTable_h__
#define _ConflictAvoidanceTable_h__

#include <vector>
#include "SearchEnvironment.h"

// Stores paths for agents, indexed by time
template <typename state, typename action>
class ConflictAvoidanceTable{
public:
  ConflictAvoidanceTable(){}
  virtual void set(std::vector<std::vector<state> > const*const ref){};
  virtual void remove(std::vector<state> const& values, SearchEnvironment<state,action> const*, unsigned agent)=0;
  virtual void insert(std::vector<state> const& values, SearchEnvironment<state,action> const*, unsigned agent)=0;
};

#endif
