#ifndef _NonUnitTimeCAT_h__
#define _NonUnitTimeCAT_h__

#include "ConflictAvoidanceTable.h"
#include "BucketHash.h"
#include <set>

struct IntervalData{
  IntervalData(uint64_t h1, uint64_t h2, uint8_t a):hash1(h1),hash2(h2),agent(a){}
  uint64_t hash1;
  uint64_t hash2;
  uint8_t agent;
  bool operator==(IntervalData const& other)const{return other.hash1==hash1 && other.hash2==hash2 && other.agent==agent;}
  bool operator<(IntervalData const& other)const{return other.hash1==hash1?(other.hash2==hash2?(other.agent==agent?false:agent<other.agent):hash2<other.hash2):hash1<other.hash1;}
};

template <typename state, typename environment, unsigned hashIntervalHundredths>
class NonUnitTimeCAT : public ConflictAvoidanceTable<state,environment>{
public:
  typedef std::set<IntervalData> ConflictSet;

  NonUnitTimeCAT():ConflictAvoidanceTable<state,environment>(){}
  virtual void remove(std::vector<state> const& thePath, environment const* env, unsigned agent){
    for(int i(0); i<thePath.size(); ++i) {
      // Populate the interval tree
      if(i) //!=0
        cat.remove(thePath[i-1].t, thePath[i].t, IntervalData(env->GetStateHash(thePath[i-1]), env->GetStateHash(thePath[i]), agent));
      else
        // No parent...
        cat.remove(thePath[i].t, thePath[i].t+.001, IntervalData(0, env->GetStateHash(thePath[i]), agent));
    }
  }
  virtual void insert(std::vector<state> const& thePath, environment const* env, unsigned agent){
    for(int i(0); i<thePath.size(); ++i) {
    // Populate the interval tree
      if(i) //!=0
        cat.insert(thePath[i-1].t, thePath[i].t, IntervalData(env->GetStateHash(thePath[i-1]), env->GetStateHash(thePath[i]), agent));
      else
        // No parent...
        cat.insert(thePath[i].t, thePath[i].t+.001, IntervalData(0, env->GetStateHash(thePath[i]), agent));
    }
  }
  void get(float t, float te, ConflictSet &result)const{
    cat.get(t,te,result);
  } 
  
private:
  BucketHash<IntervalData,hashIntervalHundredths> cat;
};

#endif
