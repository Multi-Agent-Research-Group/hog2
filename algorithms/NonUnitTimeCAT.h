/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef _NonUnitTimeCAT_h__
#define _NonUnitTimeCAT_h__

#include "ConflictAvoidanceTable.h"
#include "BucketHash.h"


struct IntervalData{
  IntervalData(uint64_t h1, uint64_t h2, uint8_t a):hash1(h1),hash2(h2),agent(a){}
  uint64_t hash1;
  uint64_t hash2;
  uint8_t agent;
  bool operator==(IntervalData const& other)const{return other.hash1==hash1 && other.hash2==hash2 && other.agent==agent;}
  bool operator<(IntervalData const& other)const{return other.hash1==hash1?(other.hash2==hash2?(other.agent==agent?false:agent<other.agent):hash2<other.hash2):hash1<other.hash1;}
};

template <typename state, typename action>
class NonUnitTimeCAT : public ConflictAvoidanceTable<state,action>{
public:
  typedef std::set<IntervalData> ConflictSet;
  static double bucketWidth;

  NonUnitTimeCAT():ConflictAvoidanceTable<state,action>(),cat(bucketWidth){}
  virtual void remove(std::vector<state> const& thePath, SearchEnvironment<state,action> const* env, unsigned agent){
    for(int i(0); i<thePath.size(); ++i) {
      // Populate the interval tree
      if(i) //!=0
        cat.remove(thePath[i-1].t, thePath[i].t, IntervalData(env->GetStateHash(thePath[i-1]), env->GetStateHash(thePath[i]), agent));
      else
        // No parent...
        cat.remove(thePath[i].t, thePath[i].t+.001, IntervalData(0, env->GetStateHash(thePath[i]), agent));
    }
  }
  virtual void insert(std::vector<state> const& thePath, SearchEnvironment<state,action> const* env, unsigned agent){
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
  BucketHash<IntervalData> cat;
};

#endif
