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
#ifndef _ConflictAvoidanceTable_h__
#define _ConflictAvoidanceTable_h__

#include "SearchEnvironment.h"
#include <vector>
#include <algorithm>

// Stores paths for agents, indexed by time
template <typename state>
class UniversalConflictAvoidanceTable{
public:
  UniversalConflictAvoidanceTable(){}
  void set(std::vector<std::vector<state>*> const*const ref){s=ref;}
  void get(unsigned t1, unsigned t2, std::vector<state const*>& out, std::vector<unsigned> const& ignore)
  {
    for(unsigned i(0); i<s->size(); ++i){
      if(std::find(ignore.begin(),ignore.end(),i)==ignore.end()){
        unsigned start(std::min(t1/state::TIME_RESOLUTION_U,unsigned(s->at(i)->size()-1)));
        auto end(std::min(t2/state::TIME_RESOLUTION_U,unsigned(s->at(i)->size()-1)));
        if(start<s->at(i)->size()){
          while(start>0 && t1<s->at(i)->at(start).t){
            --start;
          }
          
          while(end>start && t2<s->at(i)->at(end).t){
            --end;
          }
          
          while(end<s->at(i)->size()-1 && t2>s->at(i)->at(end).t){
            ++end;
          }
          if(s->at(i)->at(start).t>t1){
            --start;
          }
          if(start==end)
          {
            if (start)
            {
              --start;
            }
            else
            {
              ++end;
            }
          }
          for(unsigned j(start); j<=end; ++j){
              out.push_back(&s->at(i)->at(j));
          }
        }
      }
    }
  }
  void get(unsigned t1, unsigned t2, std::vector<state const*>& out, unsigned ignore)
  {
    static std::vector<unsigned>ig(1);
    ig.clear();
    ig.push_back(ignore);
    get(t1,t2,out,ig);
  }
private:
  std::vector<std::vector<state>*> const* s;
};

// Stores paths for agents, indexed by time
template <typename state, typename action>
class ConflictAvoidanceTable{
public:
  ConflictAvoidanceTable(){}
  virtual void set(std::vector<std::vector<state>*> const*const ref){};
  virtual void set(std::vector<std::vector<state>> const*const ref){};
  virtual void remove(std::vector<state> const& values, SearchEnvironment<state,action> const*, unsigned agent)=0;
  virtual void insert(std::vector<state> const& values, SearchEnvironment<state,action> const*, unsigned agent)=0;
};

template <typename state>
struct PtrIntervalData{
  PtrIntervalData(state* p1, state* p2, uint32_t a):s1(p1),s1(p2),agent(a){}
  state const*const s1;
  state const*const s2;
  uint32_t agent;
  bool operator==(PtrIntervalData const& other)const{return other.s1==s1 && other.s2==s2 && other.agent==agent;}
  bool operator<(PtrIntervalData const& other)const{return other.s1==s1?(s2==other.s2?agent<other.agent:s2<other.s2):s1<other.s1;}
};

// Stores paths for agents, indexed by time
template <typename state, typename ConflictSet>
class PtrConflictAvoidanceTable{
public:
  PtrConflictAvoidanceTable(){}
  virtual void remove(std::vector<state> const& thePath, unsigned agent)=0;
  virtual void insert(std::vector<state> const& thePath, unsigned agent)=0;
  virtual void get(float t, float te, ConflictSet &result)const=0;
};

#endif
