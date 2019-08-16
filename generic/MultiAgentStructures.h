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
#ifndef MultiAgentStructures_H
#define MultiAgentStructures_H

#include "GenericSearchAlgorithm.h"
#include "CollisionDetection.h"
#include "NonUnitTimeCAT.h"
#include "UnitTimeCAT.h"
#include <set>

template<typename state>
using MultiAgentState = std::vector<state>;

template<typename state>
using Solution = std::vector<std::vector<state>>;

typedef std::set<IntervalData> ConflictSet;

template<typename state, typename action>
struct EnvironmentContainer {
  EnvironmentContainer() : name("NULL ENV"), environment(nullptr), heuristic(nullptr), threshold(0), astar_weight(1.0f) {
    std::cout << "stuff\n";
  }
  EnvironmentContainer(std::string n, ConstrainedEnvironment<state,action>* e, Heuristic<state>* h, uint32_t conf, float a) : name(n), environment(e), heuristic(h), threshold(conf), astar_weight(a) {}
  void SetupSearch(GenericSearchAlgorithm<state,action,ConstrainedEnvironment<state,action>>& srch){if(heuristic)srch.SetHeuristic(heuristic); srch.SetWeight(astar_weight);}
  std::string name;
  ConstrainedEnvironment<state,action>* environment;
  Heuristic<state>* heuristic; // This could be an array if we are planning for multiple legs
  unsigned threshold;
  float astar_weight;
};

// Struct for parsing config files
struct EnvData{
  EnvData():name(),agentType('A'),threshold(0),weight(1.0),group(){}
  EnvData(unsigned g,std::string const& n, char a, unsigned t, double w):name(n),agentType(a),threshold(t),weight(w),group(g){}
  std::string name;
  char agentType; // 'A'=Air, 'G'=Ground, 'S'=Surface
  unsigned threshold;
  double weight;
  unsigned group;
};

template<typename state, typename action>
class MAPFAlgorithm{
  public:
  virtual void GetSolution(std::vector<EnvironmentContainer<state,action>*> const& env, MultiAgentState<state> const& start, MultiAgentState<state> const& goal, Solution<state>& solution, std::string& hint){}
  virtual void GetSolution(std::vector<EnvironmentContainer<state,action>*> const& env, MultiAgentState<state> const& start, MultiAgentState<state> const& goal, Solution<state>& solution, std::string& hint, std::vector<GroupConflictDetector<state>*> const& detectors){}
  virtual unsigned GetNodesExpanded()const=0;
};

template<typename state>
bool checkCollision(std::vector<state> const& p1, std::vector<state> const& p2,float radius=.5,bool loud=false){
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    Vector2D A(*ap);
    Vector2D B(*bp);
    Vector2D VA(*a);
    VA-=A;
    VA.Normalize();
    Vector2D VB(*b);
    VB-=B;
    VB.Normalize();
    if(collisionImminent(A,VA,radius,ap->t,a->t,B,VB,radius,bp->t,b->t)){
      if(loud)std::cout << "Collision: " << *ap << "-->" << *a << "," << *bp << "-->" << *b << "\n";
      return false;
    }
    if(fless(a->t,b->t)){
      ++a;
      ++ap;
    }else if(fgreater(a->t,b->t)){
      ++b;
      ++bp;
    }else{
      ++a;++b;
      ++ap;++bp;
    }
  }
  return true;
}

template<typename state>
bool validateSolution(Solution<state> const& sol, bool verbose=true){
  for(auto a(sol.begin()); a!=sol.end(); ++a){
    for(auto b(a+1); b!=sol.end(); ++b){
      if(!checkCollision<state>(*a,*b,.25,true)) return false;
    }
  }
  return true;
}

// Check if an openlist node conflicts with a node from an existing path
template<typename state>
unsigned checkForConflict(state const*const parent, state const*const node, state const*const pathParent, state const*const pathNode){
  Collision<state> v(*node,*node);
  if(v.ConflictsWith(*pathNode,*pathNode)){return 1;}
  if(parent && pathParent){
    Collision<state> e1(*parent,*node);
    if(e1.ConflictsWith(*pathParent,*pathNode)){return 1;}
  }
  return 0; 
}

#endif
