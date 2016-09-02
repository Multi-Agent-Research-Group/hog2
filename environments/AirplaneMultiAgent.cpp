//
//  AirplaneMultiAgent.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include "AirplaneMultiAgent.h"
#include <iostream>
#include <gl.h>
#include "TemplateAStar.h"
#include "Heuristic.h"

void AirplaneMultiAgentEnvironment::generatePermutations(std::vector<MultiAgentAction>& actions, std::vector<MultiAgentAction>& result, int depth, MultiAgentAction const& current) {
    if(depth == actions.size()) {
       result.push_back(current);
       return;
     }

    for(int i = 0; i < actions[depth].size(); ++i) {
        MultiAgentAction copy(current);
        copy.push_back(actions[depth][i]);
        generatePermutations(actions, result, depth + 1, copy);
    }
}

void AirplaneMultiAgentEnvironment::GetSuccessors(const MultiAgentState &nodeID, std::vector<MultiAgentState> &neighbors) const
{
  std::vector<MultiAgentAction> actions;
  GetActions(nodeID, actions);
  for (auto &act : actions) {
    MultiAgentState s(nodeID);
    ApplyAction(s,act);
    for(int i(0); i<s.size(); ++i){
      for(int j(i); j<s.size(); ++j){
        airConstraint ac(s[i]);
        airConstraint ec1(nodeID[i],s[i]);
        airConstraint ec2(nodeID[j],s[j]);
        // Only keep this state if there are no internal conflicts
        if(!ac.ConflictsWith(s[j]) && !ec1.ConflictsWith(ec2)){
          neighbors.push_back(s);
        }
      }
    }
  }
}

void AirplaneMultiAgentEnvironment::GetActions(MultiAgentState const& nodeID, std::vector<MultiAgentAction> &actions) const
{
  std::vector<MultiAgentAction> temp(nodeID.size());
  actions.resize(0);
  int i(0);
  for(auto s : nodeID){
    env->GetActions(s,temp[i++]);
  }
  MultiAgentAction c;
  generatePermutations(temp, actions, 0, c);
}

void AirplaneMultiAgentEnvironment::GetReverseActions(const MultiAgentState &nodeID, std::vector<MultiAgentAction> &actions) const
{
  actions.resize(nodeID.size());
  int i(0);
  for(auto &s : nodeID){
    env->GetReverseActions(s,actions[i++]);
  }
}

/** Gets the action required to go from node1 to node2 */
MultiAgentAction AirplaneMultiAgentEnvironment::GetAction(const MultiAgentState &node1, const MultiAgentState &node2) const
{
  MultiAgentAction a;
  for(int i(0); i<node1.size(); ++i){
    a.push_back(env->GetAction(node1[i],node2[i])); 
  }
  return a;
}

void AirplaneMultiAgentEnvironment::ApplyAction(MultiAgentState &s, MultiAgentAction dir) const
{
  int i(0);
  for(airtimeState& state : s){
    env->ApplyAction(state,dir[i++]); // Modify state in-place
  }
}

void AirplaneMultiAgentEnvironment::UndoAction(MultiAgentState &s, MultiAgentAction const& dir) const
{
  int i(0);
  for(airtimeState& state : s){
    env->UndoAction(state,dir[i++]);
  }
}

void AirplaneMultiAgentEnvironment::GetNextState(const MultiAgentState &currents, MultiAgentAction const& dir, MultiAgentState &news) const
{
    news = currents;
    ApplyAction(news, dir);
}

double AirplaneMultiAgentEnvironment::HCost(const MultiAgentState &node1, const MultiAgentState &node2) const
{
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->HCost(node1[i],node2[i]);
  }
  return total;
}



double AirplaneMultiAgentEnvironment::GCost(MultiAgentState const& node1, MultiAgentState const& node2) const {
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->GCost(node1[i],node2[i]);
  }
  return total;
}

double AirplaneMultiAgentEnvironment::GCost(MultiAgentState const& node1, MultiAgentAction const& act) const {
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->GCost(node1[i],act[i]);
  }
  return total;
}


bool AirplaneMultiAgentEnvironment::GoalTest(const MultiAgentState &node, const MultiAgentState &goal) const
{
  bool done(true);
  for(int i(0); i<node.size(); ++i){
    done = done && env->GoalTest(node[i],goal[i]);
  }
  return done;
}

double AirplaneMultiAgentEnvironment::GetPathLength(const std::vector<MultiAgentState> &sol) const
{
    double gcost(0.0);
    if(sol.size()>1)
      for(auto n(sol.begin()+1); n!=sol.end(); ++n)
        gcost += GCost(*(n-1),*n);
    return gcost;
}

uint64_t AirplaneMultiAgentEnvironment::GetStateHash(const MultiAgentState &node) const
{
    uint64_t h = 0;
    for(auto const& s : node){
      h = (h * 16777619) ^ env->GetStateHash(s); // xor
    }
    return h;
}

uint64_t AirplaneMultiAgentEnvironment::GetActionHash(MultiAgentAction act) const
{
    uint64_t h = 0;
    for(auto& s : act){
      h ^= env->GetActionHash(s); // xor
    }
    return h;
}

void AirplaneMultiAgentEnvironment::OpenGLDraw() const
{
  env->OpenGLDraw();
}

void AirplaneMultiAgentEnvironment::OpenGLDraw(const MultiAgentState &l) const
{
  for(auto const& s: l)
    env->OpenGLDraw(s);
}

void AirplaneMultiAgentEnvironment::OpenGLDraw(const MultiAgentState& o, const MultiAgentState &n, float perc) const
{
  int i(0);
  for(auto const& s: n)
    env->OpenGLDraw(s,n[i++],perc);
}

void AirplaneMultiAgentEnvironment::OpenGLDraw(const MultiAgentState &, const MultiAgentAction &) const
{
    //TODO: Implement this
}

void AirplaneMultiAgentEnvironment::GLDrawLine(const MultiAgentState &a, const MultiAgentState &b) const
{
  int i(0);
  for(auto const& s: a)
    env->GLDrawLine(s,b[i++]);
}

void AirplaneMultiAgentEnvironment::GLDrawPath(const std::vector<MultiAgentState> &p) const
{
        if(p.size()<2) return;
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}
