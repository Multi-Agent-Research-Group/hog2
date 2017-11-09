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

#ifndef MultiAgentEnvironment_h
#define MultiAgentEnvironment_h

#include <vector>
#include <cassert>
#include <cmath>
#include "SearchEnvironment.h"


// Note state must have fields: first,second
// Actual Environment
template<typename state, typename action, typename environment>
class MultiAgentEnvironment : public SearchEnvironment<std::vector<state>, std::vector<action> >
{
  public:
    typedef std::vector<state> MultiAgentState;
    typedef std::vector<action> MultiAgentAction;

    // Constructor
    MultiAgentEnvironment(environment * const base):env(base){}

    virtual void GetSuccessors(const MultiAgentState &nodeID, std::vector<MultiAgentState> &neighbors) const;

    virtual void GetActions(const MultiAgentState &nodeID, std::vector<MultiAgentAction> &actions) const{assert(false);}

    virtual void GetReverseActions(const MultiAgentState &nodeID, std::vector<MultiAgentAction> &actions) const{assert(false);}

    virtual void ApplyAction(MultiAgentState &s, MultiAgentAction dir) const{assert(false);}
    virtual void UndoAction(MultiAgentState &s, MultiAgentAction const& dir) const{assert(false);}
    virtual void GetNextState(MultiAgentState const& currents, MultiAgentAction const& dir, MultiAgentState &news) const{assert(false);}
    virtual bool InvertAction(MultiAgentAction &a) const { return false; }
    virtual MultiAgentAction GetAction(MultiAgentState const& node1, MultiAgentState const& node2) const{assert(false);}


    // Heuristics and paths
    virtual double HCost(const MultiAgentState& node1, const MultiAgentState& node2) const;
    virtual double HCost(MultiAgentState const& )  const { assert(false); return 0; }
    virtual double GCost(MultiAgentState const& node1, MultiAgentState const& node2) const;
    virtual double GCost(MultiAgentState const& node1, MultiAgentAction const& act) const{assert(false);}
    virtual double GetPathLength(std::vector<MultiAgentState> const& n) const;
    void loadPerimeterDB();

    // Goal testing
    virtual bool GoalTest(MultiAgentState const& node, MultiAgentState const& goal) const;
    virtual bool GoalTest(MultiAgentState const&) const { assert(false); return false; }

    // Hashing
    virtual uint64_t GetStateHash(MultiAgentState const& node) const;
    virtual uint64_t GetActionHash(MultiAgentAction act) const{assert(false);}

    // Drawing
    virtual void OpenGLDraw() const;
    virtual void OpenGLDraw(const MultiAgentState &l) const;
    virtual void OpenGLDraw(const MultiAgentState& oldState, const MultiAgentState &newState, float perc) const;
    virtual void OpenGLDraw(const MultiAgentState &, const MultiAgentAction &) const;
    void GLDrawLine(const MultiAgentState &a, const MultiAgentState &b) const;
    void GLDrawPath(const std::vector<MultiAgentState> &p) const;

    MultiAgentState const* goal;
    MultiAgentState const& getGoal()const{return *goal;}
    void setGoal(MultiAgentState const& g){goal=&g;}
    environment const* const getEnv()const{return env;}

  protected:

    //mutable std::vector<MultiAgentAction> internalActions;

    static void generatePermutations(std::vector<MultiAgentState>& positions, std::vector<MultiAgentState>& result, int agent, MultiAgentState const& current, uint32_t lastTime);


  private:
    environment *const env;
};

template<typename state>
static std::ostream& operator <<(std::ostream& os, std::vector<state> const& s){
  for(auto const& a : s){
    os << a.second << "/";
  }
  return os;
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::generatePermutations(std::vector<MultiAgentState>& positions, std::vector<MultiAgentState>& result, int agent, MultiAgentState const& current, uint32_t lastTime) {
  static double agentRadius=.25;
  if(agent == positions.size()) {
    result.push_back(current);
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    bool found(false);
    MultiAgentState copy(current);
    for(int j(0); j<current.size(); ++j){
      if(collisionCheck3D(positions[agent][i].first,positions[agent][i].second,current[j].first,current[j].second,agentRadius)){
        found=true;
        break;
      }
    }
    if(found) continue;
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy,lastTime);
  }
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::GetSuccessors(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &s, std::vector<MultiAgentEnvironment<state,action,environment>::MultiAgentState> &neighbors) const
{
  //Get successors into a vector
  std::vector<std::vector<state>> successors;
  successors.reserve(s.size());

  // Find minimum depth of current edges
  uint32_t sd(0xffffffff);
  for(auto const& a: s){
    sd=min(sd,a.second.t);
  }
  //std::cout << "min-depth: " << sd << "\n";

  uint32_t md(0xffffffff); // Min depth of successors
  //Add in successors for parents who are equal to the min
  for(auto const& a: s){
    std::vector<state> output;
    if(a.second.t<=sd){
      std::vector<typename state::first_type> n;
      env->GetSuccessors(a.second,n);
      //std::cout << "Keep Successors of " << *a.second << "\n";
      for(auto const& b: n){
        output.emplace_back(a.second,b);
        md=min(md,b.t);
      }
    }else{
      //std::cout << "Keep Just " << *a.second << "\n";
      output.push_back(a);
      md=min(md,a.second.t);
    }
    if(output.empty()){
      // Stay at state...
      typename state::first_type tmp(a.second,0xffffffff);
      output.emplace_back(a.second,tmp);
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
  }
  MultiAgentState tmp;
  generatePermutations(successors,neighbors,0,tmp,sd);
}

template<typename state, typename action, typename environment>
double MultiAgentEnvironment<state,action,environment>::HCost(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &node1, const MultiAgentEnvironment<state,action,environment>::MultiAgentState &node2) const
{
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->HCost(node1[i].second,node2[i].second);
  }
  return total;
}



template<typename state, typename action, typename environment>
double MultiAgentEnvironment<state,action,environment>::GCost(MultiAgentEnvironment<state,action,environment>::MultiAgentState const& node1, MultiAgentEnvironment<state,action,environment>::MultiAgentState const& node2) const {
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->GCost(node1[i].second,node2[i].second);
  }
  return total;
}

template<typename state, typename action, typename environment>
bool MultiAgentEnvironment<state,action,environment>::GoalTest(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &node, const MultiAgentEnvironment<state,action,environment>::MultiAgentState &goal) const
{
  bool done(true);
  for(int i(0); i<node.size(); ++i){
    done = done && env->GoalTest(node[i].second,goal[i].second);
  }
  return done;
}

template<typename state, typename action, typename environment>
double MultiAgentEnvironment<state,action,environment>::GetPathLength(const std::vector<MultiAgentEnvironment<state,action,environment>::MultiAgentState> &sol) const
{
    double gcost(0.0);
    if(sol.size()>1)
      for(auto n(sol.begin()+1); n!=sol.end(); ++n)
        gcost += GCost(*(n-1),*n);
    return gcost;
}

template<typename state, typename action, typename environment>
uint64_t MultiAgentEnvironment<state,action,environment>::GetStateHash(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &node) const
{
    uint64_t h = 0;
    for(auto const& s : node){
      h = (h * 16777619) ^ env->GetStateHash(s.second); // xor
    }
    return h;
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::OpenGLDraw() const
{
  env->OpenGLDraw();
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::OpenGLDraw(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &l) const
{
  for(auto const& s: l)
    env->OpenGLDraw(s.second);
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::OpenGLDraw(const MultiAgentEnvironment<state,action,environment>::MultiAgentState& o, const MultiAgentEnvironment<state,action,environment>::MultiAgentState &n, float perc) const
{
  for(auto const& s: n)
    env->OpenGLDraw(s.first,s.second,perc);
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::OpenGLDraw(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &, const MultiAgentEnvironment<state,action,environment>::MultiAgentAction &) const
{
    //TODO: Implement this
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::GLDrawLine(const MultiAgentEnvironment<state,action,environment>::MultiAgentState &a, const MultiAgentEnvironment<state,action,environment>::MultiAgentState &b) const
{
  int i(0);
  for(auto const& s: a)
    env->GLDrawLine(s.first,s.second);
}

template<typename state, typename action, typename environment>
void MultiAgentEnvironment<state,action,environment>::GLDrawPath(const std::vector<MultiAgentEnvironment<state,action,environment>::MultiAgentState> &p) const
{
        if(p.size()<2) return;
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          //GLDrawLine(*(a-1),*a);
        }
}

#endif /* MultiAgent_h */
