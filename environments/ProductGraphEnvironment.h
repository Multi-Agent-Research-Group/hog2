/*
 *  Created by Thayne Walker, Rajat Kumar Jenamani.
 *  Copyright (c) Thayne Walker 2021 All rights reserved.
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

#ifndef ProductGraphEnvironment_h
#define ProductGraphEnvironment_h

#include <vector>
#include <cassert>
#include <cmath>
#include "SearchEnvironment.h"

template<typename state>
class MAState : public std::vector<state>{
  public:
    void setT(){
      for(auto s(this->begin()); s!=this->end(); ++s){
        t=std::max(t,s->t);
      }
    }
    uint32_t t;
};

// Note state must have fields: first,second
// Actual Environment
template<typename state, typename action, typename environment>
class ProductGraphEnvironment : public SearchEnvironment<MAState<state>, std::vector<action> >
{
  public:
    //typedef std::vector<state> MAState<state>;
    typedef std::vector<action> MultiAgentAction;

    // Constructor
    ProductGraphEnvironment(std::vector<environment const*> const& base):env(base){
      heuristics.reserve(env.size());
      for(auto const& e:base){
        heuristics.push_back((Heuristic<typename state::first_type> const*)e);
      }
    }
    ProductGraphEnvironment(std::vector<environment const*> const& base, std::vector<Heuristic<typename state::first_type> const*> const& h):env(base),heuristics(h){}

    virtual void GetSuccessors(const MAState<state> &nodeID, std::vector<MAState<state>> &neighbors) const;

    virtual void GetActions(const MAState<state> &nodeID, std::vector<MultiAgentAction> &actions) const{assert(false);}

    virtual void GetReverseActions(const MAState<state> &nodeID, std::vector<MultiAgentAction> &actions) const{assert(false);}

    virtual void ApplyAction(MAState<state> &s, MultiAgentAction dir) const{assert(false);}
    virtual void UndoAction(MAState<state> &s, MultiAgentAction const& dir) const{assert(false);}
    virtual void GetNextState(MAState<state> const& currents, MultiAgentAction const& dir, MAState<state> &news) const{assert(false);}
    virtual bool InvertAction(MultiAgentAction &a) const { return false; }
    virtual MultiAgentAction GetAction(MAState<state> const& node1, MAState<state> const& node2) const{assert(false);}


    // Heuristics and paths
    virtual double HCost(const MAState<state>& node1, const MAState<state>& node2) const;
    virtual double HCost(MAState<state> const& )  const { assert(false); return 0; }
    virtual double GCost(MAState<state> const& node1, MAState<state> const& node2) const;
    virtual double GCost(MAState<state> const& node1, MultiAgentAction const& act) const{assert(false);}
    virtual double GetPathLength(std::vector<MAState<state>> const& n) const;
    void loadPerimeterDB();

    // Goal testing
    virtual bool GoalTest(MAState<state> const& node, MAState<state> const& goal) const;
    virtual bool GoalTest(MAState<state> const&) const { assert(false); return false; }

    // Hashing
    virtual uint64_t GetStateHash(MAState<state> const& node) const;
    virtual uint64_t GetStateHash(state const& node) const{return env[0]->GetStateHash(node.second);}
    virtual uint64_t GetStateHash(unsigned agent, state const& node) const{return env[agent]->GetStateHash(node.second);}
    virtual uint64_t GetActionHash(MultiAgentAction act) const{assert(false);}

    // Drawing
    virtual void OpenGLDraw() const;
    virtual void OpenGLDraw(const MAState<state> &l) const;
    virtual void OpenGLDraw(const MAState<state>& oldState, const MAState<state> &newState, float perc) const;
    virtual void OpenGLDraw(const MAState<state> &, const MultiAgentAction &) const;
    void GLDrawLine(const MAState<state> &a, const MAState<state> &b) const;
    void GLDrawPath(const std::vector<MAState<state>> &p) const;

    MAState<state> const* goal;
    MAState<state> const& getGoal()const{return *goal;}
    void setGoal(MAState<state> const& g){goal=&g;}
    environment const* const getEnv(unsigned index)const{return env[index];}
    double ViolatesConstraint(MAState<state> const& a,MAState<state> const& n){return false;}

  protected:

    //mutable std::vector<MultiAgentAction> internalActions;

    static void generatePermutations(std::vector<std::vector<state>>& positions, std::vector<MAState<state>>& result, int agent, MAState<state>& current);


  private:
    std::vector<environment const*> env;
    std::vector<Heuristic<typename state::first_type> const*> heuristics;
};

template<typename state>
static std::ostream& operator <<(std::ostream& os, MAState<state> const& s){
  for(auto const& a : s){
    os << a.second << "/";
  }
  return os;
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::generatePermutations(std::vector<std::vector<state>>& positions, std::vector<MAState<state>>& result, int agent, MAState<state>& current){
  static double agentRadius=.25;
  if(agent == positions.size()) {
    current.setT();
    result.push_back(current);
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      if(collisionCheck3D(positions[agent][i].first,positions[agent][i].second,current[j].first,current[j].second,agentRadius)){
        found=true;
        break;
      }
    }
    if(found) continue;
    MAState<state> copy(current);
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy);
  }
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::GetSuccessors(const MAState<state> &s, std::vector<MAState<state>> &neighbors) const
{
  //Get successors into a vector
  std::vector<std::vector<state>> successors;
  successors.reserve(s.size());

  // Find minimum depth of current edges
  auto sd(s[0].second.t);
  unsigned  minindex(0);
  unsigned k(0);
  for(auto const& a: s){
    if(a.second.t<sd){
      minindex=k;
      sd=a.second.t; // We only care down to the msec.
    }
    ++k;
    //sd=min(sd,a.second.t);
  }
  //std::cout << "min-depth: " << sd << "\n";

  uint32_t md(0xffffffff); // Min depth of successors
  //Add in successors for parents who are equal to the min
  k=0;
  //bool first(true);
  for(auto const& a: s){
    std::vector<state> output;
    if(k==minindex){
    //if(first && a.second.t<=sd){
      //first=false;
      std::vector<typename state::first_type> n;
      env[k]->GetSuccessors(a.second,n);
      if(env[k]->GoalTest(a.second,env[k]->getGoal())){
        // Add a wait action that goes out a long time
        auto longone(a.second);
        longone.t=0xfffff;
        n.push_back(longone);
      }
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
    ++k;
  }
  MAState<state> tmp;
  generatePermutations(successors,neighbors,0,tmp);
}

template<typename state, typename action, typename environment>
double ProductGraphEnvironment<state,action,environment>::HCost(const MAState<state> &node1, const MAState<state> &node2) const
{
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += heuristics[i]->HCost(node1[i].second,node2[i].second);
  }
  return total;
}



template<typename state, typename action, typename environment>
double ProductGraphEnvironment<state,action,environment>::GCost(MAState<state> const& node1, MAState<state> const& node2) const {
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env[i]->GCost(node1[i].second,node2[i].second);
  }
  return total;
}

template<typename state, typename action, typename environment>
bool ProductGraphEnvironment<state,action,environment>::GoalTest(const MAState<state> &node, const MAState<state> &goal) const
{
  for(int i(0); i<node.size(); ++i){
    if(!env[i]->GoalTest(node[i].second,goal[i].second))
      return false;
  }
  return true;
}

template<typename state, typename action, typename environment>
double ProductGraphEnvironment<state,action,environment>::GetPathLength(const std::vector<MAState<state>> &sol) const
{
    //double gcost(0.0);
    //if(sol.size()>1)
      //for(auto n(sol.begin()+1); n!=sol.end(); ++n)
        //gcost += GCost(*(n-1),*n);
    //return gcost;
  // Compute cost where waiting at the goal is free.
  uint32_t cost(0);
  for(auto const& path:sol){
    for(int j(path.size()-1); j>0; --j){
      if(path[j-1].second!=path[j].second){
        cost += path[j].second.t;
        break;
      }else if(j==1){
        cost += path[0].second.t;
      }
    }
  }
  return cost;
}

template<typename state, typename action, typename environment>
uint64_t ProductGraphEnvironment<state,action,environment>::GetStateHash(const MAState<state> &node) const
{
  // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
  uint64_t h(14695981039346656037UL); // Offset basis
  unsigned i(0);
  for(auto const& v : node){
    uint64_t h1(env[i++]->GetStateHash(v.second));
    uint8_t c[sizeof(uint64_t)];
    memcpy(c,&h1,sizeof(uint64_t));
    for(unsigned j(0); j<sizeof(uint64_t); ++j){
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h=h^c[j]; // Xor with octet
      h=h*1099511628211; // multiply by the FNV prime
    }
  }
  return h;
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw() const
{
  env[0]->OpenGLDraw();
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw(const MAState<state> &l) const
{
  unsigned i(0);
  for(auto const& s: l)
    env[i++]->OpenGLDraw(s.second);
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw(const MAState<state>& o, const MAState<state> &n, float perc) const
{
  unsigned i(0);
  for(auto const& s: n)
    env[i++]->OpenGLDraw(s.first,s.second,perc);
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::OpenGLDraw(const MAState<state> &, const ProductGraphEnvironment<state,action,environment>::MultiAgentAction &) const
{
    //TODO: Implement this
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::GLDrawLine(const MAState<state> &a, const MAState<state> &b) const
{
  int i(0);
  for(auto const& s: a)
    env[i++]->GLDrawLine(s.first,s.second);
}

template<typename state, typename action, typename environment>
void ProductGraphEnvironment<state,action,environment>::GLDrawPath(const std::vector<MAState<state>> &p) const
{
        if(p.size()<2) return;
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          //GLDrawLine(*(a-1),*a);
        }
}

#endif /* MultiAgent_h */
