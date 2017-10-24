//
//  AirplaneMultiAgent.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirplaneMultiAgent_h
#define AirplaneMultiAgent_h

#include <vector>
#include <cassert>
#include <cmath>
#include "SearchEnvironment.h"
#include "AirplanePerimeterDBBuilder.h"
#include "AirplaneConstrained.h"


// Actual Environment
template<typename state, typename action, typename environment>
class AirplaneMultiAgentEnvironment : public SearchEnvironment<std::vector<state>, std::vector<action> >
{
  public:
    typedef std::vector<state> MultiAgentState;
    typedef std::vector<action> MultiAgentAction;

    // Constructor
    AirplaneMultiAgentEnvironment(ConstrainedEnvironment<state,action> * const base):env(base){}

    virtual void GetSuccessors(const MultiAgentState &nodeID, std::vector<MultiAgentState> &neighbors) const;

    virtual void GetActions(const MultiAgentState &nodeID, std::vector<MultiAgentAction> &actions) const;

    virtual void GetReverseActions(const MultiAgentState &nodeID, std::vector<MultiAgentAction> &actions) const;

    virtual void ApplyAction(MultiAgentState &s, MultiAgentAction dir) const;
    virtual void UndoAction(MultiAgentState &s, MultiAgentAction const& dir) const;
    virtual void GetNextState(MultiAgentState const& currents, MultiAgentAction const& dir, MultiAgentState &news) const;
    virtual bool InvertAction(MultiAgentAction &a) const { return false; }
    virtual MultiAgentAction GetAction(MultiAgentState const& node1, MultiAgentState const& node2) const;


    // Heuristics and paths
    virtual double HCost(const MultiAgentState& node1, const MultiAgentState& node2) const;
    virtual double HCost(MultiAgentState const& )  const { assert(false); return 0; }
    virtual double GCost(MultiAgentState const& node1, MultiAgentState const& node2) const;
    virtual double GCost(MultiAgentState const& node1, MultiAgentAction const& act) const;
    virtual double GetPathLength(std::vector<MultiAgentState> const& n) const;
    void loadPerimeterDB();

    // Goal testing
    virtual bool GoalTest(MultiAgentState const& node, MultiAgentState const& goal) const;
    virtual bool GoalTest(MultiAgentState const&) const { assert(false); return false; }

    // Hashing
    virtual uint64_t GetStateHash(MultiAgentState const& node) const;
    virtual uint64_t GetActionHash(MultiAgentAction act) const;

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
    ConstrainedEnvironment<state,action> const* const getEnv()const{return env;}

  protected:

    //mutable std::vector<MultiAgentAction> internalActions;

    static void generatePermutations(std::vector<MultiAgentAction>& actions, std::vector<MultiAgentAction>& result, int depth, MultiAgentAction const& current);


  private:
    ConstrainedEnvironment<state,action> *const env;
};

template<typename state>
static std::ostream& operator <<(std::ostream& os, std::vector<state> const& s){
  for(auto const& a : s){
    os << a << "/";
  }
  return os;
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::generatePermutations(std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction>& actions, std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction>& result, int depth, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction const& current) {
    if(depth == actions.size()) {
       result.push_back(current);
       return;
     }

    for(int i = 0; i < actions[depth].size(); ++i) {
        AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction copy(current);
        copy.push_back(actions[depth][i]);
        generatePermutations(actions, result, depth + 1, copy);
    }
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::GetSuccessors(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &nodeID, std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState> &neighbors) const
{
  std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction> actions;
  GetActions(nodeID, actions);
  for (auto &act : actions) {
    AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState s(nodeID);
    
    //std::cout << "Generated from: " << s << "->" << act;
    ApplyAction(s,act);
    bool hasConflict(false);
    for(int i(0); i<s.size(); ++i){
      for(int j(i+1); j<s.size(); ++j){
        Collision<airtimeState> ec1(nodeID[i],s[i]);
        Collision<airtimeState> ec2(nodeID[j],s[j]);
        // Only keep this state if there are no internal conflicts
        if(ec1.ConflictsWith(ec2)){
          //std::cout << "CONFLICT: " << (ac.ConflictsWith(s[j])?"vertex":"edge") << s << "\n";
          hasConflict = true;
          break;
        }
      }
      if(hasConflict){break;}
    }
    if(!hasConflict){
      //std::cout << ">>>" << s << "\n";
      neighbors.push_back(s);}
  }
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::GetActions(AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState const& nodeID, std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction> &actions) const
{
  std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction> temp(nodeID.size());
  actions.resize(0);
  int i(0);
  for(auto s : nodeID){
    env->setGoal(getGoal()[i]);
    env->GetActions(s,temp[i++]);
  }
  /*std::cout << "Actions: \n";
  for(auto const& a : temp){
    std::cout << "\n";
    for(auto const& aa : a){
      std::cout << aa <<"\n";
    }
  }*/
  AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction c;
  generatePermutations(temp, actions, 0, c);
  /*std::cout << "Combined Actions: \n";
  for(auto const& a : actions){
    std::cout << "\n";
    for(auto const& aa : a){
      std::cout << " " << aa <<"\n";
    }
  }*/
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::GetReverseActions(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &nodeID, std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction> &actions) const
{
  actions.resize(nodeID.size());
  int i(0);
  for(auto &s : nodeID){
    env->GetReverseActions(s,actions[i++]);
  }
}

/** Gets the action, typename environment> required to go from node1 to node2 */
template<typename state, typename action, typename environment>
typename AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction AirplaneMultiAgentEnvironment<state,action,environment>::GetAction(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &node1, const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &node2) const
{
  AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction a;
  for(int i(0); i<node1.size(); ++i){
    env->setGoal(getGoal()[i]);
    a.push_back(env->GetAction(node1[i],node2[i])); 
  }
  return a;
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::ApplyAction(AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &s, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction dir) const
{
  int i(0);
  for(state& st : s){
    env->setGoal(getGoal()[i]);
    env->ApplyAction(st,dir[i++]); // Modify state in-place
  }
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::UndoAction(AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &s, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction const& dir) const
{
  int i(0);
  for(state& st : s){
    env->UndoAction(st,dir[i++]);
  }
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::GetNextState(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &currents, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction const& dir, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &news) const
{
    news = currents;
    ApplyAction(news, dir);
}

template<typename state, typename action, typename environment>
double AirplaneMultiAgentEnvironment<state,action,environment>::HCost(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &node1, const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &node2) const
{
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->HCost(node1[i],node2[i]);
  }
  return total;
}



template<typename state, typename action, typename environment>
double AirplaneMultiAgentEnvironment<state,action,environment>::GCost(AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState const& node1, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState const& node2) const {
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->GCost(node1[i],node2[i]);
  }
  return total;
}

template<typename state, typename action, typename environment>
double AirplaneMultiAgentEnvironment<state,action,environment>::GCost(AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState const& node1, AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction const& act) const {
  double total(0.0);
  for(int i(0); i<node1.size(); ++i){
    total += env->GCost(node1[i],act[i]);
  }
  return total;
}


template<typename state, typename action, typename environment>
bool AirplaneMultiAgentEnvironment<state,action,environment>::GoalTest(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &node, const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &goal) const
{
  bool done(true);
  for(int i(0); i<node.size(); ++i){
    done = done && env->GoalTest(node[i],goal[i]);
  }
  return done;
}

template<typename state, typename action, typename environment>
double AirplaneMultiAgentEnvironment<state,action,environment>::GetPathLength(const std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState> &sol) const
{
    double gcost(0.0);
    if(sol.size()>1)
      for(auto n(sol.begin()+1); n!=sol.end(); ++n)
        gcost += GCost(*(n-1),*n);
    return gcost;
}

template<typename state, typename action, typename environment>
uint64_t AirplaneMultiAgentEnvironment<state,action,environment>::GetStateHash(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &node) const
{
    uint64_t h = 0;
    for(auto const& s : node){
      h = (h * 16777619) ^ env->GetStateHash(s); // xor
    }
    return h;
}

template<typename state, typename action, typename environment>
uint64_t AirplaneMultiAgentEnvironment<state,action,environment>::GetActionHash(AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction act) const
{
    uint64_t h = 0;
    for(auto& s : act){
      h ^= env->GetActionHash(s); // xor
    }
    return h;
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::OpenGLDraw() const
{
  env->OpenGLDraw();
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::OpenGLDraw(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &l) const
{
  for(auto const& s: l)
    env->OpenGLDraw(s);
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::OpenGLDraw(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState& o, const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &n, float perc) const
{
  int i(0);
  for(auto const& s: n)
    env->OpenGLDraw(s,n[i++],perc);
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::OpenGLDraw(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &, const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentAction &) const
{
    //TODO: Implement this
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::GLDrawLine(const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &a, const AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState &b) const
{
  int i(0);
  for(auto const& s: a)
    env->GLDrawLine(s,b[i++]);
}

template<typename state, typename action, typename environment>
void AirplaneMultiAgentEnvironment<state,action,environment>::GLDrawPath(const std::vector<AirplaneMultiAgentEnvironment<state,action,environment>::MultiAgentState> &p) const
{
        if(p.size()<2) return;
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}

#endif /* AirplaneMultiAgent_h */
