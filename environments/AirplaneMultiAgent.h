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

typedef std::vector<airtimeState> MultiAgentState;
typedef std::vector<airplaneAction> MultiAgentAction;

// Actual Environment
class AirplaneMultiAgentEnvironment : public SearchEnvironment<MultiAgentState, MultiAgentAction>
{
  public:

    // Constructor
    AirplaneMultiAgentEnvironment(AirplaneConstrainedEnvironment * const base):env(base){}

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
    AirplaneConstrainedEnvironment const* const getEnv()const{return env;}

  protected:

    //mutable std::vector<MultiAgentAction> internalActions;

    static void generatePermutations(std::vector<MultiAgentAction>& actions, std::vector<MultiAgentAction>& result, int depth, MultiAgentAction const& current);


  private:
    AirplaneConstrainedEnvironment*const env;
};

static std::ostream& operator <<(std::ostream& os, MultiAgentState const& s){
  for(auto const& a : s){
    os << a << "/";
  }
  return os;
}

static std::ostream& operator <<(std::ostream& os, MultiAgentAction const& s){
  for(auto const& a : s){
    os << a << "/";
  }
  return os;
}

#endif /* AirplaneMultiAgent_h */
