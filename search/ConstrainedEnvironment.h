//
//  AirplaneConstrained.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__ConstrainedEnvironment__
#define __hog2_glut__ConstrainedEnvironment__

#include <vector>
#include "SearchEnvironment.h"

/*
struct Action{
  public:
    Action(){}
    virtual bool operator==(Action const&)const=0; // Equivalency check
    virtual bool operator!=(Action const& other)const{return ! this->operator==(other);} // Unequivalency check
    virtual void OpenGLDraw() const {} // Draw
};

class State{
  public:
    State(){}
    virtual bool operator==(State const&)const=0; // Equivalency check
    virtual bool operator!=(State const& other)const{return ! this->operator==(other);} // Unequivalency check
    virtual bool operator+=(Action const&)const=0; // Perform action
    virtual bool operator-=(Action const&)const=0; // Undo action
    virtual uint64_t hash()const=0; //Unique hash function
    virtual void OpenGLDraw() const {} // Draw
};

*/
template<typename State>
class Constraint {
  public:
    Constraint() {}
    Constraint(State const& start) : start_state(start), end_state(start) {}
    Constraint(State const& start, State const& end) : start_state(start), end_state(end) {}

    State start() const {return start_state;}
    State end() const {return end_state;}

    virtual bool ConflictsWith(State const& s) const {return start_state == s || end_state == s;}
    virtual bool ConflictsWith(State const& from, State const& to) const {return ConflictsWith(from) || ConflictsWith(to);}
    virtual bool ConflictsWith(Constraint const& x) const {return ConflictsWith(x.start_state, x.end_state);}
    virtual void OpenGLDraw() const {}

    State start_state;
    State end_state;
};
template<typename State, typename Action>
class ConstrainedEnvironment : public SearchEnvironment<State, Action> {
  public:
    /** Add a constraint to the model */
    virtual void AddConstraint(Constraint<State> c) = 0;
    /** Clear the constraints */
    virtual void ClearConstraints() = 0;
    /** Get the possible actions from a state */
    virtual void GetActions(const State &nodeID, std::vector<Action> &actions) const = 0;
    virtual void GetReverseActions(const State &nodeID, std::vector<Action> &actions) const = 0;
    /** Get the successor states not violating constraints */
    virtual void GetSuccessors(const State &nodeID, std::vector<State> &neighbors) const = 0;
    /** Checks to see if any constraint is violated */
    virtual bool ViolatesConstraint(const State &from, const State &to) const = 0;
};

#endif /* defined(__hog2_glut__ConstrainedEnvironment__) */
