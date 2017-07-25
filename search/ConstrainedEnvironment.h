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
#include "Map.h"
#include "SearchEnvironment.h"
#include "PositionalUtils.h"

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

class DrawableConstraint{
  public:
    virtual void OpenGLDraw(Map*) const=0;
    static int width;
    static int length;
    static int height;
};

// Represents a radial constraint where cost increases as we approach the center
template<typename State>
class SoftConstraint : public DrawableConstraint {
  public:
    SoftConstraint() {}
    SoftConstraint(State const& c, double r) : center(c),radius(r),logr(log(r)){}

    virtual double cost(State const& other, double scale) const{
      double d(Util::distance(center.x,center.y,center.height,other.x,other.y,other.height)/scale);
      return std::max(0.0,logr/d-logr/radius);
    }
    virtual void OpenGLDraw(Map*) const {}

    State center;
    double radius;
    double logr;
};

template<typename State>
class Constraint : public DrawableConstraint{
  public:
    Constraint() {}
    Constraint(State const& start) : start_state(start), end_state(start) {}
    Constraint(State const& start, State const& end) : start_state(start), end_state(end) {}

    State start() const {return start_state;}
    State end() const {return end_state;}

    virtual bool ConflictsWith(State const& s) const {return start_state == s || end_state == s;}
    virtual bool ConflictsWith(State const& from, State const& to) const {return ConflictsWith(from) || ConflictsWith(to);}
    virtual bool ConflictsWith(Constraint const& x) const {return ConflictsWith(x.start_state, x.end_state);}
    virtual void OpenGLDraw(Map*) const {}

    State start_state;
    State end_state;
};
template<typename State, typename Action>
class ConstrainedEnvironment : public SearchEnvironment<State, Action> {
  public:
    /** Add a constraint to the model */
    virtual void AddConstraint(Constraint<State> const& c) = 0;
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

// We initialize these here, but they can be changed at run-time
// These are only necessary for displaying constraints in OpenGL
/*template<typename State>
int Constraint<State>::width=80;
template<typename State>
int Constraint<State>::length=80;
template<typename State>
int Constraint<State>::height=20;
*/

#endif /* defined(__hog2_glut__ConstrainedEnvironment__) */
