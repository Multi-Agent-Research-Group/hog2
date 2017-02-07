//
//  AirplaneConstrained.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__ConstrainedEnvironment__
#define __hog2_glut__ConstrainedEnvironment__

#include <cmath>
#include <memory>
#include <limits>


template<class State>
struct Constraint {
	Constraint() {}
	Constraint(State start) : start_state(start), end_state(start) {}
	Constraint(State start, State end) : start_state(start), end_state(end) {}

	State start_state;
	State end_state;

	virtual bool ConflictsWith(const State &s) const {return start_state == s || end_state == s;}
	virtual bool ConflictsWith(const State &from, const State &to) const {return ConflictsWith(from) || ConflictsWith(to);}
	virtual bool ConflictsWith(const Constraint &x) const {return ConflictsWith(x.start_state, x.end_state);}
	virtual void OpenGLDraw() const {}
};


template<class State, class Action, class Environment>
class ConstrainedEnvironment : public Environment
{
public:
	/** Add a constraint to the model */
	virtual void AddConstraint(Constraint<State> c) = 0;
	/** Clear the constraints */
	virtual void ClearConstraints() = 0;
	/** Get the possible actions from a state */
	virtual void GetActions(const State &nodeID, std::vector<Action> &actions) const = 0;
	/** Get the successor states not violating constraints */
	virtual void GetSuccessors(const State &nodeID, std::vector<State> &neighbors) const = 0;
	/** Checks to see if any constraint is violated */
	virtual bool ViolatesConstraint(const State &from, const State &to) const = 0;
};

#endif /* defined(__hog2_glut__ConstrainedEnvironment__) */
