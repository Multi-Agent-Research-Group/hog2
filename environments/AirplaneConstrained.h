//
//  AirplaneConstrained.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__AirplaneConstrainedEnvironment__
#define __hog2_glut__AirplaneConstrainedEnvironment__

#include <iostream>

#include "Airplane.h"

/**
 * The airtimeState struct holds information about airplane state at
 * a given time, so we can check constraints
 */
struct airtimeState {
	airtimeState(airplaneState loc, int time) :l(loc), t(time) {}
	airtimeState() { l = airplaneState(); t = 0; }
	airplaneState l;
	uint16_t t;
};


/** Output the information in an airtime state */
static std::ostream& operator <<(std::ostream & out, const airtimeState &loc)
{
	out << "(" << unsigned(loc.l.x) << ", " << unsigned(loc.l.y) << ", " << unsigned(loc.l.height) << ": " << unsigned(loc.l.speed) <<
											    ": " << unsigned(loc.l.heading) << ": " << unsigned(loc.t) << ")";
	return out;
}

/** Check if two airtimeStates are equal */
bool operator==(const airtimeState &l1, const airtimeState &l2);


/**
 * The airConstraint struct holds information about a state and direction
 * which allows us to check for collisions in the environment.
 */
struct airConstraint {
	airtimeState loc;
	airplaneAction dir;
};

	

/**
 * Defines a constrained version of the airplane environment. Thus, we can add constraints 
 * consisting of airtimeStates and actions, which allow us to deal with the issues that
 * come up in CBS
 */
class AirplaneConstrainedEnvironment : public SearchEnvironment<airtimeState, airplaneAction>
{
public:

	/// CONSTRUCTORS

	/** Construct a random-ground airplane environment which forms the backbone of the model */
	AirplaneConstrainedEnvironment();
	/** Construct a constrained environment from an existing environment */
	AirplaneConstrainedEnvironment(AirplaneEnvironment* ae);

	/// CONSTRAINTS
	
	/** Add a constraint to the model */
	void AddConstraint(airConstraint c);
	/** Add a constraint consisting of only a location */
	void AddConstraint(airtimeState loc);
	/** Add a constraint consisting of a location an direction */
	void AddConstraint(airtimeState loc, airplaneAction dir);
	/** Clear the constraints */
	void ClearConstraints();


	/// STATE MANAGEMENT

	/** Get the possible actions from a state */
	virtual void GetActions(const airtimeState &nodeID, std::vector<airplaneAction> &actions) const;
	/** Get the successor states not violating constraints */
	virtual void GetSuccessors(const airtimeState &nodeID, std::vector<airtimeState> &neighbors) const;
	/** Apply an action to the base environment */
	virtual void ApplyAction(airtimeState &s, airplaneAction a) const;
	/** Undo an action on the base environment */
	virtual void UndoAction(airtimeState &s, airplaneAction a) const;
	/** Get the action required to move between two states */
	virtual airplaneAction GetAction(const airtimeState &node1, const airtimeState &node2) const;
	/** Given one state, get the next state */
	virtual void GetNextState(const airtimeState &currents, airplaneAction dir, airtimeState &news) const;
	/** Invert an action */
	virtual bool InvertAction(airplaneAction &a) const { return false; } // Actions are not invertible
	
	/// OCCUPANCY

	/** Deal with the occupancy */
	virtual OccupancyInterface<airtimeState,airplaneAction> *GetOccupancyInfo() { return 0; } //TODO: Not implemented
	
	/// HEURISTICS

	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const airtimeState &node1, const airtimeState &node2) const;
	virtual double HCost(const airtimeState &)  const { assert(false); return 0; } //No single state H-Cost implemented
	virtual double GCost(const airtimeState &node1, const airtimeState &node2) const { return 1; }
	virtual double GCost(const airtimeState &node, const airplaneAction &act) const { return 1; }
	virtual double GetPathLength(const std::vector<airtimeState> &n) const;

	/// GOAL TESTING

	virtual bool GoalTest(const airtimeState &) const { assert(false); return false; } // No support for single state goal testing
	virtual bool GoalTest(const airtimeState &node, const airtimeState &goal) const;

	/// HASHING
	
	/** Methods for dealing with state hashing */
	virtual uint64_t GetStateHash(const airtimeState &node) const;
	virtual uint64_t GetActionHash(airplaneAction act) const;

	/// DRAWING

	/** GL Drawing */
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const airtimeState&) const;
	virtual void OpenGLDraw(const airtimeState&, const airplaneAction&) const;
	virtual void GLDrawLine(const airtimeState &x, const airtimeState &y) const;
	virtual void GLDrawPath(const std::vector<airtimeState> &p) const;
	
	/// UTILS
private:
	
	/** Checks to see if any constraint is violated */
	bool ViolatesConstraint(const airplaneState &from, const airplaneState &to, int time) const;

	/** Vector holding the current constraints */
	std::vector<airConstraint> constraints;

	/** Airplane Environment holder */
	AirplaneEnvironment *ae;
};

#endif /* defined(__hog2_glut__AirplaneConstrainedEnvironment__) */
