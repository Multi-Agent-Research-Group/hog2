//
//  AirplaneConstrained.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__AirplaneConstrainedEnvironment__
#define __hog2_glut__AirplaneConstrainedEnvironment__

#include "Airplane.h"
#include <cmath>
#include <memory>
#include <limits>

/**
 * The airtimeState struct holds information about airplane state at
 * a given time, so we can check constraints
 */
struct airtimeState {
	airtimeState(airplaneState loc, float time) :l(loc), t(time) {}
	airtimeState() :l(airplaneState()), t(0) {}
	airplaneState l;
	float t;
};


/** Output the information in an airtime state */
static std::ostream& operator <<(std::ostream & out, const airtimeState &loc)
{
	out << "(x:" << loc.l.x << ", y:" << loc.l.y << ", h:" << loc.l.height << ", s:" << unsigned(loc.l.speed) <<
											    ", hdg:" << unsigned(loc.l.heading) << ", t:" << (loc.t) << ")";
	return out;
}

/** Check if two airtimeStates are equal */
bool operator==(const airtimeState &l1, const airtimeState &l2);



/**
 * The airConstraint struct holds information about a single action
 * which allows us to check for collisions in the environment. We know that
 * each one describes a box between two states.
 */
struct airConstraint 
{
	airConstraint() {}
	airConstraint(airtimeState s1) : start_state(s1), end_state(s1) {}
	airConstraint(airtimeState s1, airtimeState s2) : start_state(s1), end_state(s2) {}

	airtimeState start_state;
	airtimeState end_state;
	bool strip = false;

	bool ConflictsWith(const airtimeState &state) const;
	bool ConflictsWith(const airtimeState &from, const airtimeState &to) const;
	bool ConflictsWith(const airConstraint &x) const;
	void OpenGLDraw() const;
};

static std::ostream& operator <<(std::ostream & out, const airConstraint &loc)
{
	out << "[start: " << loc.start_state << ", end: " << loc.end_state << "]";
	return out;
}

	

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
	//AirplaneConstrainedEnvironment();
	/** Construct a constrained environment from an existing environment */
	AirplaneConstrainedEnvironment(AirplaneEnvironment* ae);

	/// CONSTRAINTS
	
	/** Add a constraint to the model */
	void AddConstraint(airConstraint c);
	void AddPointConstraint(const airtimeState &loc);
	void AddBoxConstraint(const airtimeState &loc1, const airtimeState &loc2);
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
	virtual double GCost(const airtimeState &node1, const airtimeState &node2) const { return ae->GCost(node1.l,node2.l); }
	virtual double GCost(const airtimeState &node, const airplaneAction &act) const { return ae->GCost(node.l,act); }
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
	virtual void OpenGLDraw(const airtimeState& oldState, const airtimeState &newState, float perc) const;
	virtual void GLDrawPath(const std::vector<airtimeState> &p) const;

	// Override the color method.
	virtual void SetColor(double r, double g, double b, double t = 1.0) const {SearchEnvironment::SetColor(r,g,b,t); this->ae->SetColor(r,g,b,t);}
	virtual void SetColor(double& r, double& g, double& b, double& t) const {SearchEnvironment::SetColor(r,g,b,t); this->ae->SetColor(r,g,b,t);}
	
	/// UTILS
	uint8_t GetSpeeds(){return ae->numSpeeds;}
private:
	
	/** Checks to see if any constraint is violated */
	bool ViolatesConstraint(const airplaneState &from, const airplaneState &to, int time) const;
	bool ViolatesConstraint(const airtimeState &from, const airtimeState &to) const;

	/** Vector holding the current constraints */
	std::vector<airConstraint> constraints;

	/** Airplane Environment holder */
	AirplaneEnvironment *ae;
};

#endif /* defined(__hog2_glut__AirplaneConstrainedEnvironment__) */
