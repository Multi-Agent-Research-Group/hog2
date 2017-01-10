//
//  Airplane.h
//  hog2 glut
//
//  Created by David Chan on 7/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirStates_h
#define AirStates_h

#include "Airplane.h"
#include "ConstrainedEnvironment.h"

struct RAirspaceTicket;

/**
 * The airtimeState struct holds information about airplane state at
 * a given time, so we can check constraints
 */
struct airtimeState : public airplaneState {
	airtimeState(airplaneState loc, float time, uint8_t c=0) :airplaneState(loc), t(time), nc(c) {}
	airtimeState() :airplaneState(), t(0), nc(0) {}
	float t;
        uint8_t nc; // Number of conflicts
	std::vector<RAirspaceTicket*> current_tickets;
};

/** Output the information in an airtime state */
static std::ostream& operator <<(std::ostream & out, const airtimeState &loc)
{
	out << "(x:" << loc.x << ", y:" << loc.y << ", h:" << loc.height << ", s:" << unsigned(loc.speed) <<
											    ", hdg:" << unsigned(loc.heading) << ", t:" << (loc.t) << ", l: " << unsigned (loc.landed) << ", nc: " << unsigned(loc.nc) << ")";
	return out;
}

/** Check if two airtimeStates are equal */
bool operator==(const airtimeState &l1, const airtimeState &l2);



/**
 * The airConstraint struct holds information about a single action
 * which allows us to check for collisions in the environment. We know that
 * each one describes a box between two states.
 */
struct airConstraint : public Constraint<airtimeState>
{
	airConstraint() {}
	airConstraint(airtimeState s1)  : Constraint<airtimeState>(s1) {} //: start_state(s1), end_state(s1) {}
	airConstraint(airtimeState s1, airtimeState s2) : Constraint<airtimeState>(s1, s2) {} //start_state(s1), end_state(s2) {}
	airConstraint(Constraint<airtimeState> c) : Constraint<airtimeState>(c.start_state, c.end_state) {}

	//airtimeState start_state;
	//airtimeState end_state;
	bool strip = false;
	virtual bool ConflictsWith(const airtimeState &state) const;
	virtual bool ConflictsWith(const airtimeState &from, const airtimeState &to) const;
	virtual bool ConflictsWith(const airConstraint &x) const;
	virtual void OpenGLDraw() const;
};

static std::ostream& operator <<(std::ostream & out, const airConstraint &loc)
{
	out << "[start: " << loc.start_state << ", end: " << loc.end_state << "]";
	return out;
}

#endif
