//
//  Airplane.h
//  hog2 glut
//
//  Created by David Chan on 7/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirStates_h
#define AirStates_h

#include "ConstrainedEnvironment.h"
#include <functional>

struct RAirspaceTicket;

// Define the type of airplane
enum AirplaneType {
  QUAD=0, PLANE=1
};


const int8_t k45 = 1;
const int8_t k90 = 2;
const int8_t k135 = 3;
const int8_t k180 = 4;
const uint8_t kCircleSize = 8;

const int8_t kShift = 5; // Should be outside the range [-kCircleSize/2, kCircleSize/2]
const int8_t kWait = 6;

struct airplaneAction {
public:
	airplaneAction(int8_t t=0, int8_t s=0, int8_t h=0, int8_t takeoff = 0)
	:turn(t), speed(s), height(h), takeoff(takeoff) {}
	int8_t turn;
	int8_t speed;
	int8_t height;
	int8_t takeoff; // 0 for no-land, 1 for takeoff, 2 for landing, 3 for landed-noop



	bool operator==(const airplaneAction &a2)const
	{
	return turn == a2.turn && speed==a2.speed && height==a2.height && takeoff == a2.takeoff;
	}



};

	/** Output the information in an airplane action */
	static std::ostream& operator <<(std::ostream & out, const airplaneAction &act)
	{
		out << "(turn:" << signed(act.turn) << ", speed:" << signed(act.speed) << ", height: " << signed(act.height) << ", takeoff: " << signed(act.takeoff) << ")";
		return out;
	}






// state
struct airplaneState {
  // Constructors
	airplaneState() :x(0),y(0),height(20),speed(1),heading(0),landed(false) ,type(AirplaneType::PLANE) {}
	airplaneState(uint16_t x,uint16_t y, uint16_t height, uint8_t speed, uint8_t heading, bool landed = false, AirplaneType t = AirplaneType::PLANE) :
        x(x),y(y),height(height),speed(speed),heading(heading), landed(landed), type(t) {}
  
  // Fields
	uint16_t x;
	uint16_t y;
	uint16_t height;
	uint8_t speed;
	uint8_t heading;
	bool landed;
  AirplaneType type;

  // Methods
  uint8_t headingTo(airplaneState const& other) const {
    return uint8_t(round((atan2(other.y-y,other.x-x)+(M_PI/2.0))*4.0/M_PI)+8.0)%8;
  }
  // Returns a heading of 0, 2, 4 or 6
  uint8_t cardinalHeadingTo(airplaneState const& other) const {
    return uint8_t(round((atan2(other.y-y,other.x-x)+(M_PI/2.0))*2.0/M_PI)+4.0)%4*2;
  }

  bool operator==(const airplaneState &s2)const
  {
    return (x==s2.x && y==s2.y && height==s2.height && heading == s2.heading && speed == s2.speed && landed == s2.landed);
  }


};


	/** Output the information in an airplane state */
	static std::ostream& operator <<(std::ostream & out, const airplaneState &loc)
	{
		out << "(x:" << loc.x << ", y:" << loc.y << ", h:" << loc.height << ", s:" << unsigned(loc.speed) <<
													", hdg: " << unsigned(loc.heading) << ", l: " << unsigned (loc.landed) << ", type: " << (loc.type == AirplaneType::QUAD ? "QUAD" : "PLANE") << ")";
		return out;
	}



/**
 * The airtimeState struct holds information about airplane state at
 * a given time, so we can check constraints
 */
struct airtimeState : public airplaneState {
	airtimeState(airplaneState loc, float time) :airplaneState(loc), t(time) {}
	airtimeState() :airplaneState(), t(0) {}
	float t;
	std::vector<RAirspaceTicket*> current_tickets;



	/** Operator for equivalent airtime states */
	/*bool operator==(const airtimeState &l2)
	{
		return fequal(t,l2.t) && ((airplaneState const&)(*this)== ((airplaneState const&)l2);
	}*/



};

	/** Output the information in an airtime state */
static std::ostream& operator <<(std::ostream & out, const airtimeState &loc)
{
	out << "(x:" << loc.x << ", y:" << loc.y << ", h:" << loc.height << ", s:" << unsigned(loc.speed) <<
											    ", hdg:" << unsigned(loc.heading) << ", t:" << (loc.t) << ", l: " << unsigned (loc.landed) << ", type: " << (loc.type == AirplaneType::QUAD ? "QUAD": "PLANE") <<  ")";
	return out;
}






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



// Temp hashing function for airplane states
namespace std {
	template <> struct hash<airplaneState>
	{
		size_t operator()(const airplaneState & s) const 
		{
			return hash<int>()(s.x*s.x + s.y*s.y + s.height*s.height + s.heading + s.speed + s.landed);
		}
	};
}

#endif
