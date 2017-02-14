//
//  Airplane.h
//  hog2 glut
//
//  Created by David Chan on 7/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirStates_h
#define AirStates_h

#include <cmath>
#include <memory>
#include <limits>
#include <sstream>
#include "ConstrainedEnvironment.h"
#include "constants.h"

const int8_t k45 = 1;
const int8_t k90 = 2;
const int8_t k135 = 3;
const int8_t k180 = 4;
const uint8_t kCircleSize = 8;

const int8_t kShift = 5; // Should be outside the range [-kCircleSize/2, kCircleSize/2]
const int8_t kWait = 6;

// Define the type of airplane
enum AirplaneType {
  QUAD=0, PLANE=1
};

// Utility function
namespace{
template<unsigned fullDegs>
unsigned hdgDiff(unsigned a, unsigned b){
  unsigned d(abs(a-b)%fullDegs);
  return d>(fullDegs/2.0)?fullDegs-d:d;
}
};

class HolonomicLatticeAction {
public:
	HolonomicLatticeAction(int8_t t=0, int8_t s=0, int8_t h=0)
	:turn(t), speed(s), height(h){}
	int8_t turn;
	int8_t speed;
	int8_t height;
};


class TemporalHolonomicLatticeState{
  TemporalHolonomicLatticeState() :x(0),y(0),height(20),speed(1),heading(0),t(0),landed(false){}
  TemporalHolonomicLatticeState(uint16_t x,uint16_t y, uint16_t height, uint8_t speed, uint8_t heading, float time=0.0, bool landed = false):
    x(x),y(y),height(height),speed(speed),heading(heading),t(time),landed(landed) {}
    virtual bool operator==(TemporalHolonomicLatticeState const& s2)const{
      return (x==s2.x && y==s2.y && height==s2.height && heading == s2.heading && speed == s2.speed);
    }
    virtual void operator+=(HolonomicLatticeAction const& dir){
      static const double offset[8][2] = {
        { 0, -1},
        { 1, -1},
        { 1,  0},
        { 1,  1},
        { 0,  1},
        {-1,  1},
        {-1,  0},
        {-1, -1}
      };
      int8_t hdg(heading);

      if (dir.turn == kShift) {
        hdg = (heading + kCircleSize + k45) % kCircleSize;
      } else if (dir.turn == -kShift) {
        hdg = (heading + kCircleSize - k45) % kCircleSize;
      } else {
        hdg = heading = (heading + kCircleSize + dir.turn) % kCircleSize;
      }

      if (dir.turn != kWait) {
        x += offset[hdg][0];
        y += offset[hdg][1];
      }

      speed += dir.speed;
      height += dir.height;

      static const double SPEEDS[]={0.0,0.5556,0.7778,1.0,1.2222,1.4444};
      t+=(dir.turn==kWait?1.0:(abs(dir.turn)%2?M_SQRT2:1.0)*SPEEDS[speed]);

    }
    virtual void operator-=(HolonomicLatticeAction const& dir){
      static const double offset[8][2] = {
        { 0, -1},
        { 1, -1},
        { 1,  0},
        { 1,  1},
        { 0,  1},
        {-1,  1},
        {-1,  0},
        {-1, -1}
      };

      height -= dir.height;
      speed -= dir.speed;

      int8_t hdg(heading);

      if (dir.turn == kShift) {
        hdg = (heading + kCircleSize + k45 + kCircleSize/2 ) % kCircleSize;
      } else if (dir.turn == -kShift) {
        hdg = (heading + kCircleSize - k45 + kCircleSize/2 ) % kCircleSize;
      } else {
        hdg = (heading + kCircleSize/2) % kCircleSize;
        heading = (heading + kCircleSize - dir.turn) % kCircleSize;
      }

      if (dir.turn != kWait) {
        x += offset[hdg][0];
        y += offset[hdg][1];
      }

      static const double SPEEDS[]={0.0,0.5556,0.7778,1.0,1.2222,1.4444};
      t-=(dir.turn==kWait?1.0:(abs(dir.turn)%2?M_SQRT2:1.0)*SPEEDS[speed]);
    }
    virtual uint64_t hash()const{
      uint64_t h = 0;

        h |= x;
        h = h << 16;
        h |= y;
        h = h << 10;
        h |= height & (0x400-1); // 10 bits
        h = h << 4;
        h |= speed & (0xF); // 5 bits
        h = h << 3;
        // Heading increments are in 45 degrees
        h |= heading & (0x8-1); // 3 bits
        h = h << 1;
        h |= landed;
        h = h << 12;
        h |= unsigned(t*16) & (0x1000-1);
        return h;
    }

  // Methods
  uint8_t headingTo(TemporalHolonomicLatticeState const& other) const {
    return uint8_t(round((atan2(other.y-y,other.x-x)+(M_PI/2.0))*4.0/M_PI)+8.0)%8;
  }
  // Returns a heading of 0, 2, 4 or 6
  uint8_t cardinalHeadingTo(TemporalHolonomicLatticeState const& other) const {
    return uint8_t(round((atan2(other.y-y,other.x-x)+(M_PI/2.0))*2.0/M_PI)+4.0)%4*2;
  }

private:
  // Fields
  uint16_t x;
  uint16_t y;
  uint16_t height;
  uint8_t speed;
  uint8_t heading;
  float t; // time
  bool landed;

};

struct airplaneAction {
public:
	airplaneAction(int8_t t=0, int8_t s=0, int8_t h=0, int8_t takeoff = 0)
	:turn(t), speed(s), height(h), takeoff(takeoff) {}
	int8_t turn;
	int8_t speed;
	int8_t height;
	int8_t takeoff; // 0 for no-land, 1 for takeoff, 2 for landing, 3 for landed-noop
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
};


struct landingStrip {
	landingStrip(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2, airplaneState &launch_state, airplaneState &landing_state, airplaneState &goal_state) : x1(x1), x2(x2), y1(y1), y2(y2), 
				 launch_state(launch_state), landing_state(landing_state), goal_state(goal_state) {}
	uint16_t x1;
	uint16_t x2;
	uint16_t y1;
	uint16_t y2;
	uint16_t z = 0;
	airplaneState goal_state;
	airplaneState launch_state;
	airplaneState landing_state;
};


/** Output the information in an airplane state */
static std::ostream& operator <<(std::ostream & out, const airplaneState &loc)
{
	out << "(x:" << loc.x << ", y:" << loc.y << ", h:" << loc.height << ", s:" << unsigned(loc.speed) <<
											    ", hdg: " << unsigned(loc.heading) << ", l: " << unsigned (loc.landed) << ", type: " << (loc.type == AirplaneType::QUAD ? "QUAD" : "PLANE") << ")";
	return out;
}

bool operator==(const airplaneState &s1, const airplaneState &s2);
bool operator==(const airplaneAction &a1, const airplaneAction &a2);


/**
 * The airtimeState struct holds information about airplane state at
 * a given time, so we can check constraints
 */
struct airtimeState : public airplaneState {
	airtimeState(airplaneState loc, float time, uint8_t c=0) :airplaneState(loc), t(time), nc(c) {}
	airtimeState(uint16_t x,uint16_t y, uint16_t height, uint8_t speed, uint8_t heading, bool landed = false, AirplaneType t = AirplaneType::PLANE) : airplaneState(x,y,height,speed,heading), t(0),nc(0){}

	airtimeState() :airplaneState(), t(0), nc(0) {}
	float t;
        uint8_t nc; // Number of conflicts
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
 * The Constraint<airtimeState> struct holds information about a single action
 * which allows us to check for collisions in the environment. We know that
 * each one describes a box between two states.
 */
template<>
struct Constraint<airtimeState>
{
	Constraint<airtimeState>() {}
	Constraint<airtimeState>(airtimeState s1) : start_state(s1), end_state(s1) {}
	Constraint<airtimeState>(airtimeState s1, airtimeState s2) : start_state(s1), end_state(s2) {}
	Constraint<airtimeState>(Constraint<airtimeState> const& c) : Constraint<airtimeState>(c.start_state, c.end_state) {}
        airtimeState start() const {return start_state;}
        airtimeState end() const {return end_state;}


	bool strip = false;
	virtual bool ConflictsWith(const airtimeState &state) const;
	virtual bool ConflictsWith(const airtimeState &from, const airtimeState &to) const;
	virtual bool ConflictsWith(const Constraint<airtimeState> &x) const;
	virtual void OpenGLDraw() const;

        airtimeState start_state;
        airtimeState end_state;
};

template<typename state>
static std::ostream& operator <<(std::ostream & out, const Constraint<state> &loc)
{
	out << "[start: " << loc.start() << ", end: " << loc.end() << "]";
	return out;
}

#endif
