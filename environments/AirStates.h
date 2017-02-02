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
#include "constants.h"

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

struct PlatformAction {
public:
	PlatformAction(double t=0, double p=0, int8_t s=0)
          :turnHalfDegs(uint8_t(t*2.0)),
           pitchHalfDegs(uint8_t(p*2.0)),
           speed(s) {}

	int8_t turnHalfDegs; // turn in half-degrees
	int8_t pitchHalfDegs; // pitch in half-degrees
	int8_t speed; // speed increment (+/-1)

	double turn() const { return double(turnHalfDegs)/2.0; }
	double pitch() const { return double(pitchHalfDegs)/2.0; }
};


/** Output the information in an Platform action */
static std::ostream& operator <<(std::ostream & out, PlatformAction const& act)
{
	out << "(turn:" << act.turn() << " pitch:" << act.pitch() << " speed: " << act.speed << ")";
	return out;
}


// state
struct PlatformState {
        static const double SPEEDS[];
        static const double TIMESTEP;
	// Constructors
	PlatformState() :x(0),y(0),z(0),t(0),headingHalfDegs(0),rollHalfDegs(180),pitchHalfDegs(180),speed(3){}
	PlatformState(float lt,float ln, float a, double h, double p, uint8_t s) :
		x(lt),y(ln),z(a),headingHalfDegs(h*2),rollHalfDegs(180),pitchHalfDegs((p+90)*2.0),speed(s){}

	double hdg()const{return double(headingHalfDegs)/2.0;}
	double roll()const{return double(rollHalfDegs)/2.0-90.0;}
	double pitch()const{return double(pitchHalfDegs)/2.0-90.0;}

        // Set the roll based on current turn rate.
        void setRoll(double turnDegsPerSecond){rollHalfDegs=(atan((turnDegsPerSecond*constants::degToRad*SPEEDS[speed])/constants::gravitationalConstant)*constants::radToDeg + 90.0) * 2.0;}

	uint64_t key() const {
		// We have too many bits to fit in the key (32+32+32+16+8+8)=128
		uint32_t l1(*((uint32_t*)&x));
		uint32_t l2(*((uint32_t*)&y));
		uint32_t l3(*((uint32_t*)&z));
		uint32_t l4(*((uint32_t*)&t));

                uint64_t h1(l1);
		h1 = h1 << 32;
		h1 |= l2;

                uint64_t h2(l3);
                h2 << 32;
                h2 |= l4;

                uint64_t h3(headingHalfDegs);
		h3 << 9;
		h3 |= rollHalfDegs & (0x1ff); // we only care about the range 0-360 (we normalize to +/- 90)
		h3 << 9;
		h3 |= pitchHalfDegs & (0x1ff); // we only care about the range 0-360 (we normalize to +/- 90)
                h3 << 3;
                h3 |= speed & (0x7);

		// Put the 64 bit hashes together using a prime product hash
		return (h1 * 16777619) ^ h2 ^ (h3 * 4194319);
	}

        void operator += (PlatformAction const& other){
          headingHalfDegs+=other.turnHalfDegs;
          pitchHalfDegs+=other.pitchHalfDegs;
          setRoll(other.turn());
          speed += other.speed;
          x+=cos((90.-hdg())*constants::degToRad)*SPEEDS[speed]*TIMESTEP;
          y+=sin((90.-hdg())*constants::degToRad)*SPEEDS[speed]*TIMESTEP;
          z+=sin((90.-pitch())*constants::degToRad)*SPEEDS[speed]*TIMESTEP;
        }

	bool operator == (PlatformState const& other)const{return key()==other.key();}

	// Fields
	float x;
	float y;
	float z;
	float t;
	uint16_t headingHalfDegs;
	int16_t rollHalfDegs;
	int16_t pitchHalfDegs;
	int8_t speed;  // 5 speeds: 1=100mps, 2=140, 3=180, 4=220, 5=260 mps
};


/** Output the information in a Platform state */
static std::ostream& operator <<(std::ostream & out, PlatformState const& loc)
{
	out << "(x:" << loc.x << ", y:" << loc.y << ", z:" << loc.z << ", h: " << loc.hdg() << ", r: " << signed(loc.roll()) << ", s: " << loc.speed << ", t: " << loc.t << ")";
	//out << "val<-cbind(val,c(" << loc.x << "," << loc.y << "," << loc.alt() << "," << loc.heading() << "," << signed(loc.roll) << "," << loc.sum << "," << loc.depth << "))";
	return out;
}

bool operator==(PlatformAction const& a1, PlatformAction const& a2);

#endif
