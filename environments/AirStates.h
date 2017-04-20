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
#include <iostream>

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
	airtimeState(airplaneState loc, float time, int8_t c=-1) :airplaneState(loc), t(time), nc(c) {}
	airtimeState(uint16_t x,uint16_t y, uint16_t height, uint8_t speed, uint8_t heading, bool landed = false, float time=0, int c=-1) : airplaneState(x,y,height,speed,heading,landed), t(time),nc(c){}

	airtimeState() :airplaneState(), t(0), nc(-1) {}
	float t;
        int16_t nc; // Number of conflicts
};

/** Output the information in an airtime state */
static std::ostream& operator <<(std::ostream & out, const airtimeState &loc)
{
	out << "(x:" << loc.x << ", y:" << loc.y << ", h:" << loc.height << ", s:" << unsigned(loc.speed) <<
											    ", hdg:" << unsigned(loc.heading) << ", t:" << (loc.t) << ", l: " << unsigned (loc.landed) << ", nc: " << signed(loc.nc) << ")";
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

template<>
class SoftConstraint<airtimeState> : public DrawableConstraint {
  public:
    SoftConstraint() {}
    SoftConstraint(airtimeState const& c, double r) : center(c),radius(r),logr(log(r)){}

    virtual double cost(airtimeState const& other, double scale) const{
      double d(Util::distance(center.x,center.y,center.height,other.x,other.y,other.height)/scale);
      return std::max(0.0,logr/d-logr/radius);
    }
    virtual void OpenGLDraw() const;

    airtimeState center;
    double radius;
    double logr;
};


template<typename state>
static std::ostream& operator <<(std::ostream & out, const Constraint<state> &loc)
{
	out << "[start: " << loc.start() << ", end: " << loc.end() << "]";
	return out;
}

struct PlatformAction {
public:
        PlatformAction(double t=0, double p=0, int8_t s=0)
          :turnHalfDegs(int8_t(t*2.0)),
           pitchHalfDegs(int8_t(p*2.0)),
           speed(s) {}

        int8_t turnHalfDegs; // turn in half-degrees
        int8_t pitchHalfDegs; // pitch in half-degrees
        int8_t speed; // speed increment (+/-1)

        double turn() const { return double(turnHalfDegs)/2.0; }
        void turn(double val){ turnHalfDegs=int8_t(round(val*2.0)); }
        double pitch() const { return double(pitchHalfDegs)/2.0; }
        void pitch(double val){ pitchHalfDegs=int8_t(round(val*2.0)); }
};


/** Output the information in an Platform action */
static std::ostream& operator <<(std::ostream & out, PlatformAction const& act)
{
        out << "(turn:" << act.turn() << " pitch:" << act.pitch() << " speed: " << signed(act.speed) << ")";
        return out;
}


// state
struct PlatformState {
        static const double SPEEDS[];
        static const double SPEED_COST[];
        static const double TIMESTEP;
        static const double angularPrecision;
        // Constructors
        PlatformState() :x(0),y(0),z(0),t(0),headingHalfDegs(360),rollHalfDegs(180),pitchHalfDegs(180),speed(3),landed(false),nc(0){}
        PlatformState(float lt,float ln, float a, double h, double p, int8_t s, uint32_t time=0) :
                x(lt),y(ln),z(a),t(time),headingHalfDegs(signed(360.+h*2.0)%720),rollHalfDegs(180),pitchHalfDegs(signed(180+p*2.0)%360),speed(s),landed(false),nc(0){}

        double hdg()const{return fmod(double(headingHalfDegs)/2.0+180.,360.);}
        double roll()const{return double(rollHalfDegs)/2.0-90.0;}
        double pitch()const{return double(pitchHalfDegs)/2.0-90.0;}

        // Set the roll based on current turn rate.
        void setRoll(double turnDegsPerSecond){rollHalfDegs=(atan((turnDegsPerSecond*constants::degToRad*SPEEDS[speed])/constants::gravitationalConstant)*constants::radToDeg + 90.0) * 2.0;}

        uint64_t key() const {
                // We have too many bits to fit in the key (32+32+32+16+8+8)=128
                uint32_t l1(*((uint32_t*)&x));
                uint32_t l2(*((uint32_t*)&y));
                uint32_t l3(*((uint32_t*)&z));
                //uint32_t l4(*((uint32_t*)&t));

                uint64_t h1(l1);
                h1 = h1 << 32;
                h1 |= l2;

                uint64_t h2(l3);
                h2 = h2 << 32;
                //h2 |= l4;
                h2 |= t;

                uint64_t h3(headingHalfDegs);
                h3 = h3 << 9;
                h3 |= rollHalfDegs & (0x1ff); // we only care about the range 0-360 (we normalize to +/- 90)
                h3 = h3 << 9;
                h3 |= pitchHalfDegs & (0x1ff); // we only care about the range 0-360 (we normalize to +/- 90)
                h3 = h3 << 3;
                h3 |= speed & (0x7);

                // Put the 64 bit hashes together using a prime product hash
                return (h1 * 16777619) ^ h2 ^ (h3 * 4194319);
        }

        void operator += (PlatformAction const& other){
          //std::cout  << "Apply " << other << "\n";
          //std::cout << headingHalfDegs << ":1\n";
          headingHalfDegs+=other.turnHalfDegs;
          //std::cout << headingHalfDegs << ":1\n";
          headingHalfDegs=(headingHalfDegs+720)%720;
          //std::cout << headingHalfDegs << ":1\n";
          pitchHalfDegs+=other.pitchHalfDegs;
          pitchHalfDegs=(pitchHalfDegs+360)%360;
          setRoll(other.turn());
          speed += other.speed;
          //std::cout << "yaw " << hdg() << " " << headingHalfDegs << "\n";
          //std::cout << "x. " << cos((90.-hdg())*constants::degToRad) << "\n";
          //std::cout << "y. " << sin((90.-hdg())*constants::degToRad) << "\n";
          //std::cout << "y+ " << sin((90.-hdg())*constants::degToRad)*SPEEDS[speed]*TIMESTEP << "\n";
          //std::cout << "z. " << cos((90.-pitch())*constants::degToRad) << "\n";
          //std::cout << " s " << SPEEDS[speed] << " " << SPEEDS[speed]*TIMESTEP << "\n";
          double hdist(cos(pitch()*constants::degToRad)*SPEEDS[speed]*TIMESTEP);
          x+=cos((90.-hdg())*constants::degToRad)*hdist;
          y+=sin((90.-hdg())*constants::degToRad)*hdist;
          z+=sin(pitch()*constants::degToRad)*SPEEDS[speed]*TIMESTEP;
          t++;
        }

        void operator -= (PlatformAction const& other){
          //std::cout << "_yaw " << hdg() << " " << headingHalfDegs << "\n";
          //std::cout << "_x. " << cos((90.-hdg())*constants::degToRad) << "\n";
          //std::cout << "_y. " << sin((90.-hdg())*constants::degToRad) << "\n";
          //std::cout << "_y+ " << sin((90.-hdg())*constants::degToRad)*SPEEDS[speed]*TIMESTEP << "\n";
          //std::cout << "_z. " << cos((90.-pitch())*constants::degToRad) << "\n";
          //std::cout << " _s " << SPEEDS[speed] << " " << SPEEDS[speed]*TIMESTEP << "\n";
          double hdist(cos(pitch()*constants::degToRad)*SPEEDS[speed]*TIMESTEP);
          x-=cos((90.-hdg())*constants::degToRad)*hdist;
          y-=sin((90.-hdg())*constants::degToRad)*hdist;
          z-=sin(pitch()*constants::degToRad)*SPEEDS[speed]*TIMESTEP;
          t--;
          headingHalfDegs-=other.turnHalfDegs;
          headingHalfDegs=(headingHalfDegs+720)%720;
          pitchHalfDegs-=other.pitchHalfDegs;
          pitchHalfDegs=(pitchHalfDegs+360)%360;
          setRoll(other.turn());
          speed -= other.speed;
        }

        bool operator == (PlatformState const& other)const{
          return fequal(x,other.x) && fequal(y,other.y) && fequal(z,other.z)
            && (speed==other.speed) && (headingHalfDegs==other.headingHalfDegs)
            && (pitchHalfDegs==other.pitchHalfDegs);
        }

        double distanceTo(PlatformState const& other) const {
          return sqrt((other.y-y)*(other.y-y)+(other.x-x)*(other.x-x)+(other.z-z)*(other.z-z));
        }

        double headingTo(PlatformState const& other) const {
          return double(int16_t(round((M_PI/2.0-atan2(other.y-y,other.x-x))*360.0/M_PI)+720.0)%720)/2.0;
        }

        double elevationTo(PlatformState const& other) const {
          return double(int16_t(round((atan2(other.z-z,sqrt((other.y-y)*(other.y-y)+(other.x-x)*(other.x-x))))*360.0/M_PI)))/2.0;
        }

        // Fields
        float x;
        float y;
        float z;
        uint32_t t;
        int16_t headingHalfDegs;
        int16_t rollHalfDegs;
        int16_t pitchHalfDegs;
        int8_t speed;  // 5 speeds: 1=100mps, 2=140, 3=180, 4=220, 5=260 mps
        bool landed;
        int8_t nc;
};

/** Output the information in a Platform state */
static std::ostream& operator <<(std::ostream & out, PlatformState const& loc)
{
        out << "(x:" << loc.x << ", y:" << loc.y << ", z:" << loc.z << ", h: " << loc.hdg() << ", p: " << loc.pitch() << ", r: " << signed(loc.roll()) << ", s: " << unsigned(loc.speed) << ", t: " << loc.t << ")";
        //out << "val<-cbind(val,c(" << loc.x << "," << loc.y << "," << loc.alt() << "," << loc.heading() << "," << signed(loc.roll) << "," << loc.sum << "," << loc.depth << "))";
        return out;
}


template<>
struct Constraint<PlatformState>{
  Constraint<PlatformState>() {}
  Constraint<PlatformState>(PlatformState s1) : start_state(s1), end_state(s1) {}
  Constraint<PlatformState>(PlatformState s1, PlatformState s2) : start_state(s1), end_state(s2) {}
  Constraint<PlatformState>(Constraint<PlatformState> const& c) : Constraint<PlatformState>(c.start_state, c.end_state) {}
  PlatformState start() const {return start_state;}
  PlatformState end() const {return end_state;}


  bool strip = false;
  virtual bool ConflictsWith(const PlatformState &state) const;
  virtual bool ConflictsWith(const PlatformState &from, const PlatformState &to) const;
  virtual bool ConflictsWith(const Constraint<PlatformState> &x) const;
  virtual void OpenGLDraw() const;

  PlatformState start_state;
  PlatformState end_state;
};

#endif
