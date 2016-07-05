//
//  Airplane.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef Airplane_h
#define Airplane_h

#include <vector>
#include <cassert>
#include <cmath>
#include "SearchEnvironment.h"

// turn type:
// 45, 90, 0, shift
// speed:
// faster, slower
// height:
// up / down

const uint8_t k45 = 1;
const uint8_t k90 = 2;
const uint8_t kShift = 3;

// Utility function
namespace{
template<unsigned fullDegs>
unsigned hdgDiff(unsigned a, unsigned b){
  unsigned d(abs(a-b)%fullDegs);
  return d>(fullDegs/2.0)?fullDegs-d:d;
}
};


struct airplaneAction {
public:
	airplaneAction(int8_t t=0, int8_t s=0, int8_t h=0, int8_t takeoff = 0)
	:turn(t), speed(s), height(h), takeoff(takeoff) {}
	int8_t turn;
	int8_t speed;
	int8_t height;
	int8_t takeoff; // 0 for no-land, 1 for takeoff, 2 for landing
};

/** Output the information in an airplane action */
static std::ostream& operator <<(std::ostream & out, const airplaneAction &act)
{
	out << "(turn:" << signed(act.turn) << ", speed:" << signed(act.speed) << ", height: " << signed(act.height) << ", takeoff: " << signed(act.takeoff) << ")";
	return out;
}

// state
struct airplaneState {
public:
	airplaneState() :x(0),y(0),height(20),speed(1),heading(0),landed(false) {}
	airplaneState(uint16_t x,uint16_t y, uint16_t height, uint8_t speed, uint8_t heading, bool landed = false) :x(x),y(y),height(height),speed(speed),heading(heading), landed(landed) {}
        uint8_t headingTo(airplaneState const& other) const {
          return uint8_t(round((atan2(other.y-y,other.x-x)+(M_PI/2.0))*4.0/M_PI)+8.0)%8;
        }
	uint16_t x;
	uint16_t y;
	uint16_t height;
	uint8_t speed;
	uint8_t heading;
	bool landed;
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
											    ", hdg: " << unsigned(loc.heading) << ", l: " << unsigned (loc.landed) << ")";
	return out;
}

bool operator==(const airplaneState &s1, const airplaneState &s2);
bool operator==(const airplaneAction &a1, const airplaneAction &a2);

template <class state>
class StraightLineHeuristic : public Heuristic<state> {
  public:
  double HCost(const state &a,const state &b) const {
        static const double cruiseBurnRate(.0006);
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.height-b.height)*(a.height-b.height))*cruiseBurnRate;
  }
};

template <class state>
class OctileDistanceHeuristic : public Heuristic<state> {
  public:
  double HCost(const state &node1,const state &node2) const {
          static const uint8_t numSpeeds(5); // Number of discrete speeds
          static const double cruiseBurnRate(.0006); // Fuel burn rate in liters per unit distance
          static const double speedBurnDelta(0.0001); // Extra fuel cost for non-cruise speed
          static const double climbCostRatio(1.0475); // Fuel cost ratio for climbing
          static const double descendCostRatio(0.9725); // Fuel cost ratio for descending
    // Estimate fuel cost...
    int cruise((numSpeeds+1)/2.0);
    int speedDiff1(std::max(abs(cruise-node1.speed)-1,0));
    int speedDiff2(abs(cruise-node2.speed));
    int diffx(abs(node1.x-node2.x));
    int diffy(abs(node1.y-node2.y));
    int diff(abs(diffx-diffy));
    int diag(abs((diffx+diffy)-diff)/2);
    double vertDiff(node2.height-node1.height);
    //std::cout << node1 << " " << node2 << " straight: " << diff << " diag: " << diag << "\n";
    double ratio=(fgreater(vertDiff,0)?climbCostRatio:fequal(vertDiff,0)?1.0:descendCostRatio);
    vertDiff=fabs(vertDiff);

    // Change as many diagonal moves into horizontal as we can, if vert is more than horiz
    while(vertDiff>diff+diag&&diag>0){
      diag--;
      diff+=2;
    }
      
    // Going fast or slow is expensive, so we'll go to cruiseSpeed if possible.
    // Presumably, we'll do most of our accel/decelerationg at the beginning and end
    // so that most of the track occurs at cruise speed.
      
    double total(0);
    double begin(speedDiff1);
    double end(speedDiff2);

    // Exhaust straight moves first
    int vm(vertDiff);
    int moves1(diff);
    while(moves1 > 0){
      total += (std::max(begin,end)*speedBurnDelta+cruiseBurnRate)*(vm>0?ratio:1.0);
      if(begin>end){begin-=1.0;}else{end-=1.0;}
      moves1--;
      vm--;
    }
    
    // Exhaust diagonal moves next
    moves1 = diag;
    while(moves1 > 0){
      total += (std::max(begin,end)*speedBurnDelta+cruiseBurnRate)*(vm>0?ratio:1.0)*M_SQRT2;
      if(begin>end){begin-=1.0;}else{end-=1.0;}
      moves1--;
      vm--;
    }
    
    // Finally, exhaust vertical moves.
    while(vm > 0){
      total += (std::max(begin,end)*speedBurnDelta+cruiseBurnRate)*(vm>0?ratio:1.0);
      if(begin>end){begin-=1.0;}else{end-=1.0;}
      vm--;
    }
      
    return total; 
  }
};

template <class state>
class ManhattanHeuristic : public Heuristic<state> {
  public:
  double HCost(const state &a,const state &b) const {
        static const double cruiseBurnRate(.0006);
        static const double climbCostRatio(1.0475);
        static const double descendCostRatio(0.9725);
        int vertDiff(b.height-a.height);
        double ratio=(vertDiff>0?climbCostRatio:descendCostRatio);
        vertDiff=abs(vertDiff);
        int diffx(abs(a.x-b.x));
        int diffy(abs(a.y-b.y));
        if(vertDiff > (diffx+diffy)) return vertDiff*cruiseBurnRate*ratio;
        return (diffx+diffy-vertDiff+(vertDiff*ratio))*cruiseBurnRate;
  }
};

//class GoalTester {
//public:
//	virtual ~GoalTester() {}
//	virtual bool goalTest(const airplaneState &i1) const = 0;
//};

class AirplaneEnvironment : public SearchEnvironment<airplaneState, airplaneAction>
{
public:
	AirplaneEnvironment(
          unsigned width=80,
          unsigned length=80,
          unsigned height=20,
          double climbRate=5, // in mps,
          double minSpeed=17, // in mps,
          double maxSpeed=32, // in mps
          uint8_t numSpeeds=5, // Number of discrete speeds
          double cruiseBurnRate=.0006, // Fuel burn rate in liters per unit distance
          double speedBurnDelta=0.0001, // Extra fuel cost for non-cruise speed
          double climbCostRatio=1.0475, // Fuel cost ratio for climbing
          double descendCostRatio=0.9725, // Fuel cost ratio for descending
          double gridSize=3.0); // Horizontal gird width (meters)
	virtual void GetSuccessors(const airplaneState &nodeID, std::vector<airplaneState> &neighbors) const;
	virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual void ApplyAction(airplaneState &s, airplaneAction dir) const;
	virtual void UndoAction(airplaneState &s, airplaneAction dir) const;
	virtual void GetNextState(const airplaneState &currents, airplaneAction dir, airplaneState &news) const;
	
	virtual OccupancyInterface<airplaneState,airplaneAction> *GetOccupancyInfo() { return 0; }
	virtual bool InvertAction(airplaneAction &a) const { return false; }
	
	virtual double HCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double HCost(const airplaneState &)  const { assert(false); return 0; }
	virtual double GCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double GCost(const airplaneState &node1, const airplaneAction &act) const;

	virtual double GetPathLength(const std::vector<airplaneState> &n) const;

//	void SetGoalTest(GoalTester *t) {test = t;}
    virtual airplaneAction GetAction(const airplaneState &node1, const airplaneState &node2) const;
	virtual bool GoalTest(const airplaneState &node, const airplaneState &goal) const;
	virtual bool GoalTest(const airplaneState &) const { assert(false); return false; }
	virtual uint64_t GetStateHash(const airplaneState &node) const;
	virtual uint64_t GetActionHash(airplaneAction act) const;
	
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const airplaneState &l) const;
	virtual void OpenGLDraw(const airplaneState& oldState, const airplaneState &newState, float perc) const;
	virtual void OpenGLDraw(const airplaneState &, const airplaneAction &) const;
	void GLDrawLine(const airplaneState &a, const airplaneState &b) const;
	void GLDrawPath(const std::vector<airplaneState> &p) const;
	void DrawAirplane() const;
	
	recVec GetCoordinate(int x, int y, int z) const;


	std::vector<uint8_t> getGround();
	std::vector<recVec> getGroundNormals();
	std::vector<airplaneAction> getInternalActions();

	virtual void AddLandingStrip(landingStrip x);
	virtual const std::vector<landingStrip>& GetLandingStrips() const {return landingStrips;}

        const uint8_t numSpeeds;
        const double minSpeed;       //Meters per time step
        const double maxSpeed;       //Meters per time step
        double const gridSize; // 3 meters

protected:
	void SetGround(int x, int y, uint8_t val);
	uint8_t GetGround(int x, int y) const;
	bool Valid(int x, int y);
	recVec &GetNormal(int x, int y);
	recVec GetNormal(int x, int y) const;
	void RecurseGround(int x1, int y1, int x2, int y2);
	const int width;
	const int length;
	const int height;
	std::vector<uint8_t> ground;
	std::vector<recVec> groundNormals;
	void DoNormal(recVec pa, recVec pb) const;
	mutable std::vector<airplaneAction> internalActions;

	std::vector<landingStrip> landingStrips;

        const double climbRate;      //Meters per time step
        // Assume 1 unit of movement to be 3 meters
        // 16 liters per hour/ 3600 seconds / 22 mps = 0.0002 liters per meter
        double const cruiseBurnRate;//0.0002*3.0 liters per unit
        double const speedBurnDelta;//0.0001 liters per unit
        double const climbCostRatio;//1.0475);
        double const descendCostRatio;//0.9725);

        //TODO Add wind constants
        //const double windSpeed = 0;
        //const double windDirection = 0;
};

#endif /* Airplane_h */
