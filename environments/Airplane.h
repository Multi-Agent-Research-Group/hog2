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
#include "AirplanePerimeterDBBuilder.h"

// turn type:
// 45, 90, 0, shift
// speed:
// faster, slower
// height:
// up / down

const int8_t k45 = 1;
const int8_t k90 = 2;
const int8_t k135 = 3;
const int8_t k180 = 4;
const uint8_t kCircleSize = 8;

const int8_t kShift = 5; // Should be outside the range [-kCircleSize/2, kCircleSize/2]
const int8_t kWait = 6;

// Utility function
namespace{
template<unsigned fullDegs>
unsigned hdgDiff(unsigned a, unsigned b){
  unsigned d(abs(a-b)%fullDegs);
  return d>(fullDegs/2.0)?fullDegs-d:d;
}
};

// Define the type of airplane
enum AirplaneType {
  QUAD=0, PLANE=1
};

enum SearchType {
  FORWARD=0, REVERSE=1
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


// Heuristics
template <class state>
class StraightLineHeuristic : public Heuristic<state> {
  public:
  double HCost(const state &a,const state &b) const {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.height-b.height)*(a.height-b.height));
  }
};

// Actual Environment
class AirplaneEnvironment : public SearchEnvironment<airplaneState, airplaneAction>
{
public:
	
  // Constructor
  AirplaneEnvironment(
          unsigned width=80,
          unsigned length=80,
          unsigned height=20,
          double climbRate=5,
          double minSpeed=1,
          double maxSpeed=5,
          uint8_t numSpeeds=5, // Number of discrete speeds
          double cruiseBurnRate=.006, // Fuel burn rate in liters per unit distance
          double speedBurnDelta=0.001, // Extra fuel cost for non-cruise speed
          double climbCost=0.001, // Fuel cost for climbing
          double descendCost=-0.0005, // Fuel cost for descending
          double gridSize=3.0, // Horizontal grid width (meters)
          std::string const& perimeterFile=std::string("airplanePerimeter.dat"));
	
  virtual char const*const name()const{return "AirplaneEnvironment";}
  // Successors and actions
  virtual void GetSuccessors(const airplaneState &nodeID, std::vector<airplaneState> &neighbors) const;
  virtual void GetReverseSuccessors(const airplaneState &nodeID, std::vector<airplaneState> &neighbors) const;
	
  virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
  
  virtual void GetActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
  virtual void GetActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
  virtual bool AppendLandingActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
  virtual bool AppendLandingActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;

	virtual void GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
 
  virtual void GetReverseActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
  virtual void GetReverseActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;



	virtual void ApplyAction(airplaneState &s, airplaneAction dir) const;
	virtual void UndoAction(airplaneState &s, airplaneAction dir) const;
	virtual void GetNextState(const airplaneState &currents, airplaneAction dir, airplaneState &news) const;
	virtual bool InvertAction(airplaneAction &a) const { return false; }
  virtual airplaneAction GetAction(const airplaneState &node1, const airplaneState &node2) const;
  virtual airplaneAction GetReverseAction(const airplaneState &node1, const airplaneState &node2) const;
  

  // Occupancy Info not supported
	virtual OccupancyInterface<airplaneState,airplaneAction> *GetOccupancyInfo() { return 0; }
	
  // Heuristics and paths
	virtual double HCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double OldHCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double HCost(const airplaneState &)  const { assert(false); return 0; }
	virtual double GCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double GCost(const airplaneState &node1, const airplaneAction &act) const;
	virtual double GetPathLength(const std::vector<airplaneState> &n) const;
  void loadPerimeterDB();

  // Goal testing
  virtual bool GoalTest(const airplaneState &node, const airplaneState &goal) const;
	virtual bool GoalTest(const airplaneState &) const { assert(false); return false; }

  // Hashing
	virtual uint64_t GetStateHash(const airplaneState &node) const;
	virtual uint64_t GetActionHash(airplaneAction act) const;
	
  // Drawing
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const airplaneState &l) const;
	virtual void OpenGLDraw(const airplaneState& oldState, const airplaneState &newState, float perc) const;
	virtual void OpenGLDraw(const airplaneState &, const airplaneAction &) const;
	void GLDrawLine(const airplaneState &a, const airplaneState &b) const;
	void GLDrawPath(const std::vector<airplaneState> &p) const;
	void DrawAirplane() const;
  void DrawQuadCopter() const;
	recVec GetCoordinate(int x, int y, int z) const;

  // Getters
	std::vector<uint8_t> getGround();
	std::vector<recVec> getGroundNormals();

	std::vector<airplaneAction> getInternalActions();

  // Landing Strups
	virtual void AddLandingStrip(landingStrip x);
	virtual const std::vector<landingStrip>& GetLandingStrips() const {return landingStrips;}
        
  // State information
  const uint8_t numSpeeds;  // Number of speed steps
  const double minSpeed;    // Meters per time step
  const double maxSpeed;    // Meters per time step
  double const gridSize;    // 3 meters

  airplaneState const* goal;
  airplaneState const& getGoal()const{return *goal;}
  void setGoal(airplaneState const& g){goal=&g;}

  void setSearchType(SearchType s){searchtype=s;}

protected:
  
  virtual AirplaneEnvironment& getRef() {return *this;}
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
  double const cruiseBurnRate;//0.0002*30.0 liters per unit
  double const speedBurnDelta;//0.0001 liters per unit
  double const climbCost;//1.0475;
  double const descendCost;

  // Caching for turn information
  std::vector<int8_t> turns;
  std::vector<int8_t> quad_turns;
  SearchType searchtype;

private:
	virtual double myHCost(const airplaneState &node1, const airplaneState &node2) const;
  bool perimeterLoaded;
  std::string perimeterFile;
  AirplanePerimeterDBBuilder<airplaneState, airplaneAction, AirplaneEnvironment> perimeter[2]; // One for each type of aircraft

  //TODO Add wind constants
  //const double windSpeed = 0;
  //const double windDirection = 0;
};

#endif /* Airplane_h */
