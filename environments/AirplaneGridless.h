//
//  AirplaneGridless.h
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/16.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef AirplaneGridless_h
#define AirplaneGridless_h

#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
#include "ConstrainedEnvironment.h"
#include "constants.h"
#include "AirStates.h"

struct gridlessLandingStrip {
	gridlessLandingStrip(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2, PlatformState &launch_state, PlatformState &landing_state, PlatformState &goal_state) : x1(x1), x2(x2), y1(y1), y2(y2), 
				 launch_state(launch_state), landing_state(landing_state), goal_state(goal_state) {}
	uint16_t x1;
	uint16_t x2;
	uint16_t y1;
	uint16_t y2;
	uint16_t z = 0;
	PlatformState goal_state;
	PlatformState launch_state;
	PlatformState landing_state;
};


// Actual Environment
class AirplaneGridlessEnvironment : public ConstrainedEnvironment<PlatformState, PlatformAction>
{
  public:
    // Constructor
    AirplaneGridlessEnvironment(
        unsigned width=80,
        unsigned length=80,
        unsigned height=20,
        double minSpeed=1,
        double maxSpeed=5,
        uint8_t numSpeeds=5, // Number of discrete speeds
        double cruiseBurnRate=.06, // Fuel burn rate in liters per unit distance
        double climbCost=0.01, // Fuel cost for climbing
        double descendCost=-0.00005); // Fuel cost for descending
    //std::string const& perimeterFile=std::string("airplanePerimeter.dat"));

    virtual char const*const name()const{return "AirplaneGridlessEnvironment";}
    void AddConstraint(Constraint<PlatformState> c);
    void ClearConstraints();
    void ClearStaticConstraints();
    bool ViolatesConstraint(const PlatformState &from, const PlatformState &to, int time) const;
    bool ViolatesConstraint(const PlatformState &from, const PlatformState &to) const;

    // Successors and actions
    virtual void GetSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const;
    virtual void GetReverseSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const;

    virtual void GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;


    virtual void GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;


    virtual void ApplyAction(PlatformState &s, PlatformAction dir) const;
    virtual void UndoAction(PlatformState &s, PlatformAction dir) const;
    virtual void GetNextState(const PlatformState &currents, PlatformAction dir, PlatformState &news) const;
    virtual bool InvertAction(PlatformAction &a) const { return false; }
    virtual PlatformAction GetAction(const PlatformState &node1, const PlatformState &node2) const;
    virtual PlatformAction GetReverseAction(const PlatformState &node1, const PlatformState &node2) const;


    // Occupancy Info not supported
    virtual OccupancyInterface<PlatformState,PlatformAction> *GetOccupancyInfo() { return 0; }

    // Heuristics and paths
    virtual double HCost(const PlatformState &node1, const PlatformState &node2) const;
    virtual double ReverseHCost(const PlatformState &,const PlatformState &)  const;
    virtual double ReverseGCost(const PlatformState &node1, const PlatformState &node2) const;
    virtual double HCost(const PlatformState &)  const { assert(false); return 0; }
    virtual double GCost(const PlatformState &node1, const PlatformState &node2) const;
    virtual double GCost(const PlatformState &node1, const PlatformAction &act) const;
    virtual double GetPathLength(const std::vector<PlatformState> &n) const;

    // Goal testing
    virtual bool GoalTest(const PlatformState &node, const PlatformState &goal) const;
    virtual bool GoalTest(const PlatformState &) const { assert(false); return false; }

    // Hashing
    virtual void GetStateFromHash(uint64_t hash, PlatformState&) const;
    virtual uint64_t GetStateHash(const PlatformState &node) const;
    virtual uint64_t GetActionHash(PlatformAction act) const;

    // Drawing
    virtual void OpenGLDraw() const;
    virtual void OpenGLDraw(const PlatformState &l) const;
    virtual void OpenGLDraw(const PlatformState& oldState, const PlatformState &newState, float perc) const;
    virtual void OpenGLDraw(const PlatformState &, const PlatformAction &) const;
    void GLDrawLine(const PlatformState &a, const PlatformState &b) const;
    void GLDrawPath(const std::vector<PlatformState> &p, const std::vector<PlatformState> &wpts) const;
    void DrawAirplane() const;
    void DrawQuadCopter() const;
    recVec GetCoordinate(int x, int y, int z) const;

    // Getters
    std::vector<uint8_t> getGround();
    std::vector<recVec> getGroundNormals();

    std::vector<PlatformAction> getInternalActions();

    // Landing Strups
    virtual void AddLandingStrip(gridlessLandingStrip & x);
    virtual const std::vector<gridlessLandingStrip>& GetLandingStrips() const {return landingStrips;}

    // State information
    const uint8_t numSpeeds;  // Number of speed steps
    const double minSpeed;
    const double maxSpeed;

    PlatformState const* goal;
    PlatformState const& getGoal()const{return *goal;}
    void setGoal(PlatformState const& g){goal=&g;}

    PlatformState const* start;
    PlatformState const& getStart()const{return *start;}
    void setStart(PlatformState const& s){start=&s;}

  protected:

    virtual AirplaneGridlessEnvironment& getRef() {return *this;}
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
    mutable std::vector<PlatformAction> internalActions;

    std::vector<gridlessLandingStrip> landingStrips;

    // Assume 1 unit of movement to be 3 meters
    // 16 liters per hour/ 3600 seconds / 22 mps = 0.0002 liters per meter
    double const cruiseBurnRate;//0.0002*30.0 liters per unit
    double const climbCost;//1.0475;
    double const descendCost;

    // Caching for turn information
    std::vector<int8_t> turns;
    std::vector<int8_t> quad_turns;

    std::vector<Constraint<PlatformState> > constraints;
    std::vector<Constraint<PlatformState> > static_constraints;

  private:
    virtual double myHCost(const PlatformState &node1, const PlatformState &node2) const;
    //bool perimeterLoaded;
    //std::string perimeterFile;
    //AirplanePerimeterDBBuilder<PlatformState, PlatformAction, AirplaneGridlessEnvironment> perimeter[2]; // One for each type of aircraft

    //TODO Add wind constants
    //const double windSpeed = 0;
    //const double windDirection = 0;
};

#endif /* Airplane_h */
