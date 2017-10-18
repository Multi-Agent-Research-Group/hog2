//
//  AirplaneHiFiGridless.h
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef AirplaneHiFiGridless_h
#define AirplaneHiFiGridless_h

#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
#include "ConstrainedEnvironment.h"
#include "constants.h"
#include "AirStates.h"
#include "AStarOpenClosed.h"
#include "TemplateAStar.h"

#include "UnitTimeCAT.h"

template<>
double SoftConstraint<PlatformState>::cost(PlatformState const& other, double scale) const;

// Check if an openlist node conflicts with a node from an existing path
template<typename state>
unsigned checkForTheConflict(state const*const parent, state const*const node, state const*const pathParent, state const*const pathNode){
  Constraint<state> v(*node);
  if(v.ConflictsWith(*pathNode)){return 1;}
  if(parent && pathParent){
    Constraint<state> e1(*parent,*node);
    if(e1.ConflictsWith(*pathParent,*pathNode)){return 1;}
  }
  return 0; 
}

// Heuristics
template <class state>
class StraightLineHeuristic1 : public Heuristic<state> {
  public:
  double HCost(const state &a,const state &b) const {
        return a.distanceTo(b);
  }
};

bool operator==(PlatformAction const& a1, PlatformAction const& a2);

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
class AirplaneHiFiGridlessEnvironment : public ConstrainedEnvironment<PlatformState, PlatformAction>
{
  public:
    // Constructor
    AirplaneHiFiGridlessEnvironment(
        unsigned width=80,
        unsigned length=80,
        unsigned height=20,
        unsigned minSpeed=1,
        unsigned maxSpeed=5,
        uint8_t numSpeeds=5, // Number of discrete speeds
        double goalRadius=PlatformState::SPEEDS[5],
        double maxTurn=7.5,
        double maxDive=15,
        double cruiseBurnRate=.06, // Fuel burn rate in liters per unit distance
        double climbCost=0.01, // Fuel cost for climbing
        double descendCost=-0.00005); // Fuel cost for descending
    //std::string const& perimeterFile=std::string("airplanePerimeter.dat"));

    virtual std::string name()const{return "AirplaneHiFiGridlessEnvironment";}
    void AddConstraint(Constraint<PlatformState> c);
    void AddSoftConstraint(SoftConstraint<PlatformState> c){sconstraints.push_back(c);}
    void ClearConstraints();
    void ClearStaticConstraints();
    double ViolatesConstraint(const PlatformState &from, const PlatformState &to, int time) const;
    double ViolatesConstraint(const PlatformState &from, const PlatformState &to) const;

    // Successors and actions
    virtual void GetSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const;
    virtual void GetReverseSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const;

    virtual void GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;
    void GetPerfectActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;


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
    const unsigned minSpeed;
    const unsigned maxSpeed;

    PlatformState const* goal;
    PlatformState const& getGoal()const{return *goal;}
    void setGoal(PlatformState const& g){goal=&g;}

    PlatformState const* start;
    PlatformState const& getStart()const{return *start;}
    void setStart(PlatformState const& s){start=&s;}

    void SetNilGCosts(){nilGCost=true;}
    void UnsetNilGCosts(){nilGCost=false;}

    void setSoftConstraintEffectiveness(float v){softConstraintEffectiveness=v;}
    float getSoftConstraintEffectiveness()const{return softConstraintEffectiveness;}

  protected:

    virtual AirplaneHiFiGridlessEnvironment& getRef() {return *this;}
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
    double goalRadius;
    double maxTurn;
    double maxDive;

    // Assume 1 unit of movement to be 3 meters
    // 16 liters per hour/ 3600 seconds / 22 mps = 0.0002 liters per meter
    double const cruiseBurnRate;//0.0002*30.0 liters per unit
    double const climbCost;//1.0475;
    double const descendCost;

    // Caching for turn information
    std::vector<int8_t> turns;
    std::vector<int8_t> quad_turns;

    std::vector<SoftConstraint<PlatformState> > sconstraints;
    std::vector<Constraint<PlatformState> > constraints;
    std::vector<Constraint<PlatformState> > static_constraints;
    float softConstraintEffectiveness=0.0;

  private:
    bool nilGCost;
    virtual double myHCost(const PlatformState &node1, const PlatformState &node2) const;
    //bool perimeterLoaded;
    //std::string perimeterFile;
    //AirplanePerimeterDBBuilder<PlatformState, PlatformAction, AirplaneHiFiGridlessEnvironment> perimeter[2]; // One for each type of aircraft

    //TODO Add wind constants
    //const double windSpeed = 0;
    //const double windDirection = 0;
};


template <typename state, typename action, typename environment>
class NonHolonomicComparator {
  public:
  bool operator()(const AStarOpenClosedData<state> &ci1, const AStarOpenClosedData<state> &ci2) const
  {
    // Our notion of "best" is agents near start to be facing the goal
    // failing that, we use H cost
    // Check heading first


    //double dist(std::max(ci1.data.distanceTo(currentEnv->getStart()),ci2.data.distanceTo(currentEnv->getStart())));
    //std::cout << "Comparing " << ci1.data << " to " << ci2.data << " ";

    double nnh1((ci1.data.headingTo(currentEnv->getGoal()))-ci1.data.hdg());
    if(fgreater(nnh1,180.)){nnh1=-(360.-nnh1);}
    double nnh2((ci2.data.headingTo(currentEnv->getGoal()))-ci2.data.hdg());
    if(fgreater(nnh2,180.)){nnh2=-(360.-nnh2);}

    if(fequal(fabs(nnh1),fabs(nnh2))){
      double nne1((ci1.data.elevationTo(currentEnv->getGoal()))-ci1.data.pitch());
      double nne2((ci2.data.elevationTo(currentEnv->getGoal()))-ci2.data.pitch());

      if(fequal(fabs(nne1),fabs(nne2))){
        if (fequal(ci1.g+ci1.h, ci2.g+ci2.h)){ // F-cost equal

          // Assumption of uniform time step data
          if(useCAT && CAT){
            // Make them non-const :)
            AStarOpenClosedData<state>& i1(const_cast<AStarOpenClosedData<state>&>(ci1));
            AStarOpenClosedData<state>& i2(const_cast<AStarOpenClosedData<state>&>(ci2));
            // Compute cumulative conflicts (if not already done)

            if(i1.data.nc ==-1){

              // Get number of conflicts in the parent
              state const*const parent1(i1.parentID?&(currentAstar->GetItem(i1.parentID).data):nullptr);
              unsigned nc1(parent1?parent1->nc:0);
              //std::cout << "matches " << matches.size() << "\n";

              // Count number of conflicts
              for(int agent(0); agent<CAT->numAgents(); ++agent){
                if(currentAgent == agent) continue;
                state const* p(0);
                if(i1.data.t!=0)
                  p=&(CAT->get(agent,i1.data.t-1));
                state const& n=CAT->get(agent,i1.data.t);
                nc1+=checkForTheConflict(parent1,&i1.data,p,&n);
              }
              // Set the number of conflicts in the data object
              i1.data.nc=nc1;
            }

            if(i2.data.nc ==-1){

              // Get number of conflicts in the parent
              state const*const parent2(i2.parentID?&(currentAstar->GetItem(i2.parentID).data):nullptr);
              unsigned nc2(parent2?parent2->nc:0);
              //std::cout << "matches " << matches.size() << "\n";

              // Count number of conflicts
              for(int agent(0); agent<CAT->numAgents(); ++agent){
                if(currentAgent == agent) continue;
                state const* p(0);
                if(i2.data.t!=0)
                  p=&(CAT->get(agent,i2.data.t-1));
                state const& n=CAT->get(agent,i2.data.t);
                nc2+=checkForTheConflict(parent2,&i2.data,p,&n);
              }
              // Set the number of conflicts in the data object
              i2.data.nc=nc2;
            }
            return fgreater(i1.data.nc,i2.data.nc);
          }
          else if(randomalg && fequal(ci1.g,ci2.g)){
            return rand()%2;
          } else {
            //std::cout << "GCost\n";
            return fless(ci1.g, ci2.g);  // Tie-break toward greater g-cost
          }
        }
        //std::cout << "FCost\n";
        return (fgreater(ci1.g+ci1.h, ci2.g+ci2.h));
      }
      //std::cout << "elevation\n";
      return fgreater(fabs(nne1),fabs(nne2));
    }
    //std::cout << "heading\n";
    return fgreater(fabs(nnh1),fabs(nnh2));
  }

    static TemplateAStar<state, action, environment, AStarOpenClosed<state, NonHolonomicComparator<state,action,environment> > >* currentAstar;
    static AirplaneHiFiGridlessEnvironment* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static UnitTimeCAT<PlatformState,AirplaneHiFiGridlessEnvironment>* CAT; // Conflict Avoidance Table
};

#endif /* Airplane_h */
