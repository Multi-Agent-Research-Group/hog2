//
//  AirplaneCBSUnits.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__AirplaneCBSUnits__
#define __hog2_glut__AirplaneCBSUnits__

#include <iostream>
#include <limits> 
#include <algorithm>
#include <map>

#include <queue>
#include <functional>
#include <vector>

#include <thread>
#include <mutex>

#include "Unit.h"
#include "UnitGroup.h"
#include "Airplane.h"
#include "AirplaneConstrained.h"
#include "AirplaneTicketAuthority.h"
#include "TemplateAStar.h"
#include "BFS.h"
#include "Heuristic.h"
#include "Timer.h"

extern bool highsort;
extern bool randomalg;
extern unsigned killtime;

template <class state>
struct CompareLowGCost {
  bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
  {
    if (fequal(i1.g+i1.h, i2.g+i2.h))
    {
      return fless(i1.data.t,i2.data.t);
    }
    return (fgreater(i1.g+i1.h, i2.g+i2.h));
  }
};

template <class state>
struct RandomTieBreaking {
  bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
  {
    if (fequal(i1.g+i1.h, i2.g+i2.h))
    {
      if(randomalg && fequal(i1.g,i2.g))
        return rand()%2;
      else
        return (fless(i1.g, i2.g));
    }
    return (fgreater(i1.g+i1.h, i2.g+i2.h));
  }
};


class AirCBSUnit : public Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> {
public:
	AirCBSUnit(std::vector<airtimeState> const &gs)
	:start(0), goal(1), current(gs[0]), waypoints(gs) {}
	const char *GetName() { return "AirCBSUnit"; }
	bool MakeMove(AirplaneConstrainedEnvironment *,
            OccupancyInterface<airtimeState,airplaneAction> *, 
            SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *,
            airplaneAction& a);
	void UpdateLocation(AirplaneConstrainedEnvironment *, airtimeState &newLoc, bool success, 
						SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *)
	{ if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); }
	
	void GetLocation(airtimeState &l) { l = current; }
	void OpenGLDraw(const AirplaneConstrainedEnvironment *, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *) const;
	void GetGoal(airtimeState &s) { s = waypoints[goal]; }
	void GetStart(airtimeState &s) { s = waypoints[start]; }
	void SetPath(std::vector<airtimeState> &p);
	void PushFrontPath(std::vector<airtimeState> &s)
	{
		std::vector<airtimeState> newPath;
		for (airtimeState x : s)
			newPath.push_back(x);
		for (airtimeState y : myPath)
			newPath.push_back(y);
		myPath = newPath;
	}
	inline std::vector<airtimeState> const& GetPath()const{return myPath;}
    void UpdateGoal(airtimeState &start, airtimeState &goal);
        void setUnitNumber(unsigned n){number=n;}
        unsigned getUnitNumber()const{return number;}

private:
	unsigned start, goal;
        airtimeState current;
	std::vector<airtimeState> waypoints;
	std::vector<airtimeState> myPath;
        unsigned number;
};

struct airConflict {
	airConstraint c;
	int unit1;
};

struct AirCBSTreeNode {
	AirCBSTreeNode():parent(0),satisfiable(true){}
	std::vector< std::vector<airtimeState> > paths;
	airConflict con;
	unsigned int parent;
	bool satisfiable;
};

static std::ostream& operator <<(std::ostream & out, const AirCBSTreeNode &act)
{
	out << "(paths:"<<act.paths.size()<<", parent: "<<act.parent<< ", satisfiable: "<<act.satisfiable<<")";
	return out;
}

struct EnvironmentContainer {
	EnvironmentContainer() : name("NULL ENV"), environment(0), heuristic(0), conflict_cutoff(0), astar_weight(0.0f) {}
	EnvironmentContainer(std::string n, AirplaneConstrainedEnvironment* e, Heuristic<airtimeState>* h, uint32_t conf, float a) : name(n), environment(e), heuristic(h), conflict_cutoff(conf), astar_weight(a) {}
	AirplaneConstrainedEnvironment* environment;
	Heuristic<airtimeState>* heuristic;
	uint64_t conflict_cutoff;
	float astar_weight;
	std::string name;
};


class AirCBSGroup : public UnitGroup<airtimeState, airplaneAction, AirplaneConstrainedEnvironment>
{
public:
	AirCBSGroup(std::vector<EnvironmentContainer> const&, bool u_r, bool u_w, bool);
	bool MakeMove(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, 
				  SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si, airplaneAction& a);
	void UpdateLocation(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, 
						airtimeState &loc, bool success, SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si);
	void AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u);
	void UpdateUnitGoal(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, airtimeState newGoal);
	void UpdateSingleUnitPath(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, airtimeState newGoal);
	
	void OpenGLDraw(const AirplaneConstrainedEnvironment *, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *)  const;
	double getTime() {return time;}
	bool donePlanning() {return planFinished;}
	void ExpandOneCBSNode(bool gui=true);

private:    

        unsigned IssueTicketsForNode(int location);
        unsigned LoadConstraintsForNode(int location);
        bool Bypass(int best, unsigned numConflicts, airConflict const& c1, bool gui);
	void Replan(int location);
        unsigned HasConflict(std::vector<airtimeState> const& a, std::vector<airtimeState> const& b, int x, int y, airConflict &c1, airConflict &c2, bool update, bool verbose=false);
	unsigned FindFirstConflict(AirCBSTreeNode const& location, airConflict &c1, airConflict &c2);
        void processSolution();

	void DoHAStar(airtimeState& start, airtimeState& goal, std::vector<airtimeState>& thePath);
	bool HAStarHelper(airtimeState& start, airtimeState& goal, std::vector<airtimeState>& thePath, unsigned& envConflicts, unsigned& conflicts);
	
	bool planFinished;

	/* Code for dealing with multiple environments */
	std::vector<EnvironmentContainer> environments;
	EnvironmentContainer* currentEnvironment;

	void SetEnvironment(unsigned);
    void ClearEnvironmentConstraints();
    void AddEnvironmentConstraint(airConstraint c);

	std::vector<AirCBSTreeNode> tree;
	std::vector<airtimeState> thePath;
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment, AStarOpenClosed<airtimeState, RandomTieBreaking<airtimeState> > > astar;
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment, AStarOpenClosed<airtimeState, CompareLowGCost<airtimeState> > > astar2;
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> astar3;
	double time;

	unsigned int bestNode;
	std::mutex bestNodeLock;

	struct OpenListNode {
		OpenListNode() : location(0), cost(0), nc(0) {}
		OpenListNode(uint loc, double c, uint16_t n) : location(loc), cost(c),nc(n) {}
                std::ostream& operator <<(std::ostream& out)const{
                  out << "(loc: "<<location<<", nc: "<<nc<< ", cost: "<<cost<<")";
                  return out;
                }

		uint location;
		double cost;	
                unsigned nc;
	};
	struct OpenListNodeCompare {
          bool operator() (const OpenListNode& left, const OpenListNode& right) {
            if(highsort)
               return (left.nc==right.nc)?(left.cost > right.cost):(left.nc>right.nc);
             else
              return (left.cost==right.cost)?(left.nc > right.nc):(left.cost>right.cost);
          }
	};

	std::priority_queue<AirCBSGroup::OpenListNode, std::vector<AirCBSGroup::OpenListNode>, AirCBSGroup::OpenListNodeCompare> openList;

	uint TOTAL_EXPANSIONS = 0;
        Timer* timer=0;

	TicketAuthority ticketAuthority;

	bool use_restricted = false;
	bool use_waiting = false;
	bool nobypass = false;
        std::vector<AirplaneConstrainedEnvironment*> agentEnvs;
};


#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
