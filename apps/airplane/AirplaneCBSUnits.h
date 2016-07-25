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

#include "Unit.h"
#include "UnitGroup.h"
#include "Airplane.h"
#include "AirplaneConstrained.h"
#include "AirplaneTicketAuthority.h"
#include "TemplateAStar.h"
#include "BFS.h"
#include "Heuristic.h"


class AirCBSUnit : public Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> {
public:
	AirCBSUnit(const airtimeState &s, const airtimeState &g)
	:start(s), goal(g), current(s) {}
	const char *GetName() { return "AirCBSUnit"; }
	bool MakeMove(AirplaneConstrainedEnvironment *, OccupancyInterface<airtimeState,airplaneAction> *, 
				  SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *, airplaneAction& a);
	void UpdateLocation(AirplaneConstrainedEnvironment *, airtimeState &newLoc, bool success, 
						SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *)
	{ if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); }
	
	void GetLocation(airtimeState &l) { l = current; }
	void OpenGLDraw(const AirplaneConstrainedEnvironment *, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *) const;
	void GetGoal(airtimeState &s) { s = goal; }
	void GetStart(airtimeState &s) { s = start; }
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

private:
	airtimeState start, goal, current;
	std::vector<airtimeState> myPath;
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
	AirCBSGroup(AirplaneConstrainedEnvironment *me, AirplaneConstrainedEnvironment* simple, unsigned threshold);
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

private:    

	void ExpandOneCBSNode();
	void Replan(int location);
	bool FindFirstConflict(int location, airConflict &c1, airConflict &c2);
	
	bool planFinished;

	/* Code for dealing with multiple environments */
	std::vector<EnvironmentContainer> environments;
	EnvironmentContainer* currentEnvironment;

	void SetEnvironment(unsigned);
    void ClearEnvironmentConstraints();
    void AddEnvironmentConstraint(airConstraint c);

	std::vector<AirCBSTreeNode> tree;
	std::vector<airtimeState> thePath;
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> astar;

	double time;

	unsigned int bestNode;
	struct OpenListNode {
		OpenListNode() : location(0), cost(0) {}
		OpenListNode(uint loc, double c) : location(loc), cost(c) {}
		uint location;
		double cost;	
	};
	struct OpenListNodeCompare {
		bool operator() (const OpenListNode& left, const OpenListNode& right) {
			return left.cost > right.cost;
		}
	};

	std::priority_queue<AirCBSGroup::OpenListNode, std::vector<AirCBSGroup::OpenListNode>, AirCBSGroup::OpenListNodeCompare> openList;

	uint TOTAL_EXPANSIONS = 0;

	TicketAuthority ticketAuthority;
};


#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
