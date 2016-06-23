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
#include "Unit.h"
#include "UnitGroup.h"
#include "Airplane.h"
#include "AirplaneConstrained.h"
#include "TemplateAStar.h"
#include "BFS.h"

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

private:
	airtimeState start, goal, current;
	std::vector<airtimeState> myPath;
};

struct airConflict {
	airConstraint c;
	int unit1;
};

struct AirCBSTreeNode {
	AirCBSTreeNode() { closed = false; satisfiable = true;}
	std::vector< std::vector<airtimeState> > paths;
	airConflict con;
	unsigned int parent;
	bool closed;
	bool satisfiable;
};

class AirCBSGroup : public UnitGroup<airtimeState, airplaneAction, AirplaneConstrainedEnvironment>
{
public:
	AirCBSGroup(AirplaneConstrainedEnvironment *me);
	bool MakeMove(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, 
				  SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si, airplaneAction& a);
	void UpdateLocation(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, 
						airtimeState &loc, bool success, SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si);
	void AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u);
	void OpenGLDraw(const AirplaneConstrainedEnvironment *, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *)  const;
	double getTime() {return time;}
	void incrementTime() {time += 1;}
	bool donePlanning() {return planFinished;}
private:
	void ExpandOneCBSNode();
	void Replan(int location);
	bool FindFirstConflict(int location, airConflict &c1, airConflict &c2);
	
	bool planFinished;
	AirplaneConstrainedEnvironment *ae;

	std::vector<AirCBSTreeNode> tree;
	std::vector<airtimeState> thePath;
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> astar;

	double time;
	unsigned int bestNode;
};


#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
