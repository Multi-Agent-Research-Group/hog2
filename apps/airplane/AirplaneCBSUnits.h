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

class AirCBSUnit : public Unit<airplaneState, airplaneAction, AirplaneEnvironment> {
public:
	AirCBSUnit(const airplaneState &s, const airplaneState &g)
	:start(s), goal(g), current(s) {}
	const char *GetName() { return "AirCBSUnit"; }
	bool MakeMove(AirplaneEnvironment *, OccupancyInterface<airplaneState,airplaneAction> *, 
				  SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *, airplaneAction& a);
	void UpdateLocation(AirplaneEnvironment *, airplaneState &newLoc, bool success, 
						SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *)
	{ if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); }
	
	void GetLocation(airplaneState &l) { l = current; }
	void OpenGLDraw(const AirplaneEnvironment *, const SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *) const;
	void GetGoal(airplaneState &s) { s = goal; }
	void GetStart(airplaneState &s) { s = start; }
	void SetPath(std::vector<airplaneState> &p);
private:
	airplaneState start, goal, current;
	std::vector<airplaneState> myPath;
};

struct airConflict {
	airConstraint c;
	int unit1;
};

struct AirCBSTreeNode {
	AirCBSTreeNode() { closed = false; }
	std::vector< std::vector<airplaneState> > paths;
	airConflict con;
	unsigned int parent;
	bool closed;
};

class AirCBSGroup : public UnitGroup<airplaneState, airplaneAction, AirplaneEnvironment>
{
public:
	AirCBSGroup(AirplaneEnvironment *me);
	std::vector<bool> MakeMoveAllUnits(AirplaneEnvironment *e, SimulationInfo<airplaneState, airplaneAction, AirplaneEnvironment> *si, std::vector<airplaneAction> &a);
	bool MakeMove(Unit<airplaneState, airplaneAction, AirplaneEnvironment> *u, AirplaneEnvironment *e, 
				  SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *si, airplaneAction& a);
	void UpdateLocation(Unit<airplaneState, airplaneAction, AirplaneEnvironment> *u, AirplaneEnvironment *e, 
						airplaneState &loc, bool success, SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *si);
	void AddUnit(Unit<airplaneState, airplaneAction, AirplaneEnvironment> *u);
	void OpenGLDraw(const AirplaneEnvironment *, const SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *)  const;
	double getTime() {return time;}
	void incrementTime() {time += 1;}
private:
	void ExpandOneCBSNode();
	void Replan(int location);
	bool FindFirstConflict(int location, airConflict &c1, airConflict &c2);
	
	bool planFinished;
	AirplaneConstrainedEnvironment *a2e;
	AirplaneEnvironment *ae;
	std::vector<AirCBSTreeNode> tree;
	std::vector<airtimeState> thePath;
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> astar;
	TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> anostar;
	BFS<airplaneState, airplaneAction> bfs;
	double time;
	unsigned int bestNode;
};


#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
