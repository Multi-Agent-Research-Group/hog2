/*
 *  Map2DEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MAP2DENVIRONMENT_H
#define MAP2DENVIRONMENT_H

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "BitVector.h"
#include "GraphEnvironment.h"
#include "GridStates.h"

#include <cassert>


const int numPrimitiveActions = 8;
const int numActions = 10;
const tDirection possibleDir[numActions] = { kN, kNE, kE, kSE, kS, kSW, kW, kNW, kStay, kTeleport };
const int kStayIndex = 8; // index of kStay




//typedef OccupancyInterface<xyLoc, tDirection> BaseMapOccupancyInterface;


class MapEnvironment : public SearchEnvironment<xyLoc, tDirection>
{
public:
	MapEnvironment(Map *m, bool useOccupancy = false);
	MapEnvironment(MapEnvironment *);
	virtual ~MapEnvironment();
	void SetGraphHeuristic(GraphHeuristic *h);
	GraphHeuristic *GetGraphHeuristic();
        virtual std::string name()const{std::stringstream ss; ss<<"Map2DEnvironment("<<(int)connectedness<<"-connected)"; return ss.str();}
	virtual void GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
	virtual void GetReverseSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const{GetSuccessors(nodeID,neighbors);}
	bool GetNextSuccessor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext4Successor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext5Successor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext8Successor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	void GetActions(const xyLoc &nodeID, std::vector<tDirection> &actions) const;
	tDirection GetAction(const xyLoc &s1, const xyLoc &s2) const;
	virtual void ApplyAction(xyLoc &s, tDirection dir) const;
	virtual BaseMapOccupancyInterface *GetOccupancyInfo() { return oi; }

	virtual bool InvertAction(tDirection &a) const;

//	bool Contractable(const xyLoc &where);
	
	virtual double HCost(const xyLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for MapEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyLoc &node1, const xyLoc &node2) const;
	virtual double GCost(const xyLoc &node1, const xyLoc &node2) const;
	virtual double GCost(const xyLoc &node1, const tDirection &act) const;
        bool LineOfSight(const xyLoc &node, const xyLoc &goal) const;
	bool GoalTest(const xyLoc &node, const xyLoc &goal) const;

	bool GoalTest(const xyLoc &){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for MapEnvironment\n");
		exit(1); return false;}

	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const xyLoc &node) const;
	uint64_t GetActionHash(tDirection act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const xyLoc &l) const;
	virtual void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const;
	virtual void OpenGLDraw(const xyLoc &, const tDirection &) const;
	virtual void GLLabelState(const xyLoc &, const char *) const;
	virtual void GLLabelState(const xyLoc &s, const char *str, double scale) const;
	virtual void GLDrawLine(const xyLoc &x, const xyLoc &y) const;
	
	std::string SVGHeader();
	std::string SVGDraw();
	std::string SVGDraw(const xyLoc &);
	std::string SVGLabelState(const xyLoc &, const char *, double scale) const;
	std::string SVGDrawLine(const xyLoc &x, const xyLoc &y, int width=1) const;
	std::string SVGFrameRect(int left, int top, int right, int bottom, int width = 1);

	virtual void Draw() const;
	virtual void Draw(const xyLoc &l) const;
	virtual void DrawLine(const xyLoc &x, const xyLoc &y, double width = 1.0) const;

	
	//virtual void OpenGLDraw(const xyLoc &, const tDirection &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map* GetMap() const { return map; }

	virtual void GetNextState(const xyLoc &currents, tDirection dir, xyLoc &news) const;

	void StoreGoal(xyLoc &) {} // stores the locations for the given goal state
	void ClearGoal() {}
	bool IsGoalStored() const {return false;}
	void SetDiagonalCost(double val) { DIAGONAL_COST = val; }
	double GetDiagonalCost() { return DIAGONAL_COST; }
	bool FourConnected() { return connectedness==4; }
	bool FiveConnected() { return connectedness==5; }
	bool EightConnected() { return connectedness==8; }
	bool NineConnected() { return connectedness==9; }
	bool TwentyFourConnected() { return connectedness==24; }
	bool TwentyFiveConnected() { return connectedness==25; }
	bool FortyEightConnected() { return connectedness==48; }
	bool FortyNineConnected() { return connectedness==49; }
	bool AnyAngleConnected() { return connectedness>49; }
	void SetFourConnected() { connectedness=4; }
	void SetFiveConnected() { connectedness=5; }
	void SetEightConnected() { connectedness=8; }
	void SetNineConnected() { connectedness=9; }
	void SetTwentyFourConnected() { connectedness=24; }
	void SetTwentyFiveConnected() { connectedness=25; }
	void SetFortyEightConnected() { connectedness=48; }
	void SetFortyNineConnected() { connectedness=49; }
	void SetAnyAngleConnected() { connectedness=255; }
        void SetConnectedness(int c){ connectedness=c; }
        uint8_t GetConnectedness()const{ return connectedness; }
	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Mapenv\n";return oi;}
	//virtual xyLoc GetNextState(xyLoc &s, tDirection dir);
	double GetPathLength(std::vector<xyLoc> &neighbors);
        std::vector<std::vector<std::pair<xyLoc,double>>> solution;
        void findIntervals(xyLoc curNode, std::vector<std::pair<double,double>>& intervals, std::vector<double>& EAT, int w) const;
        void SetStart(xyLoc const* s){start=s;}
protected:
	GraphHeuristic *h;
        xyLoc const* start;
	Map *map;
	BaseMapOccupancyInterface *oi;
	double DIAGONAL_COST;
	double SQRT_5;
	double SQRT_10;
	double SQRT_13;
	uint8_t connectedness;
        double _h4(unsigned dx, unsigned dy, double result=0.0)const;
        double h4(const xyLoc &l1, const xyLoc &l2)const;
        double _h8(unsigned dx,unsigned dy,double result=0)const;
        double h8(const xyLoc &l1, const xyLoc &l2)const;
        double _h24(unsigned dx,unsigned dy,double result=0)const;
        double h24(const xyLoc &l1, const xyLoc &l2)const;
        double _h48(unsigned dx,unsigned dy,double result=0)const;
        double h48(const xyLoc &l1, const xyLoc &l2)const;
};

class AbsMapEnvironment : public MapEnvironment
{
public:
	AbsMapEnvironment(MapAbstraction *ma);
	virtual ~AbsMapEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { map->OpenGLDraw(); ma->OpenGLDraw(); }
	void OpenGLDraw(const xyLoc &l) const { MapEnvironment::OpenGLDraw(l); }
	void OpenGLDraw(const xyLoc& s, const tDirection &dir) const {MapEnvironment::OpenGLDraw(s,dir);}
	void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const { MapEnvironment::OpenGLDraw(l1, l2, v); }

	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"AbsMap\n";return oi;}
protected:
	MapAbstraction *ma;
};

typedef UnitSimulation<xyLoc, tDirection, MapEnvironment> UnitMapSimulation;
typedef UnitSimulation<xyLoc, tDirection, AbsMapEnvironment> UnitAbsMapSimulation;


//template<>
//void UnitSimulation<xyLoc, tDirection, MapEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}
//
//template<>
//void UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}

#endif
