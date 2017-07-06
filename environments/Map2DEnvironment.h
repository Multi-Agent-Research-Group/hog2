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
#include "Map.h"
#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "BitVector.h"
#include "GraphEnvironment.h"

#include <cassert>

//#include "BaseMapOccupancyInterface.h"

struct xyLoc {
public:
	xyLoc() { x = -1; y = -1; }
	xyLoc(uint16_t _x, uint16_t _y) :x(_x), y(_y) {}
        bool operator<(xyLoc const& other)const{return x==other.x?y<other.y:x<other.x;}
	uint16_t x;
	uint16_t y;
};

struct xyLocHash
{
	std::size_t operator()(const xyLoc & x) const
	{
		return (x.x<<16)|(x.y);
	}
};

struct AANode : xyLoc {
  AANode(uint16_t _x, uint16_t _y):xyLoc(_x,_y),F(0),g(0),Parent(nullptr){}
  AANode():xyLoc(0,0),F(0),g(0),Parent(nullptr){}
  float   F;
  float   g;
  AANode*   Parent;
  std::pair<double,double> interval;
};


static std::ostream& operator <<(std::ostream & out, const xyLoc &loc)
{
	out << "(" << loc.x << ", " << loc.y << ")";
	return out;
}

static bool operator==(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x == l2.x) && (l1.y == l2.y);
}

static bool operator!=(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x != l2.x) || (l1.y != l2.y);
}


enum tDirection {
	kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE,
	kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE, kAll = 0xFFF,
		kNN=0x80,
		kSS=0x40,
		kEE=0x20,
		kWW=0x10,
		kNNE=kNN|kE,
		kNEE=kN|kEE,
		kNNEE=kNN|kEE,
		kSSE=kSS|kE,
		kSEE=kS|kEE,
		kSSEE=kSS|kEE,
		kSSW=kSS|kW,
		kSWW=kS|kWW,
		kSSWW=kSS|kWW,
		kNNW=kNN|kW,
		kNWW=kN|kWW,
		kNNWW=kNN|kWW,

		kNNN=0x800,
		kSSS=0x400,
		kEEE=0x200,
		kWWW=0x100,
		kNNNE=kNNN|kE,
		kNEEE=kN|kEEE,
		kNNNEE=kNNN|kEE,
		kNNEEE=kNN|kEEE,
		kNNNEEE=kNNN|kEEE,
		kSSSE=kSSS|kE,
		kSEEE=kS|kEEE,
		kSSEEE=kSS|kEEE,
		kSSSEE=kSSS|kEE,
		kSSSEEE=kSSS|kEEE,
		kSSSW=kSSS|kW,
		kSWWW=kS|kWWW,
		kSSWWW=kSS|kWWW,
		kSSSWW=kSSS|kWW,
		kSSSWWW=kSSS|kWWW,
		kNNNW=kNNN|kW,
		kNWWW=kN|kWWW,
		kNNNWW=kNNN|kWW,
		kNNWWW=kNN|kWWW,
		kNNNWWW=kNNN|kWWW
};

class BaseMapOccupancyInterface : public OccupancyInterface<xyLoc,tDirection>
{
public:
	BaseMapOccupancyInterface(Map* m);
	virtual ~BaseMapOccupancyInterface();
	virtual void SetStateOccupied(const xyLoc&, bool);
	virtual bool GetStateOccupied(const xyLoc&);
	virtual bool CanMove(const xyLoc&, const xyLoc&);
	virtual void MoveUnitOccupancy(const xyLoc &, const xyLoc&);

private:
	//BitVector *bitvec; /// For each map position, set if occupied
	std::vector<bool> bitvec;
	long mapWidth; /// Used to compute index into bitvector
	long mapHeight; /// used to compute index into bitvector

	long CalculateIndex(uint16_t x, uint16_t y);
};


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
	virtual void GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
	virtual void GetReverseSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const{GetSuccessors(nodeID,neighbors);}
	bool GetNextSuccessor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext4Successor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
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
	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Mapenv\n";return oi;}
	//virtual xyLoc GetNextState(xyLoc &s, tDirection dir);
	double GetPathLength(std::vector<xyLoc> &neighbors);
        std::vector<std::vector<std::pair<xyLoc,double>>> solution;
        void findIntervals(xyLoc curNode, std::vector<std::pair<double,double>>& intervals, std::vector<double>& EAT, int w) const;
protected:
	GraphHeuristic *h;
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
