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

struct xyzLoc {
public:
	xyzLoc():x(-1),y(-1),z(-1){}
	xyzLoc(uint16_t _x, uint16_t _y, uint16_t _z) :x(_x), y(_y) ,z(_z){}
        bool operator<(xyzLoc const& other)const{return x==other.x?(y==other.y?z<other.z:y<other.y):x<other.x;}
	uint16_t x;
	uint16_t y;
	uint16_t z;
};

struct xyzLocHash
{
	std::size_t operator()(const xyzLoc & x) const
	{
		return (x.x<<32)|(x.y<<16)|(x.z);
	}
};

static std::ostream& operator <<(std::ostream & out, const xyzLoc &loc)
{
	out << "(" << loc.x << "," << loc.y << "," << loc.z << ")";
	return out;
}

static bool operator==(const xyzLoc &l1, const xyzLoc &l2) {
	return (l1.x == l2.x) && (l1.y == l2.y) && (l1.z == l2.z);
}

static bool operator!=(const xyzLoc &l1, const xyzLoc &l2) {
	return (l1.x != l2.x) || (l1.y != l2.y) && (l1.z != l2.z);
}


enum tDirection {
  kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE,
  kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE, kAll=0x3FFF,
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
  kNNNWWW=kNNN|kWWW,

  kU=0x1000,
  kUN=kU|kN, kUS=kU|kS, kUE=kU|kE, kUW=kU|kW, kUNW=kU|kNW, kUNE=kU|kNE
  kUSE=kU|kSE, kUSW=kU|kSW,
  kUNN=kU|kNN,
  kUSS=kU|kSS,
  kUEE=kU|kEE,
  kUWW=kU|kWW,
  kUNNE=kU|kNN|kE,
  kUNEE=kU|kN|kEE,
  kUNNEE=kU|kNN|kEE,
  kUSSE=kU|kSS|kE,
  kUSEE=kU|kS|kEE,
  kUSSEE=kU|kSS|kEE,
  kUSSW=kU|kSS|kW,
  kUSWW=kU|kS|kWW,
  kUSSWW=kU|kSS|kWW,
  kUNNW=kU|kNN|kW,
  kUNWW=kU|kN|kWW,
  kUNNWW=kU|kNN|kWW,
  kUNNN=kU|kNNN,
  kUSSS=kU|kSSS,
  kUEEE=kU|kEEE,
  kUWWW=kU|kWWW,
  kUNNNE=kU|kNNN|kE,
  kUNEEE=kU|kN|kEEE,
  kUNNNEE=kU|kNNN|kEE,
  kUNNEEE=kU|kNN|kEEE,
  kUNNNEEE=kU|kNNN|kEEE,
  kUSSSE=kU|kSSS|kE,
  kUSEEE=kU|kS|kEEE,
  kUSSEEE=kU|kSS|kEEE,
  kUSSSEE=kU|kSSS|kEE,
  kUSSSEEE=kU|kSSS|kEEE,
  kUSSSW=kU|kSSS|kW,
  kUSWWW=kU|kS|kWWW,
  kUSSWWW=kU|kSS|kWWW,
  kUSSSWW=kU|kSSS|kWW,
  kUSSSWWW=kU|kSSS|kWWW,
  kUNNNW=kU|kNNN|kW,
  kUNWWW=kU|kN|kWWW,
  kUNNNWW=kU|kNNN|kWW,
  kUNNWWW=kU|kNN|kWWW,
  kUNNNWWW=kU|kNNN|kWWW,

  kD=0x2000,
  kDN=kD|kN, kDS=kD|kS, kDE=kD|kE, kDW=kD|kW, kDNW=kD|kNW, kDNE=kD|kNE
  kDSE=kD|kSE, kDSW=kD|kSW,
  kDNN=kD|kNN,
  kDSS=kD|kSS,
  kDEE=kD|kEE,
  kDWW=kD|kWW,
  kDNNE=kD|kNN|kE,
  kDNEE=kD|kN|kEE,
  kDNNEE=kD|kNN|kEE,
  kDSSE=kD|kSS|kE,
  kDSEE=kD|kS|kEE,
  kDSSEE=kD|kSS|kEE,
  kDSSW=kD|kSS|kW,
  kDSWW=kD|kS|kWW,
  kDSSWW=kD|kSS|kWW,
  kDNNW=kD|kNN|kW,
  kDNWW=kD|kN|kWW,
  kDNNWW=kD|kNN|kWW,
  kDNNN=kD|kNNN,
  kDSSS=kD|kSSS,
  kDEEE=kD|kEEE,
  kDWWW=kD|kWWW,
  kDNNNE=kD|kNNN|kE,
  kDNEEE=kD|kN|kEEE,
  kDNNNEE=kD|kNNN|kEE,
  kDNNEEE=kD|kNN|kEEE,
  kDNNNEEE=kD|kNNN|kEEE,
  kDSSSE=kD|kSSS|kE,
  kDSEEE=kD|kS|kEEE,
  kDSSEEE=kD|kSS|kEEE,
  kDSSSEE=kD|kSSS|kEE,
  kDSSSEEE=kD|kSSS|kEEE,
  kDSSSW=kD|kSSS|kW,
  kDSWWW=kD|kS|kWWW,
  kDSSWWW=kD|kSS|kWWW,
  kDSSSWW=kD|kSSS|kWW,
  kDSSSWWW=kD|kSSS|kWWW,
  kDNNNW=kD|kNNN|kW,
  kDNWWW=kD|kN|kWWW,
  kDNNNWW=kD|kNNN|kWW,
  kDNNWWW=kD|kNN|kWWW,
  kDNNNWWW=kD|kNNN|kWWW
};

class BaseMapOccupancyInterface : public OccupancyInterface<xyzLoc,tDirection>
{
public:
	BaseMapOccupancyInterface(Map* m);
	virtual ~BaseMapOccupancyInterface();
	virtual void SetStateOccupied(const xyzLoc&, bool);
	virtual bool GetStateOccupied(const xyzLoc&);
	virtual bool CanMove(const xyzLoc&, const xyzLoc&);
	virtual void MoveUnitOccupancy(const xyzLoc &, const xyzLoc&);

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




//typedef OccupancyInterface<xyzLoc, tDirection> BaseMapOccupancyInterface;


class Grid3DEnvironment : public SearchEnvironment<xyzLoc, tDirection>
{
public:
	Grid3DEnvironment(Map *m, bool useOccupancy = false);
	Grid3DEnvironment(Grid3DEnvironment *);
	virtual ~Grid3DEnvironment();
	void SetGraphHeuristic(GraphHeuristic *h);
	GraphHeuristic *GetGraphHeuristic();
	virtual void GetSuccessors(const xyzLoc &nodeID, std::vector<xyzLoc> &neighbors) const;
	bool GetNextSuccessor(const xyzLoc &currOpenNode, const xyzLoc &goal, xyzLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext4Successor(const xyzLoc &currOpenNode, const xyzLoc &goal, xyzLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext8Successor(const xyzLoc &currOpenNode, const xyzLoc &goal, xyzLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	void GetActions(const xyzLoc &nodeID, std::vector<tDirection> &actions) const;
	tDirection GetAction(const xyzLoc &s1, const xyzLoc &s2) const;
	virtual void ApplyAction(xyzLoc &s, tDirection dir) const;
	virtual BaseMapOccupancyInterface *GetOccupancyInfo() { return oi; }

	virtual bool InvertAction(tDirection &a) const;

//	bool Contractable(const xyzLoc &where);
	
	virtual double HCost(const xyzLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for Grid3DEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyzLoc &node1, const xyzLoc &node2) const;
	virtual double GCost(const xyzLoc &node1, const xyzLoc &node2) const;
	virtual double GCost(const xyzLoc &node1, const tDirection &act) const;
        bool LineOfSight(const xyzLoc &node, const xyzLoc &goal) const;
	bool GoalTest(const xyzLoc &node, const xyzLoc &goal) const;

	bool GoalTest(const xyzLoc &){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for Grid3DEnvironment\n");
		exit(1); return false;}

	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const xyzLoc &node) const;
	uint64_t GetActionHash(tDirection act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const xyzLoc &l) const;
	virtual void OpenGLDraw(const xyzLoc &l1, const xyzLoc &l2, float v) const;
	virtual void OpenGLDraw(const xyzLoc &, const tDirection &) const;
	virtual void GLLabelState(const xyzLoc &, const char *) const;
	virtual void GLLabelState(const xyzLoc &s, const char *str, double scale) const;
	virtual void GLDrawLine(const xyzLoc &x, const xyzLoc &y) const;
	
	std::string SVGHeader();
	std::string SVGDraw();
	std::string SVGDraw(const xyzLoc &);
	std::string SVGLabelState(const xyzLoc &, const char *, double scale) const;
	std::string SVGDrawLine(const xyzLoc &x, const xyzLoc &y, int width=1) const;
	std::string SVGFrameRect(int left, int top, int right, int bottom, int width = 1);

	virtual void Draw() const;
	virtual void Draw(const xyzLoc &l) const;
	virtual void DrawLine(const xyzLoc &x, const xyzLoc &y, double width = 1.0) const;

	
	//virtual void OpenGLDraw(const xyzLoc &, const tDirection &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyzLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map* GetMap() const { return map; }

	virtual void GetNextState(const xyzLoc &currents, tDirection dir, xyzLoc &news) const;

	void StoreGoal(xyzLoc &) {} // stores the locations for the given goal state
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
	void SetTwentyFiveonnected() { connectedness=25; }
	void SetFortyEightConnected() { connectedness=48; }
	void SetFortyNineConnected() { connectedness=49; }
	void SetAnyAngleConnected() { connectedness=255; }
	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Mapenv\n";return oi;}
	//virtual xyzLoc GetNextState(xyzLoc &s, tDirection dir);
	double GetPathLength(std::vector<xyzLoc> &neighbors);
        std::vector<std::vector<std::pair<xyzLoc,double>>> solution;
        void findIntervals(xyzLoc curNode, std::vector<std::pair<double,double>>& intervals, std::vector<double>& EAT, int w) const;
protected:
	GraphHeuristic *h;
	Map *map;
	BaseMapOccupancyInterface *oi;
	double DIAGONAL_COST;
	double SQRT_5;
	double SQRT_10;
	double SQRT_13;
	uint8_t connectedness;
};

class AbsGrid3DEnvironment : public Grid3DEnvironment
{
public:
	AbsGrid3DEnvironment(MapAbstraction *ma);
	virtual ~AbsGrid3DEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { map->OpenGLDraw(); ma->OpenGLDraw(); }
	void OpenGLDraw(const xyzLoc &l) const { Grid3DEnvironment::OpenGLDraw(l); }
	void OpenGLDraw(const xyzLoc& s, const tDirection &dir) const {Grid3DEnvironment::OpenGLDraw(s,dir);}
	void OpenGLDraw(const xyzLoc &l1, const xyzLoc &l2, float v) const { Grid3DEnvironment::OpenGLDraw(l1, l2, v); }

	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"AbsMap\n";return oi;}
protected:
	MapAbstraction *ma;
};

typedef UnitSimulation<xyzLoc, tDirection, Grid3DEnvironment> UnitMapSimulation;
typedef UnitSimulation<xyzLoc, tDirection, AbsGrid3DEnvironment> UnitAbsMapSimulation;


//template<>
//void UnitSimulation<xyzLoc, tDirection, Grid3DEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}
//
//template<>
//void UnitSimulation<xyzLoc, tDirection, AbsGrid3DEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}

#endif
