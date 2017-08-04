/*
 *  Grid3dEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef GRID3DENVIRONMENT_H
#define GRID3DENVIRONMENT_H

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "Map3d.h"
#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "BitVector.h"
#include "GraphEnvironment.h"

#include <cassert>


struct xyzLoc {
public:
	xyzLoc():x(-1),y(-1),z(-1),v(-1){}
	xyzLoc(uint16_t _x, uint16_t _y, uint16_t _z, uint16_t _v=0) :x(_x), y(_y) ,z(_z), v(_v){}
        bool operator<(xyzLoc const& other)const{return x==other.x?(y==other.y?(z==other.z?v<other.v:z<other.z):y<other.y):x<other.x;}
	uint16_t x;
	uint16_t y;
	uint16_t z;
	uint16_t v;
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
	return (l1.x != l2.x) || (l1.y != l2.y) || (l1.z != l2.z);
}


enum t3DDirection {
  kU
};

/*
enum t3DDirection {
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
};*/

class Grid3DEnvironment : public SearchEnvironment<xyzLoc, t3DDirection>
{
public:
	Grid3DEnvironment(Map3D *m, bool=false);
	Grid3DEnvironment(Grid3DEnvironment *);
	virtual ~Grid3DEnvironment();
	void SetGraphHeuristic(GraphHeuristic *h);
	GraphHeuristic *GetGraphHeuristic();
	virtual void GetSuccessors(const xyzLoc &nodeID, std::vector<xyzLoc> &neighbors) const;
	void GetActions(const xyzLoc &nodeID, std::vector<t3DDirection> &actions) const;
	t3DDirection GetAction(const xyzLoc &s1, const xyzLoc &s2) const;
	virtual void ApplyAction(xyzLoc &s, t3DDirection dir) const;
	virtual bool InvertAction(t3DDirection &a) const;

//	bool Contractable(const xyzLoc &where);
	
	virtual double HCost(const xyzLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for Grid3DEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyzLoc &node1, const xyzLoc &node2) const;
	virtual double GCost(const xyzLoc &node1, const xyzLoc &node2) const;
	virtual double GCost(const xyzLoc &node1, const t3DDirection &act) const;
        bool LineOfSight(const xyzLoc &node, const xyzLoc &goal) const;
	bool GoalTest(const xyzLoc &node, const xyzLoc &goal) const;

	bool GoalTest(const xyzLoc &){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for Grid3DEnvironment\n");
		exit(1); return false;}

	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const xyzLoc &node) const;
	uint64_t GetActionHash(t3DDirection act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const xyzLoc &l) const;
	virtual void OpenGLDraw(const xyzLoc &l1, const xyzLoc &l2, float v) const;
	virtual void OpenGLDraw(const xyzLoc &, const t3DDirection &) const;
	virtual void GLLabelState(const xyzLoc &, const char *) const;
	virtual void GLLabelState(const xyzLoc &s, const char *str, double scale) const;
	virtual void GLDrawLine(const xyzLoc &x, const xyzLoc &y) const;

	virtual void Draw() const;
	virtual void Draw(const xyzLoc &l) const;
	virtual void DrawLine(const xyzLoc &x, const xyzLoc &y, double width = 1.0) const;

	
	//virtual void OpenGLDraw(const xyzLoc &, const t3DDirection &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyzLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map3D* GetMap() const { return map; }

	void StoreGoal(xyzLoc &) {} // stores the locations for the given goal state
	void ClearGoal() {}
	bool IsGoalStored() const {return false;}
	void SetZeroConnected() { connectedness=0; }
	void SetOneConnected() { connectedness=1; }
	void SetTwoConnected() { connectedness=2; }
	void SetThreeConnected() { connectedness=3; }
        uint8_t GetConnectedness()const{return connectedness;}
	//virtual xyzLoc GetNextState(xyzLoc &s, t3DDirection dir);
	double GetPathLength(std::vector<xyzLoc> &neighbors);
        std::vector<std::vector<std::pair<xyzLoc,double>>> solution;
        void findIntervals(xyzLoc curNode, std::vector<std::pair<double,double>>& intervals, std::vector<double>& EAT, int w) const;
protected:
	GraphHeuristic *h;
	Map3D *map;
	uint8_t connectedness;
	bool waitAllowed;
};

/*class AbsGrid3DEnvironment : public Grid3DEnvironment
{
public:
	AbsGrid3DEnvironment(MapAbstraction *ma);
	virtual ~AbsGrid3DEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { map->OpenGLDraw(); ma->OpenGLDraw(); }
	void OpenGLDraw(const xyzLoc &l) const { Grid3DEnvironment::OpenGLDraw(l); }
	void OpenGLDraw(const xyzLoc& s, const t3DDirection &dir) const {Grid3DEnvironment::OpenGLDraw(s,dir);}
	void OpenGLDraw(const xyzLoc &l1, const xyzLoc &l2, float v) const { Grid3DEnvironment::OpenGLDraw(l1, l2, v); }

protected:
	MapAbstraction *ma;
};*/

typedef UnitSimulation<xyzLoc, t3DDirection, Grid3DEnvironment> UnitMap3DSimulation;
//typedef UnitSimulation<xyzLoc, t3DDirection, AbsGrid3DEnvironment> UnitAbsMap3DSimulation;


//template<>
//void UnitSimulation<xyzLoc, t3DDirection, Grid3DEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}
//
//template<>
//void UnitSimulation<xyzLoc, t3DDirection, AbsGrid3DEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}

#endif
