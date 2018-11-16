/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
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
#include "GridStates.h"

#include <cassert>


/*static bool operator==(const xyztLoc &l1, const xyztLoc &l2) {
	return (l1.x == l2.x) && (l1.y == l2.y) && (l1.z == l2.z);
}

static bool operator!=(const xyztLoc &l1, const xyztLoc &l2) {
	return (l1.x != l2.x) || (l1.y != l2.y) || (l1.z != l2.z);
}*/


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

class Grid3DEnvironment : public SearchEnvironment<xyztLoc, t3DDirection>
{
public:
	Grid3DEnvironment(Map3D *m, bool wait=false,Map3D::AgentType a=Map3D::air);
	//Grid3DEnvironment(Grid3DEnvironment *);
	virtual ~Grid3DEnvironment();
        virtual std::string name()const{return std::string("Grid3DEnvironment");}
	void SetGraphHeuristic(GraphHeuristic *h);
	GraphHeuristic *GetGraphHeuristic();
	virtual void GetSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const;
	void GetActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const;
	t3DDirection GetAction(const xyztLoc &s1, const xyztLoc &s2) const;
	virtual void ApplyAction(xyztLoc &s, t3DDirection dir) const;
	virtual bool InvertAction(t3DDirection &a) const;

//	bool Contractable(const xyztLoc &where);
	
	virtual double HCost(const xyztLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for Grid3DEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyztLoc &node1, const xyztLoc &node2) const;
	virtual double GCost(const xyztLoc &node1, const xyztLoc &node2) const;
	virtual double GCost(const xyztLoc &node1, const t3DDirection &act) const;
	virtual bool LineOfSight(const xyztLoc &node, const xyztLoc &goal) const;
	virtual bool LineOfSight(const std::pair<xyztLoc,xyztLoc> &node, const std::pair<xyztLoc,xyztLoc> &goal) const;
	bool GoalTest(const xyztLoc &node, const xyztLoc &goal) const;

	bool GoalTest(const xyztLoc &){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for Grid3DEnvironment\n");
		exit(1); return false;}

	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const xyztLoc &node) const;
	uint64_t GetActionHash(t3DDirection act) const;
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const xyztLoc &l) const;
	virtual void OpenGLDraw(const xyztLoc &l1, const xyztLoc &l2, float v) const;
	virtual void OpenGLDraw(const xyztLoc &, const t3DDirection &) const;
	virtual void GLLabelState(const xyztLoc &, const char *) const;
	virtual void GLLabelState(const xyztLoc &s, const char *str, double scale) const;
	virtual void GLDrawLine(const xyztLoc &x, const xyztLoc &y) const;

	virtual void Draw() const;
	virtual void Draw(const xyztLoc &l) const;
	virtual void DrawLine(const xyztLoc &x, const xyztLoc &y, double width = 1.0) const;

	
	//virtual void OpenGLDraw(const xyztLoc &, const t3DDirection &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyztLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map3D* GetMap() const { return map; }

	void SetZeroConnected() { connectedness=0; }
	void SetOneConnected() { connectedness=1; }
	void SetTwoConnected() { connectedness=2; }
	void SetThreeConnected() { connectedness=3; }
        void SetSurface() {agentType=Map3D::surface;}
        void SetGround() {agentType=Map3D::ground;}
        void SetAir() {agentType=Map3D::air;}
        uint8_t GetConnectedness()const{return connectedness;}
	//virtual xyztLoc GetNextState(xyztLoc &s, t3DDirection dir);
	double GetPathLength(std::vector<xyztLoc> &neighbors);
        std::vector<std::vector<std::pair<xyztLoc,double>>> solution;
        void findIntervals(xyztLoc curNode, std::vector<std::pair<double,double>>& intervals, std::vector<double>& EAT, int w) const;
        inline void SetWaitAllowed(){waitAllowed=true;}
        inline void SetSurface(bool v){surface=v;}
        inline void SetMaxCost(uint64_t v){maxcost=v;}
        inline void SetUniqueCosts(bool v){uniquecosts=v;}
        Map3D::AgentType agentType=Map3D::surface;
        bool fullBranching=false;
protected:
	GraphHeuristic *h=0;
	Map3D *map=0;
	uint8_t connectedness=0;
	bool waitAllowed=true;
        bool surface=true;
        bool uniquecosts=false;
        uint64_t maxcost=0;
        static double _h4(unsigned dx, unsigned dy, double result=0.0);
        static double h4(const xyztLoc &l1, const xyztLoc &l2);
        static double _h6(unsigned dx, unsigned dy, unsigned dz, double result=0.0);
        static double h6(const xyztLoc &l1, const xyztLoc &l2);
        static double _h8(unsigned dx,unsigned dy,double result=0);
        static double h8(const xyztLoc &l1, const xyztLoc &l2);
        static double _h24(unsigned dx,unsigned dy,double result=0);
        static double h24(const xyztLoc &l1, const xyztLoc &l2);
        static double _h48(unsigned dx,unsigned dy,double result=0);
        static double h48(const xyztLoc &l1, const xyztLoc &l2);
        static double _h26(unsigned dx,unsigned dy, unsigned dz, double result=0);
        static double h26(const xyztLoc &l1, const xyztLoc &l2);
        static double _h124(unsigned dx,unsigned dy, unsigned dz, double result=0);
        static double h124(const xyztLoc &l1, const xyztLoc &l2);
};

/*class AbsGrid3DEnvironment : public Grid3DEnvironment
{
public:
	AbsGrid3DEnvironment(MapAbstraction *ma);
	virtual ~AbsGrid3DEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { map->OpenGLDraw(); ma->OpenGLDraw(); }
	void OpenGLDraw(const xyztLoc &l) const { Grid3DEnvironment::OpenGLDraw(l); }
	void OpenGLDraw(const xyztLoc& s, const t3DDirection &dir) const {Grid3DEnvironment::OpenGLDraw(s,dir);}
	void OpenGLDraw(const xyztLoc &l1, const xyztLoc &l2, float v) const { Grid3DEnvironment::OpenGLDraw(l1, l2, v); }

protected:
	MapAbstraction *ma;
};*/

typedef UnitSimulation<xyztLoc, t3DDirection, Grid3DEnvironment> UnitMap3DSimulation;
//typedef UnitSimulation<xyztLoc, t3DDirection, AbsGrid3DEnvironment> UnitAbsMap3DSimulation;


//template<>
//void UnitSimulation<xyztLoc, t3DDirection, Grid3DEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}
//
//template<>
//void UnitSimulation<xyztLoc, t3DDirection, AbsGrid3DEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}

#endif
