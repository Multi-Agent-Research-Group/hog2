/*
 *  MultiObjectiveEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MultiObjectiveEnvironment_H
#define MultiObjectiveEnvironment_H

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sstream>
#include "Map.h"
#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "BitVector.h"
#include "GraphEnvironment.h"
#include "ObjectiveEnvironment.h"
#include "GridStates.h"

#include <cassert>

template <typename PhysicalEnv, typename state, typename action>
class MultiObjectiveEnvironment : public SearchEnvironment<state, action>
{
public:
	MultiObjectiveEnvironment(std::vector<ObjectiveEnvironment<state>*> const& envs, PhysicalEnv* phys):environments(envs),physicalEnvironment(phys){}
	virtual ~MultiObjectiveEnvironment(); // Delete envs in here
        virtual std::string name()const{std::stringstream ss; ss<<"MultiObjectiveEnvironment"; return ss.str();}
	virtual void GetSuccessors(const state &nodeID, std::vector<state> &neighbors) const{physicalEnvironment->GetSuccessors(nodeID,neighbors);}
	virtual void GetReverseSuccessors(const state &nodeID, std::vector<state> &neighbors) const{GetSuccessors(nodeID,neighbors);}
	void GetActions(const state &nodeID, std::vector<action> &actions) const{GetActions(nodeID,actions);}
	action GetAction(const state &s1, const state &s2) const{return physicalEnvironment->GetAction(s1,s2);}
	virtual void ApplyAction(state &s, action dir) const{physicalEnvironment->ApplyAction(s,dir);}
	//virtual BaseMapOccupancyInterface *GetOccupancyInfo() { return nullptr; }

	virtual bool InvertAction(action &a) const{return physicalEnvironment->InvertAction(a);}

//	bool Contractable(const state &where);
	
        virtual double HCost(const state &node) const{return 0;}
        virtual double HCost(const state &node1, const state &node2) const{return 0;}
        virtual double GCost(const state &node1, const state &node2) const;
        virtual double GCost(const state &node, const action &act) const{return 0;}
	virtual std::vector<float> HCostVector(const state &node1, const state &node2) const;
	virtual std::vector<float> GCostVector(const state &node1, const state &node2) const;
        virtual std::vector<float> NullVector(const state &node1, const state &node2) const{return std::vector<float>(environments.size());}
	//virtual std::vector<float> GCost(const state &node1, const action &act) const;
        bool LineOfSight(const state &node, const state &goal) const{return physicalEnvironment->LineOfSight(node,goal);}
	bool GoalTest(const state &node, const state &goal) const;

	uint64_t GetStateHash(const state &node) const{return physicalEnvironment->GetStateHash(node);}
	uint64_t GetActionHash(action act) const{return physicalEnvironment->GetActionHash(act);}
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const state &l) const{physicalEnvironment->OpenGLDraw(l);}
	virtual void OpenGLDraw(const state &l1, const state &l2, float v) const{physicalEnvironment->OpenGLDraw(l1,l2,v);}
	virtual void OpenGLDraw(const state &l, const action &d) const{physicalEnvironment->OpenGLDraw(l,d);}
	//virtual void GLLabelState(const state &s, const char *str) const{physicalEnvironment->OpenGLDraw(s,str);}
	//virtual void GLLabelState(const state &s, const char *str, double scale) const{physicalEnvironment->OpenGLDraw(s,str,scale);}
	virtual void GLDrawLine(const state &x, const state &y) const{physicalEnvironment->GLDrawLine(x,y);}
	virtual void SetColor(GLfloat rr, GLfloat g, GLfloat b, GLfloat t = 1.0) const { physicalEnvironment->SetColor(rr,g,b,t);}
	virtual void GetColor(GLfloat& rr, GLfloat& g, GLfloat& b, GLfloat &t) const { physicalEnvironment->GetColor(rr,g,b,t);}
	
	Map* GetMap() const { return physicalEnvironment->GetMap(); }

	void StoreGoal(state &l) {physicalEnvironment->StoreGoal(l);} // stores the locations for the given goal state
	void ClearGoal() {physicalEnvironment->ClearGoal();}
	bool IsGoalStored() const {return physicalEnvironment->IsGoalStored();}
	std::vector<float> GetPathLength(std::vector<state> &neighbors)const;
        std::vector<std::vector<std::pair<state,double>>> solution;
        void SetStart(state const* s){physicalEnvironment->SetStart(s);}
        PhysicalEnv* GetPhysicalEnv()const{return physicalEnvironment;}
protected:
        std::vector<ObjectiveEnvironment<state>*> environments;
        PhysicalEnv* physicalEnvironment; // For some special functions like line of sight
};

template <typename PhysicalEnv, typename state, typename action>
class AbsMultiObjectiveEnvironment : public MultiObjectiveEnvironment<PhysicalEnv, state, action>
{
public:
	AbsMultiObjectiveEnvironment(MapAbstraction *ma);
	virtual ~AbsMultiObjectiveEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { /*map->OpenGLDraw();*/ ma->OpenGLDraw(); }
	void OpenGLDraw(const state &l) const { MultiObjectiveEnvironment<PhysicalEnv, state, action>::OpenGLDraw(l); }
	void OpenGLDraw(const state& s, const action &dir) const {MultiObjectiveEnvironment<PhysicalEnv, state, action>::OpenGLDraw(s,dir);}
	void OpenGLDraw(const state &l1, const state &l2, float v) const { MultiObjectiveEnvironment<PhysicalEnv, state, action>::OpenGLDraw(l1, l2, v); }

	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"AbsMap\n";return oi;}
protected:
	MapAbstraction *ma;
};

template<typename PhysicalEnv, typename state, typename action>
MultiObjectiveEnvironment<PhysicalEnv,state,action>::~MultiObjectiveEnvironment()
{
  for(auto env:environments){
    delete env;
  }
}

template<typename PhysicalEnv, typename state, typename action>
std::vector<float> MultiObjectiveEnvironment<PhysicalEnv,state,action>::HCostVector(const state &l1, const state &l2)const{
  std::vector<float> result;
  for(auto env:environments){
    result.push_back(env->HCost(l1,l2));
  }
  return result;
}

/*template<typename PhysicalEnv, typename state, typename action>
std::vector<float> MultiObjectiveEnvironment<PhysicalEnv,state,action>::GCost(const state &l, const action &act) const
{
  assert(!"Not implemented");
  return std::vector<float>(0);
}*/

template<typename PhysicalEnv, typename state, typename action>
double MultiObjectiveEnvironment<PhysicalEnv,state,action>::GCost(const state &l1, const state &l2) const
{
  double result(0.0);
  for(auto env:environments){
    result += env->GCost(l1,l2);
  }
  return result;
}

template<typename PhysicalEnv, typename state, typename action>
std::vector<float> MultiObjectiveEnvironment<PhysicalEnv,state,action>::GCostVector(const state &l1, const state &l2) const
{
  std::vector<float> result;
  for(auto env:environments){
    result.push_back(env->GCost(l1,l2));
  }
  return result;
}

template<typename PhysicalEnv, typename state, typename action>
bool MultiObjectiveEnvironment<PhysicalEnv,state,action>::GoalTest(const state &node, const state &goal) const
{
  // This could have many criteria, but for now...
  return physicalEnvironment->GoalTest(node,goal);
}

template<typename PhysicalEnv, typename state, typename action>
AbsMultiObjectiveEnvironment<PhysicalEnv,state,action>::AbsMultiObjectiveEnvironment(MapAbstraction *_ma)
:MultiObjectiveEnvironment<PhysicalEnv,state,action>()
{
	ma = _ma;
	
}

template<typename PhysicalEnv, typename state, typename action>
AbsMultiObjectiveEnvironment<PhysicalEnv,state,action>::~AbsMultiObjectiveEnvironment()
{
	//map = 0;
	//delete ma;
}

template<typename PhysicalEnv, typename state, typename action>
std::vector<float> MultiObjectiveEnvironment<PhysicalEnv,state,action>::GetPathLength(std::vector<state> &neighbors)const{
  if(neighbors.size()>1){
    std::vector<float> result(GCostVector(neighbors[0],neighbors[1]));
    for(auto s(neighbors.begin()+2);s!=neighbors.end();++s){
      std::vector<float> cst(GCostVector(*(s-1),*s));
      for(int i(0); i<cst.size(); ++i){
        result[i]+=cst[i];
      }
    }
    return result;
  }else if(neighbors.size()==1){
    return GCostVector(neighbors[0],neighbors[0]);
  }
  return std::vector<float>(0);
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveEnvironment<PhysicalEnv,state,action>::OpenGLDraw()const{
  physicalEnvironment->OpenGLDraw();
  for(auto env:environments){
    if(env != physicalEnvironment){
      env->OpenGLDraw(physicalEnvironment->GetMap());
    }
  }
}
//typedef UnitSimulation<state, action, MultiObjectiveEnvironment<PhysicalEnv>> UnitMOSimulation;
//typedef UnitSimulation<state, action, AbsMultiObjectiveEnvironment<PhysicalEnv>> UnitAbsMOSimulation;

#endif
