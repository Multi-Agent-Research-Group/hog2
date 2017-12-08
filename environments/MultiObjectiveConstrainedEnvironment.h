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

#ifndef __hog2_glut__MultiObjectiveConstrainedEnvironment__
#define __hog2_glut__MultiObjectiveConstrainedEnvironment__

#include <iostream>

#include "MultiObjectiveEnvironment.h"
#include "Vector2D.h"
#include "VelocityObstacle.h"
#include "NonUnitTimeCAT.h"
#include "ConstrainedEnvironment.h"
#include "PositionalUtils.h"
#include "TemplateAStar.h"
#include "GridStates.h"
#include "NAMOAStar.h"

extern double agentRadius;

template <typename PhysicalEnv, typename state, typename action>
class MultiObjectiveConstrainedEnvironment : public ConstrainedEnvironment<state, action>
{
public:
	MultiObjectiveConstrainedEnvironment(MultiObjectiveEnvironment<PhysicalEnv,state,action> *m);
	virtual void AddConstraint(Constraint<state> const* c){mapEnv->GetPhysicalEnv()->AddConstraint(c);}
	//virtual void AddConstraint(Constraint<TemporalVector> const& c){mapEnv->GetPhysicalEnv()->AddConstraint(c);}
	void ClearConstraints(){mapEnv->GetPhysicalEnv()->ClearConstraints();}
        virtual std::string name()const{return mapEnv->name();}
	virtual void GetSuccessors(const state &nodeID, std::vector<state> &neighbors) const{mapEnv->GetPhysicalEnv()->GetSuccessors(nodeID,neighbors);}
	virtual void GetAllSuccessors(const state &nodeID, std::vector<state> &neighbors) const{mapEnv->GetPhysicalEnv()->GetAllSuccessors(nodeID,neighbors);}
	virtual void GetActions(const state &nodeID, std::vector<action> &actions) const{mapEnv->GetPhysicalEnv()->GetActions(nodeID,actions);}
	virtual action GetAction(const state &s1, const state &s2) const{return mapEnv->GetPhysicalEnv()->GetAction(s1,s2);}
	virtual void ApplyAction(state &s, action a) const{mapEnv->GetPhysicalEnv()->ApplyAction(s,a);}
	virtual void UndoAction(state &s, action a) const{mapEnv->GetPhysicalEnv()->UndoAction(s,a);}
	virtual void GetReverseActions(const state &nodeID, std::vector<action> &actions) const{mapEnv->GetPhysicalEnv()->GetReverseActions(nodeID,actions);}
	double ViolatesConstraint(const state &from, const state &to) const{return mapEnv->GetPhysicalEnv()->ViolatesConstraint(from,to);}
        void setSoftConstraintEffectiveness(double e){mapEnv->GetPhysicalEnv()->setSoftConstraintEffectiveness(e);}
	
	virtual bool InvertAction(action &a) const{return InvertAction(a);}
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const state &node1, const state &node2) const { return mapEnv->HCost(node1,node2); }
	virtual std::vector<float> HCostVector(const state &node1, const state &node2) const { return mapEnv->HCostVector(node1,node2); }
	virtual double GCost(const state &node1, const state &node2) const { return mapEnv->GCost(node1,node2); }
	virtual std::vector<float> GCostVector(const state &node1, const state &node2) const { return mapEnv->GCostVector(node1,node2); }
        virtual std::vector<float> NullVector(const state &node1, const state &node2) const{return mapEnv->NullVector(node1,node2);}
	virtual double GCost(const state &node, const action &act) const { return  mapEnv->GCost(node,act); }
	virtual bool GoalTest(const state &node, const state &goal) const { return mapEnv->GoalTest(node,goal); }
	
	virtual uint64_t GetStateHash(const state &node) const{return mapEnv->GetPhysicalEnv()->GetStateHash(node);}
        virtual void GetStateFromHash(uint64_t hash, state &s) const{mapEnv->GetPhysicalEnv()->GetStateFromHash(hash,s);}
	virtual uint64_t GetActionHash(action act) const{return mapEnv->GetPhysicalEnv()->GetActionHash(act);}
	std::vector<float> GetPathLength(std::vector<state> &neighbors)const{return mapEnv->GetPathLength(neighbors);}

	virtual void OpenGLDraw() const;
        void OpenGLDraw(const state& s, const state& t, float perc) const;
	virtual void OpenGLDraw(const state&) const;
	virtual void OpenGLDraw(const state&, const action&) const;
	virtual void GLDrawLine(const state &x, const state &y) const;
        void GLDrawPath(const std::vector<state> &p, const std::vector<state> &waypoints) const;
	virtual void SetColor(GLfloat rr, GLfloat g, GLfloat b, GLfloat t = 1.0) const { mapEnv->SetColor(rr,g,b,t);}
	virtual void GetColor(GLfloat& rr, GLfloat& g, GLfloat& b, GLfloat &t) const { mapEnv->GetColor(rr,g,b,t);}
        virtual Map* GetMap()const{return mapEnv->GetMap();}
        virtual MultiObjectiveEnvironment<PhysicalEnv,state,action>* GetEnv()const{return mapEnv;}
        bool LineOfSight(const state &x, const state &y)const{return mapEnv->GetPhysicalEnv()->LineOfSight(x,y);}
        bool collisionCheck(const state &x,const state &y,float,state const&z, state const&a, float){return true;}
private:
	double ViolatesConstraint(const state &from, const state &to, float time, float inc) const{return  mapEnv->GetPhysicalEnv()->ViolatesConstraint(from,to,time,inc);}

	std::vector<Constraint<state>> constraints;
	std::vector<Constraint<TemporalVector>> vconstraints;
        // TODO: will this work?
	MultiObjectiveEnvironment<PhysicalEnv,state,action> *mapEnv;
};

template<typename PhysicalEnv, typename state, typename action>
MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::MultiObjectiveConstrainedEnvironment(MultiObjectiveEnvironment<PhysicalEnv,state,action> *m)
{
	mapEnv = m;
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::OpenGLDraw() const
{
	mapEnv->OpenGLDraw();
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::OpenGLDraw(const state& s, const state& e, float perc) const
{
  mapEnv->OpenGLDraw(s,e,perc);
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::OpenGLDraw(const state& l) const
{
   mapEnv->OpenGLDraw(l);
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::OpenGLDraw(const state&, const action&) const
{
       std::cout << "Draw move\n";
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::GLDrawLine(const state &x, const state &y) const
{
  mapEnv->GLDrawLine(x,y);
}

template<typename PhysicalEnv, typename state, typename action>
void MultiObjectiveConstrainedEnvironment<PhysicalEnv,state,action>::GLDrawPath(const std::vector<state> &p, const std::vector<state> &waypoints) const
{
  mapEnv->GetPhysicalEnv()->GLDrawPath(p,waypoints);
}

typedef std::set<IntervalData> ConflictSet;

#define HASH_INTERVAL 1.0
#define HASH_INTERVAL_HUNDREDTHS 100

template <typename state, typename action, unsigned dim>
class LexicographicTieBreaking {
  public:
  bool operator()(const NAMOAOpenClosedData<state,dim> &ci1, const NAMOAOpenClosedData<state,dim> &ci2) const
  {
    // Check f-costs first
    for(int i(0); i<dim; ++i){
      if (!fequal(ci1.g.value[i]+ci1.h.value[i], ci2.g.value[i]+ci2.h.value[i]))
      {
        return (fgreater(ci1.g.value[i]+ci1.h.value[i], ci2.g.value[i]+ci2.h.value[i]));
      }
    }
    for(int i(0); i<dim; ++i){
      // Update the conflict avoidance table
      if(useCAT && CAT){
        // Make them non-const :)
        NAMOAOpenClosedData<state,dim>& i1(const_cast<NAMOAOpenClosedData<state,dim>&>(ci1));
        NAMOAOpenClosedData<state,dim>& i2(const_cast<NAMOAOpenClosedData<state,dim>&>(ci2));
        // Compute cumulative conflicts (if not already done)
        ConflictSet matches;
        if(i1.data.nc ==-1){
          //std::cout << "Getting NC for " << i1.data << ":\n";
          CAT->get(i1.data.t,i1.data.t+HASH_INTERVAL,matches);

          // Get number of conflicts in the parent
          state const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
          unsigned nc1(parent1?parent1->nc:0);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for(auto const& m: matches){
            if(currentAgent == m.agent) continue;
            state p;
            currentEnv->GetStateFromHash(m.hash1,p);
            //p.t=m.start;
            state n;
            currentEnv->GetStateFromHash(m.hash2,n);
            //n.t=m.stop;
            nc1+=checkForConflict(parent1,&i1.data,&p,&n);
            //if(!nc1){std::cout << "NO ";}
            //std::cout << "conflict(1): " << i1.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          i1.data.nc=nc1;
        }
        if(i2.data.nc ==-1){
          //std::cout << "Getting NC for " << i2.data << ":\n";
          CAT->get(i2.data.t,i2.data.t+HASH_INTERVAL,matches);

          // Get number of conflicts in the parent
          state const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
          unsigned nc2(parent2?parent2->nc:0);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for(auto const& m: matches){
            if(currentAgent == m.agent) continue;
            state p;
            currentEnv->GetStateFromHash(m.hash2,p);
            //p.t=m.start;
            state n;
            currentEnv->GetStateFromHash(m.hash2,n);
            //n.t=m.stop;
            nc2+=checkForConflict(parent2,&i2.data,&p,&n);
            //if(!nc2){std::cout << "NO ";}
            //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          i2.data.nc=nc2;
        }
        if(fequal(i1.data.nc,i2.data.nc)){
          if(!fequal(i1.g.value[i], i2.g.value[i]))
          {
            return (fless(i1.g.value[i], i2.g.value[i]));
          }
        }
      }else{
        if(!fequal(ci1.g.value[i], ci2.g.value[i]))
        {
          return (fless(ci1.g.value[i], ci2.g.value[i]));
        }
      }
    }
    return randomalg?rand()%2:true;
  }
    static OpenClosedInterface<state,NAMOAOpenClosedData<state,dim>>* openList;
    static SearchEnvironment<state,action>* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action, unsigned dim>
OpenClosedInterface<state,NAMOAOpenClosedData<state,dim>>* LexicographicTieBreaking<state,action,dim>::openList=0;
template <typename state, typename action, unsigned dim>
SearchEnvironment<state,action>* LexicographicTieBreaking<state,action,dim>::currentEnv=0;
template <typename state, typename action, unsigned dim>
uint8_t LexicographicTieBreaking<state,action,dim>::currentAgent=0;
template <typename state, typename action, unsigned dim>
bool LexicographicTieBreaking<state,action,dim>::randomalg=false;
template <typename state, typename action, unsigned dim>
bool LexicographicTieBreaking<state,action,dim>::useCAT=false;
template <typename state, typename action, unsigned dim>
NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* LexicographicTieBreaking<state,action,dim>::CAT=0;

template <typename state, typename action, unsigned dim>
class LexicographicGoalTieBreaking {
  public:
  bool operator()(const NAMOAOpenClosedData<state,dim> &ci1, const NAMOAOpenClosedData<state,dim> &ci2) const
  {
    // Check f-costs first
    for(int i(0); i<dim; ++i){
      //float a(std::max(0.0f,cost<dim>::goals[i]-ci1.g[i]+ci1.h[i])); // Constrain leq
      //float b(std::max(0.0f,cost<dim>::goals[i]-ci2.g[i]+ci2.h[i]));
      float a(fabs(cost<dim>::goals[i]-ci1.g[i]+ci1.h[i])); // Constrain leq
      float b(fabs(cost<dim>::goals[i]-ci2.g[i]+ci2.h[i]));
      if(!fequal(a,b)){
        return fgreater(a,b);
      }
    }
    for(int i(0); i<dim; ++i){
      // Update the conflict avoidance table
      if(useCAT && CAT){
        // Make them non-const :)
        NAMOAOpenClosedData<state,dim>& i1(const_cast<NAMOAOpenClosedData<state,dim>&>(ci1));
        NAMOAOpenClosedData<state,dim>& i2(const_cast<NAMOAOpenClosedData<state,dim>&>(ci2));
        // Compute cumulative conflicts (if not already done)
        ConflictSet matches;
        if(i1.data.nc ==-1){
          //std::cout << "Getting NC for " << i1.data << ":\n";
          CAT->get(i1.data.t,i1.data.t+HASH_INTERVAL,matches);

          // Get number of conflicts in the parent
          state const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
          unsigned nc1(parent1?parent1->nc:0);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for(auto const& m: matches){
            if(currentAgent == m.agent) continue;
            state p;
            currentEnv->GetStateFromHash(m.hash1,p);
            //p.t=m.start;
            state n;
            currentEnv->GetStateFromHash(m.hash2,n);
            //n.t=m.stop;
            nc1+=checkForConflict(parent1,&i1.data,&p,&n);
            //if(!nc1){std::cout << "NO ";}
            //std::cout << "conflict(1): " << i1.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          i1.data.nc=nc1;
        }
        if(i2.data.nc ==-1){
          //std::cout << "Getting NC for " << i2.data << ":\n";
          CAT->get(i2.data.t,i2.data.t+HASH_INTERVAL,matches);

          // Get number of conflicts in the parent
          state const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
          unsigned nc2(parent2?parent2->nc:0);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for(auto const& m: matches){
            if(currentAgent == m.agent) continue;
            state p;
            currentEnv->GetStateFromHash(m.hash2,p);
            //p.t=m.start;
            state n;
            currentEnv->GetStateFromHash(m.hash2,n);
            //n.t=m.stop;
            nc2+=checkForConflict(parent2,&i2.data,&p,&n);
            //if(!nc2){std::cout << "NO ";}
            //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          i2.data.nc=nc2;
        }
        if(fequal(i1.data.nc,i2.data.nc)){
          //float a(std::max(0.0f,cost<dim>::goals[i]-ci1.g[i])); // Constrain leq
          //float b(std::max(0.0f,cost<dim>::goals[i]-ci2.g[i]));
          float a(fabs(cost<dim>::goals[i]-ci1.g[i]));
          float b(fabs(cost<dim>::goals[i]-ci2.g[i]));
          if(!fequal(a,b)){
            return fless(a,b);
          }
        }
      }else{
        //float a(std::max(0.0f,cost<dim>::goals[i]-ci1.g[i])); // Constrain leq
        //float b(std::max(0.0f,cost<dim>::goals[i]-ci2.g[i]));
        float a(fabs(cost<dim>::goals[i]-ci1.g[i]));
        float b(fabs(cost<dim>::goals[i]-ci2.g[i]));
        if(!fequal(a,b)){
          return fless(a,b);
        }
      }
    }
    return randomalg?rand()%2:true;
  }
    static OpenClosedInterface<state,NAMOAOpenClosedData<state,dim>>* openList;
    static SearchEnvironment<state,action>* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action, unsigned dim>
OpenClosedInterface<state,NAMOAOpenClosedData<state,dim>>* LexicographicGoalTieBreaking<state,action,dim>::openList=0;
template <typename state, typename action, unsigned dim>
SearchEnvironment<state,action>* LexicographicGoalTieBreaking<state,action,dim>::currentEnv=0;
template <typename state, typename action, unsigned dim>
uint8_t LexicographicGoalTieBreaking<state,action,dim>::currentAgent=0;
template <typename state, typename action, unsigned dim>
bool LexicographicGoalTieBreaking<state,action,dim>::randomalg=false;
template <typename state, typename action, unsigned dim>
bool LexicographicGoalTieBreaking<state,action,dim>::useCAT=false;
template <typename state, typename action, unsigned dim>
NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* LexicographicGoalTieBreaking<state,action,dim>::CAT=0;

template <typename state, typename action, unsigned dim>
class DominanceTieBreaking {
  public:
  bool operator()(const NAMOAOpenClosedData<state,dim> &ci1, const NAMOAOpenClosedData<state,dim> &ci2) const
  {
    // Check f-costs first
    cost<dim> a(ci1.g+ci1.h);
    cost<dim> b(ci2.g+ci2.h);
    if(a!=b)
    {
      //std::cout << ci1.data << "!=" << ci2.data << "\n";
      return a>b;
    }
    //std::cout << ci1.data << "==" << ci2.data << "\n";
    // Update the conflict avoidance table
    if(useCAT && CAT){
      // Make them non-const :)
      NAMOAOpenClosedData<state,dim>& i1(const_cast<NAMOAOpenClosedData<state,dim>&>(ci1));
      NAMOAOpenClosedData<state,dim>& i2(const_cast<NAMOAOpenClosedData<state,dim>&>(ci2));
      // Compute cumulative conflicts (if not already done)
      ConflictSet matches;
      if(i1.data.nc ==-1){
        //std::cout << "Getting NC for " << i1.data << ":\n";
        CAT->get(i1.data.t,i1.data.t+HASH_INTERVAL,matches);

        // Get number of conflicts in the parent
        state const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
        unsigned nc1(parent1?parent1->nc:0);
        //std::cout << "  matches " << matches.size() << "\n";

        // Count number of conflicts
        for(auto const& m: matches){
          if(currentAgent == m.agent) continue;
          state p;
          currentEnv->GetStateFromHash(m.hash1,p);
          //p.t=m.start;
          state n;
          currentEnv->GetStateFromHash(m.hash2,n);
          //n.t=m.stop;
          nc1+=checkForConflict(parent1,&i1.data,&p,&n);
          //if(!nc1){std::cout << "NO ";}
          //std::cout << "conflict(1): " << i1.data << " " << n << "\n";
        }
        // Set the number of conflicts in the data object
        i1.data.nc=nc1;
      }
      if(i2.data.nc ==-1){
        //std::cout << "Getting NC for " << i2.data << ":\n";
        CAT->get(i2.data.t,i2.data.t+HASH_INTERVAL,matches);

        // Get number of conflicts in the parent
        state const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
        unsigned nc2(parent2?parent2->nc:0);
        //std::cout << "  matches " << matches.size() << "\n";

        // Count number of conflicts
        for(auto const& m: matches){
          if(currentAgent == m.agent) continue;
          state p;
          currentEnv->GetStateFromHash(m.hash2,p);
          //p.t=m.start;
          state n;
          currentEnv->GetStateFromHash(m.hash2,n);
          //n.t=m.stop;
          nc2+=checkForConflict(parent2,&i2.data,&p,&n);
          //if(!nc2){std::cout << "NO ";}
          //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
        }
        // Set the number of conflicts in the data object
        i2.data.nc=nc2;
      }
      if(fequal(i1.data.nc,i2.data.nc)){
        if(ci1.g!=ci2.g){
          return ci1.g<ci2.g;
        }
      }
    }else{
      if(ci1.g!=ci2.g){
        return ci1.g<ci2.g;
      }
    }
    return randomalg?rand()%2:true;
  }
    static OpenClosedInterface<state,NAMOAOpenClosedData<state,dim>>* openList;
    static SearchEnvironment<state,action>* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action, unsigned dim>
OpenClosedInterface<state,NAMOAOpenClosedData<state,dim>>* DominanceTieBreaking<state,action,dim>::openList=0;
template <typename state, typename action, unsigned dim>
SearchEnvironment<state,action>* DominanceTieBreaking<state,action,dim>::currentEnv=0;
template <typename state, typename action, unsigned dim>
uint8_t DominanceTieBreaking<state,action,dim>::currentAgent=0;
template <typename state, typename action, unsigned dim>
bool DominanceTieBreaking<state,action,dim>::randomalg=false;
template <typename state, typename action, unsigned dim>
bool DominanceTieBreaking<state,action,dim>::useCAT=false;
template <typename state, typename action, unsigned dim>
NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* DominanceTieBreaking<state,action,dim>::CAT=0;
#endif /* defined(__hog2_glut__MultiObjectiveConstrainedEnvironment__) */
