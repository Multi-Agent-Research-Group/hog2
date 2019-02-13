//
//  Map2DConstrainedEnvironment.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//
//  Modified by Thayne Walker 2017.
//

#ifndef __hog2_glut__Map2DConstrainedEnvironment__
#define __hog2_glut__Map2DConstrainedEnvironment__

#include <iostream>

#include "Map2DEnvironment.h"
#include "Vector2D.h"
#include "VelocityObstacle.h"
#include "ConstrainedEnvironment.h"
#include "PositionalUtils.h"
#include "TemplateAStar.h"
#include "MultiAgentStructures.h"

extern double agentRadius;

class Map2DConstrainedEnvironment : public ConstrainedEnvironment<xytLoc, tDirection>
{
public:
	Map2DConstrainedEnvironment(Map *m);
	Map2DConstrainedEnvironment(MapEnvironment *m);
	virtual void AddConstraint(Constraint<TemporalVector3D> const* c);
	void ClearConstraints();
        virtual std::string name()const{return mapEnv->name();}
	bool GetNextSuccessor(const xytLoc &currOpenNode, const xytLoc &goal, xytLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	virtual void GetSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const;
	virtual void GetAllSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const;
	virtual void GetActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const;
	virtual tDirection GetAction(const xytLoc &s1, const xytLoc &s2) const;
	virtual void ApplyAction(xytLoc &s, tDirection a) const;
	virtual void UndoAction(xytLoc &s, tDirection a) const;
	virtual void GetReverseActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const;
	//double ViolatesConstraint(const xytLoc &from, const xytLoc &to) const;
        void setSoftConstraintEffectiveness(double){}
	double GetPathLength(std::vector<xytLoc> &neighbors);
        virtual bool collisionCheck(const xytLoc &s1, const xytLoc &d1, float r1, const xytLoc &s2, const xytLoc &d2, float r2);
	
	virtual bool InvertAction(tDirection &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const xytLoc &node1, const xytLoc &node2) const;
	virtual double GCost(const xytLoc &node1, const xytLoc &node2) const { return fequal(node1.t,node2.t)?0.0:mapEnv->GCost(node1,node2); }
	virtual double GCost(const xytLoc &node, const tDirection &act) const { return  mapEnv->GCost(node,act); }
	virtual bool GoalTest(const xytLoc &node, const xytLoc &goal) const;
	
	virtual uint64_t GetStateHash(const xytLoc &node) const;
        virtual void GetStateFromHash(uint64_t hash, xytLoc &s) const;
	virtual uint64_t GetActionHash(tDirection act) const;

	virtual void OpenGLDraw() const;
        void OpenGLDraw(const xytLoc& s, const xytLoc& t, float perc) const;
	virtual void OpenGLDraw(const xytLoc&) const;
	virtual void OpenGLDraw(const xytLoc&, const tDirection&) const;
	virtual void GLDrawLine(const xytLoc &x, const xytLoc &y) const;
	virtual void GLLabelState(const xytLoc &s, const char *str) const{mapEnv->GLLabelState(s,str);}
	virtual void GLLabelState(const xytLoc &s, const char *str, double scale) const{mapEnv->GLLabelState(s,str,scale);}
        void GLDrawPath(const std::vector<xytLoc> &p, const std::vector<xytLoc> &waypoints) const;
        virtual Map* GetMap()const{return mapEnv->GetMap();}
        virtual bool LineOfSight(const xytLoc &x, const xytLoc &y)const{return mapEnv->LineOfSight(x,y) && !ViolatesConstraint(x,y);}
        void SetIgnoreTime(bool i){ignoreTime=i;}
        bool GetIgnoreTime()const{return ignoreTime;}
        void SetIgnoreHeading(bool i){ignoreHeading=i;}
        bool GetIgnoreHeading()const{return ignoreHeading;}
        inline void SetMaxTurn(float val){maxTurnAzimuth=val*HDG_RESOLUTON;}
        uint16_t maxTurnAzimuth=0;
        static const float HDG_RESOLUTON;
        MapEnvironment* GetEnv()const{return mapEnv;}
        MapEnvironment* GetMapEnv()const{return mapEnv;}
        void SetConnectedness(int c){ mapEnv->SetConnectedness(c); }
        uint8_t GetConnectedness()const{ return mapEnv->GetConnectedness(); }

private:
        bool ignoreTime;
        bool ignoreHeading;

	std::vector<Constraint<TemporalVector3D> const*> vconstraints;
	MapEnvironment *mapEnv;
};
#define HASH_INTERVAL 1.0
template <typename state, typename action>
class TieBreaking {
  public:
  bool operator()(const AStarOpenClosedData<state> &ci1, const AStarOpenClosedData<state> &ci2) const
  {
    if (fequal(ci1.g+ci1.h, ci2.g+ci2.h)) // F-cost equal
    {
      if(useCAT && CAT){
        // Make them non-const :)
        AStarOpenClosedData<state>& i1(const_cast<AStarOpenClosedData<state>&>(ci1));
        AStarOpenClosedData<state>& i2(const_cast<AStarOpenClosedData<state>&>(ci2));
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
          if(randomalg && fequal(ci1.g,ci2.g)){
            return rand()%2;
          }
          return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
        }
        return fgreater(i1.data.nc,i2.data.nc);
      }else{
        if(randomalg && fequal(ci1.g,ci2.g)){
          return rand()%2;
        }
        return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
      }
    }
    return (fgreater(ci1.g+ci1.h, ci2.g+ci2.h));
  }
    static OpenClosedInterface<state,AStarOpenClosedData<state>>* openList;
    static ConstrainedEnvironment<state,action>* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,action>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action>
OpenClosedInterface<state,AStarOpenClosedData<state>>* TieBreaking<state,action>::openList=0;
template <typename state, typename action>
ConstrainedEnvironment<state,action>* TieBreaking<state,action>::currentEnv=0;
template <typename state, typename action>
uint8_t TieBreaking<state,action>::currentAgent=0;
template <typename state, typename action>
bool TieBreaking<state,action>::randomalg=false;
template <typename state, typename action>
bool TieBreaking<state,action>::useCAT=false;
template <typename state, typename action>
NonUnitTimeCAT<state,action>* TieBreaking<state,action>::CAT=0;

#endif /* defined(__hog2_glut__Map2DConstrainedEnvironment__) */
