//
//  Map2DConstrainedEnvironment.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__Map2DConstrainedEnvironment__
#define __hog2_glut__Map2DConstrainedEnvironment__

#include <iostream>

#include "Map2DEnvironment.h"
#include "Vector2D.h"
#include "VelocityObstacle.h"
#include "NonUnitTimeCAT.h"
#include "ConstrainedEnvironment.h"
#include "PositionalUtils.h"
#include "TemplateAStar.h"

struct xytLoc : xyLoc {
	xytLoc(xyLoc loc, float time):xyLoc(loc), t(time) {}
	xytLoc(uint16_t _x, uint16_t _y, float time):xyLoc(_x,_y), t(time) {}
	xytLoc():xyLoc(),t(0),nc(-1){}
	float t;
        int16_t nc; // Number of conflicts, for conflict avoidance table
        operator Vector2D()const{return Vector2D(x,y);}
};

static std::ostream& operator <<(std::ostream & out, const xytLoc &loc)
{
	out << "(" << loc.x << ", " << loc.y << ": " << loc.t << ")";
	return out;
}
	
bool operator==(const xytLoc &l1, const xytLoc &l2);


class Map2DConstrainedEnvironment : public ConstrainedEnvironment<xytLoc, tDirection>
{
public:
	Map2DConstrainedEnvironment(Map *m);
	Map2DConstrainedEnvironment(MapEnvironment *m);
	virtual void AddConstraint(Constraint<xytLoc> const& c);
	//void AddConstraint(xytLoc const& loc);
	void AddConstraint(xytLoc const& loc, tDirection dir);
	void ClearConstraints();
        virtual char const*const name()const{return mapEnv->name();}
	virtual void GetSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const;
	virtual void GetActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const;
	virtual tDirection GetAction(const xytLoc &s1, const xytLoc &s2) const;
	virtual void ApplyAction(xytLoc &s, tDirection a) const;
	virtual void UndoAction(xytLoc &s, tDirection a) const;
	virtual void GetReverseActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const;
	bool ViolatesConstraint(const xytLoc &from, const xytLoc &to) const;
        void setSoftConstraintEffectiveness(double){}
	
	virtual bool InvertAction(tDirection &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const xytLoc &node1, const xytLoc &node2) const;
	virtual double GCost(const xytLoc &node1, const xytLoc &node2) const { return mapEnv->GCost(node1,node2); }
	virtual double GCost(const xytLoc &node, const tDirection &act) const { return  mapEnv->GCost(node,act); }
	virtual bool GoalTest(const xytLoc &node, const xytLoc &goal) const;
	
	virtual uint64_t GetStateHash(const xytLoc &node) const;
	virtual uint64_t GetActionHash(tDirection act) const;

	virtual void OpenGLDraw() const;
        void OpenGLDraw(const xytLoc& s, const xytLoc& t, float perc) const;
	virtual void OpenGLDraw(const xytLoc&) const;
	virtual void OpenGLDraw(const xytLoc&, const tDirection&) const;
	virtual void GLDrawLine(const xytLoc &x, const xytLoc &y) const;
        virtual Map* GetMap()const{return mapEnv->GetMap();}
        bool LineOfSight(const xytLoc &x, const xytLoc &y)const{return mapEnv->LineOfSight(x,y);}
private:
	bool ViolatesConstraint(const xyLoc &from, const xyLoc &to, float time, float inc) const;

	std::vector<Constraint<xytLoc>> constraints;
	MapEnvironment *mapEnv;
};
typedef std::set<IntervalData> ConflictSet;

// Check if an openlist node conflicts with a node from an existing path
template<typename state>
unsigned checkForConflict(state const*const parent, state const*const node, state const*const pathParent, state const*const pathNode){
  Constraint<state> v(*node);
  if(v.ConflictsWith(*pathNode)){return 1;}
  if(parent && pathParent){
    Constraint<state> e1(*parent,*node);
    if(e1.ConflictsWith(*pathParent,*pathNode)){return 1;}
  }
  return 0; 
}


#define HASH_INTERVAL 0.09
#define HASH_INTERVAL_HUNDREDTHS 9

template <typename state, typename action, typename environment>
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
    static Map2DConstrainedEnvironment* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,environment,HASH_INTERVAL_HUNDREDTHS>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action, typename environment>
OpenClosedInterface<state,AStarOpenClosedData<state>>* TieBreaking<state,action,environment>::openList=0;
template <typename state, typename action, typename environment>
Map2DConstrainedEnvironment* TieBreaking<state,action,environment>::currentEnv=0;
template <typename state, typename action, typename environment>
uint8_t TieBreaking<state,action,environment>::currentAgent=0;
template <typename state, typename action, typename environment>
bool TieBreaking<state,action,environment>::randomalg=false;
template <typename state, typename action, typename environment>
bool TieBreaking<state,action,environment>::useCAT=false;
template <typename state, typename action, typename environment>
NonUnitTimeCAT<state,environment,HASH_INTERVAL_HUNDREDTHS>* TieBreaking<state,action,environment>::CAT=0;

#endif /* defined(__hog2_glut__Map2DConstrainedEnvironment__) */
