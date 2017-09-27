//
//  Grid3DConstrainedEnvironment.h
//  hog2 glut
//
//  Created by Thayne Walker on 8/4/2017
//

#ifndef __hog2_glut__Grid3DConstrainedEnvironment__
#define __hog2_glut__Grid3DConstrainedEnvironment__

#include <iostream>

#include "Grid3DEnvironment.h"
#include "Vector3D.h"
#include "VelocityObstacle.h"
#include "NonUnitTimeCAT.h"
#include "ConstrainedEnvironment.h"
#include "PositionalUtils.h"
#include "TemplateAStar.h"

struct xyztLoc : xyzLoc {
	xyztLoc(xyzLoc loc, float time):xyzLoc(loc), h(0), p(0), t(time), nc(-1){}
	xyztLoc(xyzLoc loc, uint16_t _h, int16_t _p, float time):xyzLoc(loc), h(_h), p(_p), t(time), nc(-1){}
	xyztLoc(uint16_t _x, uint16_t _y, uint16_t _z, float time):xyzLoc(_x,_y,_z), h(0), p(0), t(time) ,nc(-1){}
	xyztLoc(uint16_t _x, uint16_t _y, uint16_t _z, uint16_t _h, int16_t _p, float time):xyzLoc(_x,_y,_z), h(_h), p(_p), t(time) ,nc(-1){}
	xyztLoc(uint16_t _x, uint16_t _y, uint16_t _z, double _h, double _p, float time):xyzLoc(_x,_y,_z), h(_h*xyztLoc::HDG_RESOLUTON), p(_p*xyztLoc::PITCH_RESOLUTON), t(time) ,nc(-1){}
	xyztLoc(uint16_t _x, uint16_t _y, uint16_t _z, uint16_t _v, uint16_t _h, int16_t _p, float time):xyzLoc(_x,_y,_z,_v), h(_h), p(_p), t(time) ,nc(-1){}
	xyztLoc():xyzLoc(),h(0),p(0),t(0),nc(-1){}
        int16_t nc; // Number of conflicts, for conflict avoidance table
        uint16_t h; // Heading
        int16_t p; // Pitch
	float t;
        operator Vector3D()const{return Vector3D(x,y,z);}
        bool sameLoc(xyztLoc const& other)const{return x==other.x&&y==other.y&&z==other.z;}
        static const float HDG_RESOLUTON;
        static const float PITCH_RESOLUTON;
};

struct TemporalVector3D : Vector3D {
	TemporalVector3D(Vector3D const& loc, double time):Vector3D(loc), t(time){}
	TemporalVector3D(xyztLoc const& loc):Vector3D(loc), t(loc.t){}
	TemporalVector3D(double _x, double _y, double _z, float time):Vector3D(_x,_y,_z), t(time){}
	TemporalVector3D():Vector3D(),t(0){}
	double t;
};

static std::ostream& operator <<(std::ostream & out, const TemporalVector3D &loc)
{
	out << "<" << loc.x << ", " << loc.y << ", " << loc.z << ": " << loc.t << ">";
	return out;
}
	
static std::ostream& operator <<(std::ostream & out, const xyztLoc &loc)
{
	out << "(" << loc.x << "," << loc.y << "," << loc.z << "," << loc.h/xyztLoc::HDG_RESOLUTON << "," << loc.p/xyztLoc::PITCH_RESOLUTON << ": " << loc.t << ")";
	return out;
}
	
bool operator==(const xyztLoc &l1, const xyztLoc &l2);

class Grid3DConstrainedEnvironment : public ConstrainedEnvironment<xyztLoc, t3DDirection>
{
public:
	Grid3DConstrainedEnvironment(Map3D *m);
	Grid3DConstrainedEnvironment(Grid3DEnvironment *m);
	virtual void AddConstraint(Constraint<xyztLoc> const& c);
	virtual void AddConstraint(Constraint<TemporalVector3D> const& c);
	//void AddConstraint(xyztLoc const& loc);
	void AddConstraint(xyztLoc const& loc, t3DDirection dir);
	void ClearConstraints();
        virtual std::string name()const{return mapEnv->name();}
	virtual void GetSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const;
	virtual void GetAllSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const;
	virtual void GetActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const;
	virtual t3DDirection GetAction(const xyztLoc &s1, const xyztLoc &s2) const;
	virtual void ApplyAction(xyztLoc &s, t3DDirection a) const;
	virtual void UndoAction(xyztLoc &s, t3DDirection a) const;
	virtual void GetReverseActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const;
	bool ViolatesConstraint(const xyztLoc &from, const xyztLoc &to) const;
        void setSoftConstraintEffectiveness(double){}
	
	virtual bool InvertAction(t3DDirection &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const xyztLoc &node1, const xyztLoc &node2) const;
	virtual double GCost(const xyztLoc &node1, const xyztLoc &node2) const { return mapEnv->GCost(node1,node2); }
	virtual double GCost(const xyztLoc &node, const t3DDirection &act) const { return  mapEnv->GCost(node,act); }
	virtual bool GoalTest(const xyztLoc &node, const xyztLoc &goal) const;
	
	virtual uint64_t GetStateHash(const xyztLoc &node) const;
        virtual void GetStateFromHash(uint64_t hash, xyztLoc &s) const;
	virtual uint64_t GetActionHash(t3DDirection act) const;

	virtual void OpenGLDraw() const;
        void OpenGLDraw(const xyztLoc& s, const xyztLoc& t, float perc) const;
	virtual void OpenGLDraw(const xyztLoc&) const;
	virtual void OpenGLDraw(const xyztLoc&, const t3DDirection&) const;
	virtual void GLDrawLine(const xyztLoc &x, const xyztLoc &y) const;
        void GLDrawPath(const std::vector<xyztLoc> &p, const std::vector<xyztLoc> &waypoints) const;
        virtual Map3D* GetMap()const{return mapEnv->GetMap();}
        bool LineOfSight(const xyztLoc &x, const xyztLoc &y)const{return mapEnv->LineOfSight(x,y) && !ViolatesConstraint(x,y);}
        void SetIgnoreTime(bool i){ignoreTime=i;}

        void SetMaxTurnAzimuth(float val){maxTurnAzimuth=val*xyztLoc::HDG_RESOLUTON;}
        void SetMaxPitch(float val){maxPitch=val*xyztLoc::PITCH_RESOLUTON;}
        uint16_t maxTurnAzimuth=0; // 0 means "turn off"
        int16_t maxPitch=0;
private:
        bool ignoreTime;
	bool ViolatesConstraint(const xyzLoc &from, const xyzLoc &to, float time, float inc) const;

	std::vector<Constraint<xyztLoc>> constraints;
	std::vector<Constraint<TemporalVector3D>> vconstraints;
	Grid3DEnvironment *mapEnv;
};
typedef std::set<IntervalData> ConflictSet;


#define HASH_INTERVAL 0.50
#define HASH_INTERVAL_HUNDREDTHS 50

template <typename state, typename action>
class TieBreaking3D {
  public:
// Check if an openlist node conflicts with a node from an existing path
unsigned checkForConflict(state const*const parent, state const*const node, state const*const pathParent, state const*const pathNode){
  Constraint<state> v(*node);
  if(v.ConflictsWith(*pathNode)){return 1;}
  if(parent && pathParent){
    Constraint<state> e1(*parent,*node);
    if(e1.ConflictsWith(*pathParent,*pathNode)){return 1;}
  }
  return 0; 
}

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
    static Grid3DConstrainedEnvironment* currentEnv;
    static uint8_t currentAgent;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action>
OpenClosedInterface<state,AStarOpenClosedData<state>>* TieBreaking3D<state,action>::openList=0;
template <typename state, typename action>
Grid3DConstrainedEnvironment* TieBreaking3D<state,action>::currentEnv=0;
template <typename state, typename action>
uint8_t TieBreaking3D<state,action>::currentAgent=0;
template <typename state, typename action>
bool TieBreaking3D<state,action>::randomalg=false;
template <typename state, typename action>
bool TieBreaking3D<state,action>::useCAT=false;
template <typename state, typename action>
NonUnitTimeCAT<state,action,HASH_INTERVAL_HUNDREDTHS>* TieBreaking3D<state,action>::CAT=0;

#endif /* defined(__hog2_glut__Grid3DConstrainedEnvironment__) */
