//
//  Grid3DConstrainedEnvironment.h
//  hog2 glut
//
//  Created by Thayne Walker on 8/4/2017
//

#ifndef __hog2_glut__Grid3DConstrainedEnvironment__
#define __hog2_glut__Grid3DConstrainedEnvironment__

#include <iostream>
#include <memory>

#include "Grid3DEnvironment.h"
#include "Vector3D.h"
#include "VelocityObstacle.h"
#include "ConstrainedEnvironment.h"
#include "PositionalUtils.h"
#include "TemplateAStar.h"
#include "MultiAgentStructures.h"

	
//bool operator==(const xyztLoc &l1, const xyztLoc &l2);

class Grid3DConstrainedEnvironment : public ConstrainedEnvironment<xyztAABB, t3DDirection>
{
public:
	Grid3DConstrainedEnvironment(Map3D *m, unsigned agent);
	Grid3DConstrainedEnvironment(Grid3DEnvironment *m, unsigned agent);
        virtual std::string name()const{return mapEnv->name();}
	virtual void GetSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const;
	virtual void GetAllSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const;
	virtual void GetActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const;
	virtual t3DDirection GetAction(const xyztLoc &s1, const xyztLoc &s2) const;
	virtual void ApplyAction(xyztLoc &s, t3DDirection a) const;
	virtual void UndoAction(xyztLoc &s, t3DDirection a) const;
	virtual void GetReverseActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const;
	
	virtual bool InvertAction(t3DDirection &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const xyztLoc &node1, const xyztLoc &node2) const;
	virtual double GCost(const xyztLoc &node1, const xyztLoc &node2) const { return mapEnv->GCost(node1,node2); }
	virtual double GCost(const xyztLoc &node, const t3DDirection &act) const { return  mapEnv->GCost(node,act); }
	virtual bool GoalTest(const xyztLoc &node, const xyztLoc &goal) const;
	
	virtual uint64_t GetStateHash(const xyztLoc &node) const;
        virtual void GetStateFromHash(uint64_t hash, xyztLoc &s) const;
	virtual uint64_t GetActionHash(t3DDirection act) const;
	virtual double GetPathLength(std::vector<xyztAABB> &neighbors);

        virtual inline double ViolatesConstraint(xyztLoc const& from, xyztLoc const& to) const {
          return ConstrainedEnvironment<xyztAABB, t3DDirection>::ViolatesConstraint(from,to)*xyztLoc::TIME_RESOLUTION_D;
        }

	virtual void OpenGLDraw() const;
        void OpenGLDraw(const xyztLoc& s, const xyztLoc& t, float perc) const;
	virtual void OpenGLDraw(const xyztLoc&) const;
	virtual void OpenGLDraw(const xyztLoc&, const t3DDirection&) const;
	virtual void GLDrawLine(const xyztLoc &x, const xyztLoc &y) const;
        void GLDrawPath(const std::vector<xyztLoc> &p, const std::vector<xyztLoc> &waypoints) const;
        virtual Map3D* GetMap()const{return mapEnv->GetMap();}
        virtual bool LineOfSight(const xyztLoc &x, const xyztLoc &y)const{return mapEnv->LineOfSight(x,y);}// && !ViolatesConstraint(x,y);}
        void SetIgnoreTime(bool i){ignoreTime=i;}
        bool GetIgnoreTime()const{return ignoreTime;}
        void SetIgnoreHeading(bool i){ignoreHeading=i;}
        bool GetIgnoreHeading()const{return ignoreHeading;}

        uint16_t maxTurnAzimuth=0; // 0 means "turn off"
        int16_t maxPitch=0;
        virtual bool collisionCheck(const xyztLoc &s1, const xyztLoc &d1, float r1, const xyztLoc &s2, const xyztLoc &d2, float r2);
        virtual bool collisionCheck(const xyztAABB &s1, float r1, const xyztAABB &s2, float r2);
        inline Grid3DEnvironment* GetMapEnv()const{return mapEnv;}
        virtual void setGoal(xyztLoc const& s){mapEnv->setGoal(s);};
        virtual xyztLoc const& getGoal()const{return mapEnv->getGoal();}
private:
        bool ignoreTime;
        bool ignoreHeading;

	Grid3DEnvironment *mapEnv;
};

// Check if an openlist node conflicts with a node from an existing path
unsigned checkForTheConflict(xyztLoc const*const parent, xyztLoc const*const node, xyztLoc const*const pathParent, xyztLoc const*const pathNode){
  if(parent && pathParent){
    CollisionDetector<xyztAABB> e1(.25);
    if(e1.HasConflict(*parent,*node,*pathParent,*pathNode)){return 1;}
  }
  return 0;
}


template <typename BB, typename action>
class TieBreaking3D {
  public:

  bool operator()(const AStarOpenClosedData<typename BB::State> &ci1, const AStarOpenClosedData<typename BB::State> &ci2) const
  {
    if (fequal(ci1.g+ci1.h, ci2.g+ci2.h)) // F-cost equal
    {

      if(useCAT && CAT){
        // Make them non-const :)
        AStarOpenClosedData<typename BB::State>& i1(const_cast<AStarOpenClosedData<typename BB::State>&>(ci1));
        AStarOpenClosedData<typename BB::State>& i2(const_cast<AStarOpenClosedData<typename BB::State>&>(ci2));
        // Compute cumulative conflicts (if not already done)
        ConflictSet matches;
        if(i1.data.nc ==-1){
          //std::cout << "Getting NC for " << i1.data << ":\n";
          CAT->get(i1.data.t,i1.data.t+xyztLoc::TIME_RESOLUTION_U,matches);

          // Get number of conflicts in the parent
          typename BB::State const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
          unsigned nc1(parent1?parent1->nc:0);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for(auto const& m: matches){
            if(currentAgent == m.agent) continue;
            typename BB::State p;
            currentEnv->GetStateFromHash(m.hash1,p);
            //p.t=m.start;
            typename BB::State n;
            currentEnv->GetStateFromHash(m.hash2,n);
            //n.t=m.stop;
            collchecks++;
            nc1+=checkForConflict(parent1,&i1.data,&p,&n);
            //if(!nc1){std::cout << "NO ";}
            //std::cout << "conflict(1): " << i1.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          i1.data.nc=nc1;
        }
        if(i2.data.nc ==-1){
          //std::cout << "Getting NC for " << i2.data << ":\n";
          CAT->get(i2.data.t,i2.data.t+xyztLoc::TIME_RESOLUTION_U,matches);

          // Get number of conflicts in the parent
          typename BB::State const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
          unsigned nc2(parent2?parent2->nc:0);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for(auto const& m: matches){
            if(currentAgent == m.agent) continue;
            typename BB::State p;
            currentEnv->GetStateFromHash(m.hash2,p);
            //p.t=m.start;
            typename BB::State n;
            currentEnv->GetStateFromHash(m.hash2,n);
            //n.t=m.stop;
            collchecks++;
            nc2+=checkForConflict(parent2,&i2.data,&p,&n);
            //if(!nc2){std::cout << "NO ";}
            //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          i2.data.nc=nc2;
        }
        if(fequal(i1.data.nc,i2.data.nc)){
          if(fequal(ci1.g,ci2.g)){
            if(randomalg && ci1.data.t==ci2.data.t){
              return rand()%2;
            }
            return ci1.data.t<ci2.data.t;  // Tie-break toward greater time (relevant for waiting at goal)
          }
          return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
        }
        return fgreater(i1.data.nc,i2.data.nc);
      }else{
        if(fequal(ci1.g,ci2.g)){
          if(randomalg && ci1.data.t==ci2.data.t){
            return rand()%2;
          }
          return ci1.data.t<ci2.data.t;  // Tie-break toward greater time (relevant for waiting at goal)
        }
        return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
      }
    }
    return (fgreater(ci1.g+ci1.h, ci2.g+ci2.h));
  }
    static OpenClosedInterface<typename BB::State,AStarOpenClosedData<typename BB::State>>* openList;
    static ConstrainedEnvironment<BB,action>* currentEnv;
    static uint8_t currentAgent;
    static unsigned collchecks;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<BB,action>* CAT; // Conflict Avoidance Table
};

template <typename BB, typename action>
OpenClosedInterface<typename BB::State,AStarOpenClosedData<typename BB::State>>* TieBreaking3D<BB,action>::openList=0;
template <typename BB, typename action>
ConstrainedEnvironment<BB,action>* TieBreaking3D<BB,action>::currentEnv=0;
template <typename BB, typename action>
uint8_t TieBreaking3D<BB,action>::currentAgent=0;
template <typename BB, typename action>
unsigned TieBreaking3D<BB,action>::collchecks=0;
template <typename BB, typename action>
bool TieBreaking3D<BB,action>::randomalg=false;
template <typename BB, typename action>
bool TieBreaking3D<BB,action>::useCAT=false;
template <typename BB, typename action>
NonUnitTimeCAT<BB,action>* TieBreaking3D<BB,action>::CAT=0;

template <typename BB, typename action>
class UnitTieBreaking3D {
  public:
    bool operator()(const AStarOpenClosedData<typename BB::State> &ci1, const AStarOpenClosedData<typename BB::State> &ci2) const
    {
      if (fequal(ci1.g+ci1.h, ci2.g+ci2.h)) // F-cost equal
      {
        if(useCAT && CAT){
          // Make them non-const :)
          AStarOpenClosedData<typename BB::State>& i1(const_cast<AStarOpenClosedData<typename BB::State>&>(ci1));
          AStarOpenClosedData<typename BB::State>& i2(const_cast<AStarOpenClosedData<typename BB::State>&>(ci2));

          // Compute cumulative conflicts (if not already done)
          if(i1.data.nc ==-1){
            //std::cout << "Getting NC for " << i1.data << ":\n";

            // Get number of conflicts in the parent
            typename BB::State const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
            unsigned nc1(parent1?parent1->nc:0);
            //std::cout << "  matches " << matches.size() << "\n";

            // Count number of conflicts
            for(int agent(0); agent<CAT->numAgents(); ++agent){
              if(currentAgent == agent) continue;
              typename BB::State const* p(0);
              if(i1.data.t!=0)
                p=&(CAT->get(agent,(i1.data.t-xyztLoc::TIME_RESOLUTION_U)/xyztLoc::TIME_RESOLUTION_D));
              typename BB::State const& n=CAT->get(agent,i1.data.t/xyztLoc::TIME_RESOLUTION_D);
              collchecks++;
              nc1+=checkForTheConflict(parent1,&i1.data,p,&n);
            }
            // Set the number of conflicts in the data object
            i1.data.nc=nc1;
          }
          if(i2.data.nc ==-1){
            //std::cout << "Getting NC for " << i2.data << ":\n";

            // Get number of conflicts in the parent
            typename BB::State const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
            unsigned nc2(parent2?parent2->nc:0);
            //std::cout << "  matches " << matches.size() << "\n";

            // Count number of conflicts
            for(int agent(0); agent<CAT->numAgents(); ++agent){
              if(currentAgent == agent) continue;
              typename BB::State const* p(0);
              if(i2.data.t!=0)
                p=&(CAT->get(agent,(i2.data.t-xyztLoc::TIME_RESOLUTION_U)/xyztLoc::TIME_RESOLUTION_D));
              typename BB::State const& n=CAT->get(agent,i2.data.t/xyztLoc::TIME_RESOLUTION_D);
              collchecks++;
              nc2+=checkForTheConflict(parent2,&i2.data,p,&n);
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
    static OpenClosedInterface<typename BB::State,AStarOpenClosedData<typename BB::State>>* openList;
    static ConstrainedEnvironment<BB,action>* currentEnv;
    static uint8_t currentAgent;
    static unsigned collchecks;
    static bool randomalg;
    static bool useCAT;
    static UnitTimeCAT<BB,action>* CAT; // Conflict Avoidance Table
};

template <typename BB, typename action>
OpenClosedInterface<typename BB::State,AStarOpenClosedData<typename BB::State>>* UnitTieBreaking3D<BB,action>::openList=0;
template <typename BB, typename action>
ConstrainedEnvironment<BB,action>* UnitTieBreaking3D<BB,action>::currentEnv=0;
template <typename BB, typename action>
uint8_t UnitTieBreaking3D<BB,action>::currentAgent=0;
template <typename BB, typename action>
unsigned UnitTieBreaking3D<BB,action>::collchecks=0;
template <typename BB, typename action>
bool UnitTieBreaking3D<BB,action>::randomalg=false;
template <typename BB, typename action>
bool UnitTieBreaking3D<BB,action>::useCAT=false;
template <typename BB, typename action>
UnitTimeCAT<BB,action>* UnitTieBreaking3D<BB,action>::CAT=0;

#endif /* defined(__hog2_glut__Grid3DConstrainedEnvironment__) */
