//
//  CBSUnits.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//
//  Modified by Thayne Walker 2017.
//

#ifndef __hog2_glut__AirplaneCBSUnits__
#define __hog2_glut__AirplaneCBSUnits__

#include <iostream>
#include <limits> 
#include <algorithm>
#include <map>

#include <queue>
#include <functional>
#include <vector>

#include <thread>
#include <mutex>

#include "Unit.h"
#include "UnitGroup.h"
#include "ConstrainedEnvironment.h"
#include "VelocityObstacle.h"
//#include "TemplateIntervalTree.h"
#include "MultiAgentStructures.h"
#include "GridStates.h"
#include "TemporalAStar.h"
#include "Heuristic.h"
#include "Timer.h"
#include <string.h>

#define NO_CONFLICT    0
#define NON_CARDINAL   1
#define LEFT_CARDINAL  2
#define RIGHT_CARDINAL 4
#define BOTH_CARDINAL  (LEFT_CARDINAL|RIGHT_CARDINAL)

float collisionTime(0);
float planTime(0);
float replanTime(0);
float bypassplanTime(0);
template <class state>
struct CompareLowGCost;

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
class CBSUnit;

extern double agentRadius;

template <class state>
struct CompareLowGCost {
  bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2)const{
    if(fequal(i1.g+i1.h, i2.g+i2.h)){
      if(fequal(i1.g, i2.g)){
        return fless(i1.data.t,i2.data.t); // Break ties by time
      }
      return fless(i1.g, i2.g);
    }
    return fgreater(i1.g+i1.h, i2.g+i2.h);
  }
};

// Helper functions

// Merge path between waypoints
template <typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void MergeLeg(std::vector<state> const& path, std::vector<state>& thePath, std::vector<int>& wpts, const unsigned s, unsigned g, double minTime){
  int insertPoint(wpts[s]); // Starting index of this leg
  float origTime(thePath[wpts[g]].t); // Original ending time of leg
  unsigned deletes(wpts[g]-wpts[s]+1); // Number of path points in this leg.
  // Remove points from the original path (if they exist)
  if(thePath.empty()){
    assert(!"Path being merged into is empty");
  }
  if(path.empty()){
    assert(!"Path to merge is empty");
  }
  while(thePath.size()>wpts[g]+1 && thePath[wpts[g]].sameLoc(thePath[++wpts[g]])){deletes++;}
  float newTime(path.rbegin()->t); // Save the track end time of the new leg

  // Insert new path in front of the insert point
  thePath.insert(thePath.begin()+insertPoint,path.begin(),path.end());
  insertPoint += path.size();

  //Erase the original subpath including the start node
  thePath.erase(thePath.begin()+insertPoint,thePath.begin()+insertPoint+deletes);

  // Update waypoint indices
  int legLenDiff(path.size()-deletes);
  if(legLenDiff){
    for(int i(g); i<wpts.size(); ++i){
      wpts[i]+=legLenDiff;
    }
  }

  if(thePath.size()-1!=wpts[g] && !fequal(newTime,origTime)){
      // Increase times through the end of the track
      auto newEnd(thePath.begin()+insertPoint);
      while(newEnd++ != thePath.end()){
          newEnd->t+=(newTime-origTime);
      }
  }

  while(wpts[g]>wpts[s]+1 && thePath[wpts[g]].sameLoc(thePath[wpts[g]-1])){
    wpts[g]--;
  }
}

// Plan path between waypoints
template <typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
unsigned ReplanLeg(CBSUnit<state,action,comparison,conflicttable,searchalgo>* c, searchalgo& astar, ConstrainedEnvironment<state,action>* env, std::vector<state>& thePath, std::vector<int>& wpts, unsigned s, unsigned g, double minTime){
  //env->setSoftConstraintEffectiveness(1.0);
  int insertPoint(wpts[s]); // Starting index of this leg
  float origTime(thePath[wpts[g]].t); // Original ending time of leg
  unsigned deletes(wpts[g]-wpts[s]+1); // Number of path points in this leg.
  // Remove points from the original path (if they exist)
  if(thePath.empty()){
    assert(false && "Expected a valid path for re-planning.");
  }
  while(thePath.size()>wpts[g]+1 && thePath[wpts[g]].sameLoc(thePath[++wpts[g]])){deletes++;}
  //std::cout << "Agent: " << "re-planning path from " << s << " to " << g << " on a path of len:" << thePath.size() << "\n";
  state start(c->GetWaypoint(s));
  state goal(c->GetWaypoint(g));
  // Preserve proper start time
  start.t = thePath[wpts[s]].t;

  //std::cout << start << " to " << goal << "\n";

  // Perform search for the leg
  std::vector<state> path;
  env->setGoal(goal);
  Timer tmr;
  tmr.StartTimer();
  astar.GetPath(env, start, goal, path, minTime);
  replanTime+=tmr.EndTimer();
  //std::cout << "Replan took: " << tmr.EndTimer() << std::endl;
  //std::cout << "New leg " << path.size() << "\n";
  //for(auto &p: path){std::cout << p << "\n";}
  if(path.empty()){
    thePath.resize(0);
    return astar.GetNodesExpanded(); //no solution found
  }
  float newTime(path.rbegin()->t); // Save the track end time of the new leg

  // Insert new path in front of the insert point
  //std::cout << "SIZE " << thePath.size() << "\n";
  //std::cout << "Insert path of len " << path.size() << " before " << insertPoint << "\n";
  thePath.insert(thePath.begin()+insertPoint,path.begin(),path.end());
  //std::cout << "SIZE " << thePath.size() << "\n";
  insertPoint += path.size();

  //Erase the original subpath including the start node
  //std::cout << "Erase path from " << insertPoint << " to " << (insertPoint+deletes) << "\n";
  thePath.erase(thePath.begin()+insertPoint,thePath.begin()+insertPoint+deletes);
  //std::cout << "SIZE " << thePath.size() << "\n";

  // Update waypoint indices
  int legLenDiff(path.size()-deletes);
  if(legLenDiff){
    for(int i(g); i<wpts.size(); ++i){
      wpts[i]+=legLenDiff;
    }
  }

  if(thePath.size()-1!=wpts[g] && !fequal(newTime,origTime)){
      // Increase times through the end of the track
      auto newEnd(thePath.begin()+insertPoint);
      while(newEnd++ != thePath.end()){
          newEnd->t+=(newTime-origTime);
      }
  }

  while(wpts[g]>wpts[s]+1 && thePath[wpts[g]].sameLoc(thePath[wpts[g]-1])){
    wpts[g]--;
  }
  //std::cout << "Replanned path\n";
  //for(auto &p: thePath){std::cout << p << "\n";}
  //std::cout << "exp replan " << astar.GetNodesExpanded() << "\n";
  return astar.GetNodesExpanded();
}

// Plan path between waypoints
template <typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
unsigned GetFullPath(CBSUnit<state,action,comparison,conflicttable,searchalgo>* c, searchalgo& astar, ConstrainedEnvironment<state,action>* env, std::vector<state>& thePath, std::vector<int>& wpts, unsigned agent)
{
  //env->setSoftConstraintEffectiveness(c->getVisibility());
  int insertPoint(0);
  float origTime(0.0);
  float newTime(0.0);
  unsigned deletes(0);
  unsigned expansions(0);
  // We should only call this function for initial empty paths.
  if(thePath.size()) assert(!"Tried to plan on top of an existing path!");
  //thePath.resize(0);
  wpts.resize(c->GetNumWaypoints());
  wpts[0]=0;

  // Perform search for all legs
  unsigned offset(0);
  comparison::currentEnv=(ConstrainedEnvironment<state,action>*)env;
  if(comparison::useCAT){
    comparison::openList=astar.GetOpenList();
    comparison::currentAgent=agent;
  }
  for(int i(0); i<wpts.size()-1; ++i){
    std::vector<state> path;
    state start(thePath.size()?thePath.back():c->GetWaypoint(i));
    start.landed=false;
    //start.t=0;
    state goal(c->GetWaypoint(i+1));
    env->setGoal(goal);

    Timer tmr;
    tmr.StartTimer();
    astar.GetPath(env, start, goal, path);
    planTime+=tmr.EndTimer();
    //std::cout << start <<"-->"<<goal<<" took: " << tmr.EndTimer() << std::endl;

    expansions += astar.GetNodesExpanded();
    //std::cout << "exp full " << astar.GetNodesExpanded() << "\n";
    if(path.empty()){return expansions;} //no solution found

    // Append to the entire path, omitting the first node for subsequent legs
    thePath.insert(thePath.end(),path.begin()+offset,path.end());
    
    offset=1;
    int ix(thePath.size()-1);
    wpts[i+1]=ix;
    while(ix>0&&thePath[ix].sameLoc(thePath[ix-1])){
      wpts[i+1]=--ix;
    }
  }
  return expansions;
}

template<typename state, typename action, typename comparison, typename conflicttable,class searchalgo=TemporalAStar<state, action, ConstrainedEnvironment<state,action>, AStarOpenClosed<state, comparison>>>
class CBSUnit : public Unit<state, action, ConstrainedEnvironment<state,action>> {
public:
  CBSUnit(std::vector<state> const &gs, float viz=0)
:start(0), goal(1), current(gs[0]), waypoints(gs), visibility(viz) {}
  const char *GetName() { return "CBSUnit"; }
  bool MakeMove(ConstrainedEnvironment<state,action> *,
      OccupancyInterface<state,action> *,
      SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *,
      action& a);
  void UpdateLocation(ConstrainedEnvironment<state,action> *, state &newLoc, bool success,
      SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *)
  { if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); }

  void GetLocation(state &l) { l = current; }
  void OpenGLDraw(const ConstrainedEnvironment<state,action> *, const SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *) const;
  void GetGoal(state &s) { s = waypoints[goal]; }
  void GetStart(state &s) { s = waypoints[start]; }
  inline std::vector<state> const & GetWaypoints()const{return waypoints;}
  inline state GetWaypoint(size_t i)const{ return waypoints[std::min(i,waypoints.size()-1)]; }
  inline unsigned GetNumWaypoints()const{return waypoints.size();}
  void SetPath(std::vector<state> &p);
  /*void PushFrontPath(std::vector<state> &s)
  {
    std::vector<state> newPath;
    for (state x : s)
      newPath.push_back(x);
    for (state y : myPath)
      newPath.push_back(y);
    myPath = newPath;
  }*/
  inline std::vector<state> const& GetPath()const{return myPath;}
  void UpdateGoal(state &start, state &goal);
  void setUnitNumber(unsigned n){number=n;}
  unsigned getUnitNumber()const{return number;}
  float getVisibility()const{return visibility;}

private:
  unsigned start, goal;
  state current;
  std::vector<state> waypoints;
  std::vector<state> myPath;
  unsigned number;
  float visibility;
};

template<typename state>
struct Conflict {
  Conflict<state>():c(nullptr),unit1(9999999),prevWpt(0){}
  Conflict<state>(Conflict<state>const& from):c(from.c.release()),unit1(from.unit1),prevWpt(from.prevWpt){}
  Conflict<state>& operator=(Conflict<state>const& from){c.reset(from.c.release());unit1=from.unit1;prevWpt=from.prevWpt;}
  mutable std::unique_ptr<Constraint<state>> c;
  unsigned unit1;
  unsigned prevWpt;
};

template<typename state, typename conflicttable, class searchalgo>
struct CBSTreeNode {
	CBSTreeNode():parent(0),satisfiable(true),cat(){}
	CBSTreeNode(CBSTreeNode<state,conflicttable,searchalgo> const& from):wpts(from.wpts),paths(from.paths),con(from.con),parent(from.parent),satisfiable(from.satisfiable),cat(from.cat){}
        CBSTreeNode(CBSTreeNode<state,conflicttable,searchalgo> const& node, Conflict<state> const& c, unsigned p, bool s):wpts(node.wpts),paths(node.paths),con(c),parent(p),satisfiable(s),cat(node.cat){}
        CBSTreeNode& operator=(CBSTreeNode<state,conflicttable,searchalgo> const& from){
          wpts=from.wpts;
          paths=from.paths;
          con=from.con;
          parent=from.parent;
          satisfiable=from.satisfiable;
          cat=from.cat;
        }
	std::vector< std::vector<int> > wpts;
	Solution<state> paths;
	Conflict<state> con;
	unsigned int parent;
	bool satisfiable;
        //IntervalTree cat; // Conflict avoidance table
        conflicttable cat; // Conflict avoidance table
};

template<typename state, typename conflicttable, class searchalgo>
static std::ostream& operator <<(std::ostream & out, const CBSTreeNode<state,conflicttable,searchalgo> &act)
{
	out << "(paths:"<<act.paths.size()<<", parent: "<<act.parent<< ", satisfiable: "<<act.satisfiable<<")";
	return out;
}

template<class T, class C, class Cmp>
struct ClearablePQ:public std::priority_queue<T,C,Cmp>{
  void clear(){
    //std::cout << "Clearing pq\n";
    //while(this->size()){std::cout<<this->size()<<"\n";this->pop();}
    this->c.resize(0);
  }
};

template<typename state, typename action, typename comparison, typename conflicttable,class searchalgo=TemporalAStar<state, action, ConstrainedEnvironment<state,action>, AStarOpenClosed<state, comparison>>>
class CBSGroup : public UnitGroup<state, action, ConstrainedEnvironment<state,action>>
{
  public:
    CBSGroup(std::vector<std::vector<EnvironmentContainer<state,action>>>&, bool v=false);
    bool MakeMove(Unit<state, action, ConstrainedEnvironment<state,action>> *u, ConstrainedEnvironment<state,action> *e, 
        SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *si, action& a);
    void UpdateLocation(Unit<state, action, ConstrainedEnvironment<state,action>> *u, ConstrainedEnvironment<state,action> *e, 
        state &loc, bool success, SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *si);
    void AddUnit(Unit<state, action, ConstrainedEnvironment<state,action>> *u);
    double GetMaxTime(int location, int agent);
    void StayAtGoal(int location);

    void OpenGLDraw(const ConstrainedEnvironment<state,action> *, const SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *)  const;
    double getTime() {return time;}
    bool donePlanning() {return planFinished;}
    bool ExpandOneCBSNode();

    std::vector<CBSTreeNode<state,conflicttable,searchalgo> > tree;
    void processSolution(double);
    searchalgo astar;
  private:    

    unsigned LoadConstraintsForNode(int location, int agent=-1);
    bool Bypass(int best, std::pair<unsigned,unsigned> const& numConflicts, Conflict<state> const& c1, unsigned otherunit, double minTime);
    void Replan(int location);
    void HasConflict(std::vector<state> const& a, std::vector<int> const& wa, std::vector<state> const& b, std::vector<int> const& wb, int x, int y, Conflict<state> &c1, Conflict<state> &c2, std::pair<unsigned,unsigned>& conflict);
    std::pair<unsigned,unsigned> FindHiPriConflict(CBSTreeNode<state,conflicttable,searchalgo>  const& location, Conflict<state> &c1, Conflict<state> &c2);
    unsigned FindFirstConflict(CBSTreeNode<state,conflicttable,searchalgo>  const& location, Conflict<state> &c1, Conflict<state> &c2);

    bool planFinished;

    /* Code for dealing with multiple environments */
    std::vector<std::vector<EnvironmentContainer<state,action>>> environments;
    std::vector<EnvironmentContainer<state,action>*> currentEnvironment;

    void SetEnvironment(unsigned conflicts,unsigned agent);
    void ClearEnvironmentConstraints(unsigned agent);
    void AddEnvironmentConstraint(Constraint<state>* c, unsigned agent);

    double time;

    unsigned int bestNode;
    std::mutex bestNodeLock;

    struct OpenListNode {
      OpenListNode() : location(0), cost(0), nc(0) {}
      OpenListNode(uint loc, double c, uint16_t n) : location(loc), cost(c),nc(n) {}
      std::ostream& operator <<(std::ostream& out)const{
        out << "(loc: "<<location<<", nc: "<<nc<< ", cost: "<<cost<<")";
        return out;
      }

      uint location;
      double cost;	
      unsigned nc;
    };
    struct OpenListNodeCompare {
      bool operator() (const OpenListNode& left, const OpenListNode& right) {
        if(greedyCT)
          return (left.nc==right.nc)?(fgreater(left.cost,right.cost)):(left.nc>right.nc);
        else
          return fequal(left.cost,right.cost)?(left.nc > right.nc):(fgreater(left.cost,right.cost));
      }
    };

    ClearablePQ<CBSGroup::OpenListNode, std::vector<CBSGroup::OpenListNode>, CBSGroup::OpenListNodeCompare> openList;

    uint TOTAL_EXPANSIONS = 0;

    //std::vector<SearchEnvironment<state,action>*> agentEnvs;
public:
    // Algorithm parameters
    bool verify;
    bool nobypass;
    bool ECBSheuristic; // For ECBS
    unsigned killex; // Kill after this many expansions
    bool keeprunning; // Whether to keep running after finding the answer (e.g. for ui)
    int seed;
    Timer* timer;
    int animate; // Add pauses for animation
    static bool greedyCT;
    bool verbose=false;
    bool quiet=false;
    bool disappearAtGoal=true;
};

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSGroup<state,action,comparison,conflicttable,searchalgo>::greedyCT=false;

/** AIR CBS UNIT DEFINITIONS */

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSUnit<state,action,comparison,conflicttable,searchalgo>::SetPath(std::vector<state> &p)
{
  myPath = p;
  std::reverse(myPath.begin(), myPath.end());
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSUnit<state,action,comparison,conflicttable,searchalgo>::OpenGLDraw(const ConstrainedEnvironment<state,action> *ae, 
    const SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *si) const
{
  GLfloat r, g, b;
  this->GetColor(r, g, b);
  ae->SetColor(r, g, b);

  if (myPath.size() > 1) {
    // Interpolate between the two given the timestep
    state start_t = myPath[myPath.size()-1];
    state stop_t = myPath[myPath.size()-2];

    if(si->GetSimulationTime() <= stop_t.t && si->GetSimulationTime() >= start_t.t)
    {
      float perc = (stop_t.t - si->GetSimulationTime())/(stop_t.t - start_t.t);
      ae->OpenGLDraw(stop_t, start_t, perc);
      //Constraint<state> c(stop_t, start_t);
      //glColor3f(1, 0, 0);
      //c.OpenGLDraw();
    } else {		
      //ae->OpenGLDraw(stop_t);
      //glColor3f(1, 0, 0);
      //Constraint<state> c(stop_t);
      //c.OpenGLDraw();
    }
  } else {
    if (current.landed)
      return;
    //ae->OpenGLDraw(current);
    //Constraint<state> c(current);
    //glColor3f(1, 0, 0);
    //c.OpenGLDraw(si->GetEnvironment()->GetMap());
  }
}

/*void CBSUnit<state,action,comparison,conflicttable,searchalgo>::UpdateGoal(state &s, state &g)
  {
  start = s;
  goal = g;
  }*/

//----------------------------------------------------------------------------------------------------------------------------------------------//

/** CBS GROUP DEFINITIONS */


template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::ClearEnvironmentConstraints(unsigned agent){
  for (EnvironmentContainer<state,action> env : this->environments[agent]) {
    env.environment->ClearConstraints();
  }
}


template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::AddEnvironmentConstraint(Constraint<state>* c, unsigned agent){
  //if(verbose)std::cout << "Add constraint " << c.start_state << "-->" << c.end_state << "\n";
  for (EnvironmentContainer<state,action> env : this->environments[agent]) {
    env.environment->AddConstraint(c);
  }
}

/** constructor **/
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
CBSGroup<state,action,comparison,conflicttable,searchalgo>::CBSGroup(std::vector<std::vector<EnvironmentContainer<state,action>>>& environvec,bool v)
: time(0), bestNode(0), planFinished(false), verify(false), nobypass(false)
    , ECBSheuristic(false), killex(INT_MAX), keeprunning(false),animate(0),
    seed(1234567), timer(0), verbose(v), quiet(true)
{
  //std::cout << "THRESHOLD " << threshold << "\n";

  tree.resize(1);
  tree[0].parent = 0;

  // Sort the environment container by the number of conflicts
  unsigned agent(0);
  for(auto& environs: environvec){
    std::sort(environs.begin(), environs.end(), 
        [](const EnvironmentContainer<state,action>& a, const EnvironmentContainer<state,action>& b) -> bool 
        {
        return a.threshold < b.threshold;
        }
        );
    environments.push_back(environs);
    // Set the current environment to that with 0 conflicts
    SetEnvironment(0,agent);
    ++agent;
  }

  astar.SetVerbose(verbose);
}


/** Expand a single CBS node */
// Return true while processing
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSGroup<state,action,comparison,conflicttable,searchalgo>::ExpandOneCBSNode()
{
  openList.pop();
  // There's no reason to expand if the plan is finished.
  if (planFinished)
    return false;

  Conflict<state> c1, c2;
  unsigned long last = tree.size();

  auto numConflicts(FindHiPriConflict(tree[bestNode], c1, c2));
  // If no conflicts are found in this node, then the path is done
  if (numConflicts.first==0)
  {
    processSolution(timer->EndTimer());
  }else{
    // If the conflict is NON_CARDINAL, try the bypass
    // if semi-cardinal, try bypass on one and create a child from the other
    // if both children are cardinal, create children for both

    // Swap units
    //unsigned tmp(c1.unit1);
    //c1.unit1=c2.unit1;
    //c2.unit1=tmp;
    // Notify the user of the conflict
    if(!quiet)std::cout << "TREE " << bestNode <<"("<<tree[bestNode].parent << ") " <<(numConflicts.second==7?"CARDINAL":(numConflicts.second==3?"LEFT-CARDINAL":(numConflicts.second==5?"RIGHT-CARDINAL":"NON-CARDIANL")))<< " conflict found between unit " << c1.unit1 << " and unit " << c2.unit1 << " @:" << c2.c->start() << "-->" << c2.c->end() <<  " and " << c1.c->start() << "-->" << c1.c->end() << " NC " << numConflicts.first << " prev-W " << c1.prevWpt << " " << c2.prevWpt << "\n";
    if(verbose){
      std::cout << c1.unit1 << ":\n";
      for(auto const& a:tree[bestNode].paths[c1.unit1]){
        std::cout << a << "\n";
      }
      std::cout << c2.unit1 << ":\n";
      for(auto const& a:tree[bestNode].paths[c2.unit1]){
        std::cout << a << "\n";
      }
    }
    if(animate){
      c1.c->OpenGLDraw(currentEnvironment[0]->environment->GetMap());
      c2.c->OpenGLDraw(currentEnvironment[0]->environment->GetMap());
      usleep(animate*1000);
    }

    double minTime(0.0);
    // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
    if(bestNode==0 || tree[bestNode].con.prevWpt+1==tree[bestNode].wpts[c1.unit1].size()-1){
      minTime=GetMaxTime(bestNode,c1.unit1)-1.0; // Take off a 1-second wait action, otherwise paths will grow over and over.
    }
    if((numConflicts.second&LEFT_CARDINAL) || !Bypass(bestNode,numConflicts,c1,c2.unit1,minTime)){
      last = tree.size();
      tree.resize(last+1);
      tree[last] = CBSTreeNode<state,conflicttable,searchalgo>(tree[bestNode],c1,bestNode,true);
      Replan(last);
      unsigned nc1(numConflicts.first);
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++){
        if(verbose){
          std::cout << "Agent " << y <<":\n";
          for(auto const& ff:tree[last].paths[y]){
            std::cout << ff << "\n";
          }
          std::cout << "cost: " << currentEnvironment[y]->environment->GetPathLength(tree[last].paths[y]) << "\n";
        }
        cost += currentEnvironment[y]->environment->GetPathLength(tree[last].paths[y]);
      }
      if(verbose){
        std::cout << "New CT NODE: " << last << " replanned: " << c1.unit1 << " cost: " << cost << " " << nc1 << "\n";
      }
      openList.emplace(last, cost, nc1);
    }
    if(bestNode==0 || tree[bestNode].con.prevWpt+1==tree[bestNode].wpts[c2.unit1].size()-1){
      minTime=GetMaxTime(bestNode,c2.unit1)-1.0; // Take off a 1-second wait action, otherwise paths will grow over and over.
    }
    if((numConflicts.second&RIGHT_CARDINAL) || !Bypass(bestNode,numConflicts,c2,c1.unit1,minTime)){
      last = tree.size();
      tree.resize(last+1);
      tree[last] = CBSTreeNode<state,conflicttable,searchalgo>(tree[bestNode],c2,bestNode,true);
      Replan(last);
      unsigned nc1(numConflicts.first);
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++){
        if(verbose){
          std::cout << "Agent " << y <<":\n";
          for(auto const& ff:tree[last].paths[y]){
            std::cout << ff << "\n";
          }
          std::cout << "cost: " << currentEnvironment[y]->environment->GetPathLength(tree[last].paths[y]) << "\n";
        }
        cost += currentEnvironment[y]->environment->GetPathLength(tree[last].paths[y]);
      }
      if(verbose){
        std::cout << "New CT NODE: " << last << " replanned: " << c2.unit1 << " cost: " << cost << " " << nc1 << "\n";
      }
      openList.emplace(last, cost, nc1);
    }

    // Get the best node from the top of the open list, and remove it from the list
    int count(0);
    do{
    bestNode = openList.top().location;
    if(!tree[bestNode].satisfiable)openList.pop();
    if(++count > tree.size()) assert(!"No solution!?!");
    }while(!tree[bestNode].satisfiable);

    // Set the visible paths for every unit in the node
    if(keeprunning)
    for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
    {
      // Grab the unit
      CBSUnit<state,action,comparison,conflicttable,searchalgo> *unit = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*) this->GetMember(x);

      // Prune these paths to the current simulation time
      state current;
      unit->GetLocation(current);
      std::vector<state> newPath;
      newPath.push_back(current); // Add the current simulation node to the new path

      // For everything in the path that's new, add the path back
      for (state xNode : tree[bestNode].paths[x]) {
        if (current.t < xNode.t - 0.0001) {
          newPath.push_back(xNode);
        }
      }

      // Update the actual unit path
      unit->SetPath(newPath);
    }

  }
  return true;
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSUnit<state,action,comparison,conflicttable,searchalgo>::MakeMove(ConstrainedEnvironment<state,action> *ae, OccupancyInterface<state,action> *,
							 SimulationInfo<state,action,ConstrainedEnvironment<state,action>> * si, action& a)
{
  if (myPath.size() > 1 && si->GetSimulationTime() > myPath[myPath.size()-2].t)
  {

    //std::cout << "Moved from " << myPath[myPath.size()-1] << " to " << myPath[myPath.size()-2] << std::endl;
    a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
    //std::cout << "Used action " << a << "\n";
    myPath.pop_back();
    return true;
  }
  return false;
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSGroup<state,action,comparison,conflicttable,searchalgo>::MakeMove(Unit<state, action, ConstrainedEnvironment<state,action>> *u, ConstrainedEnvironment<state,action> *e,
    SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *si, action& a)
{
  if (planFinished && si->GetSimulationTime() > time)
  {
    return u->MakeMove(e,0,si,a);
  }
  else if ((si->GetSimulationTime() - time) < 0.0001)
  {
    return false;
  }
  else {
    time = si->GetSimulationTime();
    ExpandOneCBSNode();
  }
  return false;
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::processSolution(double elapsed)
{

  double cost(0.0);
  unsigned total(0);
  double maxTime(GetMaxTime(bestNode,9999999));
  // For every unit in the node
  bool valid(true);
  for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
  {
    for(int j(tree[bestNode].paths[x].size()-1); j>0; --j){
      if(!tree[bestNode].paths[x][j-1].sameLoc(tree[bestNode].paths[x][j])){
        cost += tree[bestNode].paths[x][j].t;
        total += j;
        if(verbose)std::cout << "Adding " << tree[bestNode].paths[x][j].t << "\n";
        break;
      }else if(j==1){
        cost += tree[bestNode].paths[x][0].t;
        if(verbose)std::cout << "Adding_" << tree[bestNode].paths[x][0].t << "\n";
        total += 1;
      }
    }

    // Grab the unit
    CBSUnit<state,action,comparison,conflicttable,searchalgo>* unit((CBSUnit<state,action,comparison,conflicttable,searchalgo>*) this->GetMember(x));

    // Prune these paths to the current simulation time
    /*state current;
      unit->GetLocation(current);
      std::vector<state> newPath;
      newPath.push_back(current); // Add the current simulation node to the new path

    // For everything in the path that's new, add the path back
    for (state xNode : tree[bestNode].paths[x]) {
    if (current.t < xNode.t - 0.0001) {
    newPath.push_back(xNode);
    }
    }*/

    // Update the actual unit path
    // Add an extra wait action for "visualization" purposes,
    // This should not affect correctness...
    if(tree[bestNode].paths[x].size() && tree[bestNode].paths[x].back().t<maxTime){
      tree[bestNode].paths[x].push_back(tree[bestNode].paths[x].back());
      tree[bestNode].paths[x].back().t=maxTime;
    }
    unit->SetPath(tree[bestNode].paths[x]);
    if(tree[bestNode].paths[x].size()){
      if(!quiet)std::cout << "Agent " << x << ": " << "\n";
      unsigned wpt(0);
      signed ix(0);
      for(auto &a: tree[bestNode].paths[x])
      {
        //std::cout << a << " " << wpt << " " << unit->GetWaypoint(wpt) << "\n";
        if(ix++==tree[bestNode].wpts[x][wpt])
        {
          if(!quiet)std::cout << " *" << a << "\n";
          if(wpt<tree[bestNode].wpts[x].size()-1)wpt++;
        }
        else
        {
          if(!quiet)std::cout << "  " << a << "\n";
        }
      }
    }else{
      if(!quiet)std::cout << "Agent " << x << ": " << "NO Path Found.\n";
    }
    // Only verify the solution if the run didn't time out
    if(verify&&elapsed>0){
      for(unsigned int y = x+1; y < tree[bestNode].paths.size(); y++){
        for(unsigned i(1); i<tree[bestNode].paths[x].size(); ++i){
          for(unsigned j(1); j<tree[bestNode].paths[y].size(); ++j){
            if(currentEnvironment[0]->environment->collisionCheck(tree[bestNode].paths[x][i-1],tree[bestNode].paths[x][i],agentRadius,tree[bestNode].paths[y][j-1],tree[bestNode].paths[y][j],agentRadius)){
              valid=false;
              std::cout << "ERROR: Solution invalid; collision at: " << x <<":" << tree[bestNode].paths[x][i-1] << "-->" << tree[bestNode].paths[x][i] << ", " << y <<":" << tree[bestNode].paths[y][j-1] << "-->" << tree[bestNode].paths[y][j] << std::endl;
            }
          }
        }
      }
    }
  }
  fflush(stdout);
  std::cout<<"elapsed,planTime,replanTime,bypassplanTime,collisionTime,expansions,collisions,cost,actions\n";
  if(verify)std::cout << (valid?"VALID":"INVALID")<<std::endl;
  if(elapsed<0){
    //std::cout << seed<<":FAILED\n";
    std::cout << seed<<":" << elapsed*(-1.0) << ",";
  }else{
    std::cout << seed<<":" << elapsed << ",";
  }
  std::cout << planTime << ",";
  std::cout << replanTime << ",";
  std::cout << bypassplanTime << ",";
  std::cout << collisionTime << ",";
  std::cout << TOTAL_EXPANSIONS << ",";
  std::cout << tree.size() << ",";
  std::cout << cost << ","; 
  std::cout << total << std::endl;
  TOTAL_EXPANSIONS = 0;
  planFinished = true;
  if(!keeprunning)exit(0);
}

/** Update the location of a unit */
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::UpdateLocation(Unit<state, action, ConstrainedEnvironment<state,action>> *u, ConstrainedEnvironment<state,action> *e, state &loc, 
    bool success, SimulationInfo<state,action,ConstrainedEnvironment<state,action>> *si)
{
  u->UpdateLocation(e, loc, success, si);
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::SetEnvironment(unsigned numConflicts, unsigned agent){
  bool set(false);
  if(currentEnvironment.size()<agent+1){
    currentEnvironment.resize(agent+1); // We make the assumption that agents are continuously numbered
  }
  for (int i = 0; i < this->environments[agent].size(); i++) {
    if (numConflicts >= environments[agent][i].threshold) {
      if(verbose)std::cout << "Setting to env# " << i << " b/c " << numConflicts << " >= " << environments[agent][i].threshold<<environments[agent][i].environment->name()<<std::endl;
      //std::cout<<environments[agent][i].environment->getGoal()<<"\n";
      currentEnvironment[agent] = &(environments[agent][i]);
      set=true;
    } else {
      break;
    }
  }
  if(!set)assert(false&&"No env was set - you need -cutoffs of zero...");

  astar.SetHeuristic(currentEnvironment[agent]->heuristic);
  astar.SetWeight(currentEnvironment[agent]->astar_weight);
}

/** Add a new unit with a new start and goal state to the CBS group */
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::AddUnit(Unit<state, action, ConstrainedEnvironment<state,action>> *u)
{
  astar.SetExternalExpansionsPtr(&TOTAL_EXPANSIONS);
  astar.SetExternalExpansionLimit(killex);

  CBSUnit<state,action,comparison,conflicttable,searchalgo> *c = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*)u;
  c->setUnitNumber(this->GetNumMembers());
  // Add the new unit to the group, and construct an CBSUnit
  UnitGroup<state,action,ConstrainedEnvironment<state,action>>::AddUnit(u);

  // Clear the constraints from the environment set
  ClearEnvironmentConstraints(this->GetNumMembers()-1);
  SetEnvironment(0,this->GetNumMembers()-1);

  // Setup the state and goal in the graph
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Resize the number of paths in the root of the tree
  tree[0].paths.resize(this->GetNumMembers());
  tree[0].wpts.resize(this->GetNumMembers());
  //agentEnvs.resize(this->GetNumMembers());

  // Recalculate the optimum path for the root of the tree
  //std::cout << "AddUnit "<<(this->GetNumMembers()-1) << " getting path." << std::endl;
  //std::cout << "Search using " << currentEnvironment->environment->name() << "\n";
  //agentEnvs[c->getUnitNumber()]=currentEnvironment[this->GetNumMembers()-1]->environment;
  comparison::CAT = &(tree[0].cat);
  comparison::CAT->set(&tree[0].paths);
  GetFullPath<state,action,comparison,conflicttable,searchalgo>(c, astar, currentEnvironment[this->GetNumMembers()-1]->environment, tree[0].paths.back(),tree[0].wpts.back(),this->GetNumMembers()-1);
  if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
      processSolution(-timer->EndTimer());
  //std::cout << "AddUnit agent: " << (this->GetNumMembers()-1) << " expansions: " << astar.GetNodesExpanded() << "\n";

  // Create new conflict avoidance table instance
  if(this->GetNumMembers()<2) tree[0].cat=conflicttable();
  // We add the optimal path to the root of the tree
  if(comparison::useCAT){
    tree[0].cat.insert(tree[0].paths.back(),currentEnvironment[this->GetNumMembers()-1]->environment,tree[0].paths.size()-1);
  }
  StayAtGoal(0); // Do this every time a unit is added because these updates are taken into consideration by the CAT

  // Set the plan finished to false, as there's new updates
  planFinished = false;

  // Clear up the rest of the tree and clean the open list
  tree.resize(1);
  openList.clear();
  openList.emplace(0, 0, 0);
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
double CBSGroup<state,action,comparison,conflicttable,searchalgo>::GetMaxTime(int location,int agent){

  float maxDuration(0.0);
  if(disappearAtGoal)return 0.0;

  int i(0);
  // Find max duration of all paths
  for(auto const& n:tree[location].paths){
    if(i++!=agent)
      maxDuration=std::max(maxDuration,n.back().t);
  }
  return maxDuration;
}


template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::StayAtGoal(int location){

  if(disappearAtGoal)return;
  float maxDuration(0.0);

  // Find max duration of all paths
  for(auto const& n:tree[location].paths){
    maxDuration=std::max(maxDuration,n.back().t);
  }

  // Add wait actions (of 1 second) to goal states less than max
  for(auto& n:tree[location].paths){
    while(n.back().t<maxDuration-1.){
      state x(n.back());
      x.t+=1.0;
      n.push_back(x);
    }
  }
}

// Loads conflicts into environements and returns the number of conflicts loaded.
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
unsigned CBSGroup<state,action,comparison,conflicttable,searchalgo>::LoadConstraintsForNode(int location, int agent){
  // Select the unit from the tree with the new constraint
  int theUnit(agent<0?tree[location].con.unit1:agent);
  unsigned numConflicts(0);

  // Reset the constraints in the test-environment
  ClearEnvironmentConstraints(theUnit);

  // Add all of the constraints in the parents of the current node to the environment
  while(location!=0){
    if(theUnit == tree[location].con.unit1)
    {
      numConflicts++;
      AddEnvironmentConstraint(tree[location].con.c.get(),theUnit);
      if(verbose)std::cout << "Adding constraint (in accumulation)" << tree[location].con.c->start_state << "-->" << tree[location].con.c->end_state << " for unit " << theUnit << "\n";
    }
    location = tree[location].parent;
  }// while (location != 0);
  //Implement ECBS prioritization which penalizes the second element in a path.
  if(ECBSheuristic && strstr(currentEnvironment[theUnit]->environment->name().c_str(),"Highway")==NULL && tree[location].paths[tree[location].con.unit1].size()>1)
    AddEnvironmentConstraint((Constraint<state>*)new Collision<state>(tree[location].paths[tree[location].con.unit1][0],tree[location].paths[tree[location].con.unit1][1],agentRadius),theUnit);
  return numConflicts;
}

// Attempts a bypass around the conflict using an alternate optimal path
// Returns whether the bypass was effective
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSGroup<state,action,comparison,conflicttable,searchalgo>::Bypass(int best, std::pair<unsigned,unsigned> const& numConflicts, Conflict<state> const& c1, unsigned otherunit, double minTime)
{
  if(nobypass)return false;
  LoadConstraintsForNode(best,c1.unit1);
  AddEnvironmentConstraint(c1.c.get(),c1.unit1); // Add this constraint

  //std::cout << "Attempt to find a bypass.\n";

  bool success(false);
  Conflict<state> c3, c4;
  std::vector<state> newPath(tree[best].paths[c1.unit1]);
  std::vector<int> newWpts(tree[best].wpts[c1.unit1]);
  // Re-perform the search with the same constraints (since the start and goal are the same)
  CBSUnit<state,action,comparison,conflicttable,searchalgo> *c = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*)this->GetMember(c1.unit1);

  // Never use conflict avoidance tree for bypass
  bool orig(comparison::useCAT);
  comparison::useCAT=false;

  state start(c->GetWaypoint(c1.prevWpt));
  state goal(c->GetWaypoint(c1.prevWpt+1));
  // Preserve proper start time
  start.t = newPath[newWpts[c1.prevWpt]].t;
  // Cost of the previous path
  double cost(currentEnvironment[c1.unit1]->environment->GetPathLength(newPath));
  currentEnvironment[c1.unit1]->environment->setGoal(goal);
  std::vector<state> path;
  
  CBSTreeNode<state,conflicttable,searchalgo> newNode(tree[best]);
  Conflict<state> t1,t2; // Temp variables
  // Perform search for the leg
  if(verbose)std::cout << "Bypass for unit " << c1.unit1 << " on:\n";
  if(verbose)for(auto const& a:newPath){std::cout << a << "\n";}
  if(verbose)std::cout << cost << " cost\n";
  if(verbose)std::cout << openList.top().nc << " conflicts\n";
  unsigned pnum(0);
  unsigned nc1(openList.top().nc);
  // Initialize A*, etc.
  Timer tmr;
  tmr.StartTimer();
  astar.GetPath(currentEnvironment[c1.unit1]->environment,start,goal,path,minTime); // Get the path with the new constraint
  bypassplanTime+=tmr.EndTimer();
  MergeLeg<state,action,comparison,conflicttable,searchalgo>(path,newPath,newWpts,c1.prevWpt, c1.prevWpt+1,minTime);
  if(fleq(currentEnvironment[c1.unit1]->environment->GetPathLength(newPath),cost)){
    do{
      pnum++;
      if(path.size()==0)continue;
      MergeLeg<state,action,comparison,conflicttable,searchalgo>(path,newPath,newWpts,c1.prevWpt, c1.prevWpt+1,minTime);
      newNode.paths[c1.unit1] = newPath;
      newNode.wpts[c1.unit1] = newWpts;

      if(verbose)for(auto const& a:newPath){std::cout << a << "\n";}
      // TODO do full conflict count here
      auto pconf(FindHiPriConflict(newNode,c3,c4));
      if(verbose)std::cout<<"Path number " << pnum << "\n";
      if(verbose)for(auto const& a:newPath){std::cout << a << "\n";}
      if(verbose)std::cout << pconf.first << " conflicts\n";
      if(nc1>pconf.first){ // Is this bypass helpful?
        tree[best].paths[c1.unit1]=newPath;
        tree[best].wpts[c1.unit1]=newWpts;
        nc1=pconf.first;
        success=true;
      }
      if(pconf.first==0){
        if(verbose){std::cout << "BYPASS -- solution\n";}
        processSolution(timer->EndTimer());
        break;
      }
    }while(fleq(astar.GetNextPath(currentEnvironment[c1.unit1]->environment,start,goal,path,minTime),cost));
  }
  TOTAL_EXPANSIONS+=astar.GetNodesExpanded();
  if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
    processSolution(-timer->EndTimer());

  if(!success)return false;
  // Add CT node with the "best" bypass
  unsigned last = tree.size();
  tree.resize(last+1);
  tree[last] = CBSTreeNode<state,conflicttable,searchalgo>(tree[bestNode],c1,bestNode,true);
  cost=0;
  for (int y = 0; y < tree[last].paths.size(); y++){
    if(verbose){
      std::cout << "Agent " << y <<":\n";
      for(auto const& ff:tree[last].paths[y]){
        std::cout << ff << "\n";
      }
      std::cout << "cost: " << currentEnvironment[c1.unit1]->environment->GetPathLength(tree[last].paths[y]) << "\n";
    }
    cost += currentEnvironment[c1.unit1]->environment->GetPathLength(tree[last].paths[y]);
  }
  if(verbose){
    std::cout << "New BYPASS NODE: " << last << " replanned: " << c1.unit1 << " cost: " << cost << " " << nc1 << "\n";
  }
  openList.emplace(last, cost, nc1);

  comparison::useCAT=orig;

  // Make sure that the current location is satisfiable
  if (newPath.size() == 0 && !(*tree[best].paths[c1.unit1].begin() == *tree[best].paths[c1.unit1].rbegin()))
  {
    return false;
  }

  return success;
}


/** Replan a node given a constraint */
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::Replan(int location)
{
  // Select the unit from the tree with the new constraint
  int theUnit = tree[location].con.unit1;

  unsigned numConflicts(LoadConstraintsForNode(location));

  // Set the environment based on the number of conflicts
  SetEnvironment(numConflicts,theUnit);

  // Select the air unit from the group
  CBSUnit<state,action,comparison,conflicttable,searchalgo> *c = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*)this->GetMember(theUnit);

  // Retreive the unit start and goal
  //state start, goal;
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Recalculate the path
  //std::cout << "#conflicts for " << tempLocation << ": " << numConflicts << "\n";
  //currentEnvironment[theUnit]->environment->setGoal(goal);
  //std::cout << numConflicts << " conflicts " << " using " << currentEnvironment[theUnit]->environment->name() << " for agent: " << tree[location].con.unit1 << "?="<<c->getUnitNumber()<<"\n";
  //agentEnvs[c->getUnitNumber()]=currentEnvironment[theUnit]->environment;
  //astar.GetPath(currentEnvironment[theUnit]->environment, start, goal, thePath);
  //std::vector<state> thePath(tree[location].paths[theUnit]);
  comparison::openList=astar.GetOpenList();
  comparison::currentEnv=(ConstrainedEnvironment<state,action>*)currentEnvironment[theUnit]->environment;
  comparison::currentAgent=theUnit;
  comparison::CAT=&(tree[location].cat);
  comparison::CAT->set(&tree[location].paths);
  
  if(comparison::useCAT){
    comparison::CAT->remove(tree[location].paths[theUnit],currentEnvironment[theUnit]->environment,theUnit);
  }

  double minTime(0.0);
  // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
  if(location==0 || tree[location].con.prevWpt+1==tree[location].wpts[theUnit].size()-1){
    minTime=GetMaxTime(location,theUnit)-1.0; // Take off a 1-second wait action, otherwise paths will grow over and over.
  }


  //std::cout << "Replan agent " << theUnit << "\n";
  ReplanLeg<state,action,comparison,conflicttable,searchalgo>(c, astar, currentEnvironment[theUnit]->environment, tree[location].paths[theUnit], tree[location].wpts[theUnit], tree[location].con.prevWpt, tree[location].con.prevWpt+1,minTime);
  //for(int i(0); i<tree[location].paths.size(); ++i)
  //std::cout << "Replanned agent "<<i<<" path " << tree[location].paths[i].size() << "\n";

  if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
    processSolution(-timer->EndTimer());

  //DoHAStar(start, goal, thePath);
  //TOTAL_EXPANSIONS += astar.GetNodesExpanded();
  //std::cout << "Replan agent: " << location << " expansions: " << astar.GetNodesExpanded() << "\n";

  // Make sure that the current location is satisfiable
  if (tree[location].paths[theUnit].size() < 1){
    tree[location].satisfiable = false;
  }

  // Add the path back to the tree (new constraint included)
  //tree[location].paths[theUnit].resize(0);
  if(comparison::useCAT)
    comparison::CAT->insert(tree[location].paths[theUnit],currentEnvironment[theUnit]->environment,theUnit);

  /*for(int i(0); i<thePath.size(); ++i) {
    tree[location].paths[theUnit].push_back(thePath[i]);
  }*/

  // Issue tickets on the path
}

// Returns a pair containing:
// Number of Conflicts (NC)
// and
// an enum:
// 0=no-conflict    0x0000
// 1=non-cardinal   0x0001
// 2=left-cardinal  0x0010
// 4=right-cardinal 0x0100
// 6=both-cardinal  0x0110
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::HasConflict(std::vector<state> const& a, std::vector<int> const& wa, std::vector<state> const& b, std::vector<int> const& wb, int x, int y, Conflict<state> &c1, Conflict<state> &c2, std::pair<unsigned,unsigned>& conflict)
{
  // The conflict parameter contains the conflict count so far (conflict.first)
  // and the type of conflict found so far (conflict.second=BOTH_CARDINAL being the highest)

  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax(a.size());
  int ymax(b.size());

  //if(verbose)std::cout << "Checking for conflicts between: "<<x << " and "<<y<<" ranging from:" << xmax <<"," << ymax << "\n";

  //CBSUnit<state,action,comparison,conflicttable,searchalgo>* A = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*) this->GetMember(x);
  //CBSUnit<state,action,comparison,conflicttable,searchalgo>* B = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*) this->GetMember(y);
  //std::cout << "x,y "<<x<<" "<<y<<"\n";

  signed pwptA(-1); // waypoint number directly after which a conflict occurs
  signed pwptB(-1);
  int pxTime(-1);
  int pyTime(-1);
  for (int i = 0, j = 0; j < ymax && i < xmax;) // If we've reached the end of one of the paths, then time is up and 
    // no more conflicts could occur
  {
    // I and J hold the current step in the path we are comparing. We need 
    // to check if the current I and J have a conflict, and if they do, then
    // we have to deal with it.

    // Figure out which indices we're comparing
    int xTime(max(0, min(i, xmax-1)));
    int yTime(max(0, min(j, ymax-1)));
    int xNextTime(min(xmax-1, xTime+1));
    int yNextTime(min(ymax-1, yTime+1));

    // Check if we're looking directly at a waypoint.
    // Increment so that we know we've passed it.
    //std::cout << "if(xTime != pxTime && A->GetWaypoint(pwptA+1)==a[xTime]){++pwptA; pxTime=xTime;}\n";
    //std::cout << " " << xTime << " " << pxTime << " " << pwptA;std::cout << " " << A->GetWaypoint(pwptA+1) << " " << a[xTime] << "==?" << (A->GetWaypoint(pwptA+1)==a[xTime]) <<  "\n";
    //std::cout << "if(yTime != pyTime && B->GetWaypoint(pwptB+1)==b[yTime]){++pwptB; pyTime=yTime;}\n";
    //std::cout << " " << yTime << " " << pyTime << " " << pwptB;std::cout << " " << B->GetWaypoint(pwptB+1) << " " << b[yTime] << "==?" << (B->GetWaypoint(pwptB+1)==b[yTime]) <<  "\n";
    if(xTime != pxTime && pwptA+2<wa.size() && xTime == wa[pwptA+1]){++pwptA; pxTime=xTime;}
    if(yTime != pyTime && pwptB+2<wb.size() && yTime == wb[pwptB+1]){++pwptB; pyTime=yTime;}

    //if(verbose)std::cout << "Looking at positions " << xTime <<":"<<a[xTime].t << "," << j<<":"<<b[yTime].t << std::endl;

    // Check the point constraints
    //Constraint<state> x_c(a[xTime]);
    //state y_c =b[yTime];


    // Deal with landing conflicts, we don't conflict if one of the planes stays landed at
    // the entire time
    if (!(a[xTime].landed && a[xNextTime].landed || 
          b[yTime].landed && b[yNextTime].landed)) 
    {
      state const& aGoal(a[wa[pwptA+1]]);
      state const& bGoal(b[wb[pwptB+1]]);
      if(currentEnvironment[x]->environment->collisionCheck(a[xTime],a[xNextTime],agentRadius,b[yTime],b[yNextTime],agentRadius)){
        ++conflict.first;
        if(verbose)std::cout<<conflict.first<<" conflicts; #"<<x<<":" << a[xTime]<<"-->"<<a[xNextTime]<<" #"<<y<<":"<<b[yTime]<<"-->"<<b[yNextTime]<<"\n";
        if(BOTH_CARDINAL!=(conflict.second&BOTH_CARDINAL)){ // Keep searching until we find a both-cardinal conflict
          // Determine conflict type
          // If there are other legal successors with succ.f()=child.f(), this is non-cardinal
          unsigned conf(NO_CONFLICT); // Left is cardinal?
          {
            double childf(currentEnvironment[x]->environment->GCost(a[xTime],a[xNextTime])+currentEnvironment[x]->environment->HCost(a[xNextTime],aGoal));
            std::vector<state> succ;
            currentEnvironment[x]->environment->GetSuccessors(a[xTime],succ);
            bool found(false);
            for(auto const& s:succ){ // Is there at least one successor with same g+h as child?
              if(s.sameLoc(a[xNextTime])){continue;}
              if(fleq(currentEnvironment[x]->environment->GCost(a[xTime],s)+currentEnvironment[x]->environment->HCost(s,aGoal),childf)){found=true;break;}
            }
            if(!found){conf|=LEFT_CARDINAL;}
          }
          // Right is cardinal
          {
            double childf(currentEnvironment[y]->environment->GCost(b[yTime],b[yNextTime])+currentEnvironment[y]->environment->HCost(b[yNextTime],bGoal));
            std::vector<state> succ;
            currentEnvironment[y]->environment->GetSuccessors(b[yTime],succ);
            bool found(false);
            for(auto const& s:succ){ // Is there at least one successor with same g+h as child?
              if(s.sameLoc(b[yNextTime])){continue;}
              if(fleq(currentEnvironment[y]->environment->GCost(b[yTime],s)+currentEnvironment[y]->environment->HCost(s,bGoal),childf)){found=true;break;}
            }
            if(!found){conf|=RIGHT_CARDINAL;}
          }
          // Have we increased form non-cardinal to semi-cardinal or both-cardinal?
          if(NO_CONFLICT==conflict.second || ((conflict.second<=NON_CARDINAL)&&conf) || BOTH_CARDINAL==conf){
            conflict.second=conf+1;

            c1.c.reset((Constraint<state>*)new Collision<state>(a[xTime], a[xNextTime]));
            c2.c.reset((Constraint<state>*)new Collision<state>(b[yTime], b[yNextTime]));

            c1.unit1 = x;
            c2.unit1 = y;

            c1.prevWpt = pwptB;
            c2.prevWpt = pwptA;
          }
        }
      }
    }

    // Increment the counters based on the time

    // First we check to see if either is at the end
    // of the path. If so, immediately increment the 
    // other counter.
    if (i == xmax)
    {
      j++;
      continue;
    } else if (j == ymax)
    {
      i++;
      continue;
    }

    // Otherwise, we figure out which ends soonest, and
    // we increment that counter.
    if(fless(a[xNextTime].t,b[yNextTime].t)){
      // If the end-time of the x unit segment is before the end-time of the y unit segment
      // we have in increase the x unit but leave the y unit time the same
      i++;
    } else if(fequal(a[xNextTime].t,b[yNextTime].t)){
      i++;
      j++;
    } else {
      // Otherwise, the y unit time has to be incremented
      j++;
    }

  } // End time loop
}

/** Find the highest priority conflict **/
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
std::pair<unsigned,unsigned> CBSGroup<state,action,comparison,conflicttable,searchalgo>::FindHiPriConflict(CBSTreeNode<state,conflicttable,searchalgo> const& location, Conflict<state> &c1, Conflict<state> &c2)
{
  if(verbose)std::cout<<"Checking for conflicts\n";
  // prefer cardinal conflicts
  std::pair<std::pair<unsigned,unsigned>,std::pair<Conflict<state>,Conflict<state>>> best;

  // For each pair of units in the group
  Timer tmr;
  tmr.StartTimer();
  for(int x = 0; x < this->GetNumMembers(); x++)
  {
    for(int y = x+1; y < this->GetNumMembers(); y++)
    {
      // This call will update "best" with the number of conflicts and
      // with the *most* cardinal conflicts
      HasConflict(location.paths[x],location.wpts[x],location.paths[y],location.wpts[y],x,y,best.second.first,best.second.second,best.first);
      //if((best.first.second&BOTH_CARDINAL)==BOTH_CARDINAL)break;
    }
    //if((best.first.second&BOTH_CARDINAL)==BOTH_CARDINAL)break;
  }
  collisionTime+=tmr.EndTimer();
  c1=best.second.first;
  c2=best.second.second;
  return best.first;
}

/** Find the first place that there is a conflict in the tree */
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
unsigned CBSGroup<state,action,comparison,conflicttable,searchalgo>::FindFirstConflict(CBSTreeNode<state,conflicttable,searchalgo> const& location, Conflict<state> &c1, Conflict<state> &c2)
{

  unsigned numConflicts(0);

  // For each pair of units in the group
  for (int x = 0; x < this->GetNumMembers(); x++)
  {
    for (int y = x+1; y < this->GetNumMembers(); y++)
    {

      //numConflicts += HasConflict(location.paths[x],location.wpts[x],location.paths[y],location.wpts[y],x,y,c1,c2,numConflicts==0);
      if(!greedyCT&&numConflicts) return numConflicts;
    }
  }

  return numConflicts;
}

/** Draw the AIR CBS group */
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,searchalgo>::OpenGLDraw(const ConstrainedEnvironment<state,action> *ae, const SimulationInfo<state,action,ConstrainedEnvironment<state,action>> * sim)  const
{
	/*
	GLfloat r, g, b;
	glLineWidth(2.0);
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		CBSUnit<state,action,comparison,conflicttable,searchalgo> *unit = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*)this->GetMember(x);
		unit->GetColor(r, g, b);
		ae->SetColor(r, g, b);
		for (unsigned int y = 0; y < tree[bestNode].paths[x].size(); y++)
		{
			ae->OpenGLDraw(tree[bestNode].paths[x][y]);
		}
	}
	glLineWidth(1.0);
	*/
}


#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
