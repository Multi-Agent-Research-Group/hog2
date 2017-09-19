//
//  AirplaneCBSUnits.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
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

template <class state>
struct CompareLowGCost;

template<typename state>
using Solution = std::vector<std::vector<state>>;

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
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
template <typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
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
template <typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
unsigned ReplanLeg(CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>* c, searchalgo& astar, environment* env, std::vector<state>& thePath, std::vector<int>& wpts, unsigned s, unsigned g, double minTime){
  env->setSoftConstraintEffectiveness(1.0);
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
template <typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
unsigned GetFullPath(CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>* c, searchalgo& astar, environment* env, std::vector<state>& thePath, std::vector<int>& wpts, unsigned agent)
{
  env->setSoftConstraintEffectiveness(c->getVisibility());
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
  comparison::currentEnv=(environment*)env;
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

template<typename state, typename action, typename environment, typename comparison, typename conflicttable,class searchalgo=TemporalAStar<state, action, environment, AStarOpenClosed<state, comparison>>>
class CBSUnit : public Unit<state, action, environment> {
public:
  CBSUnit(std::vector<state> const &gs, float viz=0)
:start(0), goal(1), current(gs[0]), waypoints(gs), visibility(viz) {}
  const char *GetName() { return "CBSUnit"; }
  bool MakeMove(environment *,
      OccupancyInterface<state,action> *,
      SimulationInfo<state,action,environment> *,
      action& a);
  void UpdateLocation(environment *, state &newLoc, bool success,
      SimulationInfo<state,action,environment> *)
  { if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); }

  void GetLocation(state &l) { l = current; }
  void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *) const;
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

struct MetaAgent {
  MetaAgent(){}
  MetaAgent(unsigned agent){units.push_back(x);}
  std::vector<Constraint<state>> c;
  std::vector<unsigned> units;
};

template<typename state>
struct Conflict {
  Constraint<state> c; // constraint representing one agent in the meta-state
  unsigned unit1;
  std::vector<unsigned> prevWpt;
};

template<typename state, typename conflicttable, class searchalgo, class maplanner>
struct CBSTreeNode {
	CBSTreeNode():parent(0),satisfiable(true),cat(){}
	std::vector< std::vector<int> > wpts;
	Solution<state> paths;
	Conflict<state> con;
	unsigned int parent;
	bool satisfiable;
        //IntervalTree cat; // Conflict avoidance table
        conflicttable cat; // Conflict avoidance table
};

template<typename state, typename conflicttable, class searchalgo, class maplanner>
static std::ostream& operator <<(std::ostream & out, const CBSTreeNode<state,conflicttable,searchalgo> &act)
{
	out << "(paths:"<<act.paths.size()<<", parent: "<<act.parent<< ", satisfiable: "<<act.satisfiable<<")";
	return out;
}

template<typename state, typename action, typename environ>
struct EnvironmentContainer {
	EnvironmentContainer() : name("NULL ENV"), environment(0), conflict_cutoff(0), astar_weight(0.0f) {}
	EnvironmentContainer(std::string n, environ * e, Heuristic<state>* h, uint32_t conf, float a) : name(n), environment(e), conflict_cutoff(conf), astar_weight(a) {}
	std::vector<environ*> environment;
	uint64_t conflict_cutoff;
	float astar_weight;
	std::string name;
};


template<typename state, typename action, typename environment, typename comparison, typename conflicttable,class searchalgo=TemporalAStar<state, action, environment, AStarOpenClosed<state, comparison>>>
class CBSGroup : public UnitGroup<state, action, environment>
{
  public:
    CBSGroup(std::vector<EnvironmentContainer<state,action,environment> > const&, bool v=false);
    bool MakeMove(Unit<state, action, environment> *u, environment *e, 
        SimulationInfo<state,action,environment> *si, action& a);
    void UpdateLocation(Unit<state, action, environment> *u, environment *e, 
        state &loc, bool success, SimulationInfo<state,action,environment> *si);
    void AddUnit(Unit<state, action, environment> *u);
    double GetMaxTime(int location, int agent);
    void StayAtGoal(int location);

    void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *)  const;
    double getTime() {return time;}
    bool donePlanning() {return planFinished;}
    bool ExpandOneCBSNode();

    std::vector<CBSTreeNode<state,conflicttable,searchalgo> > tree;
    void processSolution(double);
    searchalgo astar;
    maplanner maPlanner;
    
  private:    

    unsigned LoadConstraintsForNode(int location, int agent=-1);
    bool Bypass(int best, Conflict<state> const& c1, unsigned otherunit, double minTime);
    void Replan(int location);
    unsigned HasConflict(std::vector<state> const& a, std::vector<int> const& wa, std::vector<state> const& b, std::vector<int> const& wb, int x, int y, Conflict<state> &c1, Conflict<state> &c2, std::pair<unsigned,unsigned>& conflict, bool update);
    std::pair<unsigned,unsigned> FindHiPriConflict(CBSTreeNode<state,conflicttable,searchalgo>  const& location, Conflict<state> &c1, Conflict<state> &c2, bool update);

    bool planFinished;

    /* Code for dealing with multiple environments */
    std::vector<EnvironmentContainer<state,action,environment> > environments;
    EnvironmentContainer<state,action,environment>* currentEnvironment;

    void SetEnvironment(unsigned conflicts);
    void ClearEnvironmentConstraints();
    void AddEnvironmentConstraint(Constraint<state> const& c, unsigned agent);

    double time;
    unsigned mergeThreshold(5);

    std::unordered_map<unsigned, MetaAgent> unitToMetaAgentMap;
    std::vector<MetaAgent> activeMetaAgents;
    std::vector<std::vector<unsigned>> metaAgentConflictMatrix;
    bool CheckForMerge(std::pair<unsigned, unsigned> &toMerge);

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

    std::priority_queue<CBSGroup::OpenListNode, std::vector<CBSGroup::OpenListNode>, CBSGroup::OpenListNodeCompare> openList;

    uint TOTAL_EXPANSIONS = 0;

    std::vector<SearchEnvironment<state,action>*> agentEnvs;
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
    bool disappearAtGoal=true;
};

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
bool CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::greedyCT=false;

/** AIR CBS UNIT DEFINITIONS */

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::SetPath(std::vector<state> &p)
{
  myPath = p;
  std::reverse(myPath.begin(), myPath.end());
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::OpenGLDraw(const environment *ae, 
    const SimulationInfo<state,action,environment> *si) const
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

/*void CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::UpdateGoal(state &s, state &g)
  {
  start = s;
  goal = g;
  }*/

//----------------------------------------------------------------------------------------------------------------------------------------------//

/** CBS GROUP DEFINITIONS */


template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::ClearEnvironmentConstraints(){
  for (EnvironmentContainer<state,action,environment> env : this->environments) {
    env.environment->ClearConstraints();
  }
}


template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::AddEnvironmentConstraint(Constraint<state>const& c, unsigned agent){
  //if(verbose)std::cout << "Add constraint " << c.start_state << "-->" << c.end_state << "\n";
  for (EnvironmentContainer<state,action,environment> env : this->environments) {
    env.environment[agent]->AddConstraint(c);
  }
}

/** constructor **/
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::CBSGroup(std::vector<EnvironmentContainer<state,action,environment> > const& environs,bool v)
: time(0), bestNode(0), planFinished(false), verify(false), nobypass(false)
    , ECBSheuristic(false), killex(INT_MAX), keeprunning(false),animate(0),
    seed(1234567), timer(0), verbose(v)
{
  //std::cout << "THRESHOLD " << threshold << "\n";

  tree.resize(1);
  tree[0].parent = 0;

  // Sort the environment container by the number of conflicts
  std::sort(environs.begin(), environs.end(), 
      [](const EnvironmentContainer<state,action,environment>& a, const EnvironmentContainer<state,action,environment>& b) -> bool 
      {
      return a.conflict_cutoff < b.conflict_cutoff;
      }
      );
  environments=environs;

  // Set the current environment to that with 0 conflicts
  SetEnvironment(0);
  astar.SetVerbose(verbose);
  maPlanner.SetVerbose(verbose);
}


/** Expand a single CBS node */
// Return true while processing
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
bool CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::ExpandOneCBSNode()
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
    if(verbose){
      std::cout << "TREE " << bestNode <<"("<<tree[bestNode].parent << ") " <<(numConflicts.second==7?"CARDINAL":(numConflicts.second==3?"LEFT-CARDINAL":(numConflicts.second==5?"RIGHT-CARDINAL":"NON-CARDIANL")))<< " conflict found between MA " << c1.unit1 << " and MA " << c2.unit1 << " @:" << c2.c.start() << "-->" << c2.c.end() <<  " and " << c1.c.start() << "-->" << c1.c.end() << " NC " << numConflicts.first << " prev-W " << c1.prevWpt << " " << c2.prevWpt << "\n";
      //std::cout << c1.unit1 << ":\n";
      //for(auto const& a:tree[bestNode].paths[c1.unit1]){
        //std::cout << a << "\n";
      //}
      //std::cout << c2.unit1 << ":\n";
      //for(auto const& a:tree[bestNode].paths[c2.unit1]){
        //std::cout << a << "\n";
      //}
    }
    if(animate){
      c1.c.OpenGLDraw(currentEnvironment->environment[0]->GetMap());
      c2.c.OpenGLDraw(currentEnvironment->environment[0]->GetMap());
      usleep(animate*1000);
    }

    for(unsigned i(0); i < activeMetaAgents.size(); ++i){
      for(unsigned j(i+1); j < activeMetaAgents.size(); ++j){
        if(metaAgentConflictMatrix[i][j] > mergeThreshold){
          std::cout << "Merging " << i << " and " << j << "\n";    
          // Merge i and j
          for(unsigned x : activeMetaAgents[j].units){
            activeMetaAgents[i].units.push_back(x);
          }
          // Remove j from the active list
          activeMetaAgents.erase(activeMetaAgents.begin() + j);
          // Remove j from the conflict matrix
          for(int x(0); x < metaAgentConflictMatrix.size(); ++x){
            metaAgentConflictMatrix[x].erase(metaAgentConflictMatrix[x].begin() + j);
          }
          metaAgentConflictMatrix.erase(metaAgentConflictMatrix.begin() + j);

          // Reset the search (This is the merge and restart enhancement)
          // Clear up the rest of the tree and clean the open list
          tree.resize(1);
          while(!openList.empty()) openList.pop();
          openList.push(OpenListNode(0, 0, 0));
          // Re-Plan the first node
          for (unsigned a(0); a < activeMetaAgents.size(); a++) {
            // Build the MultiAgentState
            MultiAgentState start;
            MultiAgentState goal;
            for (unsigned x(0); x < activeMetaAgents[a].units.size(); x++) {
              // Select the air unit from the group
              CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *c((CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*)GetMember(activeMetaAgents[a].units[x]));
              // Retreive the unit start and goal
              state s,g;
              c->GetStart(s);
              c->GetGoal(g);
              start.push_back(s);
              goal.push_back(g);
            }

            Solution<state> solution;
            env->setGoal(goal);
            maPlanner.GetPath(env, start, goal, solution);
            if(goal.size()>1)
              std::cout << "Merged plan took " << astar.GetNodesExpanded() << " expansions\n";

            TOTAL_EXPANSIONS += maPlanner.GetNodesExpanded();
            tree[0].paths.resize(GetNumMembers());

            for (unsigned k(0); k < activeMetaAgents[a].units.size(); k++) {
              // Add the path back to the tree (new constraint included)
              tree[0].paths[activeMetaAgents[a].units[k]].resize(0);
              for (unsigned l(0); l < tp.size(); l++)
              {
                tree[0].paths[activeMetaAgents[a].units[k]].push_back(tp[l][k]);
              }
            }
          }
          // Finished merging - return from the unit

          // Get the best node from the top of the open list, and remove it from the list
          bestNode = openList.top().location;
          openList.pop();

          // Set the visible paths for every unit in the node
          for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
          {
            // Grab the unit
            CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>* unit((CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*)GetMember(x));

            // Prune these paths to the current simulation time
            airtimeState current;
            unit->GetLocation(current);
            std::vector<airtimeState> newPath;
            newPath.push_back(current); // Add the current simulation node to the new path

            // For everything in the path that's new, add the path back
            for (airtimeState xNode : tree[bestNode].paths[x]) {
              if (current.t < xNode.t - 0.0001) {
                newPath.push_back(xNode);
              }
            }

            // Update the actual unit path
            unit->SetPath(newPath);
          }
          std::cout << "Merged MAs " << i << " and " << j << std::endl;
          return; 
        }
      }
    }
    double minTime(0.0);
    // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
    if(activeMetaAgents[c1.unit1].size()==1 && tree[bestNode].con.prevWpt+1==tree[bestNode].wpts[activeMetaAgents[c1.unit1].units[0]].size()-1){
      minTime=GetMaxTime(bestNode,c1.unit1)-1.0; // Take off a 1-second wait action, otherwise paths will grow over and over.
    }
    if((numConflicts.second&LEFT_CARDINAL) || !Bypass(bestNode,c1,c2.unit1,minTime)){
      last = tree.size();
      tree.resize(last+1);
      tree[last] = tree[bestNode];
      tree[last].con = c1;
      tree[last].parent = bestNode;
      tree[last].satisfiable = true;
      Replan(last);
      unsigned nc1(numConflicts.first);
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++){
        if(verbose){
          std::cout << "Agent " << y <<":\n";
          for(auto const& ff:tree[last].paths[y]){
            std::cout << ff << "\n";
          }
          std::cout << "cost: " << currentEnvironment->environment[y]->GetPathLength(tree[last].paths[y]) << "\n";
        }
        cost += currentEnvironment->environment[y]->GetPathLength(tree[last].paths[y]);
      }
      OpenListNode l1(last, cost, nc1);
      if(verbose){
        std::cout << "New CT NODE: " << last << " replanned: " << c1.unit1 << " cost: " << cost << " " << nc1 << "\n";
      }
      openList.push(l1);
    }
    if(tree[bestNode].con.prevWpt+1==tree[bestNode].wpts[c2.unit1].size()-1){
      minTime=GetMaxTime(bestNode,c2.unit1)-1.0; // Take off a 1-second wait action, otherwise paths will grow over and over.
    }
    if((numConflicts.second&RIGHT_CARDINAL) || !Bypass(bestNode,c2,c1.unit1,minTime)){
      last = tree.size();
      tree.resize(last+1);
      tree[last] = tree[bestNode];
      tree[last].con = c2;
      tree[last].parent = bestNode;
      tree[last].satisfiable = true;
      Replan(last);
      unsigned nc1(numConflicts.first);
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++){
        if(verbose){
          std::cout << "Agent " << y <<":\n";
          for(auto const& ff:tree[last].paths[y]){
            std::cout << ff << "\n";
          }
          std::cout << "cost: " << currentEnvironment->environment[y]->GetPathLength(tree[last].paths[y]) << "\n";
        }
        cost += currentEnvironment->environment[y]->GetPathLength(tree[last].paths[y]);
      }
      OpenListNode l1(last, cost, nc1);
      if(verbose){
        std::cout << "New CT NODE: " << last << " replanned: " << c2.unit1 << " cost: " << cost << " " << nc1 << "\n";
      }
      openList.push(l1);
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
      CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *unit = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*) this->GetMember(x);

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

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
bool CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::MakeMove(environment *ae, OccupancyInterface<state,action> *,
							 SimulationInfo<state,action,environment> * si, action& a)
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

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
bool CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::MakeMove(Unit<state, action, environment> *u, environment *e,
    SimulationInfo<state,action,environment> *si, action& a)
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

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::processSolution(double elapsed)
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
        std::cout << "Adding " << tree[bestNode].paths[x][j].t << "\n";
        break;
      }else if(j==1){
        cost += tree[bestNode].paths[x][0].t;
        std::cout << "Adding_" << tree[bestNode].paths[x][0].t << "\n";
        total += 1;
      }
    }

    //cost += currentEnvironment->environment[0]->GetPathLength(tree[bestNode].paths[x]);
    // Grab the unit
    CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *unit = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*) this->GetMember(x);

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
      std::cout << "Agent " << x << ": " << "\n";
      unsigned wpt(0);
      signed ix(0);
      for(auto &a: tree[bestNode].paths[x])
      {
        //std::cout << a << " " << wpt << " " << unit->GetWaypoint(wpt) << "\n";
        if(ix++==tree[bestNode].wpts[x][wpt])
        {
          std::cout << " *" << a << "\n";
          wpt++;
        }
        else
        {
          std::cout << "  " << a << "\n";
        }
      }
    }else{
      std::cout << "Agent " << x << ": " << "NO Path Found.\n";
    }
    if(verify){
      for(unsigned int y = x+1; y < tree[bestNode].paths.size(); y++){
        for(unsigned i(1); i<tree[bestNode].paths[x].size(); ++i){
          Vector2D A(tree[bestNode].paths[x][i-1]);
          Vector2D VA(tree[bestNode].paths[x][i]);
          VA-=A; // Direction vector
          VA.Normalize();
          for(unsigned j(1); j<tree[bestNode].paths[y].size(); ++j){
            Vector2D B(tree[bestNode].paths[y][j-1]);
            Vector2D VB(tree[bestNode].paths[y][j]);
            VB-=B; // Direction vector
            VB.Normalize();

            if(collisionImminent(A,VA,agentRadius,tree[bestNode].paths[x][i-1].t,tree[bestNode].paths[x][i].t,B,VB,agentRadius,tree[bestNode].paths[y][j-1].t,tree[bestNode].paths[y][j].t)){
              valid=false;
              std::cout << "ERROR: Solution invalid; collision at: " << tree[bestNode].paths[x][i-1] << "-->" << tree[bestNode].paths[x][i] << ", " << tree[bestNode].paths[y][j-1] << "-->" << tree[bestNode].paths[y][j] << std::endl;
            }
          }
        }
      }
    }
  }
  fflush(stdout);
  if(verify&&valid)std::cout << "VALID"<<std::endl;
  if(elapsed<0){
    std::cout << seed<<":FAILED\n";
    std::cout << seed<<":Time elapsed: " << elapsed*(-1.0) << "\n";
  }else{
    std::cout << seed<<":Time elapsed: " << elapsed << "\n";
  }
  std::cout << seed<<":expansions: " << TOTAL_EXPANSIONS << " expansions.\n";
  /*for(auto e:environments)
  {
    unsigned total=0;
    for(auto a: agentEnvs)
      if(e.environment==a)
        total++;
    std::string tmp;
    if(e.astar_weight > 1)
      tmp = "Weighted";
    std::cout << seed<<":%Environment used: " << tmp<<e.environment->name() <<": "<< total/double(agentEnvs.size())<<"\n";
  }*/
  std::cout << seed<<":Total conflicts: " << tree.size() << std::endl;
  TOTAL_EXPANSIONS = 0;
  planFinished = true;
  std::cout << seed<<":Solution cost: " << cost << "\n"; 
  std::cout << seed<<":solution length: " << total << std::endl;
  if(!keeprunning)exit(0);
}

/** Update the location of a unit */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::UpdateLocation(Unit<state, action, environment> *u, environment *e, state &loc, 
    bool success, SimulationInfo<state,action,environment> *si)
{
  u->UpdateLocation(e, loc, success, si);
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::SetEnvironment(unsigned numConflicts){
  bool set(false);
  for (int i = 0; i < this->environments[agent].size(); i++) {
    if (numConflicts >= environments[agent][i].conflict_cutoff) {
      if(verbose)std::cout << "Setting to env# " << i << " b/c " << numConflicts << " >= " << environments[i].conflict_cutoff<<environments[i].environment->name()<<std::endl;
      //std::cout<<environments[i].environment->getGoal()<<"\n";
      currentEnvironment = &(environments[i]);
      set=true;
    } else {
      break;
    }
  }
  if(!set)assert(false&&"No env was set - you need -cutoffs of zero...");

  astar.SetWeight(currentEnvironment->astar_weight);
}

/** Add a new unit with a new start and goal state to the CBS group */
// Note: this should never be called with a meta agent of size>1
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::AddUnit(Unit<state, action, environment> *u)
{
  astar.SetExternalExpansionsPtr(&TOTAL_EXPANSIONS);
  astar.SetExternalExpansionLimit(killex);

  CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *c = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*)u;
  unsigned theUnit(this->GetNumMembers());
  c->setUnitNumber(theUnit);
  // Add the new unit to the group, and construct an CBSUnit
  UnitGroup<state,action,environment>::AddUnit(u);

  activeMetaAgents.push_back(MetaAgent(theUnit));
  unitToMetaAgentMap[theUnit] = activeMetaAgents.size()-1;

  // Add the new meta-agent to the conflict matrix
  metaAgentConflictMatrix.push_back(std::vector<unsigned>(activeMetaAgents.size()));
  for(unsigned i(0); i < activeMetaAgents.size(); ++i){
    metaAgentConflictMatrix[i].push_back(0);
    metaAgentConflictMatrix.back()[i] = 0;
  }

  // Clear the constraints from the environment set
  ClearEnvironmentConstraints();

  // Setup the state and goal in the graph
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Resize the number of paths in the root of the tree
  tree[0].paths.resize(this->GetNumMembers());
  tree[0].wpts.resize(this->GetNumMembers());
  agentEnvs.resize(this->GetNumMembers());

  // Recalculate the optimum path for the root of the tree
  //std::cout << "AddUnit "<<(theUnit) << " getting path." << std::endl;
  //std::cout << "Search using " << currentEnvironment->environment[theUnit]->name() << "\n";
  agentEnvs[c->getUnitNumber()]=currentEnvironment->environment[theUnit];
  comparison::CAT = &(tree[0].cat);
  comparison::CAT->set(&tree[0].paths);
  GetFullPath<state,action,environment,comparison,conflicttable,searchalgo,maplanner>(c, astar, currentEnvironment->environment[theUnit], tree[0].paths.back(),tree[0].wpts.back(),theUnit);
  if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
      processSolution(-timer->EndTimer());
  //std::cout << "AddUnit agent: " << (theUnit) << " expansions: " << astar.GetNodesExpanded() << "\n";

  // Create new conflict avoidance table instance
  if(this->GetNumMembers()<2) tree[0].cat=conflicttable();
  // We add the optimal path to the root of the tree
  if(comparison::useCAT){
    tree[0].cat.insert(tree[0].paths.back(),currentEnvironment->environment[theUnit],tree[0].paths.size()-1);
  }
  StayAtGoal(0); // Do this every time a unit is added because these updates are taken into consideration by the CAT

  // Set the plan finished to false, as there's new updates
  planFinished = false;

  // Clear up the rest of the tree and clean the open list
  tree.resize(1);
  while(!openList.empty()) openList.pop();
  openList.push(OpenListNode(0, 0, 0));
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
double CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::GetMaxTime(int location,int agent){

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


template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::StayAtGoal(int location){

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
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
unsigned CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::LoadConstraintsForNode(int location, int agent){
  // Select the unit from the tree with the new constraint
  int theUnit(agent<0?tree[location].con.unit1:agent);
  unsigned numConflicts(0);

  // Reset the constraints in the test-environment
  ClearEnvironmentConstraints();

  // Add all of the constraints in the parents of the current node to the environment
  while(location!=0){
    if(theUnit == tree[location].con.unit1)
    {
      numConflicts++;
      AddEnvironmentConstraint(tree[location].con.c,theUnit);
      if(verbose)std::cout << "Adding constraint (in accumulation)" << tree[location].con.c.start_state << "-->" << tree[location].con.c.end_state << " for unit " << theUnit << "\n";
    }
    location = tree[location].parent;
  }// while (location != 0);
  return numConflicts;
}

// Attempts a bypass around the conflict using an alternate optimal path
// Returns whether the bypass was effective
// Note: should only be called for meta-agents with size=1
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
bool CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::Bypass(int best, Conflict<state> const& c1, unsigned otherunit, double minTime)
{
  unsigned theUnit(activeMetaAgents[c1.unit1].units[0]);
  if(nobypass)return false;
  LoadConstraintsForNode(best,theUnit);
  AddEnvironmentConstraint(c1.c,theUnit); // Add this constraint

  //std::cout << "Attempt to find a bypass.\n";

  bool success(false);
  Conflict<state> c3, c4;
  std::vector<state> newPath(tree[best].paths[theUnit]);
  std::vector<int> newWpts(tree[best].wpts[theUnit]);
  // Re-perform the search with the same constraints (since the start and goal are the same)
  CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *c = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*)this->GetMember(c1.unit1);

  // Never use conflict avoidance tree for bypass
  bool orig(comparison::useCAT);
  comparison::useCAT=false;

  state start(c->GetWaypoint(c1.prevWpt));
  state goal(c->GetWaypoint(c1.prevWpt+1));
  // Preserve proper start time
  start.t = newPath[newWpts[c1.prevWpt]].t;
  // Cost of the previous path
  double cost(currentEnvironment->environment[theUnit]->GetPathLength(newPath));
  currentEnvironment->environment[theUnit]->setGoal(goal);
  std::vector<state> path;
  
  CBSTreeNode<state,conflicttable,searchalgo> newNode(tree[best]);
  Conflict<state> t1,t2; // Temp variables
  // Perform search for the leg
  if(verbose)std::cout << "Bypass for unit " << theUnit << " on:\n";
  if(verbose)for(auto const& a:newPath){std::cout << a << "\n";}
  if(verbose)std::cout << cost << " cost\n";
  if(verbose)std::cout << openList.top().nc << " conflicts\n";
  unsigned pnum(0);
  unsigned nc1(openList.top().nc);
  // Initialize A*, etc.
  astar.GetPath(currentEnvironment->environment[theUnit],start,goal,path,minTime); // Get the path with the new constraint
  MergeLeg<state,action,environment,comparison,conflicttable,searchalgo,maplanner>(path,newPath,newWpts,c1.prevWpt, c1.prevWpt+1,minTime);
  if(fleq(currentEnvironment->environment[theUnit]->GetPathLength(newPath),cost)){
    do{
      pnum++;
      if(path.size()==0)continue;
      MergeLeg<state,action,environment,comparison,conflicttable,searchalgo,maplanner>(path,newPath,newWpts,c1.prevWpt, c1.prevWpt+1,minTime);
      newNode.paths[theUnit] = newPath;
      newNode.wpts[theUnit] = newWpts;

      if(verbose)for(auto const& a:newPath){std::cout << a << "\n";}
      // TODO do full conflict count here
      auto pconf(FindHiPriConflict(newNode,c3,c4,false)); // false since we don't care about finding cardinal conflicts
      if(verbose)std::cout<<"Path number " << pnum << "\n";
      if(verbose)for(auto const& a:newPath){std::cout << a << "\n";}
      if(verbose)std::cout << pconf.first << " conflicts\n";
      if(nc1>pconf.first){ // Is this bypass helpful?
        tree[best].paths[theUnit]=newPath;
        tree[best].wpts[theUnit]=newWpts;
        nc1=pconf.first;
        success=true;
      }
      if(pconf.first==0){
        if(verbose){std::cout << "BYPASS -- solution\n";}
        processSolution(timer->EndTimer());
        break;
      }
    }while(fleq(astar.GetNextPath(currentEnvironment->environment[theUnit],start,goal,path,minTime),cost));
  }
  TOTAL_EXPANSIONS+=astar.GetNodesExpanded();
  if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
    processSolution(-timer->EndTimer());

  if(!success)return false;
  // Add CT node with the "best" bypass
  unsigned last = tree.size();
  tree.resize(last+1);
  tree[last] = tree[bestNode];
  tree[last].con = c1;
  tree[last].parent = bestNode;
  tree[last].satisfiable = true;
  cost=0;
  for (int y = 0; y < tree[last].paths.size(); y++){
    if(verbose){
      std::cout << "Agent " << y <<":\n";
      for(auto const& ff:tree[last].paths[y]){
        std::cout << ff << "\n";
      }
      std::cout << "cost: " << currentEnvironment->environment[theUnit]->GetPathLength(tree[last].paths[y]) << "\n";
    }
    cost += currentEnvironment->environment[theUnit]->GetPathLength(tree[last].paths[y]);
  }
  OpenListNode l1(last, cost, nc1);
  if(verbose){
    std::cout << "New BYPASS NODE: " << last << " replanned: " << theUnit << " cost: " << cost << " " << nc1 << "\n";
  }
  openList.push(l1);

  comparison::useCAT=orig;

  // Make sure that the current location is satisfiable
  if (newPath.size() == 0 && !(*tree[best].paths[theUnit].begin() == *tree[best].paths[theUnit].rbegin()))
  {
    return false;
  }

  return success;
}


/** Replan a node given a constraint */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::Replan(int location)
{
  // Select the unit from the tree with the new constraint
  unsigned theMA(tree[location].con.unit1);
  if(activeMetaAgents[theMA].units.size()==1){
    unsigned theUnit(activeMetaAgents[theMA].units[0]);

    unsigned numConflicts(LoadConstraintsForNode(location));

    // Set the environment based on the number of conflicts
    SetEnvironment(numConflicts);

    // Select the unit from the group
    CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *c((CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*)GetMember(theUnit));

    // Retreive the unit start and goal
    //state start, goal;
    //c->GetStart(start);
    //c->GetGoal(goal);

    // Recalculate the path
    //std::cout << "#conflicts for " << tempLocation << ": " << numConflicts << "\n";
    //currentEnvironment->environment[theUnit]->setGoal(goal);
    //std::cout << numConflicts << " conflicts " << " using " << currentEnvironment->environment[theUnit]->name() << " for agent: " << tree[location].con.unit1 << "?="<<c->getUnitNumber()<<"\n";
    //agentEnvs[c->getUnitNumber()]=currentEnvironment->environment[theUnit];
    //astar.GetPath(currentEnvironment->environment[theUnit], start, goal, thePath);
    //std::vector<state> thePath(tree[location].paths[theUnit]);
    comparison::openList=astar.GetOpenList();
    comparison::currentEnv=(environment*)currentEnvironment->environment[theUnit];
    comparison::currentAgent=theUnit;
    comparison::CAT=&(tree[location].cat);
    comparison::CAT->set(&tree[location].paths);

    if(comparison::useCAT){
      comparison::CAT->remove(tree[location].paths[theUnit],currentEnvironment->environment[theUnit],theUnit);
    }

    double minTime(0.0);
    // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
    if(tree[location].con.prevWpt+1==tree[location].wpts[theUnit].size()-1){
      minTime=GetMaxTime(location,theUnit)-1.0; // Take off a 1-second wait action, otherwise paths will grow over and over.
    }


    //std::cout << "Replan agent " << theUnit << "\n";
    ReplanLeg<state,action,environment,comparison,conflicttable,searchalgo,maplanner>(c, astar, currentEnvironment->environment[theUnit], tree[location].paths[theUnit], tree[location].wpts[theUnit], tree[location].con.prevWpt, tree[location].con.prevWpt+1,minTime);
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
      comparison::CAT->insert(tree[location].paths[theUnit],currentEnvironment->environment[theUnit],theUnit);

    /*for(int i(0); i<thePath.size(); ++i) {
      tree[location].paths[theUnit].push_back(thePath[i]);
      }*/
  }else{
    std::vector<environment*> envs(activeMetaAgents[theMA].units.size());
    std::vector<std::pair<state,state>> instances(activeMetaAgents[theMA].units.size());
    int i(0);
    for(auto theUnit: activeMetaAgents[theMA]){
      ClearEnvironmentConstraints(theUnit);

      // Add all of the constraints in the parents of the current node to the environment
      while(location!=0){
        if(theUnit == tree[location].con.unit1)
        {
          AddEnvironmentConstraint(tree[location].con.c,theUnit);
          if(verbose)std::cout << "Adding constraint (in accumulation)" << tree[location].con.c.start_state << "-->" << tree[location].con.c.end_state << " for unit " << theUnit << "\n";
        }
        location = tree[location].parent;
      }// while (location != 0);
      envs[i]=currentEnvironment->environment[theUnit];
      // TODO: Break out waypoints and plan each leg separately! (if possible!?!)
      instances[i]={tree[location].wpts[0],tree[location].wpts[1]};
      ++i;
    }
    Solution<state> partial;
    maPlanner.GetSolution(envs,instances,partial);
    i=0;
    for(auto theUnit: activeMetaAgents[theMA]){
      double minTime(GetMaxTime(location,theUnit)-1.0); // Take off a 1-second wait action, otherwise paths will grow over and over.
      MergeLeg<state,action,environment,comparison,conflicttable,searchalgo,maplanner>(partial[i],tree[location].path[theUnit],tree[location].wpts[theUnit],0,partial[i].size()-1,minTime);

      // Make sure that the current location is satisfiable
      if (tree[location].paths[theUnit].size() < 1){
        tree[location].satisfiable = false;
      }

      // Add the path back to the tree (new constraint included)
      //tree[location].paths[theUnit].resize(0);
      if(comparison::useCAT)
        comparison::CAT->insert(tree[location].paths[theUnit],currentEnvironment->environment[theUnit],theUnit);
    }
  }
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
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
unsigned CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::HasConflict(std::vector<state> const& a, std::vector<int> const& wa, std::vector<state> const& b, std::vector<int> const& wb, int x, int y, Conflict<state> &c1, Conflict<state> &c2, std::pair<unsigned,unsigned>& conflict, bool update)
{
  // The conflict parameter contains the conflict count so far (conflict.first)
  // and the type of conflict found so far (conflict.second=BOTH_CARDINAL being the highest)

  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax(a.size());
  int ymax(b.size());
  unsigned orig(conflict.first); // Save off the original conflict count

  //if(verbose)std::cout << "Checking for conflicts between: "<<x << " and "<<y<<" ranging from:" << xmax <<"," << ymax << "\n";

  //CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>* A = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*) this->GetMember(x);
  //CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>* B = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*) this->GetMember(y);
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
      // There are 4 states landed->landed landed->not_landed not_landed->landed and not_landed->not_landed. we
      // need to check edge conflicts on all except the landed->landed states, which we do above. If one plane
      // stays landed the whole time, then there's no edge-conflict -> but if they don't stay landed the whole time
      // then there's obviously a conflict, because they both have to be doing something fishy.

      // Check for edge conflicts
      Vector2D A(a[xTime]);
      Vector2D VA(a[xNextTime]);
      VA-=A; // Direction vector
      VA.Normalize();
      Vector2D B(b[yTime]);
      Vector2D VB(b[yNextTime]);
      VB-=B; // Direction vector
      VB.Normalize();

      if(collisionImminent(A,VA,agentRadius,a[xTime].t,a[xNextTime].t,B,VB,agentRadius,b[yTime].t,b[yNextTime].t)){
        ++conflict.first;
        if(verbose)std::cout<<conflict.first<<" conflicts; #"<<x<<":" << a[xTime]<<"-->"<<a[xNextTime]<<" #"<<y<<":"<<b[yTime]<<"-->"<<b[yNextTime]<<"\n";
        if(update && (BOTH_CARDINAL!=(conflict.second&BOTH_CARDINAL))){ // Keep searching until we find a both-cardinal conflict
          // Determine conflict type
          // If there are other legal successors with succ.f()=child.f(), this is non-cardinal
          unsigned conf(NO_CONFLICT); // Left is cardinal?
          {
            double childf(currentEnvironment->environment[theUnit]->GCost(a[xTime],a[xNextTime])+currentEnvironment->environment[theUnit]->HCost(a[xNextTime],aGoal));
            std::vector<state> succ;
            currentEnvironment->environment[theUnit]->GetSuccessors(a[xTime],succ);
            bool found(false);
            for(auto const& s:succ){ // Is there at least one successor with same g+h as child?
              if(s.sameLoc(a[xNextTime])){continue;}
              if(fleq(currentEnvironment->environment[theUnit]->GCost(a[xTime],s)+currentEnvironment->environment[theUnit]->HCost(s,aGoal),childf)){found=true;break;}
            }
            if(!found){conf|=LEFT_CARDINAL;}
          }
          // Right is cardinal
          {
            double childf(currentEnvironment->environment[theUnit]->GCost(b[yTime],b[yNextTime])+currentEnvironment->environment[theUnit]->HCost(b[yNextTime],bGoal));
            std::vector<state> succ;
            currentEnvironment->environment[theUnit]->GetSuccessors(b[yTime],succ);
            bool found(false);
            for(auto const& s:succ){ // Is there at least one successor with same g+h as child?
              if(s.sameLoc(b[yNextTime])){continue;}
              if(fleq(currentEnvironment->environment[theUnit]->GCost(b[yTime],s)+currentEnvironment->environment[theUnit]->HCost(s,bGoal),childf)){found=true;break;}
            }
            if(!found){conf|=RIGHT_CARDINAL;}
          }
          // Have we increased form non-cardinal to semi-cardinal or both-cardinal?
          if(NO_CONFLICT==conflict.second || ((conflict.second<=NON_CARDINAL)&&conf) || BOTH_CARDINAL==conf){
            conflict.second=conf+1;

            Constraint<state> x_e_c(a[xTime], a[xNextTime]);
            Constraint<state> y_e_c(b[yTime], b[yNextTime]);

            c1.c = x_e_c;
            c2.c = y_e_c;

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
  return conflict.first-orig;
}

/** Find the highest priority conflict **/
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
std::pair<unsigned,unsigned> CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::FindHiPriConflict(CBSTreeNode<state,conflicttable,searchalgo> const& location, Conflict<state> &c1, Conflict<state> &c2, bool update=true)
{
  if(verbose)std::cout<<"Checking for conflicts\n";
  // prefer cardinal conflicts
  std::pair<std::pair<unsigned,unsigned>,std::pair<Conflict<state>,Conflict<state>>> best;

  for(unsigned a(0); a < activeMetaAgents.size(); ++a){
    for(unsigned b(a+1); b < activeMetaAgents.size(); ++b){
      unsigned intraConflicts(0); // Conflicts between meta-agents
      unsigned previous(best.first.second);
      // For each pair of units in the group
      for(unsigned x : activeMetaAgents.at(a).units){
        for(unsigned y : activeMetaAgents.at(b).units){
          // This call will update "best" with the number of conflicts and
          // with the *most* cardinal conflicts
          intraConflicts+=HasConflict(location.paths[x],location.wpts[x],location.paths[y],location.wpts[y],x,y,best.second.first,best.second.second,best.first,update);
        }
      }
      // Make sure that the conflict counted is the one being returned (and being translated to meta-agent indices)
      if(best.first.second>previous && (intraConflicts)){
        best.second.first.unit1=a;
        best.second.second.unit1=b;
      }
    }
  }
  metaAgentConflictMatrix[best.second.first.unit1][best.second.second.unit1]++;
  c1=best.second.first;
  c2=best.second.second;
  return best.first;
}

/** Draw the AIR CBS group */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable, class searchalgo, class maplanner>
void CBSGroup<state,action,environment,comparison,conflicttable,searchalgo,maplanner>::OpenGLDraw(const environment *ae, const SimulationInfo<state,action,environment> * sim)  const
{
	/*
	GLfloat r, g, b;
	glLineWidth(2.0);
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner> *unit = (CBSUnit<state,action,environment,comparison,conflicttable,searchalgo,maplanner>*)this->GetMember(x);
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
