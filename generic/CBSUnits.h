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
//#include "TemplateIntervalTree.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "Timer.h"
#include <string.h>

template <class state>
struct CompareLowGCost;

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
class CBSUnit;


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

// Plan path between waypoints
template <typename state, typename action, typename environment, typename comparison, typename conflicttable>
unsigned ReplanLeg(CBSUnit<state,action,environment,comparison,conflicttable>* c, TemplateAStar<state, action, environment, AStarOpenClosed<state, comparison> >& astar, environment* env, std::vector<state>& thePath, unsigned s, unsigned g)
{
  int insertPoint(-1);
  float origTime(0.0);
  float newTime(0.0);
  unsigned deletes(0);
  // Remove points from the original path (if they exist)
  if(thePath.empty()){
    assert(false && "Expected a valid path for re-planning.");
  }
  //std::cout << "re-planning path from " << s << " to " << g << " on a path of len:" << thePath.size() << "\n";
  state start(c->GetWaypoint(s));
  state goal(c->GetWaypoint(g));
  //std::cout << start << " to " << goal << "\n";
  for(unsigned n(0); n<thePath.size(); ++n){
    if(thePath[n]==start){
      insertPoint=n;
      break;
    }
  }
  for(auto n(thePath.begin()+insertPoint); n!=thePath.end(); ++n){
    deletes++;
    if(*n==goal){
      origTime=n->t;
      break;
    }
  }

  // Perform search for the leg
  std::vector<state> path;
  env->setGoal(goal);
  astar.GetPath(env, start, goal, path);
  //std::cout << "New leg " << path.size() << "\n";
  //for(auto &p: path){std::cout << p << "\n";}
  if(path.empty())return astar.GetNodesExpanded(); //no solution found
  // Update path times
  if(thePath.size()){
    float offsetTime(thePath[insertPoint].t);
    for(auto &p: path){
      p.t+=offsetTime;
    }
    newTime=path.rbegin()->t; // Save the track end time of the new leg
  }

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
  if(thePath.size()>path.size()){
      // Increase times through the end of the track
      auto newEnd(thePath.begin()+insertPoint);
      while(newEnd++ != thePath.end()){
          newEnd->t+=(newTime-origTime);
      }
  }
  //std::cout << "exp replan " << astar.GetNodesExpanded() << "\n";
  return astar.GetNodesExpanded();

}

// Plan path between waypoints
template <typename state, typename action, typename environment, typename comparison, typename conflicttable>
unsigned GetFullPath(CBSUnit<state,action,environment,comparison,conflicttable>* c, TemplateAStar<state, action, environment, AStarOpenClosed<state,comparison> >& astar, environment* env, std::vector<state>& thePath, unsigned s, unsigned g, unsigned agent)
{
  int insertPoint(-1);
  float origTime(0.0);
  float newTime(0.0);
  unsigned deletes(0);
  unsigned expansions(0);
  // Remove points from the original path (if they exist)
  if(!thePath.empty()){
    //std::cout << "re-planning path from " << s << " to " << g << " on a path of len:" << thePath.size() << "\n";
    state start(c->GetWaypoint(s));
    state goal(c->GetWaypoint(g));
    for(unsigned n(0); n<thePath.size(); ++n){
      if(thePath[n]==start){
        insertPoint=n;
        break;
      }
    }
    for(auto n(thePath.begin()+insertPoint); n!=thePath.end(); ++n){
      deletes++;
      if(*n==goal){
        origTime=n->t;
        break;
      }
    }
  }

  // Perform search for all legs
  unsigned offset(0);
  comparison::currentEnv=(environment*)env;
  if(comparison::useCAT){
    comparison::currentAstar=&astar;
    comparison::currentAgent=agent;
  }
  for(int i=s; i<g; ++i){
    std::vector<state> path;
    state start(thePath.size()?thePath.back():c->GetWaypoint(i));
    start.landed=false;
    start.t=0;
    state goal(c->GetWaypoint(i+1));
    env->setGoal(goal);
    astar.GetPath(env, start, goal, path);
    expansions += astar.GetNodesExpanded();
    //std::cout << "exp full " << astar.GetNodesExpanded() << "\n";
    if(path.empty()){return expansions;} //no solution found
    // Update path times
    if(thePath.size()){
      float offsetTime(insertPoint==-1?thePath.rbegin()->t:thePath[insertPoint].t);
      for(auto &p: path){
        p.t+=offsetTime;
      }
      newTime=path.rbegin()->t; // Save the track end time of the new leg
    }
    //std::cout << "Got path of len " << path.size() << "\nAdding to main path of len "<<thePath.size() << "\n";
    // Append to the entire path, omitting the first node for subsequent legs
    bool append(insertPoint==-1);

    // Insert new path in front of the insert point
    thePath.insert(append?thePath.end():thePath.begin()+insertPoint,path.begin()+offset,path.end());
    if(!append)
      insertPoint += path.size()-offset;

    if(!append){
        //Erase the original subpath including the start node
        thePath.erase(thePath.begin()+insertPoint,thePath.begin()+insertPoint+deletes);
        if(thePath.size()>path.size()){
            // Increase times through the end of the track
            auto newEnd(thePath.begin()+insertPoint);
            while(newEnd++ != thePath.end()){
                newEnd->t+=(newTime-origTime);
            }
        }
    }
    offset=1;
    //std::cout << "Planned leg " << goal << "\n";
  }
  return expansions;
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
class CBSUnit : public Unit<state, action, environment> {
public:
	CBSUnit(std::vector<state> const &gs)
	:start(0), goal(1), current(gs[0]), waypoints(gs) {}
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
	void PushFrontPath(std::vector<state> &s)
	{
		std::vector<state> newPath;
		for (state x : s)
			newPath.push_back(x);
		for (state y : myPath)
			newPath.push_back(y);
		myPath = newPath;
	}
	inline std::vector<state> const& GetPath()const{return myPath;}
        void UpdateGoal(state &start, state &goal);
        void setUnitNumber(unsigned n){number=n;}
        unsigned getUnitNumber()const{return number;}

private:
	unsigned start, goal;
        state current;
	std::vector<state> waypoints;
	std::vector<state> myPath;
        unsigned number;
};

template<typename state>
struct Conflict {
	Constraint<state> c;
	int unit1;
        int prevWpt;
};

template<typename state, typename conflicttable>
struct CBSTreeNode {
	CBSTreeNode():parent(0),satisfiable(true),cat(){}
	std::vector< std::vector<state> > paths;
	Conflict<state> con;
	unsigned int parent;
	bool satisfiable;
        //IntervalTree cat; // Conflict avoidance table
        conflicttable cat; // Conflict avoidance table
};

template<typename state, typename conflicttable>
static std::ostream& operator <<(std::ostream & out, const CBSTreeNode<state,conflicttable> &act)
{
	out << "(paths:"<<act.paths.size()<<", parent: "<<act.parent<< ", satisfiable: "<<act.satisfiable<<")";
	return out;
}

template<typename state, typename action, typename environ>
struct EnvironmentContainer {
	EnvironmentContainer() : name("NULL ENV"), environment(0), heuristic(0), conflict_cutoff(0), astar_weight(0.0f) {}
	EnvironmentContainer(std::string n, environ * e, Heuristic<state>* h, uint32_t conf, float a) : name(n), environment(e), heuristic(h), conflict_cutoff(conf), astar_weight(a) {}
	environ * environment;
	Heuristic<state>* heuristic;
	uint64_t conflict_cutoff;
	float astar_weight;
	std::string name;
};


template <typename state, typename action, typename environment, typename comparison, typename conflicttable>
class CBSGroup : public UnitGroup<state, action, environment>
{
  public:
    CBSGroup(std::vector<EnvironmentContainer<state,action,environment> > const&);
    bool MakeMove(Unit<state, action, environment> *u, environment *e, 
        SimulationInfo<state,action,environment> *si, action& a);
    void UpdateLocation(Unit<state, action, environment> *u, environment *e, 
        state &loc, bool success, SimulationInfo<state,action,environment> *si);
    void AddUnit(Unit<state, action, environment> *u);
    void UpdateUnitGoal(Unit<state, action, environment> *u, state newGoal);
    void UpdateSingleUnitPath(Unit<state, action, environment> *u, state newGoal);

    void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *)  const;
    double getTime() {return time;}
    bool donePlanning() {return planFinished;}
    bool ExpandOneCBSNode();

    std::vector<CBSTreeNode<state,conflicttable> > tree;
    void processSolution(double);
  private:    

    unsigned LoadConstraintsForNode(int location);
    bool Bypass(int best, unsigned numConflicts, Conflict<state> const& c1);
    void Replan(int location);
    unsigned HasConflict(std::vector<state> const& a, std::vector<state> const& b, int x, int y, Conflict<state> &c1, Conflict<state> &c2, bool update, bool verbose=false);
    unsigned FindFirstConflict(CBSTreeNode<state,conflicttable>  const& location, Conflict<state> &c1, Conflict<state> &c2);

    bool planFinished;

    /* Code for dealing with multiple environments */
    std::vector<EnvironmentContainer<state,action,environment> > environments;
    EnvironmentContainer<state,action,environment>* currentEnvironment;

    void SetEnvironment(unsigned);
    void ClearEnvironmentConstraints();
    void AddEnvironmentConstraint(Constraint<state> c);

    TemplateAStar<state, action, environment, AStarOpenClosed<state, comparison > > astar;
    TemplateAStar<state, action, environment, AStarOpenClosed<state, CompareLowGCost<state> > > astar2;
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
          return (left.nc==right.nc)?(left.cost > right.cost):(left.nc>right.nc);
        else
          return (left.cost==right.cost)?(left.nc > right.nc):(left.cost>right.cost);
      }
    };

    std::priority_queue<CBSGroup::OpenListNode, std::vector<CBSGroup::OpenListNode>, CBSGroup::OpenListNodeCompare> openList;

    uint TOTAL_EXPANSIONS = 0;

    std::vector<SearchEnvironment<state,action>*> agentEnvs;
public:
    // Algorithm parameters
    bool nobypass;
    bool ECBSheuristic; // For ECBS
    unsigned killex; // Kill after this many expansions
    bool keeprunning; // Whether to keep running after finding the answer (e.g. for ui)
    int seed;
    Timer* timer;
    static bool greedyCT;
};

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
bool CBSGroup<state,action,environment,comparison,conflicttable>::greedyCT=false;

/** AIR CBS UNIT DEFINITIONS */

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSUnit<state,action,environment,comparison,conflicttable>::SetPath(std::vector<state> &p)
{
  myPath = p;
  std::reverse(myPath.begin(), myPath.end());
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSUnit<state,action,environment,comparison,conflicttable>::OpenGLDraw(const environment *ae, 
    const SimulationInfo<state,action,environment> *si) const
{
  GLfloat r, g, b;
  this->GetColor(r, g, b);
  ae->SetColor(r, g, b);

  if (myPath.size() > 1) {
    // Interpolate between the two given the timestep
    state start_t = myPath[myPath.size()-1];
    state stop_t = myPath[myPath.size()-2];

    if (si->GetSimulationTime() <= stop_t.t && si->GetSimulationTime() >= start_t.t)
    {
      float perc = (stop_t.t - si->GetSimulationTime())/(stop_t.t - start_t.t);
      ae->OpenGLDraw(stop_t, start_t, perc);
      Constraint<state> c(stop_t, start_t);
      glColor3f(1, 0, 0);
      c.OpenGLDraw();
    } else {		
      ae->OpenGLDraw(current);
      glColor3f(1, 0, 0);
      Constraint<state> c(current);
      c.OpenGLDraw();
    }
  } else {
    if (current.landed)
      return;
    ae->OpenGLDraw(current);
    Constraint<state> c(current);
    glColor3f(1, 0, 0);
    c.OpenGLDraw();
  }
}

/*void CBSUnit<state,action,environment,comparison,conflicttable>::UpdateGoal(state &s, state &g)
  {
  start = s;
  goal = g;
  }*/

//----------------------------------------------------------------------------------------------------------------------------------------------//

/** CBS GROUP DEFINITIONS */


template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::ClearEnvironmentConstraints(){
  for (EnvironmentContainer<state,action,environment> env : this->environments) {
    env.environment->ClearConstraints();
  }
}


template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::AddEnvironmentConstraint(Constraint<state>  c){
  for (EnvironmentContainer<state,action,environment> env : this->environments) {
    env.environment->AddConstraint(c);
  }
}

/** constructor **/
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
CBSGroup<state,action,environment,comparison,conflicttable>::CBSGroup(std::vector<EnvironmentContainer<state,action,environment> > const& environs) : time(0), bestNode(0), planFinished(false), nobypass(false)
    , ECBSheuristic(false), killex(INT_MAX), keeprunning(false), seed(1234567),
    timer(0)
{
  //std::cout << "THRESHOLD " << threshold << "\n";

  tree.resize(1);
  tree[0].parent = 0;
  environments = environs;

  // Sort the environment container by the number of conflicts
  std::sort(this->environments.begin(), this->environments.end(), 
      [](const EnvironmentContainer<state,action,environment>& a, const EnvironmentContainer<state,action,environment>& b) -> bool 
      {
      return a.conflict_cutoff < b.conflict_cutoff;
      }
      );

  // Set the current environment to that with 0 conflicts
  SetEnvironment(0);

}


/** Expand a single CBS node */
// Return true while processing
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
bool CBSGroup<state,action,environment,comparison,conflicttable>::ExpandOneCBSNode()
{
  // There's no reason to expand if the plan is finished.
  if (planFinished)
    return false;

  Conflict<state> c1, c2;
  unsigned long last = tree.size();

  unsigned numConflicts(FindFirstConflict(tree[bestNode], c1, c2));
  // If not conflicts are found in this node, then the path is done
  if (numConflicts==0)
  {
    processSolution(timer->EndTimer());
  } 

  // Otherwise, we need to deal with the conflicts
  else
  {
    // Notify the user of the conflict
    std::cout << "Conflict found between unit " << c1.unit1 << " and unit " << c2.unit1 << " @:" << c2.c.start() <<  " and " << c1.c.start() << " NC " << numConflicts << " prev-W " << c1.prevWpt << " " << c2.prevWpt << "\n";

    // Don't create new nodes if either bypass was successful
    // Note, these calls will add nodes to the openList
    if(!Bypass(bestNode,numConflicts,c1) && !Bypass(bestNode,numConflicts,c2))
      //Bypass(bestNode,numConflicts,c1,gui);
      //Bypass(bestNode,numConflicts,c2,gui);
    {
      last = tree.size();

      // Add two nodes to the tree for each of the children
      tree.resize(last+2);
      //std::cout << "Tree has " << tree.size() << "\n";
      // The first node contains the conflict c1
      tree[last] = tree[bestNode];
      tree[last].con = c1;
      tree[last].parent = bestNode;
      tree[last].satisfiable = true;

      // The second node constains the conflict c2
      tree[last+1] = tree[bestNode];
      tree[last+1].con = c2;
      tree[last+1].parent = bestNode;
      tree[last+1].satisfiable = true;

      // We now replan in the tree for both of the child nodes
      Replan(last);
      Replan(last+1);
      unsigned nc1(numConflicts);
      unsigned nc2(numConflicts);
      //unsigned nc1(FindFirstConflict(tree[last], c1, c2));
      //unsigned nc2(FindFirstConflict(tree[last+1], c1, c2));


      // Add the new nodes to the open list
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++)
        cost += currentEnvironment->environment->GetPathLength(tree[last].paths[y]);
      OpenListNode l1(last, cost, nc1);
      //std::cout << "New CT NODE: " << last << " " << cost << " " << nc1 << "\n";
      openList.push(l1);

      cost = 0;
      for (int y = 0; y < tree[last+1].paths.size(); y++)
        cost += currentEnvironment->environment->GetPathLength(tree[last+1].paths[y]);
      OpenListNode l2(last+1, cost, nc2);
      //std::cout << "New CT NODE: " << last+1 << " " << cost << " " << nc2 << "\n";
      openList.push(l2);
    }

    // Get the best node from the top of the open list, and remove it from the list
    bestNode = openList.top().location;
    openList.pop();

    // Set the visible paths for every unit in the node
    for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
    {
      // Grab the unit
      CBSUnit<state,action,environment,comparison,conflicttable> *unit = (CBSUnit<state,action,environment,comparison,conflicttable>*) this->GetMember(x);

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

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
bool CBSUnit<state,action,environment,comparison,conflicttable>::MakeMove(environment *ae, OccupancyInterface<state,action> *,
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
/*else if (false){//myPath.size() <= 1) { // Don't plan a next goal...
     if (current.landed)
     {
       return false;
     } else {

  // With a random probability - either land or keep flying around.
  if (rand()%5 == 0) {
  // Replan the node to a landing location
  state land(18, 23, 0, 0, 0, true);
  state newGoal(land, 0);
  CBSGroup* g = (CBSGroup*) this->GetUnitGroup();
  //g->UpdateUnitGoal(this, newGoal);			
  g->UpdateSingleUnitPath(this, newGoal);
  } else {
  // Replan the node to a random location
  state rs(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
  state newGoal(rs, 0);
  CBSGroup* g = (CBSGroup*) this->GetUnitGroup();
  //g->UpdateUnitGoal(this, newGoal);
  g->UpdateSingleUnitPath(this, newGoal);
  }
  }
  }*/
  return false;
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
bool CBSGroup<state,action,environment,comparison,conflicttable>::MakeMove(Unit<state, action, environment> *u, environment *e,
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

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::processSolution(double elapsed)
{

  double cost(0.0);
  unsigned total(0);
  // For every unit in the node
  for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
  {
    cost += currentEnvironment->environment->GetPathLength(tree[bestNode].paths[x]);
    total += tree[bestNode].paths[x].size();
    // Grab the unit
    CBSUnit<state,action,environment,comparison,conflicttable> *unit = (CBSUnit<state,action,environment,comparison,conflicttable>*) this->GetMember(x);

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
    /*std::cout << "Agent " << x << ": " << "\n";
    unsigned wpt(0);
    for(auto &a: tree[bestNode].paths[x])
    {
        //std::cout << a << " " << wpt << " " << unit->GetWaypoint(wpt) << "\n";
        if(a==unit->GetWaypoint(wpt))
        {
            std::cout << " *" << a << "\n";
            wpt++;
        }
        else
        {
            std::cout << "  " << a << "\n";
        }
    }*/
  }
  fflush(stdout);
  if(elapsed<0){
    std::cout << "FAILED\n";
    std::cout << "Finished with failure using " << TOTAL_EXPANSIONS << " expansions.\n";
    std::cout << seed<<":Time elapsed: " << elapsed*(-1.0) << "\n";
  }else{
    std::cout << "Finished the plan using " << TOTAL_EXPANSIONS << " expansions.\n";
    std::cout << seed<<":Time elapsed: " << elapsed << "\n";
  }
  for(auto e:environments)
  {
    unsigned total=0;
    for(auto a: agentEnvs)
      if(e.environment==a)
        total++;
    std::string tmp;
    if(e.astar_weight > 1)
      tmp = "Weighted";
    std::cout << seed<<":%Environment used: " << tmp<<e.environment->name() <<": "<< total/double(agentEnvs.size())<<"\n";
  }
  std::cout << seed<<":Total conflicts: " << tree.size() << std::endl;
  TOTAL_EXPANSIONS = 0;
  planFinished = true;
  std::cout << seed<<":Solution cost: " << cost << "\n"; 
  std::cout << seed<<":solution length: " << total << std::endl;
  if(!keeprunning)exit(0);
}

/** Update the location of a unit */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::UpdateLocation(Unit<state, action, environment> *u, environment *e, state &loc, 
    bool success, SimulationInfo<state,action,environment> *si)
{
  u->UpdateLocation(e, loc, success, si);
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::SetEnvironment(unsigned numConflicts){
  bool set(false);
  for (int i = 0; i < this->environments.size(); i++) {
    if (numConflicts >= environments[i].conflict_cutoff) {
      //std::cout << "Setting to env# " << i << " b/c " << numConflicts << " >= " << environments[i].conflict_cutoff<<environments[i].environment->name()<<std::endl;
      //std::cout<<environments[i].environment->getGoal()<<"\n";
      currentEnvironment = &(environments[i]);
      set=true;
    } else {
      break;
    }
  }
  if(!set)assert(false&&"No env was set - you need -cutoffs of zero...");

  astar.SetHeuristic(currentEnvironment->heuristic);
  astar.SetWeight(currentEnvironment->astar_weight);
}

/** Add a new unit with a new start and goal state to the CBS group */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::AddUnit(Unit<state, action, environment> *u)
{
  CBSUnit<state,action,environment,comparison,conflicttable> *c = (CBSUnit<state,action,environment,comparison,conflicttable>*)u;
  c->setUnitNumber(this->GetNumMembers());
  // Add the new unit to the group, and construct an CBSUnit
  UnitGroup<state,action,environment>::AddUnit(u);

  // Clear the constraints from the environment set
  ClearEnvironmentConstraints();

  // Setup the state and goal in the graph
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Resize the number of paths in the root of the tree
  tree[0].paths.resize(this->GetNumMembers());
  agentEnvs.resize(this->GetNumMembers());

  // Recalculate the optimum path for the root of the tree
  //std::cout << "AddUnit "<<(this->GetNumMembers()-1) << " getting path." << std::endl;
  //std::cout << "Search using " << currentEnvironment->environment->name() << "\n";
  std::vector<state> thePath;
  agentEnvs[c->getUnitNumber()]=currentEnvironment->environment;
  comparison::CAT = &(tree[0].cat);
  comparison::CAT->set(&tree[0].paths);
  TOTAL_EXPANSIONS+=GetFullPath<state,action,environment,comparison,conflicttable>(c, astar, currentEnvironment->environment, thePath, 0,c->GetNumWaypoints()-1,this->GetNumMembers()-1);
  if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
      processSolution(-timer->EndTimer());
  //std::cout << "AddUnit agent: " << (this->GetNumMembers()-1) << " expansions: " << astar.GetNodesExpanded() << "\n";

  // Create new conflict avoidance table instance
  if(this->GetNumMembers()<2) tree[0].cat=conflicttable();
  //IntervalTree::intervalVector info;
  // We add the optimal path to the root of the tree
  for(int i(0); i<thePath.size(); ++i) {
    // Populate the interval tree
    if(comparison::useCAT){
      tree[0].cat.insert(thePath,currentEnvironment->environment,tree[0].paths.size()-1);
    }
    tree[0].paths.back().push_back(thePath[i]);
    //std::cout << "PATH " << thePath.size() << " tree SIZE,DEPTH " << tree[0].cat.size() << "," << tree[0].cat.depth() << "\n";
  }
  /*if(info.size()){
    tree[0].cat=conflicttable();
  }*/

  // Set the plan finished to false, as there's new updates
  planFinished = false;

  // Clear up the rest of the tree and clean the open list
  tree.resize(1);
  while(!openList.empty()) openList.pop();
  openList.push(OpenListNode(0, 0, 0));
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::UpdateUnitGoal(Unit<state, action, environment> *u, state newGoal)
{

  //std::cout << "Replanning units..." << std::endl;

  //std::cout << "Clearing open list..." << std::endl;
  // Clear the tree and the open list
  tree.resize(1);
  while(!openList.empty()) openList.pop();

  //std::cout << "Resizing the main tree..." << std::endl;
  // Resize the number of paths in the root of the tree
  tree[0].paths.resize(this->GetNumMembers());

  //std::cout << "Clear the environmental constraints" << std::endl;
  // Clear the constraints from the environment set
  ClearEnvironmentConstraints();


  //std::cout << "Beginning to update members..." << std::endl;
  // Update the start for all of the units
  for (int x = 0; x < this->GetNumMembers(); x++)
  {

    //std::cout << "Updating member " << (x + 1) << " of " << this->GetNumMembers() << std::endl;

    // Get the unit
    CBSUnit<state,action,environment,comparison,conflicttable> *c = (CBSUnit<state,action,environment,comparison,conflicttable>*)this->GetMember(x);
    if(c!=u) continue;

    // Obtain the unit's current location and current goal
    //state current = *(tree[0].paths[x].rbegin());
    state current, goal;
    if (c->GetPath().size() > 1) {
      current = c->GetPath()[c->GetPath().size()-2];
    } else {
      c->GetLocation(current);
    }
    c->GetGoal(goal);

    // Update the start of that unit to be their current location and the goal to be the new goal
    //c->UpdateGoal(current, (this->GetMember(x) != u ? goal : newGoal));


    //std::cout << "Planning optimal path from " << current << " to " << goal << std::endl;
    // Replan the unit's optimal path
    //astar.GetPath(currentEnvironment->environment, current, goal, thePath);
    std::vector<state> thePath;
    DoHAStar(current, goal, thePath);
    TOTAL_EXPANSIONS += astar.GetNodesExpanded();
    if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
      processSolution(-timer->EndTimer());
    //std::cout << "exp replan " << astar.GetNodesExpanded() << "\n";

    //std::cout << "Got optimal path" << std::endl;
    // Add the optimal path to the root of the tree
    tree[0].paths[x].resize(0);
    if (c->GetPath().size() > 1){
      state cur;
      c->GetLocation(cur);
      tree[0].paths[x].push_back(cur);
    }
    for (unsigned int i = 0; i < thePath.size(); i++)
    {
      tree[0].paths[x].push_back(thePath[i]);
    }

    //for(auto &a : tree[0].paths[x]){
      //std::cout << " --  " << a << "\n";
    //}
  }

  //std::cout << "Adding the root to the open list" << std::endl;

  // Add the root of the node to the open list
  openList.push(OpenListNode(0, 0, 0));

  // Clean up the root node
  tree[0].parent = 0;
  tree[0].satisfiable = true;
  bestNode = 0;

  // Set if the plan is finished to false
  planFinished = false;
}

/** Update Unit Path */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::UpdateSingleUnitPath(Unit<state, action, environment> *u, state newGoal)
{

  //std::cout << "Replanning Single Unit Path..." << std::endl;

  //std::cout << "Clear the environmental constraints" << std::endl;
  // Clear the constraints from the environment set
  ClearEnvironmentConstraints();

  // Add constraints for the rest of the units' pre-planned paths
  for (int x = 0; x < this->GetNumMembers(); x++) {
    if (this->GetMember(x) != u) {
      CBSUnit<state,action,environment,comparison,conflicttable> *c = (CBSUnit<state,action,environment,comparison,conflicttable>*)this->GetMember(x);

      // Loop over the pre-planned path
      for (int i = 0; i < c->GetPath().size(); i++) {
        // Add a vertex constraint. If you're landed, you don't collide with
        // anything
        if (!(c->GetPath()[i].landed)) {
          Constraint<state> c1(c->GetPath()[i]);
          AddEnvironmentConstraint(c1);
        }

        // If we can add an edge constraint (because we know where we're going...)
        if (i +1 < c->GetPath().size()) {
          // Add edge consraints if the plane is landing, taking off, or 
          // just in the air. If both states stay landed, no edge constraint
          if (!(c->GetPath()[i].landed && !c->GetPath()[i+1].landed) ||
              !(c->GetPath()[i].landed && c->GetPath()[i+1].landed) || 
              (c->GetPath()[i].landed && !c->GetPath()[i+1].landed)
             ) 
          {
            Constraint<state> c2(c->GetPath()[i], c->GetPath()[i+1]);
            AddEnvironmentConstraint(c2);
          }
        }
      } /* End loop over pre-planned path */
    } /* End if */
  } /* End for */


  // Get the unit location
  CBSUnit<state,action,environment,comparison,conflicttable> *c = (CBSUnit<state,action,environment,comparison,conflicttable>*)u;
  state current = c->GetPath()[0];

  // Plan a new path for the unit
  //std::cout << "Going from " << current << " to " << newGoal << std::endl;
  //astar.GetPath(currentEnvironment->environment, current, newGoal, thePath);
  std::vector<state> thePath;
  DoHAStar(current, newGoal, thePath);
  //std::cout << "Finished replanning with " << astar.GetNodesExpanded() << " expansions." << std::endl;

  //std::cout << "New Path: ";
  //for (auto &a : thePath)
  //std::cout << a << " ";
  //std::cout << std::endl;

  // Update the Unit Path
  c->SetPath(thePath);
}

// Loads conflicts into environements and returns the number of conflicts loaded.
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
unsigned CBSGroup<state,action,environment,comparison,conflicttable>::LoadConstraintsForNode(int location){
  // Select the unit from the tree with the new constraint
  int theUnit(tree[location].con.unit1);
  unsigned numConflicts(0);

  // Reset the constraints in the test-environment
  ClearEnvironmentConstraints();

  // Add all of the constraints in the parents of the current node to the environment
  do {
    if(theUnit == tree[location].con.unit1)
    {
      numConflicts++;
      AddEnvironmentConstraint(tree[location].con.c);
    }
    location = tree[location].parent;
  } while (location != 0);
  //Implement ECBS prioritization which penalizes the second element in a path.
  if(ECBSheuristic && strstr(currentEnvironment->environment->name(),"Highway")==NULL && tree[location].paths[tree[location].con.unit1].size()>1)
    AddEnvironmentConstraint(Constraint<state>(tree[location].paths[tree[location].con.unit1][1]));
  return numConflicts;
}

// Attempts a bypass around the conflict using an alternate optimal path
// Returns whether the bypass was effective
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
bool CBSGroup<state,action,environment,comparison,conflicttable>::Bypass(int best, unsigned numConflicts, Conflict<state> const& c1)
{
  if(nobypass)return false;
  LoadConstraintsForNode(best);

  //std::cout << "Attempt to find a bypass.\n";

  bool success(false);
  Conflict<state> c3, c4;
  std::vector<state> newPath(tree[best].paths[c1.unit1]);
  // Re-perform the search with the same constraints (since the start and goal are the same)
  CBSUnit<state,action,environment,comparison,conflicttable> *c = (CBSUnit<state,action,environment,comparison,conflicttable>*)this->GetMember(c1.unit1);
  //currentEnvironment->environment->setGoal(*tree[best].paths[c1.unit1].rbegin());
  //astar2.SetStopAfterAllOpt(true);
  astar2.noncritical=true; // Because it's bypass, we can kill early if the search prolongs. this var is reset internally by the routine.
  astar2.SetWeight(currentEnvironment->astar_weight);
  //astar2.GetPath(currentEnvironment->environment, *tree[best].paths[c1.unit1].begin(), *tree[best].paths[c1.unit1].rbegin(), newPath);
  //TOTAL_EXPANSIONS += astar2.GetNodesExpanded();

  // Never use conflict avoidance tree for bypass
  bool orig(comparison::useCAT);
  comparison::useCAT=false;
  // TODO fix this to replan in reverse
  TOTAL_EXPANSIONS += ReplanLeg<state,action,environment,comparison,conflicttable>(c, astar, currentEnvironment->environment, newPath, c1.prevWpt, c1.prevWpt+1);
    if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
      processSolution(-timer->EndTimer());
  comparison::useCAT=orig;

  // Make sure that the current location is satisfiable
  if (newPath.size() == 0 && !(*tree[best].paths[c1.unit1].begin() == *tree[best].paths[c1.unit1].rbegin()))
  {
    return false;
  }

  CBSTreeNode<state,conflicttable> newNode(tree[best]);
  newNode.paths[c1.unit1] = newPath;
  unsigned bypassConflicts(FindFirstConflict(newNode, c3, c4));
  if(bypassConflicts < numConflicts)// && newPath.size() <= tree[best].paths[c1.unit1].size())
  {
    success = true;
    //std::cout << "Found a bypass! expansions:" << astar2.GetNodesExpanded() << " len" <<newPath.size()<<" == "<<tree[best].paths[c1.unit1].size() << "\n\n\n";
    if(bypassConflicts==0)
    {
      tree[best].paths[c1.unit1]=newPath;
      processSolution(timer->EndTimer());
      return true;
    }
    else
    {
      // Add two nodes to the tree for each of the children
      unsigned long last(tree.size());

      tree.resize(last+2);
      tree[last]=newNode;
      tree[last].con = c3;
      // Note: parent does not change
      tree[last].satisfiable = true;

      tree[last+1] = newNode;
      tree[last+1].con = c4;
      // Note: parent does not change
      tree[last+1].satisfiable = true;

      // Issue tickets on the path
      //Replan(last); //Replan not necessary since we already have the path...
      //Replan(last+1);
      unsigned nc1(numConflicts);
      unsigned nc2(numConflicts);
      //unsigned nc1(FindFirstConflict(tree[last], c3, c4));
      //unsigned nc2(FindFirstConflict(tree[last+1], c3, c4));

      // Add the new nodes to the open list
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++)
        cost += currentEnvironment->environment->GetPathLength(tree[last].paths[y]);
      OpenListNode l1(last, cost, nc1);
      //std::cout << "New CT NODE: " << last << " " << cost << " " << nc1 << "\n";
      openList.push(l1);

      cost = 0;
      for (int y = 0; y < tree[last+1].paths.size(); y++)
        cost += currentEnvironment->environment->GetPathLength(tree[last+1].paths[y]);
      OpenListNode l2(last+1, cost, nc2);
      //std::cout << "New CT NODE: " << last+1 << " " << cost << " " << nc2 << "\n";
      openList.push(l2);
    }
  }


  return success;
}


/** Replan a node given a constraint */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::Replan(int location)
{
  // Select the unit from the tree with the new constraint
  int theUnit = tree[location].con.unit1;

  unsigned numConflicts(LoadConstraintsForNode(location));

  // Set the environment based on the number of conflicts
  SetEnvironment(numConflicts);

  // Select the air unit from the group
  CBSUnit<state,action,environment,comparison,conflicttable> *c = (CBSUnit<state,action,environment,comparison,conflicttable>*)this->GetMember(theUnit);

  // Retreive the unit start and goal
  //state start, goal;
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Recalculate the path
  //std::cout << "#conflicts for " << tempLocation << ": " << numConflicts << "\n";
  //currentEnvironment->environment->setGoal(goal);
  std::cout << numConflicts << " conflicts " << " using " << currentEnvironment->environment->name() << " for agent: " << tree[location].con.unit1 << "?="<<c->getUnitNumber()<<"\n";
  //agentEnvs[c->getUnitNumber()]=currentEnvironment->environment;
  //astar.GetPath(currentEnvironment->environment, start, goal, thePath);
  std::vector<state> thePath(tree[location].paths[theUnit]);
  comparison::currentAstar=&astar;
  comparison::currentEnv=(environment*)currentEnvironment->environment;
  comparison::currentAgent=theUnit;
  comparison::CAT=&(tree[location].cat);
  comparison::CAT->set(&tree[location].paths);
  
  if(comparison::useCAT){
    comparison::CAT->remove(thePath,currentEnvironment->environment,theUnit);
  }

  TOTAL_EXPANSIONS += ReplanLeg<state,action,environment,comparison,conflicttable>(c, astar, currentEnvironment->environment, thePath, tree[location].con.prevWpt, tree[location].con.prevWpt+1);
    if(killex != INT_MAX && TOTAL_EXPANSIONS>killex)
      processSolution(-timer->EndTimer());

  //DoHAStar(start, goal, thePath);
  //TOTAL_EXPANSIONS += astar.GetNodesExpanded();
  //std::cout << "Replan agent: " << location << " expansions: " << astar.GetNodesExpanded() << "\n";

  // Make sure that the current location is satisfiable
  if (thePath.size() > 1){
    tree[location].satisfiable = false;
  }

  // Add the path back to the tree (new constraint included)
  tree[location].paths[theUnit].resize(0);
  if(comparison::useCAT)
    comparison::CAT->insert(thePath,currentEnvironment->environment,theUnit);

  for(int i(0); i<thePath.size(); ++i) {
    tree[location].paths[theUnit].push_back(thePath[i]);
  }

  // Issue tickets on the path
}

template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
unsigned CBSGroup<state,action,environment,comparison,conflicttable>::HasConflict(std::vector<state> const& a, std::vector<state> const& b, int x, int y, Conflict<state> &c1, Conflict<state> &c2, bool update, bool verbose)
{
  unsigned numConflicts(0);
  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax = a.size();
  int ymax = b.size();

  if(verbose)std::cout << "Checking for conflicts between: "<<x << " and "<<y<<" ranging from:" << xmax <<"," << ymax << " update: " << update << "\n";

  CBSUnit<state,action,environment,comparison,conflicttable>* A = (CBSUnit<state,action,environment,comparison,conflicttable>*) this->GetMember(x);
  CBSUnit<state,action,environment,comparison,conflicttable>* B = (CBSUnit<state,action,environment,comparison,conflicttable>*) this->GetMember(y);
  //std::cout << "x,y "<<x<<" "<<y<<"\n";
  signed pwptA(-1);
  signed pwptB(-1);
  int pxTime(-1);
  int pyTime(-1);
  for (int i = 0, j = 0; j < ymax && i < xmax;) // If we've reached the end of one of the paths, then time is up and 
    // no more conflicts could occur
  {
    // I and J hold the current step in the path we are comparing. We need 
    // to check if the current I and J have a conflict, and if they do, then
    // we have to deal with it, if not, then we don't.

    // Figure out which indices we're comparing
    int xTime = max(0, min(i, xmax-1));
    int yTime = max(0, min(j, ymax-1));

    // Check if we're looking directly at a waypoint.
    // Increment so that we know we've passed it.
    if(update){
        //std::cout << "if(xTime != pxTime && A->GetWaypoint(pwptA+1)==a[xTime]){++pwptA; pxTime=xTime;}\n";
        //std::cout << " " << xTime << " " << pxTime << " " << pwptA;std::cout << " " << A->GetWaypoint(pwptA+1) << " " << a[xTime] << "==?" << (A->GetWaypoint(pwptA+1)==a[xTime]) <<  "\n";
        //std::cout << "if(yTime != pyTime && B->GetWaypoint(pwptB+1)==b[yTime]){++pwptB; pyTime=yTime;}\n";
        //std::cout << " " << yTime << " " << pyTime << " " << pwptB;std::cout << " " << B->GetWaypoint(pwptB+1) << " " << b[yTime] << "==?" << (B->GetWaypoint(pwptB+1)==b[yTime]) <<  "\n";
        if(xTime != pxTime && A->GetWaypoint(pwptA+1)==a[xTime]){++pwptA; pxTime=xTime;}
        if(yTime != pyTime && B->GetWaypoint(pwptB+1)==b[yTime]){++pwptB; pyTime=yTime;}
    }

    if(verbose)std::cout << "Looking at positions " << xTime <<":"<<a[xTime].t << "," << j<<":"<<b[yTime].t << std::endl;

    // Check the point constraints
    Constraint<state> x_c(a[xTime]);
    state y_c =b[yTime];


    // Deal with landing conflicts, we don't conflict if one of the planes stays landed at
    // the entire time
    if (!(a[xTime].landed && a[min(xTime + 1, xmax-1)].landed || 
          b[yTime].landed && b[min(yTime + 1, xmax-1)].landed)) 
    {
      // There are 4 states landed->landed landed->not_landed not_landed->landed and not_landed->not_landed. we
      // need to check edge conflicts on all except the landed->landed states, which we do above. If one plane
      // stays landed the whole time, then there's no edge-conflict -> but if they don't stay landed the whole time
      // then there's obviously a conflict, because they both have to be doing something fishy.


      // Check the vertex conflict
      if (x_c.ConflictsWith(y_c) && ++numConflicts && update)
      {
        c1.c = x_c;
        c2.c = y_c;

        c1.unit1 = y;
        c2.unit1 = x;

        c1.prevWpt = pwptB;
        c2.prevWpt = pwptA;

        update = false;
        return 1;
      }

      // Check the edge conflicts
      Constraint<state> x_e_c(a[xTime], a[min(xmax-1, xTime+1)]);
      Constraint<state> y_e_c(b[yTime], b[min(ymax-1, yTime+1)]);

      if (x_e_c.ConflictsWith(y_e_c) && ++numConflicts && update)
      {
        c1.c = x_e_c;
        c2.c = y_e_c;

        c1.unit1 = y;
        c2.unit1 = x;

        c1.prevWpt = pwptB;
        c2.prevWpt = pwptA;

        update = false;
        return 1;
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
    if (a[min(xmax-1, xTime+1)].t < b[min(ymax-1, yTime+1)].t)
    {
      // If the end-time of the x unit segment is before the end-time of the y unit segment
      // we have in increase the x unit but leave the y unit time the same
      i ++;
    } 
    else 
    {
      // Otherwise, the y unit time has to be incremented
      j ++;
    }

  } // End time loop
  return numConflicts;
}

/** Find the first place that there is a conflict in the tree */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
unsigned CBSGroup<state,action,environment,comparison,conflicttable>::FindFirstConflict(CBSTreeNode<state,conflicttable> const& location, Conflict<state> &c1, Conflict<state> &c2)
{

  unsigned numConflicts(0);

  // For each pair of units in the group
  for (int x = 0; x < this->GetNumMembers(); x++)
  {
    for (int y = x+1; y < this->GetNumMembers(); y++)
    {

      numConflicts += HasConflict(location.paths[x],location.paths[y],x,y,c1,c2,numConflicts==0);
      if(!greedyCT&&numConflicts) return numConflicts;
    }
  }

  return numConflicts;
}

/** Draw the AIR CBS group */
template<typename state, typename action, typename environment, typename comparison, typename conflicttable>
void CBSGroup<state,action,environment,comparison,conflicttable>::OpenGLDraw(const environment *ae, const SimulationInfo<state,action,environment> * sim)  const
{
	/*
	GLfloat r, g, b;
	glLineWidth(2.0);
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		CBSUnit<state,action,environment,comparison,conflicttable> *unit = (CBSUnit<state,action,environment,comparison,conflicttable>*)this->GetMember(x);
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
