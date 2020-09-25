/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2018 All rights reserved.
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

#ifndef __hog2_glut__CBICSUnits__
#define __hog2_glut__CBICSUnits__

#include <iostream>
#include <limits> 
#include <algorithm>
#include <unordered_set>

#include <queue>
#include <functional>
#include <vector>

#include <thread>
#include <mutex>

#include "Unit.h"
#include "UnitGroup.h"
#include "ConstrainedEnvironment.h"
#include "CollisionDetection.h"
#include "GridStates.h"
#include "MultiAgentStructures.h"
#include "TemporalAStar2.h"
#include "MultiAgentEnvironment.h"
#include "Heuristic.h"
#include "BiClique.h"
#include "Timer.h"
#include "MutexProp.h"
#include "ICTSAlgorithm.h"
#include "VertexCover.h"
#include <string.h>
#include <unordered_map>
#include <math.h>

#define NO_CONFLICT    0
#define NON_CARDINAL   1
#define LEFT_CARDINAL  2
#define RIGHT_CARDINAL 4
#define BOTH_CARDINAL  (LEFT_CARDINAL|RIGHT_CARDINAL)

#define PRE_NONE 0
#define PRE_AABB 1
#define PRE_HULL 2
#define PRE_SAP  4

#define MAXNAGENTS 1000

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
class CBSUnit;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
class CBSGroup;

struct Params {
  static bool greedyCT;
  static bool skip;
  static unsigned conn;
  static bool prioritizeConf;
  static bool identicalconstraints;
  static bool crossconstraints;
  static bool boxconstraints;
  static bool timerangeconstraints;
  static bool pyramidconstraints;
  static bool xorconstraints;
  static bool extrinsicconstraints;
  static bool mutualconstraints;
  static bool mutualtimerange;
  static bool apriori;
  static bool extended;
  static bool overlapconstraints;
  static bool verbose;
  static bool astarverbose;
  static bool asym; // Asymmetric set
  static bool vc; // vertex collisions
  static bool overload; // Overload constraints (add extra constraints for every conflict)
  static bool complete; // Ensure completeness (only relevant when crossconstraints=true)
  static bool conditional; // Use conditional constraints to promote completeness
  static bool mutexprop; // Use mutex propagation logic
  static bool pairwise; // Only evaluate conflict groups of size 2...
  static std::vector<unsigned> array;
  static std::vector<unsigned> indices;
  static std::vector<float> ivls;
};

unsigned Params::conn = 1;
bool Params::greedyCT = false;
bool Params::skip = false;
bool Params::prioritizeConf = false;
bool Params::crossconstraints = false;
bool Params::identicalconstraints = true;
bool Params::timerangeconstraints = false;
bool Params::pyramidconstraints = false;
bool Params::xorconstraints = false;
bool Params::extrinsicconstraints = false;
bool Params::mutualconstraints = false;
bool Params::mutualtimerange = false;
bool Params::apriori = false;
bool Params::extended = false;
bool Params::overlapconstraints = false;
bool Params::complete = false;
bool Params::verbose = false;
bool Params::astarverbose = false;
bool Params::asym = false;
bool Params::vc = false;
bool Params::overload = false;
bool Params::conditional = false;
bool Params::mutexprop = false;
bool Params::pairwise = true;
std::vector<unsigned> Params::array;
std::vector<unsigned> Params::indices;
std::vector<float> Params::ivls;

template<class state>
struct CompareLowGCost {
  bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const {
    if (fequal(i1.g + i1.h, i2.g + i2.h)) {
      if (fequal(i1.g, i2.g)) {
        return i1.data.t < i2.data.t; // Break ties by time
      }
      return fless(i1.g, i2.g);
    }
    return fgreater(i1.g + i1.h, i2.g + i2.h);
  }
};

// To copy pointers of an object into the destination array...
template<typename state, typename aabb>
void addAABBs(std::vector<state> const& v, std::vector<aabb>& d, uint32_t agent) {
  d.reserve(v.size() - 1);
  auto first(v.cbegin());
  while (first + 1 != v.end()) {
    d.emplace_back(*first, *(first + 1), agent);
    ++first;
  }
}

// Merge path between waypoints
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void MergeLeg(std::vector<state> const& path, std::vector<state>& thePath, std::vector<int>& wpts, unsigned s, unsigned g,
    unsigned minTime) {
  int insertPoint(wpts[s]); // Starting index of this leg
  unsigned origTime(thePath[wpts[g]].t); // Original ending time of leg
  unsigned deletes(wpts[g] - wpts[s] + 1); // Number of path points in this leg.
  // Remove points from the original path (if they exist)
  if (thePath.empty()) {
    assert(!"Path being merged into is empty");
  }
  if (path.empty()) {
    assert(!"Path to merge is empty");
  }
  while (thePath.size() > wpts[g] + 1 && thePath[wpts[g]].sameLoc(thePath[++wpts[g]])) {
    deletes++;
  }
  unsigned newTime(path.rbegin()->t); // Save the track end time of the new leg

  // Insert new path in front of the insert point
  thePath.insert(thePath.begin() + insertPoint, path.begin(), path.end());
  insertPoint += path.size();

  //Erase the original subpath including the start node
  thePath.erase(thePath.begin() + insertPoint, thePath.begin() + insertPoint + deletes);

  // Update waypoint indices
  int legLenDiff(path.size() - deletes);
  if (legLenDiff) {
    for (int i(g); i < wpts.size(); ++i) {
      wpts[i] += legLenDiff;
    }
  }

  if (thePath.size() - 1 != wpts[g] && newTime != origTime) {
    // Increase times through the end of the track
    auto newEnd(thePath.begin() + insertPoint);
    while (newEnd++ != thePath.end()) {
      newEnd->t += (newTime - origTime);
    }
  }

  while (wpts[g] > wpts[s] + 1 && thePath[wpts[g]].sameLoc(thePath[wpts[g] - 1])) {
    wpts[g]--;
  }
}

// Plan path between waypoints
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo, class singleHeuristic>
void GetFullPath(CBSUnit<state, action, comparison, conflicttable, searchalgo>* c, searchalgo& astar,
    EnvironmentContainer<state, action>& env, std::vector<state>& thePath, std::vector<int>& wpts, unsigned minTime,
    unsigned agent, float& replanTime, unsigned& expansions) {
  static std::unordered_map<uint64_t,std::unique_ptr<singleHeuristic>> heuristics;
  // We should only call this function for initial empty paths.
  if (thePath.size())
    assert(!"Tried to plan on top of an existing path!");
  //thePath.resize(0);
  wpts.resize(c->GetNumWaypoints()+env.environment->pconstraints.size()*2);
  wpts[0] = 0;
  std::vector<state> const* fpts(&c->GetWaypoints());
  if(Params::xorconstraints){
    static std::set<state> pts;
    pts.clear();
    // Assume there are only 2 waypoints at this time...
    pts.insert(c->GetWaypoint(0));
    for(auto const& p: env.environment->pconstraints){ // Implies that xor is turned on
      if(Params::extrinsicconstraints){
        pts.insert(p->start_state);
        pts.insert(p->end_state);
      }else{
        pts.insert(((XOR<state> const*)p)->pos_start);
        pts.insert(((XOR<state> const*)p)->pos_end);
      }
    }
    static std::vector<state> ffpts;
    ffpts.insert(ffpts.begin(),pts.begin(),pts.end());
    if(!(*ffpts.rbegin()==c->GetWaypoint(1))){
      //ffpts.reserve(ffpts.size()+1);
      ffpts.push_back(c->GetWaypoint(1));
      if(Params::verbose)std::cout << "Plan eventually" << *ffpts.begin() <<"-->"<<*ffpts.rbegin()<<"\n";
    }
    // All waypoints are the same
    if(ffpts.size()==1){
      thePath.push_back(ffpts[0]);
      return;
    }
    fpts=&ffpts;
  }

  // Perform search for all legs
  unsigned offset(0);
  comparison::currentEnv = (ConstrainedEnvironment<state, action>*) env.environment.get();
  if (comparison::useCAT) {
    comparison::openList = astar.GetOpenList();
    comparison::currentAgent = agent;
  }
  for (int i(0); i < fpts->size() - 1; ++i) {
    state start(fpts->at(i));
    //start.landed=false;
    //start.t=0;
    state goal(fpts->at(i + 1));
    goal.t=env.environment->getGoal().t;
    env.environment->setGoal(goal);
    if(Params::verbose)std::cout << "Plan " << start <<"-->"<<goal<<"\n";
    Timer tmr;
    tmr.StartTimer();
    if (env.heuristic)
    { //There is a specific heuristic that we should use
      if (i == fpts->size() - 2)
      {
        astar.SetHeuristic(env.heuristic.get());
      }
      else
      {
        singleHeuristic *h(nullptr);
        // TODO fix this so we get a unique key (its ok for xyztLoc, but not others.)
        uint64_t key(*((uint64_t *)&goal));
        auto hh(heuristics.find(key));
        if (hh == heuristics.end())
        {
          h = new singleHeuristic(env.environment->GetMap(), env.environment.get());
          heuristics[key].reset(h);
        }
        else
        {
          h = hh->second.get();
        }
        astar.SetHeuristic(h);
      }
    }
    goal.t=minTime;
    static std::vector<state> path;
    path.resize(0);
    astar.GetPath(env.environment.get(), start, goal, path, goal.t?goal.t:minTime);
    replanTime += tmr.EndTimer();
    if(Params::verbose)std::cout << start <<"-->"<<goal<<" took: " << tmr.EndTimer() << std::endl;

    expansions += astar.GetNodesExpanded();
    //std::cout << "exp full " << astar.GetNodesExpanded() << "\n";
    if (path.empty()) {
      thePath.clear();
      return;
    } //no solution found

    // Append to the entire path, omitting the first node for subsequent legs
    thePath.insert(thePath.end(), path.begin() + offset, path.end());

    offset = 1;
    int ix(thePath.size() - 1);
    wpts[i + 1] = ix;
    while (ix > 0 && thePath[ix].sameLoc(thePath[ix - 1])) {
      wpts[i + 1] = --ix;
    }
  }
}

// Plan path between waypoints
template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo, class singleHeuristic>
void ReplanLeg(CBSUnit<state, action, comparison, conflicttable, searchalgo>* c, searchalgo& astar,
    EnvironmentContainer<state, action>& env, std::vector<state>& thePath, std::vector<int>& wpts, unsigned s, unsigned g,
    unsigned minTime, unsigned agent, float& replanTime, unsigned& expansions) {
  if (thePath.empty()) {
    GetFullPath<state, action, comparison, conflicttable, searchalgo, singleHeuristic>(c, astar, env, thePath, wpts, minTime, agent, replanTime, expansions);
    return;
    //assert(false && "Expected a valid path for re-planning.");
  }
  //std::cout << "REPLANLEG\n";
  int insertPoint(wpts[s]); // Starting index of this leg
  unsigned origTime(thePath[wpts[g]].t); // Original ending time of leg
  unsigned deletes(wpts[g] - wpts[s] + 1); // Number of path points in this leg.
  // Remove points from the original path (if they exist)
  if (thePath.empty()) {
    assert(false && "Expected a valid path for re-planning.");
  }
  while (thePath.size() > wpts[g] + 1 && thePath[wpts[g]].sameLoc(thePath[++wpts[g]])) {
    deletes++;
  }
  state start(c->GetWaypoint(s));
  state goal(c->GetWaypoint(g));
  // Preserve proper start time
  start.t = thePath[wpts[s]].t;

  //std::cout << start << " to " << goal << "\n";

  // Perform search for the leg
  static std::vector<state> path;
  path.resize(0);
  env.environment->setGoal(goal);
  Timer tmr;
  tmr.StartTimer();
  astar.GetPath(env.environment.get(), start, goal, path, minTime);
  replanTime += tmr.EndTimer();
  //std::cout << "Replan took: " << tmr.EndTimer() << std::endl;
  //std::cout << "New leg " << path.size() << "\n";
  //for(auto &p: path){std::cout << p << "\n";}
  if (path.empty()) {
    thePath.resize(0);
    expansions+= astar.GetNodesExpanded(); //no solution found
    return;
  }
  unsigned newTime(path.rbegin()->t); // Save the track end time of the new leg

  // Insert new path in front of the insert point
  //std::cout << "SIZE " << thePath.size() << "\n";
  //std::cout << "Insert path of len " << path.size() << " before " << insertPoint << "\n";
  thePath.insert(thePath.begin() + insertPoint, path.begin(), path.end());
  //std::cout << "SIZE " << thePath.size() << "\n";
  insertPoint += path.size();

  //Erase the original subpath including the start node
  //std::cout << "Erase path from " << insertPoint << " to " << (insertPoint+deletes) << "\n";
  thePath.erase(thePath.begin() + insertPoint, thePath.begin() + insertPoint + deletes);
  //std::cout << "SIZE " << thePath.size() << "\n";

  // Update waypoint indices
  int legLenDiff(path.size() - deletes);
  if (legLenDiff) {
    for (int i(g); i < wpts.size(); ++i) {
      wpts[i] += legLenDiff;
    }
  }

  if (thePath.size() - 1 != wpts[g] && newTime != origTime) {
    // Increase times through the end of the track
    auto newEnd(thePath.begin() + insertPoint);
    while (newEnd++ != thePath.end()) {
      newEnd->t += (newTime - origTime);
    }
  }

  while (wpts[g] > wpts[s] + 1 && thePath[wpts[g]].sameLoc(thePath[wpts[g] - 1])) {
    wpts[g]--;
  }
  //std::cout << "Replanned path\n";
  //for(auto &p: thePath){std::cout << p << "\n";}
  //std::cout << "exp replan " << astar.GetNodesExpanded() << "\n";
  expansions += astar.GetNodesExpanded();
}


template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo = TemporalAStar<
    state, action, ConstrainedEnvironment<state, action>, AStarOpenClosed<state, comparison>>>
class CBSUnit: public Unit<state, action, ConstrainedEnvironment<state, action>> {
public:
  CBSUnit(std::vector<state> const &gs, 
  EnvironmentContainer<state, action>* e,
  float r, float viz = 0)
      : start(0), goal(1), current(gs[0]), waypoints(gs), env(e), radius(r), visibility(viz) ,number(-1){
  }
  const char *GetName() {
    return "CBSUnit";
  }
  bool MakeMove(ConstrainedEnvironment<state, action> *, OccupancyInterface<state, action> *,
      SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *, state& a);
  bool MakeMove(ConstrainedEnvironment<state, action> *, OccupancyInterface<state, action> *,
      SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *, action& a);
  void UpdateLocation(ConstrainedEnvironment<state, action> *, state &newLoc, bool success,
      SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *) {
    if (success)
      current = newLoc;
    else
      assert(!"CBS Unit: Movement failed");
  }

  void GetLocation(state &l) {
    l = current;
  }
  void OpenGLDraw(const ConstrainedEnvironment<state, action> *,
      const SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *) const;
  void GetGoal(state &s) {
    s = waypoints[goal];
  }
  void GetStart(state &s) {
    s = waypoints[start];
  }
  inline std::vector<state> const & GetWaypoints() const {
    return waypoints;
  }
  inline state GetWaypoint(size_t i) const {
    return waypoints[std::min(i, waypoints.size() - 1)];
  }
  inline unsigned GetNumWaypoints() const {
    return waypoints.size();
  }
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
  inline std::vector<state> const& GetPath() const {
    return myPath;
  }
  void UpdateGoal(state &start, state &goal);
  void setUnitNumber(unsigned n) {
    number = n;
  }
  unsigned getUnitNumber() const {
    return number;
  }
  float getVisibility() const {
    return visibility;
  }
  float radius;
  EnvironmentContainer<state, action>* env;

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
  Conflict<state>(){}
  Conflict<state>(std::vector<unsigned> const& agents):c(agents.size()),costs(agents.size()),units(agents){}
  Conflict<state>(Conflict<state> const& from)
      : c(std::move(from.c)),
      costs(std::move(from.costs)),
      units(std::move(from.units)){}
  Conflict<state>& operator=(Conflict<state> const& from) {
    c=std::move(from.c);
    units=std::move(from.units);
    costs=std::move(from.costs);
    return *this;
  }
  mutable std::vector<std::vector<std::unique_ptr<Constraint<state>>>> c; // constraint representing one agent in the meta-state
  std::vector<unsigned> units;
  std::vector<std::pair<unsigned,unsigned>> costs;
};

template<typename state>
static std::ostream& operator<<(std::ostream& os, Conflict<state> const& c){
  os << "{ units: "<<c.units<<", costs: "<<c.costs<<", constraints: "<<c.c<<"}";
  return os;
}

// populate vector aabb with (min),(max)
template<typename state>
void computeAABB(std::vector<Vector2D>& aabb, std::vector<state> const& path) {
  aabb.resize(2);
  aabb[0].x = path[0].x;
  aabb[0].y = path[0].y;
  //aabb[0].z = 0x3FF;
  for (auto const& p : path) {
    if (p.x < aabb[0].x) {
      aabb[0].x = p.x;
    }
    if (p.x > aabb[1].x) {
      aabb[1].x = p.x;
    }
    if (p.y < aabb[0].y) {
      aabb[0].y = p.y;
    }
    if (p.y > aabb[1].y) {
      aabb[1].y = p.y;
    }
    /*if (p.z < aabb[0].z) {
      aabb[0].z = p.z;
    }
    if (p.z > aabb[1].z) {
      aabb[1].z = p.z;
    }*/
  }
}

struct agentpair_t{
  agentpair_t():a1(0xffff),a2(0xffff){}
  agentpair_t(unsigned i1, unsigned i2):a1(i1),a2(i2){if(a2<a1){a1=i2;a2=i1;}}
  uint16_t a1;
  uint16_t a2;
  size_t operator==(agentpair_t const&pt)const{return pt.a1==a1 && pt.a2==a2;}
  size_t operator()(agentpair_t const&pt)const{return pt.a1+pt.a2*0xffff;}
};

struct conflict_t{
  conflict_t():ix1(0xffff),ix2(0xffff){}
  conflict_t(unsigned i1, unsigned i2):ix1(i1),ix2(i2){}
  uint16_t ix1;
  uint16_t ix2;
  size_t operator==(conflict_t const&pt)const{return pt.ix1==ix1 && pt.ix2==ix2;}
  size_t operator()(conflict_t const&pt)const{return pt.ix1+pt.ix2*0xffff;}
};

template <typename state, typename conflicttable>
struct CBSTreeNode
{
  CBSTreeNode()
      : path(), parent(0), depth(0), satisfiable(true), cat(), cct() {}
  // Copy ctor takes over memory for path member

  CBSTreeNode(CBSTreeNode<state, conflicttable> const &from)
      : wpts(from.wpts), path(from.path.size()),
        paths(from.paths), con(from.con), parent(from.parent), depth(from.depth),
        satisfiable(from.satisfiable), cat(from.cat), cct(from.cct)
  {
    // Hand ownership of re-planned paths over to this new node

    for (unsigned i(0); i < con.units.size(); ++i)
    {
      path[i].reset(from.path[i].release());
    }
  }

// Construction from parent node...
  CBSTreeNode(CBSTreeNode<state, conflicttable> const &from, Conflict<state> const &c, unsigned p, bool s)
      : wpts(from.wpts), path(c.units.size()),
        paths(from.paths), con(c), parent(p), depth(from.depth+1),
        satisfiable(s), cat(from.cat), cct(from.cct)
  {
    unsigned i(0);
    for (auto const &u : c.units)
    {
      // Allocate new mem for replanned path
      path[i].reset(new std::vector<state>());
      // point to new path
      paths[u] = path[i++].get();
    }
    for (auto const &a : c.units)
      clearcct(a); // Clear the cct for this unit so that we can re-count collisions
  }

  inline void getCollisionPair(unsigned &a1, unsigned &a2) const
  {
    for (auto const &e : cct)
    {
      a1 = e.first.a1;
      a2 = e.first.a2;
      return;
    }
  }

  inline bool hasCollisions() const
  {
    return !cct.empty();
  }

  inline unsigned numCollisions() const
  {
    unsigned total(0);
    for (auto const &e : cct)
    {
      total += e.second.size();
    }
    return total;
  }

  inline void mostConflicting(unsigned &a1, unsigned &a2) const
  {
    unsigned total1(0);
    for (auto const &e : cct)
    {
      if (e.second.size() > total1)
      {
        a1 = e.first.a1;
        a2 = e.first.a2;
        total1=e.second.size();
      }
    }
  }

  inline void mostConflicting(unsigned &a1, unsigned &a2, unsigned &s1, unsigned &s2) const
  {
    unsigned total1(0);
    for (auto const &e : cct)
    {
      if (e.second.size() > total1)
      {
        a1 = e.first.a1;
        a2 = e.first.a2;
        s1 = e.second.begin()->ix1;
        s2 = e.second.begin()->ix2;
        total1=e.second.size();
      }
    }
  }

  inline void setcct(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2) const
  {
    agentpair_t p(a1, a2);
    if (p.a1 == a1)
      cct[p].emplace(ix1, ix2);
    else
      cct[p].emplace(ix2, ix1);
  }

  inline void unsetcct(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2) const
  {
    agentpair_t p(a1, a2);
    if (p.a1 == a1)
      cct[p].erase({ix1, ix2});
    else
      cct[p].erase({ix2, ix1});
  }

  inline void clearcct(unsigned a1) const
  {
    for (unsigned a2(0); a2 < paths.size(); ++a2)
    {
      //std::cout << "cct size " << cct.size() << "\n";
      auto ix(cct.find({a1, a2}));
      if (ix != cct.end())
      {
        cct.erase(ix);
      }
    }
  }

  std::vector<std::vector<int>> wpts;
  mutable std::vector<std::unique_ptr<std::vector<state>>> path;
  std::vector<std::vector<state> *> paths;
  mutable std::unordered_map<agentpair_t, std::unordered_set<conflict_t, conflict_t>, agentpair_t> cct;

  Conflict<state> con;
  unsigned parent;
  unsigned depth;
  bool satisfiable;
  conflicttable cat; // Conflict avoidance table
};

template<typename state, typename conflicttable, class searchalgo>
static std::ostream& operator <<(std::ostream & out, const CBSTreeNode<state, conflicttable> &act) {
  out << "(paths:" << act.paths->size() << ", parent: " << act.parent << ", satisfiable: " << act.satisfiable << ")";
  return out;
}

typedef std::vector<uint16_t> Group;

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic,
    class searchalgo = TemporalAStar<state, action, ConstrainedEnvironment<state, action>,
        AStarOpenClosed<state, comparison>>>
class CBSGroup: public UnitGroup<state, action, ConstrainedEnvironment<state, action>> {
public:
  CBSGroup(Group const& g);
  CBSUnit<state, action, comparison, conflicttable, searchalgo>* GetUnit(unsigned a)const{
    return (CBSUnit<state, action, comparison, conflicttable, searchalgo> *)this->GetMember(a);
  }
  ConstrainedEnvironment<state,action>* GetEnv(unsigned a)const{
    return GetUnit(a)->env->environment.get();
  }
  Heuristic<state>* GetHeuristic(unsigned a){
    return GetUnit(a)->env->heuristic.get();
  }
  bool MakeMove(Unit<state, action, ConstrainedEnvironment<state, action>> *u, ConstrainedEnvironment<state, action> *e,
      SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si, action& a);
  bool MakeMove(Unit<state, action, ConstrainedEnvironment<state, action>> *u, ConstrainedEnvironment<state, action> *e,
      SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si, state& a);
  void UpdateLocation(Unit<state, action, ConstrainedEnvironment<state, action>> *u,
      ConstrainedEnvironment<state, action> *e, state &loc, bool success,
      SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si);
  void AddUnit(Unit<state, action, ConstrainedEnvironment<state, action>> *u);
  void AddUnit(Unit<state, action, ConstrainedEnvironment<state, action>> *u, std::vector<state> const& path);
  unsigned GetMaxTime(int location, int agent);
  void StayAtGoal(int location);

  void OpenGLDraw(const ConstrainedEnvironment<state, action> *,
      const SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *) const;
  double getTime() {
    return time;
  }
  bool donePlanning() {
    return planFinished;
  }
  bool Pop();
  bool ExpandOneCBSNode();
  void Init();

  std::vector<CBSTreeNode<state, conflicttable> > tree;
  void processSolution(double);
  searchalgo astar;
  Group agents;

private:

  unsigned LoadConstraintsForNode(int location, int agent);
  void LoadConstraints(std::vector<unsigned> const& agents, unsigned location, std::vector<unsigned> const& costs);
  void Replan(int location, unsigned a, std::pair<unsigned,unsigned> const& limits, std::vector<unsigned> const& costs);

  bool IsCardinal(int x, state const&, state const&, int y, state const&, state const&, bool asym=false);
  unsigned CountConflicts(std::vector<state> const& a, std::vector<int> const& wa, std::vector<state> const& b,
      std::vector<int> const& wb, int x, int y);

  void GetBiclique( state const& a1, state const& a2, state const& b1, state const& b2, unsigned x, unsigned y, Conflict<state>& c1, Conflict<state>& c2, bool conditional=false);
  void GetConfGroup(CBSTreeNode<state, conflicttable> const& location, std::vector<unsigned>& confset, std::vector<unsigned>& stateIndices);
  bool GetConstraints(CBSTreeNode<state, conflicttable> const& location,
  std::vector<unsigned>& confset,
  std::vector<unsigned> const& allCosts,
  std::vector<std::pair<unsigned,unsigned>> const& extents,
  std::vector<Conflict<state>>& constraints,
  std::vector<Solution<state>> & fixed);
  unsigned FindConflictGroupAllPairs(CBSTreeNode<state, conflicttable> const& location, std::vector<unsigned>& confset, std::vector<unsigned>& stateIndices);
  unsigned FindConflictGroupOneVsAll(CBSTreeNode<state, conflicttable> const& location, unsigned x, std::vector<unsigned>& confset, std::vector<unsigned>& stateIndices, bool update);
      
  void SetEnvironment(unsigned agent);
  void ClearEnvironmentConstraints(unsigned agent);
  void AddEnvironmentConstraint(Constraint<state>* c, unsigned agent, bool negative);

  double time;

  struct OpenListNode {
    OpenListNode()
        : location(0), cost(0), nc(0){
    }
    OpenListNode(uint loc, double c, uint16_t n)
        : location(loc), cost(c), nc(n){
    }
    inline bool operator<(OpenListNode const& other)const{
      if (Params::greedyCT){
        return ( nc == other.nc ? cost<other.cost : nc>other.nc);
      }else{
        return ( cost==other.cost ? (nc > other.nc) : cost > other.cost);
      }
    }

    uint location;
    double cost;
    uint16_t nc;
  };
  /*struct OpenListNodeCompare {
    bool operator()(const OpenListNode& left, const OpenListNode& right) {
      if (Params::greedyCT)
        return (left.nc == right.nc) ? (fgreater(left.cost, right.cost)) : (left.nc > right.nc);
      else
        return left.cardinal==right.cardinal ? fequal(left.cost, right.cost) ? (left.nc > right.nc) : (fgreater(left.cost, right.cost) : left.cardinal>right.cardinal);
    }
  };*/

  ClearablePQ<CBSGroup::OpenListNode, std::vector<CBSGroup::OpenListNode>> openList;


  //std::vector<SearchEnvironment<state,action>*> agentEnvs;
public:
  std::vector<float> radii;
  bool planFinished;
  unsigned incumbent;
  unsigned currCost;
  unsigned bestCost;
  unsigned bestNode;

  // Algorithm parameters
  bool verify;
  bool ECBSheuristic; // For ECBS
  unsigned killex; // Kill after this many expansions
  bool keeprunning; // Whether to keep running after finding the answer (e.g. for ui)
  int seed;
  static Timer* timer;
  int animate; // Add pauses for animation
  bool quiet = false;
  bool disappearAtGoal = true;
  Solution<state> basepaths;
  static unsigned TOTAL_EXPANSIONS;
  static unsigned collchecks;
  static float collisionTime;
  static float planTime;
  static float replanTime;
  static uint64_t constraintsz;
  static uint64_t constrainttot;
};

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
Timer* CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer=new Timer();
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::TOTAL_EXPANSIONS=0;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::collchecks=0;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
float CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::collisionTime=0;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
float CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::planTime=0;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
uint64_t CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::constraintsz=0;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
uint64_t CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::constrainttot=0;
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
float CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::replanTime=0;


/** CBS UNIT DEFINITIONS */

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSUnit<state, action, comparison, conflicttable, searchalgo>::SetPath(std::vector<state> &p) {
  myPath = p;
  std::reverse(myPath.begin(), myPath.end());
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSUnit<state, action, comparison, conflicttable, searchalgo>::OpenGLDraw(
    const ConstrainedEnvironment<state, action> *ae,
    const SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si) const {
  //GLfloat r, g, b;
  //this->GetColor(r, g, b);
  //ae->SetColor(r, g, b);

  if (myPath.size() > 1) {
    // Interpolate between the two given the timestep
    state start_t = myPath[myPath.size() - 1];
    state stop_t = myPath[myPath.size() - 2];

    glLineWidth(2.0);
    GLfloat r, g, b;
    this->GetColor(r, g, b);
    ae->SetColor(r, g, b);
    ae->GLDrawPath(this->GetPath(), this->GetWaypoints());
    if (si->GetSimulationTime() * state::TIME_RESOLUTION_D <= stop_t.t
        && si->GetSimulationTime() * state::TIME_RESOLUTION_D >= start_t.t) {
      float perc = (stop_t.t - si->GetSimulationTime() * state::TIME_RESOLUTION_D) / (stop_t.t - start_t.t);
      ae->OpenGLDraw(stop_t, start_t, perc, radius);
      //Constraint<state> c(stop_t, start_t);
      //glColor3f(1, 0, 0);
      //c.OpenGLDraw();
    } else {
      //ae->OpenGLDraw(stop_t);
      //glColor3f(1, 0, 0);
      //Constraint<state> c(stop_t);
      //c.OpenGLDraw();
    }
    //GLfloat r, g, b;
    //this->GetColor(r, g, b);
    //ae->SetColor(r, g, b);
  } else {
    //if (current.landed)
    //return;
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

//------------------------------------------------------------------------------------------------//
/** CBS GROUP DEFINITIONS */

template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::ClearEnvironmentConstraints(
    unsigned agent)
{
  this->GetEnv(agent)->ClearConstraints();
  this->GetEnv(agent)->resetGoal();
}

template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::AddEnvironmentConstraint(
    Constraint<state> *c, unsigned agent, bool negative)
{
  //if(verbose)std::cout << "Add constraint " << c.start_state << "-->" << c.end_state << "\n";
  if (negative)
  {
    if (typeid(*c) == typeid(MinCost<state>))
    { // Assume this is a MinCost Constraint
      if (c->start_state.t > this->GetEnv(agent)->getGoal().t)
      {
        auto goal(this->GetEnv(agent)->getGoal());
        goal.t = c->start_state.t;
        this->GetEnv(agent)->setGoal(goal);
      }
    }
    else
    {
      this->GetEnv(agent)->AddConstraint(c);
    }
  }
  else
  {
    this->GetEnv(agent)->AddPositiveConstraint(c);
  }
}

/** constructor **/
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::CBSGroup(Group const& g)
    : time(0), incumbent(0), bestCost(INT_MAX), bestNode(INT_MAX), planFinished(false), verify(false), ECBSheuristic(false), killex(INT_MAX), keeprunning(
        false), animate(0), seed(1234567), quiet(true), agents(g){
  //std::cout << "THRESHOLD " << threshold << "\n";

  tree.resize(1);
  tree[0].parent = 0;

  // Sort the environment container by the number of conflicts
  astar.SetVerbose(Params::astarverbose);
  basepaths.resize(MAXNAGENTS);
}

/** Get the constraints for varying cost vectors */
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::GetConstraints(
  CBSTreeNode<state, conflicttable> const& location,
  std::vector<unsigned>& agents,
  std::vector<unsigned> const& allCosts,
  std::vector<std::pair<unsigned,unsigned>> const& extents,
  std::vector<Conflict<state>>& conflicts,
  std::vector<Solution<state>>& fixed)
{
  // Set up a bunch of stuff
  auto n(agents.size());
  static const unsigned epsilon(1);
  static std::vector<Node<state> *> toDelete;
  toDelete.clear();
  static MultiState<state> root;
  root.assign(n, nullptr);
  static MultiEdge<state> start;
  start.resize(n);
  static std::vector<DAG<state>> dags;
  dags.resize(n);
  clear(dags);
  static std::vector<EdgeSet<state>> terminals;
  terminals.resize(n);
  static std::vector<EdgeSet<state>> mutexes;
  mutexes.resize(n);

  static std::vector<unsigned> origCost;
  origCost.clear();
  origCost.reserve(n);
  static std::vector<state> goals;
  goals.clear();
  goals.reserve(n);
  static std::vector<state> starts;
  starts.clear();
  starts.reserve(n);
  std::vector<EnvironmentContainer<state, action> *> ec;
  //std::vector<ConstrainedEnvironment<state, action> *> env;
  ec.clear();
  ec.reserve(n);
  static std::vector<float> radi;
  radi.clear();
  radi.resize(n);
  unsigned ix(0);
  //std::vector<unsigned> nc(agents.size());

  static std::vector<std::vector<uint32_t>> somecosts;
  somecosts.clear();
  somecosts.reserve(n);
  somecosts.resize(1);
  for (auto const &a : agents)
  {
    origCost.push_back(extents[ix].first);
    //origCost.push_back(GetEnv(a)->GetPathLength(*location.paths[a]));
    goals.push_back(this->GetUnit(a)->GetWaypoint(1));
    starts.push_back(this->GetUnit(a)->GetWaypoint(0));
    ec.push_back(this->GetUnit(a)->env);
    ///*nc[ix]=*/LoadConstraintsForNode(incumbent,a); // Count constraints per agent
    radi[ix++] = radii[a];
    //somecosts[0].push_back(origCost[somecosts[0].size()]-GetEnv(a)->GetPathLength(*tree[0].paths[a]));
    somecosts[0].push_back(extents[somecosts[0].size()].first-GetEnv(a)->GetPathLength(*tree[0].paths[a]));
  }
  //static std::vector<unsigned> maxs;
  //maxs = origCost;
  static std::vector<unsigned> best;
  best.assign(n, INT_MAX);

  ICTSAlgorithm<state, action> ictsalgo(radi);
  ictsalgo.SetVerbose(Params::astarverbose);
  ictsalgo.SetQuiet(true);//!Params::unquiet);
  LoadConstraints(agents,incumbent);
  if(!ictsalgo.GetSolutionCosts(ec, starts, goals, somecosts))
  {
    return false; // Signal that no solution is possible
  }
  PairwiseConstrainedSearch<ConstrainedEnvironment<state, action>, state, action> srch2(
      0, 1,
      ec[0]->environment.get(),
      ec[1]->environment.get(),
      ec[0]->heuristic.get(),
      ec[1]->heuristic.get(),
      0, INT_MAX, 0, INT_MAX,
      starts[0], goals[0], radi[0],
      starts[1], goals[1], radi[1],
      0,true); // Solution must be greater cost than soc
  srch2.getRangesAndConstraints();

  somecosts.resize(1); // Throw away all but one for now...
  if(verbose)std::cout << agents << " optimal costs " << somecosts << "\n";
  // Results here indicate an upper bound on other more expensive solutions
  // E.g. an initial cost of <2,2> with lowest-cost solution of <3,4> indicates
  // there is an overall solution where both agents are bounded by <[3,inf),[4,inf)>
  // there may still be solutions in the ranges <[2,3),[x>4,inf)> or <[y>3,inf),[2,4)>
  //
  // Supposing there are multiple lowest-cost solutions possible: <3,4>,<4,3>, then the poss.
  // solutions may be in the complement of the union:
  // <3,4>\cup<4,3>=<[3,4],[3,4]>
  // <[2,inf),[2,inf)\setminus<[3,4],[3,4]>=<\<{[2,3),(4,inf]>\},\{[2,3),(4,inf]\}>.
  // Now test all members of cartesian product that min sum > (7)
  // <[2,3),(4,inf]>,<(4,inf],[2,3)]>
  std::vector<unsigned> mins(n,INT_MAX);
  std::vector<unsigned> maxs(origCost);
  unsigned totalCost(std::accumulate(somecosts[0].begin(),somecosts[0].end(),0));
  for (auto const &costs : somecosts)
  {
    for (unsigned i(0); i < costs.size(); ++i)
    {
      mins[i] = std::min(mins[i], costs[i]);
      maxs[i] = std::max(maxs[i], costs[i]);
    }
  }
  // TODO:
  // Hard-code for two agents... (instead of the k-agent case below)
  //static std::vector<std::vector<std::pair<unsigned,unsigned>>> combinations;
  // a[0] < min[0], a[1] (min[1],max[1])
  // a[0] (min[0],max[0]), a[1] < min[1]
  // a[0] (min[0],max[0]), a[1] (min[1],max[1])
  // a[0] (min[0],max[0]), a[1] (max[1],inf)
  // a[0] (max[0],inf), a[1] (min[1],max[1])
  // a[0] (max[0]+eps,inf), a[1] (max[1]+eps,inf)


  // Make all intervals closed by adding or subtracting epsilon from open ends
  std::vector<std::vector<std::pair<unsigned,unsigned>>> intervals(n);
  for (unsigned i(0); i < n; ++i)
  {
    if (mins[i] > origCost[i])
    {
      // Never go below or above the original extents from the parents
      // If this interval doesn't have at least partial overlap, ignore it.
      if ((mins[i] - epsilon >= extents[i].first && mins[i]-epsilon <= extents[i].second) ||
          (origCost[i] >= extents[i].first && origCost[i] <= extents[i].second))
      {
        intervals[i].emplace_back(std::max(extents[i].first,origCost[i]), std::min(extents[i].second,mins[i] - epsilon));
      }
    }
    if (maxs[i] >= origCost[i])
    {
      if (maxs[i] <= extents[i].second)
      {
        intervals[i].emplace_back(std::max(extents[i].first, maxs[i]), std::min(extents[i].second, unsigned(INT_MAX)));
      }
    }
    if (mins[i] != maxs[i]){
      if ((mins[i] >= extents[i].first && mins[i] <= extents[i].second) ||
          (maxs[i]-epsilon >= extents[i].first && maxs[i]-epsilon <= extents[i].second))
      {
        intervals[i].emplace_back(std::max(extents[i].first, mins[i]), std::min(extents[i].second, maxs[i] - epsilon));
      }
    }/*else{
      if (mins[i] >= extents[i].first && mins[i] <= extents[i].second)
      {
        intervals[i].emplace_back(mins[i], maxs[i]);
      }
    }*/
  }
  // Now have intervals of: [orig,min),(min,max),(max,inf)
  // Check for duplicates
  for (unsigned i(0); i < n; ++i)
  {
    for (auto combo(intervals[i].begin()); combo != intervals[i].end(); ++combo)
    {
      auto dup(std::find(combo + 1, intervals[i].end(), *combo));
      if (dup != intervals[i].end())
      {
        intervals[i].erase(dup);
      }
    }
  }

  // Generate Cartesian product...

  static std::vector<std::vector<std::pair<unsigned,unsigned>>> combinations;
  combinations = {{}};
  combinations.reserve(somecosts.size()*n);
  static std::vector<std::vector<std::pair<unsigned,unsigned>>> r;
  r.reserve(somecosts.size()*n);
  for (const auto &u : intervals)
  {
    r.clear();
    for (const auto &x : combinations)
    {
      for (const auto y : u)
      {
        r.push_back(x);
        r.back().push_back(y);
      }
    }
    combinations = std::move(r);
  }

  if(!quiet)std::cout << "agents " << agents << "optcosts " << somecosts << "original combinations " << combinations << "\n";

  // All of these combinations represent the cost frontier. It is like a
  // pareto-frontier, we can't decrease the cost of one agent without
  // increasing the cost of another.
  // Ignore combinations that have: all agents decrease
  // Additionally, we are interested in combinations that have:
  //   No increase or decrease (a new cost combination)
  // Delete inline...
  auto bounds(combinations); // holds mutex prop bounds
  unsigned b(0);
  for(auto c(combinations.begin()); c!=combinations.end(); /*++c*/)
  {
    bool allDecrease(true);
    bool noDecrease(true);
    bool inst(true);
    unsigned nDecrease(0);
    unsigned nUnbounded(0);
    uint64_t sumMax(0);
    for (unsigned i(0); i < n; ++i)
    {
      if(c->at(i).second==INT_MAX){
        nUnbounded++;
        allDecrease=noDecrease=false;
      }else if(c->at(i).first>=mins[i]){
        allDecrease=false;
      }else{
        nDecrease++;
        noDecrease=false;
      }
      if(c->at(i).first!=c->at(i).second){
        inst=false;
      }
      sumMax+=c->at(i).second;
    }
    // If ranges are all a single point, erase
    if (inst)
    {
      combinations.erase(c);
      bounds.erase(bounds.begin() + b);
      continue;
    }
    // All decreasing or all increasing unbounded doesn't tell us anything new
    // (no solution poss. in case of former, definitely poss. in case of latter)
    if (allDecrease)
    {
      combinations.erase(c);
      bounds.erase(bounds.begin() + b);
      continue;
    }else if(noDecrease && sumMax<totalCost){
      combinations.erase(c);
      bounds.erase(bounds.begin() + b);
      continue;
    //}else if((nUnbounded>0 && nDecrease==0) || (nUnbounded==0 && nDecrease>0)){
      // If one or more is unbounded and there is none that decrease,
      // we've already covered this case in the original search
      //combinations.erase(c);
    }else{
      if (noDecrease && sumMax >= totalCost)
      {
        // TODO:
        // If all have an upper bound, make sure the cost will increase
        // Note, this combination could be a waste of time if we have unit costs
        // because no cost combination will fall in the range
        // Or if margins are very wide. ex: suppose <2,5>,<3,4>,<4,3>,<5,2>.
        // min,max=(2,5)(2,5). Obviously, solutions <3,4>,<4,3> exist so we won't
        // learn anything new. Should probably use: <(2,3),(4,5)>,<(3,4),(3,4)>,
        // <(4,5),(2,3)>...
        // We're only ok if there are two or less solns
        assert(somecosts.size() <= n);
      }
      // This combination checks out...
      // Now see if it is satisfiable, that is, with the limits, can we
      // solve the sub-problem?
      if (nUnbounded > 0)
      {
        // We have to resolve to an upper bound for ICTS, however, in the future, we will do this with A*...
        // Get MDDs and mutexes with new costs
        best.assign(n, INT_MAX);
        // Re-initialize DAGs
        root.assign(n, nullptr);
        clear(terminals);
        clear(mutexes);
        clear(dags);

        // Fix the upper limit on all unlimited agents. Do an ICTS search until
        // bounded agents are all at their goals. Finally, put all of those nodes for the
        // unbounded agent into A* as starting points and see if you can reach the goal.
        // At least one other agent has a cost decrease in this scenario
        auto theCosts(allCosts);
        unsigned j(0);
        for (unsigned a(0); a < c->size(); ++a)
        {
          theCosts[a]=c->at(j).first;
          ++j;
        }
        LoadConstraints(agents,incumbent,theCosts);
        bool blocked(false);
        std::vector<unsigned> unboundedAgents;
        unboundedAgents.reserve(nUnbounded);
        std::vector<unsigned> minCost(n);
        for (unsigned a(0); a < c->size(); ++a)
        {
          auto cost(c->at(a));
          minCost[a]=cost.first;
          if (cost.second == INT_MAX)
          {
            unboundedAgents.push_back(a);
            cost.second = totalCost;
          }
          if (!getMDD(starts[a], goals[a], dags[a],
                      root[a], terminals[a], cost.first, cost.second,
                      best[a], ec[a]->environment.get(), blocked))
          {
            // Infeasible, just exit loop and try next combo
            if (Params::verbose)std::cout << cost << " has no MDD\n";
            if (cost.second != INT_MAX)
            {
              blocked = true;
              break;
            }
          }
        }
        if (blocked)
        {
          combinations.erase(c);
          bounds.erase(bounds.begin() + b);
          continue;
        }
        else
        {
          /*for (unsigned a(0); a < root.size(); ++a)
          {
            start[a] = {root[a], root[a]};
          }
          if (!getTerminals(start, goals, ec, toDelete,
                            terminals, radi, minCost,
                            unboundedAgents, disappearAtGoal))
          {
            combinations.erase(c);
            continue;
          }
          else*/
          {
            PairwiseConstrainedSearch<ConstrainedEnvironment<state, action>, state, action> srch(
                0, 1,
                ec[0]->environment.get(),
                ec[1]->environment.get(),
                ec[0]->heuristic.get(),
                ec[1]->heuristic.get(),
                c->at(0).first, c->at(0).second,
                c->at(1).first, c->at(1).second,
                starts[0], goals[0], radi[0],
                starts[1], goals[1], radi[1],
                totalCost); // Solution must be greater cost than soc
            srch.getRangesAndConstraints();
            if (srch.finalcost.size() < 2)
            {
              if(verbose)std::cout << agents << "w/" << *c << " infeasible\n";
              combinations.erase(c);
              bounds.erase(bounds.begin() + b);
              continue;
            }
            bounds[b][0].second=srch.finalcost[0][0];
            bounds[b][1].second=srch.finalcost[0][1];
                                      /*
                                      if (nUnbounded == 1)
            {
              auto agent(unboundedAgents[0]);
              c->at(agent).second=c->at(agent).first+state::TIME_RESOLUTION_U;
              */
             /*
              searchalgo srch;
              srch.SetHeuristic(GetHeuristic(agent));
              std::vector<state> starts;
              std::vector<state> path;
              std::vector<double> scosts;
              for(auto const& t:terminals[agent]){
                starts.push_back(t.second->n);
                scosts.push_back(t.second->n.t);
              }
              srch.GetPath(ec[agent]->environment.get(), starts, scosts, goals[agent], path, minCost[agent]);
              if(path.size()){
                // Set an upper bound...
                c->at(agent).second=path[0].t+ec[agent]->environment->GetPathLength(path);
              }else{
                combinations.erase(c);
                continue;
              }*/
            /*}
            else if (nUnbounded > 1)
            {
              // For now, just add 1 to the lower bound...
              for (unsigned agent(0); agent < unboundedAgents.size(); ++agent)
              {
                c->at(agent).first+=epsilon; // This is assuming only 2 agents! WARNING!
                c->at(agent).second=c->at(agent).first+state::TIME_RESOLUTION_U;
              }
              */

/*
              // Set up multi-agent env
              std::vector<SearchEnvironment<state, action> const*> agentEnvs(nUnbounded);
              std::vector<std::pair<state,state>> goal; goal.reserve(nUnbounded);
              for (auto const &a : unboundedAgents)
              {
                agentEnvs[a] = ec[a]->environment.get();
                goal.emplace_back(goals[a],goals[a]);
              }
              class Edge : public std::pair<state, state>
              {
              public:
                Edge(state const &a, state const &b) : std::pair<state, state>(a, b), t(b.t) {}
                uint32_t t;
              };
              typedef MultiAgentEnvironment<Edge, action, SearchEnvironment<state, action>> MultiEnv;
              MultiEnv mae(agentEnvs);
              TemporalAStar<MAState<Edge>, typename MultiEnv::MultiAgentAction, MultiEnv> srch;
              std::vector<MAState<Edge>> starts;
              std::vector<double> scosts;

              for (unsigned agent(0); agent < unboundedAgents.size(); ++agent)
              {
                for (auto const &t : terminals[agent]) // TODO get termstates as vector of terminals
                {
                  starts.emplace_back(t.first->n,t.second->n);
                  scosts.push_back(t.second->n.t);
                }
              }

              std::vector<MAState<Edge>> path;
              //srch.SetVerbose(verbose);
              srch.GetPath(&mae, starts, scosts, goal, path);
              if(path.size()){
                // Set an upper bound...
                unsigned i(0);
                for (auto const &a : unboundedAgents)
                {
                  std::vector<state> p;
                  p.reserve(path[i].size());
                  p.push_back(path[i][0].first);
                  for(auto const& q:path[i]){
                    p.push_back(q.second);
                  }
                  c->at(a).second = ec[a]->environment->GetPathLength(p);
                  ++i;
                }
                ++c;
              }else{
                combinations.erase(c);
              }
              */
            /*}*/
          }
        }
      }
    }
    ++c;
    ++b;
  }

  // Check for duplicates
  b=0;
  for (auto combo(combinations.begin()); combo != combinations.end(); ++combo)
  {
    auto dup(std::find(combo + 1, combinations.end(), *combo));
    if (dup != combinations.end())
    {
      combinations.erase(dup);
      bounds.erase(bounds.begin()+b);
    }
    else
    {
      ++b;
    }
  }

  unsigned i(0);
  //std::vector<bool> increased(n);
  for (auto& costs : bounds)
  {
    if(verbose)std::cout << "Check: " << combinations[i] << "("<<costs<<")\n";
    auto theCosts(allCosts);
    for(unsigned j(0); j<costs.size(); ++j)
    {
      theCosts[agents[j]]=costs[j].first;
    }
    LoadConstraints(agents,incumbent,theCosts);
    // Get MDDs and mutexes with new costs
    best.assign(n,INT_MAX);
    // Re-initialize DAGs
    root.assign(n,nullptr);
    clear(terminals);
    clear(mutexes);
    clear(dags);
    bool blocked(false);
    for(unsigned a(0); a<costs.size(); ++a)
    {
      auto cost = costs[a];
      if (!getMDD(starts[a], goals[a], dags[a],
                  root[a], terminals[a], cost.first, cost.second,
                  best[a], ec[a]->environment.get(), blocked))
      {
        // Infeasible, just exit loop and try next combo
        if (Params::verbose)std::cout << cost << " has no MDD\n";
        blocked = true;
        break;
      }
      //if(Params::verbose)std::cout << "MDD"<<agents[a]<<":\n" << root[a] << "\n";
    }
    if(blocked){
      ++i;
      continue;
    }

    for(unsigned a(0); a<root.size(); ++a){
      start[a]={root[a],root[a]};
    }

    std::vector<unsigned> mincost;
    mincost.reserve(costs.size());
    for(auto const& cc:costs){
      mincost.push_back(cc.first);
    }

    fixed.resize(fixed.size() + 1);
  // TODO:
  // Constraints need to be disabled if the opposing agent's cost in which they were
  // made has increased. To do this, add an annotation on the constraint itself: agent,maxCost
  // Note that maxCost can actually be determined on a per-constraint basis, however, for now
  // we'll just use the opposing agent's MDD size at the time of mutex propagation.
    if (!getMutexes(start, goals, ec, toDelete,
                    terminals, mutexes, radi, mincost,
                    fixed.back(), totalCost, disappearAtGoal))
    {
      if(verbose)std::cout << costs << "Unable to find solution during mutex propagation\n";
      fixed.resize(fixed.size()-1);
      ++i;
      continue;
    }
    //std::cout << "Solution found: " << hasSolution << "\n";
    for (auto &d : toDelete)
    {
      delete d;
    }

    // Adjust cost ranges if necessary
    bool changed(false);
    for(unsigned q(0); q<n; ++q){
      auto newCost(ec[q]->environment->GetPathLength(fixed.back()[q]));
      if(newCost>costs[q].first){
        if(verbose)std::cout << "increase " << q << " from " << costs[q].first << " to " << newCost << "\n";
        combinations[i][q].first=costs[q].first=newCost;
        changed=true;
      }
    }
    // Check for duplicates
    /*if (changed)
    {
      for (auto combo(combinations.begin()); combo != combinations.end(); ++combo)
      {
        auto dup(std::find(combo + 1, combinations.end(), *combo));
        if (dup != combinations.end())
        {
          combinations.erase(dup);
        }
      }
    }*/

    // Add min-cost constraints (???) (forces paths to have minimum cost).
    //state dummy(a.back());
    //dummy.t = costs[0];
    //c1.c.emplace_back((Constraint<state> *)new MinCost<state>(dummy));

    // Set cardinal type
    // Remove redundancies: Any edge which has all parents mutexed can be removed.
    if(verbose)std::cout << "Mutexes:\n";
    bool haveMutexes(false);
    for (int a(0); a < mutexes.size(); ++a)
    {
      if(verbose)std::cout << "  " << a << "\n";
      auto edge(mutexes[a].rbegin());
      while (edge != mutexes[a].rend())
      {
        if(verbose)std::cout << "    " << *edge->first << " " << *edge->second << "\n";
        haveMutexes=true;
        if (edge->first->parents.size())
        {
          bool all(true);
          // Check that all parents of this edge are in the mutexed list
          for (auto const &p : edge->first->parents)
          {
            bool found(false);
            for (auto k : mutexes[a])
            {
              if (*edge == k)
                continue;
              if (k.first == p && k.second == edge->first)
              {
                found = true;
                break;
              }
            }
            if (!found)
            {
              all = false;
              break;
            }
          }
          if (all)
          {
            // convert to regular iterator
            edge = Util::make_reverse_iter(mutexes[a].erase((++edge).base())--);
          }
          else
          {
            ++edge;
          }
        }
        else
        {
          ++edge;
        }
      }
    }

    if (haveMutexes)
    {
      conflicts.resize(conflicts.size() + 1);
      if(verbose)std::cout << "agents: " << agents << " " << "cost: " << combinations[i] << "\n";
      conflicts.back().units = agents;
      conflicts.back().costs = combinations[i];
      /*for(unsigned j(0); j<conflicts[i].costs.size(); ++j){
      if(conflicts[i].costs[j].second > maxs[j]) conflicts[i].costs[j].second=UINT_MAX;
    }*/

      conflicts.back().c.resize(mutexes.size());
      for (unsigned a(0); a < mutexes.size(); ++a)
      {
        for (auto const &m : mutexes[a])
        {
          if(verbose)std::cout << "Add constraint to agent " << agents[a] << ": " << m.first->n << "-->" << m.second->n << " refAgent=" << agents[a ? 0 : 1] << "\n";
          conflicts.back().c[a].emplace_back((Constraint<state> *)new ContingentIdentical<state>(m.first->n, m.second->n, agents[a], agents[a ? 0 : 1], combinations[i][a?0:1].first, bounds[i][a?0:1].second));
        }
      }
    }
    else
    {
      if(verbose)std::cout << "NO MUTEXES: agents: " << agents << " " << "cost: " << combinations[i] << "\n";
      conflicts.resize(conflicts.size() + 1);
      if(verbose)std::cout << "agents: " << agents << " cost: " << combinations[i] << "\n";
      conflicts.back().units = agents;
      conflicts.back().costs = combinations[i];
      conflicts.back().c.resize(2);
    }
    ++i;
  }
  if(!quiet)std::cout << "agents " << agents << "optcosts " << somecosts << "combinations " << combinations << "\n";
  /*
  for (auto const &i : increase)
  {
    if (!i)
    {
      // Try increasing this and not the other(s)
      for (unsigned a(0); a < costs.size(); ++a)
      {
        auto cost(origCosts[a]);
        cost = costs[a];
        auto minCost = costs[a] - epsilon;
        while (!getMDD(starts[a], goals[a], dags[a],
                       root[a], terminals[a], minCost, cost,
                       best[a], ec[a]->environment.get(), blocked))
        {
          if (blocked)
            break;
          minCost = cost;
          cost += epsilon;
          root[a] = nullptr;
          dags[a].clear();
          terminals[a].clear();
        }
        if (Params::verbose)
          std::cout << "MDD" << a << ":\n"
                    << root[a] << "\n";
      }
    }
  }*/
  return true;
}

// Get the best node from the top of the open list, and remove it from the list
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::Pop()
{
  int count(0);
  do
  {
    incumbent = openList.top().location;
    currCost = openList.top().cost;
    if (!tree[incumbent].satisfiable)
      openList.pop();
    if (++count > tree.size())
      assert(!"No solution!?!");
  } while (!tree[incumbent].satisfiable);
}


/** Expand a single CBS node */
// Return true while processing
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::ExpandOneCBSNode()
{
  //for(auto const& o:openList.getContainer()){
  //std::cout << "tree["<<o.location <<"]="<<o.cost<<","<<o.nc<<"\n";
  //}
  auto currentCost=openList.top().cost;
  //auto currentConf=openList.top().nc;
  if(openList.empty()){
    return false;
  }
  openList.pop();
  if(!quiet)std::cout << "Expanding " << incumbent << " cost: " << currentCost << "(" << tree[incumbent].parent << ")" << tree[incumbent].con.units << " " << tree[incumbent].con.costs << " \n";
  // Check current cost of solution equal to currentCost
    //if(cost==currentCost ||
  if(currentCost>=bestCost){
    processSolution(CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());
    return false;
  }
  // Compute extents based on all parents costs
  unsigned cs(tree[incumbent].paths.size());
  std::vector<unsigned> mins(cs);
  std::vector<unsigned> maxs(cs);
  auto node(incumbent);
  unsigned updated(0);
  while(node&&updated<cs){
    unsigned i(0);
    for (auto const &u : tree[node].con.units)
    {
      if (mins[u] == 0)
      {
        ++updated;
        mins[u] = tree[node].con.costs[i].first;
      }
      if (maxs[u] == 0)
      {
        maxs[u] = tree[node].con.costs[i].second;
      }
      ++i;
    }
    node=tree[node].parent;
  }

  std::vector<unsigned> origcosts(cs);
  // For agents with no extents, use the minimum path length at the root
  for (int i(0); i < cs; ++i){
    origcosts[i] = GetEnv(i)->GetPathLength(*tree[incumbent].paths[i]);
    if(mins[i]==0){mins[i]=GetEnv(i)->GetPathLength(*tree[0].paths[i]);}
    if(maxs[i]==0){maxs[i]=UINT_MAX;}
  }

  if (!quiet){
    //if (incumbent)
    //{
    std::cout << "<";
      for (int i(0); i < cs; ++i)
      {
        if (maxs[i] == UINT_MAX)
          std::cout << (i ? "," : "") << i << ":(" << mins[i] << ","
                    << "inf"
                    << ")";
        else
          std::cout << (i ? "," : "") << i << ":(" << mins[i] << "," << maxs[i] << ")";
      }
    //}
    /*else
    {
      for (int i(0); i < tree[incumbent].paths.size(); ++i)
      {
        std::cout << (i ? "," : "") << i << ":(" << GetEnv(i)->GetPathLength(*tree[incumbent].paths[i])
        << ",inf)";
      }
    }*/
    std::cout << ">\n";
    // print all constraints for this node
    std::cout << "costs: "<< origcosts << "\n";

/*
    for (auto const &a : this->agents)
    {
      std::cout << "Constraints for agent: "<<a<<":\n";
      // Go up the tree
      auto location(incumbent);
      auto go(location+1);
      while (go)
      {
        go=location;
        // Agents involved in this node
        for (unsigned i(0); i < tree[location].con.units.size(); ++i)
        {
          // Agents to add constraints for
          // This node has constraints for the agent
          if (a == tree[location].con.units[i])
          {
            // Add all of the constraints
            for (auto const &c : tree[location].con.c[i])
            {
              if (typeid(*c) == typeid(ContingentIdentical<state>))
              {
                ContingentIdentical<state> const &cc((ContingentIdentical<state> &)*c);
                if (origcosts[cc.refAgent] < cc.lb || origcosts[cc.refAgent] > cc.ub)
                {
                  //std::cout << "    Ignoring:   " << cc.start_state << "-->" << cc.end_state << " "<<cc.a<<" " << cc.refAgent << " [" << cc.lb << "," << cc.ub << "]\n";
                }else{
                std::cout << "    Contingent:("<<cc.a<<") " << cc.start_state << "-->" << cc.end_state << " "<<cc.a<<" " << cc.refAgent << " [" << cc.lb << "," << cc.ub << "]\n";
                }
              }else if (typeid(*c) == typeid(Identical<state>))
              {
                  std::cout << "    Identical:(" << c->a << ") " << c->start_state << "-->" << c->end_state << "\n";
              }else if (typeid(*c) == typeid(Collision<state>))
              {
                  std::cout << "    Collision:(" << c->a << ") " << c->start_state << "-->" << c->end_state << "\n";
              }
            }
          }
        }
        location = tree[location].parent;
      }
    }*/
  }
  // There's no reason to expand if the plan is finished.
  if (planFinished)
    return false;

  std::vector<unsigned> conflictset;
  std::vector<unsigned> stateIndices;
  unsigned numconflicts(0);
  
  if (incumbent != 0 || tree[incumbent].path.size())
  {
    // Check all of the agents that were just replanned
    for (auto const &a : tree[incumbent].con.units)
    {
      numconflicts = FindConflictGroupOneVsAll(tree[incumbent], a, conflictset, stateIndices, a==tree[incumbent].con.units.back());
      //if(numconflicts){break;} // can't do this...
    }
  }
  else
  {
    tree[0].cct.clear();
    //numconflicts=currentConf;
    numconflicts = FindConflictGroupAllPairs(tree[incumbent], conflictset, stateIndices);
  }
  // If no conflicts are found in this node, then the path is done
  if(numconflicts)
  {
    // Load constraints, assuming one of the two agents being replanned may change
    // up to the upper bound.
    auto costs(mins);
    costs[conflictset[0]]=maxs[conflictset[0]]-1;
    costs[conflictset[1]]=maxs[conflictset[1]]-1;
    LoadConstraints(conflictset,incumbent,costs);

    // currentCost lies somewhere inbetween the sums of upper and lower bounds
    // Therefore, the current bound is not min[0]+min[1], nor orig[0]+orig[1]
    // it is currentCost minus costs of all non-conflicting agents (this 
    // guarantees that the costs of agents goes up)
    unsigned combinedUB(currentCost);
    for (unsigned i(0); i < cs; ++i)
    {
      if(i!=conflictset[0] && i!=conflictset[1]){
        combinedUB-=origcosts[i];
      }
    }
    //std::cout << "CombinedUB: " << combinedUB << "\n";

    PairwiseConstrainedSearch<ConstrainedEnvironment<state, action>, state, action> srch(
      0, 1,
      GetEnv(conflictset[0]),
      GetEnv(conflictset[1]),
      GetHeuristic(conflictset[0]),
      GetHeuristic(conflictset[1]),
      mins[conflictset[0]],maxs[conflictset[0]],
      mins[conflictset[1]],maxs[conflictset[1]],
      GetUnit(conflictset[0])->GetWaypoint(0), GetUnit(conflictset[0])->GetWaypoint(1), radii[0],
      GetUnit(conflictset[1])->GetWaypoint(0), GetUnit(conflictset[1])->GetWaypoint(1), radii[1],
      combinedUB,true); // Solution must be greater cost than soc
    srch.getRangesAndConstraints();
    std::vector< std::vector< std::pair< std::array<unsigned,3>,Action<state> > const* > > cons(2);
    
    if (srch.finalcost.empty())
    {
      // Infeasible
      if (verbose)
        std::cout << "TREE[" << incumbent << "] infeasible\n";
      Pop();
      return true;
    }
    else
    {
      auto &node(tree[incumbent]);
      for (unsigned s(0); s < srch.finalcost.size(); ++s)
      {
        // Filter the constraints for the costs
        for (unsigned i(0); i < 2; ++i)
        {
          auto other(i ? 0 : 1);
          //std::cout << "Filtered for agent " << conflictset[other] << " cost " << srch.finalcost[s][other] << " for agent " << conflictset[i] << "\n";
          //auto a(std::lower_bound(std::pair<std::pair<
          for (auto const &a : srch.intervals[i])
          {
            if (a.first[0] >= srch.finalcost[s][other])
            { // does upper bound encapsulate the target cost?
              if (a.first[1] <= srch.finalcost[s][other])
              {
                //std::cout << a << "\n";
                cons[i].push_back(&a);
              }
            }
          }
        }

        Conflict<state> con(conflictset);
        // The last cost combination has unlimited bounds
        bool unbounded(srch.finalcost.size()>1 && s == srch.finalcost.size() - 1);
        if (unbounded)
        {
          // Only create this node if this isn't a "bypass"
          con.costs = {{mins[conflictset[0]], std::min(maxs[conflictset[0]],UINT_MAX)}, {mins[conflictset[1]], std::min(maxs[conflictset[1]],UINT_MAX)}};
        }
        else
        {
          con.costs = {{srch.finalcost[s][0], srch.finalcost[s][0]+1},
                       {srch.finalcost[s][1], srch.finalcost[s][1]+1}};
        }
        // TODO: UPDATE in other ALGO... (removed bypass)
        if (cons[0].size() && cons[1].size())
        {
          // Notify the user of the conflict
          if (!quiet)
            std::cout << "TREE " << incumbent << "(" << node.parent << "): " << mins<<"/"<<maxs << "\n";

          if (Params::verbose)
          {
            std::cout << "  child node " << s << ": ";
            for (unsigned qq(0); qq < mins.size(); ++qq)
            {
              if (qq == conflictset[0])
              {
                std::cout << "*"<<con.costs[0].first<<"-" <<con.costs[0].second<< ",";
              }
              else if (qq == conflictset[1])
              {
                std::cout << "*"<<con.costs[1].first<<"-" <<con.costs[1].second<< ",";
              }
              else
              {
                std::cout << mins[qq] <<"-"<<maxs[qq]<< ",";
              }
            }
            std::cout << "\n";
          }
          auto last(tree.size());

          // The last cost combination has unlimited bounds
          if (s != srch.finalcost.size() - 1)
          {
            // Add constraints
            for (unsigned j(0); j < 2; ++j)
            {
              for (auto const &act : cons[j])
              {
                con.c[j].emplace_back((Constraint<state> *)new ContingentIdentical<state>(act->second.first, act->second.second, conflictset[j], conflictset[j ? 0 : 1], act->first[1], act->first[1]));
              }
            }
          }
          tree.emplace_back(tree[incumbent], con, incumbent, true);
          // We can fill in the paths...
          if(unbounded)
          {
            //tree[last].path=tree[0].path;
            tree[last].paths=tree[0].paths;
            tree[last].cct=tree[0].cct;
          }
          else
          {
            unsigned j(0);
            for (auto const &a : conflictset)
            {
              tree[last].path[j].reset(new std::vector<state>(srch.paths[s][j]));
              tree[last].paths[a] = tree[last].path[j].get();
              tree[last].clearcct(a); // Clear the cct for this unit so that we can re-count collisions
              ++j;
            }
          }
          unsigned cost(0);
          for (unsigned qq(0); qq < mins.size(); ++qq)
          {
            if (qq == conflictset[0])
            {
              cost += srch.finalcost[s][0];
            }
            else if (qq == conflictset[1])
            {
              cost += srch.finalcost[s][1];
            }
            else
            {
              if(unbounded)
              {
                cost += mins[qq];
              }
              else
              {
                cost += origcosts[qq];
              }
            }
          }
          if (srch.finalcost.size() > 1 && s == srch.finalcost.size() - 1 && cost <= currentCost)
          {
            cost = currentCost + 1;
          }

          if (Params::verbose)
          {
            std::cout << "New CT NODE: " << incumbent << ">" << last << " replanned: " << conflictset << " cost: " << cost << " " << numconflicts << " <";
            std::cout << tree[last].con.costs << "\n";
          }
          openList.emplace(last, cost, numconflicts);
        }
        else
        {
          // CBS Split
          // Re-filter constraints
          unsigned i(1);
          if (srch.finalcost[s][0] == origcosts[conflictset[0]])
          {
            i = 0; // Agent 0 had no change, reduce cost of agent 1
          }
          else if (srch.finalcost[s][1] != origcosts[conflictset[1]] && srch.finalcost[s][0] < srch.finalcost[s][1])
          {
            i = 0; // Both agents had change, but agent 0 had smaller cost, reduce cost of agent 1
          }
          auto other(i ? 0 : 1);
          //std::cout << "Filtered to agent " << conflictset[other] << " cost " << srch.finalcost[s][other] - 1 << " for agent " << conflictset[i] << "\n";
          //auto a(std::lower_bound(std::pair<std::pair<
          for (auto const &a : srch.intervals[i])
          {
            if (a.first[0] >= srch.finalcost[s][other] - 1)
            { // does upper bound encapsulate the target cost?
              if (a.first[1] <= srch.finalcost[s][other] - 1)
              {
                //std::cout << a << "\n";
                cons[i].push_back(&a);
              }
            }
          }

          if (cons[0].empty() || cons[1].empty())
          {
            cons[0].clear();
            cons[1].clear();
          }
          // Create the two nodes
          for (int i(0); i < 2; ++i)
          {
            auto last(tree.size());

            Conflict<state> tmpcon(conflictset);
            if (cons[i].empty())
            {
              // Resort to old-school CBS constraints
              if (i)
              {
                tmpcon.c[i].emplace_back((Constraint<state> *)new Identical<state>(tree[incumbent].paths[conflictset[1]]->at(stateIndices[1]), tree[incumbent].paths[conflictset[1]]->at(stateIndices[1] + 1), conflictset[i]));
              }
              else
              {
                tmpcon.c[i].emplace_back((Constraint<state> *)new Collision<state>(tree[incumbent].paths[conflictset[1]]->at(stateIndices[1]), tree[incumbent].paths[conflictset[1]]->at(stateIndices[1] + 1), radii[i], conflictset[i]));
              }
            }
            else
            {
              for (auto const &act : cons[i])
              {
                tmpcon.c[i].emplace_back((Constraint<state> *)new Identical<state>(act->second.first, act->second.second, conflictset[i]));
              }
            }
            tmpcon.costs=con.costs;
            tree.emplace_back(tree[incumbent], tmpcon, incumbent, true);
            tree[last].path.resize(2);
            // We can fill in the paths...
            unsigned j(0);
            for (auto const &a : conflictset)
            {
              tree[last].path[j].reset(new std::vector<state>(srch.paths[s][j]));
              tree[last].paths[a] = tree[last].path[j].get();
              tree[last].clearcct(a); // Clear the cct for this unit so that we can re-count collisions
              ++j;
            }
            if (Params::verbose)
            {
              std::cout << "  child node " << i << " (CBS): ";
              for (unsigned qq(0); qq < mins.size(); ++qq)
              {
                if (qq == conflictset[0])
                {
                  std::cout << "*" << con.costs[0].first << "-" << con.costs[0].second << ",";
                }
                else if (qq == conflictset[1])
                {
                  std::cout << "*" << con.costs[1].first << "-" << con.costs[1].second << ",";
                }
                else
                {
                  std::cout << mins[qq] << "-" << maxs[qq] << ",";
                }
              }
              std::cout << "\n";
            }
            unsigned cost(0);
            for (unsigned qq(0); qq < mins.size(); ++qq)
            {
              if (qq == conflictset[0])
              {
                cost += srch.finalcost[s][0];
              }
              else if (qq == conflictset[1])
              {
                cost += srch.finalcost[s][1];
              }
              else
              {
                cost += mins[qq];
              }
            }
            if (Params::verbose)
            {
              std::cout << "New CT NODE: " << incumbent << ">" << last << " units: " << conflictset << " cost: " << cost << " " << numconflicts << " \n";
            }
            openList.emplace(last, cost, numconflicts);
          }
          }
      }
    }
  }
  else
  {
    double cost = 0;
    for (int y = 0; y < tree[incumbent].paths.size(); y++)
    {
      auto cc(GetEnv(y)->GetPathLength(*tree[incumbent].paths[y]));
      cost += cc;
    }
    if (cost < bestCost)
    {
      bestNode = incumbent;
      bestCost = cost;
      if(openList.size()==0 || (openList.size()&&openList.top().cost>=bestCost)){
        processSolution(CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());
        return false;
      }
    }
  }

  // Set the visible paths for every unit in the node
  if (keeprunning)
    for (unsigned int x = 0; x < tree[incumbent].paths.size(); x++)
    {
      // Grab the unit
      auto unit(this->GetUnit(x));

      // Prune these paths to the current simulation time
      state current;
      unit->GetLocation(current);
      static std::vector<state> newPath;
      newPath.resize(0);
      newPath.push_back(current); // Add the current simulation node to the new path

      // For everything in the path that's new, add the path back
      for (state xNode : *tree[incumbent].paths[x])
      {
        if (current.t < xNode.t)
        {
          newPath.push_back(xNode);
        }
      }

      // Update the actual unit path
      unit->SetPath(newPath);
    }

  Pop();
  return true;
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSUnit<state, action, comparison, conflicttable, searchalgo>::MakeMove(ConstrainedEnvironment<state, action> *ae,
    OccupancyInterface<state, action> *, SimulationInfo<state, action, ConstrainedEnvironment<state, action>>* si,
    state& a) {
  if (myPath.size() > 1 && si->GetSimulationTime() * state::TIME_RESOLUTION_D > myPath[myPath.size() - 2].t) {
    a = myPath[myPath.size() - 2];

    //std::cout << "Moved from " << myPath[myPath.size()-1] << " to " << myPath[myPath.size()-2] << std::endl;
    //a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
    //std::cout << "Used action " << a << "\n";
    myPath.pop_back();
    return true;
  }
  return false;
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
bool CBSUnit<state, action, comparison, conflicttable, searchalgo>::MakeMove(ConstrainedEnvironment<state, action> *ae,
    OccupancyInterface<state, action> *, SimulationInfo<state, action, ConstrainedEnvironment<state, action>> * si,
    action& a) {
  if (myPath.size() > 1 && si->GetSimulationTime() * state::TIME_RESOLUTION_D > myPath[myPath.size() - 2].t) {

    //std::cout << "Moved from " << myPath[myPath.size()-1] << " to " << myPath[myPath.size()-2] << std::endl;
    //a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
    //std::cout << "Used action " << a << "\n";
    myPath.pop_back();
    return true;
  }
  return false;
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::MakeMove(
    Unit<state, action, ConstrainedEnvironment<state, action>> *u, ConstrainedEnvironment<state, action> *e,
    SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si, state& a) {
  if (planFinished && si->GetSimulationTime() > time) {
    return u->MakeMove(e, 0, si, a);
  } else if ((si->GetSimulationTime() - time) < 0.0001) {
    return false;
  } else {
    time = si->GetSimulationTime();
    ExpandOneCBSNode();
  }
  return false;
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::MakeMove(
    Unit<state, action, ConstrainedEnvironment<state, action>> *u, ConstrainedEnvironment<state, action> *e,
    SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si, action& a) {
  if (planFinished && si->GetSimulationTime() > time) {
    return u->MakeMove(e, 0, si, a);
  } else if ((si->GetSimulationTime() - time) < 0.0001) {
    return false;
  } else {
    time = si->GetSimulationTime();
    ExpandOneCBSNode();
  }
  return false;
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::processSolution(double elapsed) {
  double cost(0.0);
  unsigned total(0);
  unsigned maxTime(GetMaxTime(bestNode, 9999999));
  // For every unit in the node
  bool valid(true);
  for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++) {
    cost += GetEnv(x)->GetPathLength(*tree[bestNode].paths[x]);
    total += tree[bestNode].paths[x]->size();

    // Grab the unit
    auto unit(this->GetUnit(x));

    // Prune these paths to the current simulation time
    /*state current;
     unit->GetLocation(current);
     std::vector<state> newPath;
     newPath.push_back(current); // Add the current simulation node to the new path

     // For everything in the path that's new, add the path back
     for (state xNode : *tree[bestNode].paths[x]) {
     if (current.t < xNode.t - 0.0001) {
     newPath.push_back(xNode);
     }
     }*/

    // Update the actual unit path
    // Add an extra wait action for "visualization" purposes,
    // This should not affect correctness...
    if (!disappearAtGoal && tree[bestNode].paths[x]->size() && tree[bestNode].paths[x]->back().t < maxTime) {
      tree[bestNode].paths[x]->push_back(tree[bestNode].paths[x]->back());
      tree[bestNode].paths[x]->back().t = maxTime;
    }

    // For everything in the path that's new, add the path back
    static std::vector<state> newPath;
    newPath.resize(0);
    for (state const& xNode : *tree[bestNode].paths[x]) {
      newPath.push_back(xNode);
    }
    newPath.push_back(tree[bestNode].paths[x]->back());
    unit->SetPath(newPath);
    if (tree[bestNode].paths[x]->size()) {
      if (!quiet)std::cout << "Agent " << x << "(" << GetEnv(x)->GetPathLength(*tree[bestNode].paths[x]) << "): " << "\n";
      unsigned wpt(0);
      signed ix(0);
      for (auto &a : *tree[bestNode].paths[x]) {
        //std::cout << a << " " << wpt << " " << unit->GetWaypoint(wpt) << "\n";
        if (ix++ == tree[bestNode].wpts[x][wpt]) {
          if (!quiet)std::cout << " *" << a << "\n";
          if (wpt < tree[bestNode].wpts[x].size() - 1)
            wpt++;
        } else {
          if (!quiet)std::cout << "  " << a << "\n";
        }
      }
    } else {
      if (!quiet)std::cout << "Agent " << x << ": " << "NO Path Found.\n";
    }
    // Only verify the solution if the run didn't time out
    if (verify && elapsed > 0) {
      for (unsigned int y = x + 1; y < tree[bestNode].paths.size(); y++) {
        auto ap(tree[bestNode].paths[x]->begin());
        auto a(ap + 1);
        auto bp(tree[bestNode].paths[y]->begin());
        auto b(bp + 1);
        while (a != tree[bestNode].paths[x]->end() && b != tree[bestNode].paths[y]->end()) {
          if(collisionCheck3D(*ap, *a, *bp, *b, radii[x],radii[y])){
            valid = false;
            std::cout << "ERROR: Solution invalid; collision at: " << x << ":" << *ap << "-->" << *a << ", " << y << ":"
              << *bp << "-->" << *b << std::endl;
          }
          if (a->t < b->t) {
            ++a;
            ++ap;
          } else if (a->t > b->t) {
            ++b;
            ++bp;
          } else {
            ++a;
            ++b;
            ++ap;
            ++bp;
          }
        }
      }
    }
  }
  if(keeprunning){
    fflush(stdout);
    std::cout << "elapsed,collisions,cost,planTime,replanTime,collisionTime,expansions,CATcollchecks,collchecks,actions,maxCSet,meanCSet\n";
    if (verify && elapsed > 0)
      std::cout << (valid ? "VALID" : "INVALID") << std::endl;
    if (elapsed < 0) {
      //std::cout << seed<<":FAILED\n";
      std::cout << seed << ":" << elapsed * (-1.0) << ",";
    } else {
      std::cout << seed << ":" << elapsed << ",";
    }
    std::cout << tree.size() << ",";
    std::cout << cost / state::TIME_RESOLUTION_D << ",";
    std::cout << planTime << ",";
    std::cout << replanTime << ",";
    std::cout << collisionTime << ",";
    std::cout << TOTAL_EXPANSIONS << ",";
    std::cout << comparison::collchecks << ",";
    std::cout << collchecks << ",";
    std::cout << total << ",";
    std::cout << constraintsz/std::max(1ul,constrainttot) << std::endl;
  }
  planFinished = true;
  //if (!keeprunning)
    //exit(0);
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state,action,comparison,conflicttable,singleHeuristic,searchalgo>::Init(){
  //unsigned total(0);
  //for(auto const& p:tree[0].paths){
    //total+=p->size();
  //}
  // Do an initial conflict check, then compute the minimum composite cost
  std::vector<unsigned> conflictset;
  std::unordered_map<agentpair_t,unsigned,agentpair_t> edgeWeights;
  std::vector<unsigned> stateIndices;
  std::vector<unsigned> pathCosts;
  pathCosts.reserve(agents.size());
  for(auto const& a:agents)
  {
    pathCosts.push_back(GetEnv(a)->GetPathLength(*tree[0].paths[a]));
  }
  unsigned numconflicts(FindConflictGroupAllPairs(tree[0], conflictset, stateIndices));
  conflictset.clear();
  std::set<unsigned> vertices;
  for(auto const& pair:tree[0].cct){
    auto a1(pair.first.a1);
    auto a2(pair.first.a2);
    PairwiseConstrainedSearch<ConstrainedEnvironment<state, action>, state, action> srch(
      0, 1,
      GetEnv(a1), GetEnv(a2),
      GetHeuristic(a1), GetHeuristic(a2),
      0,UINT_MAX,0,UINT_MAX,
      GetUnit(a1)->GetWaypoint(0), GetUnit(a1)->GetWaypoint(1), radii[0],
      GetUnit(a2)->GetWaypoint(0), GetUnit(a2)->GetWaypoint(1), radii[1],
      0,true); // Solution must be greater cost than soc
    srch.getRangesAndConstraints();
    
    if (!srch.finalcost.empty())
    {
      auto increase(srch.finalcost[0][0]-pathCosts[a1] + srch.finalcost[0][1]-pathCosts[a2]);
      if (increase)
      {
        edgeWeights[{pair.first}] = increase;
        vertices.insert(a1);
        vertices.insert(a2);
      }
    }
  }
  auto sz(vertices.size());
  std::vector<unsigned> adjmat(sz*sz);
  for(auto const& e:edgeWeights)
  {
    adjmat[e.first.a1*sz+e.first.a2]=e.second;
    adjmat[e.first.a2*sz+e.first.a1]=e.second;
  }
  auto startCost(weightedVertexCover(adjmat));
  startCost+=openList.top().cost;
  openList.pop();
  openList.emplace(0,startCost,numconflicts);
}

/** Update the location of a unit */
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::UpdateLocation(
    Unit<state, action, ConstrainedEnvironment<state, action>> *u, ConstrainedEnvironment<state, action> *e, state &loc,
    bool success, SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si) {
  u->UpdateLocation(e, loc, success, si);
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::SetEnvironment(unsigned agent) {
  astar.SetHeuristic(GetHeuristic(agent));
  astar.SetWeight(GetUnit(agent)->env->astar_weight);
  astar.SetGoalSteps(!disappearAtGoal && !Params::extrinsicconstraints);
}

/** Add a new unit with a new start and goal state to the CBS group */
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::AddUnit(
    Unit<state, action, ConstrainedEnvironment<state, action>> *u) {
  astar.SetExternalExpansionsPtr(&TOTAL_EXPANSIONS);
  astar.SetExternalExpansionLimit(killex);

  CBSUnit<state, action, comparison, conflicttable, searchalgo> *c = (CBSUnit<state, action, comparison, conflicttable,
      searchalgo>*) u;
  radii.push_back(c->radius);
  unsigned theUnit(this->GetNumMembers());
  c->setUnitNumber(theUnit);
  // Add the new unit to the group, and construct an CBSUnit
  UnitGroup<state, action, ConstrainedEnvironment<state, action>>::AddUnit(u);
  basepaths.resize(this->GetNumMembers());

  // Clear the env constraints
  for (unsigned i(0); i < basepaths.size(); ++i) {
    // Clear the constraints from the environment set
    ClearEnvironmentConstraints(i);
  }

  SetEnvironment(theUnit);

  // Setup the state and goal in the graph
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Resize the number of paths in the root of the tree
  tree[0].paths.resize(this->GetNumMembers());
  tree[0].paths.back() = &basepaths[this->GetNumMembers()-1];
  tree[0].wpts.resize(this->GetNumMembers());
  //agentEnvs.resize(this->GetNumMembers());

  // Recalculate the optimum path for the root of the tree
  //std::cout << "AddUnit "<<(theUnit) << " getting path." << std::endl;
  //std::cout << "Search using " << GetEnv(theUnit)->name() << "\n";
  //agentEnvs[c->getUnitNumber()]=GetEnv(theUnit);
  comparison::CAT = &(tree[0].cat);
  comparison::CAT->set(&tree[0].paths);
  GetFullPath<state, action, comparison, conflicttable, searchalgo, singleHeuristic>(c, astar, *(GetUnit(theUnit)->env),
      *tree[0].paths.back(), tree[0].wpts.back(), 0, theUnit, replanTime, TOTAL_EXPANSIONS);
  assert(tree[0].paths.back()->size());
  if (killex != INT_MAX && TOTAL_EXPANSIONS > killex)
    processSolution(-CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());
  //std::cout << "AddUnit agent: " << (theUnit) << " expansions: " << astar.GetNodesExpanded() << "\n";

  // Create new conflict avoidance table instance
  if(comparison::useCAT && this->GetNumMembers() < 2){
    tree[0].cat = conflicttable();
  // We add the optimal path to the root of the tree
    tree[0].cat.insert(*tree[0].paths.back(), GetEnv(theUnit), tree[0].paths.size() - 1);
  }
  //StayAtGoal(0); // Do this every time a unit is added because these updates are taken into consideration by the CAT

  // Set the plan finished to false, as there's new updates
  planFinished = false;

  // Clear up the rest of the tree and clean the open list
  tree.resize(1);
  incumbent = 0;
  openList.clear();
  unsigned cost(0);
  for(unsigned i(0); i<basepaths.size(); ++i){
    cost+=GetEnv(i)->GetPathLength(basepaths[i]);
  }
  openList.emplace(0, cost, 0);
}

/** Add a new unit with an existing path **/
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::AddUnit(
    Unit<state, action, ConstrainedEnvironment<state, action>> *u, std::vector<state> const& path) {
  CBSUnit<state, action, comparison, conflicttable, searchalgo> *c = (CBSUnit<state, action, comparison, conflicttable, searchalgo>*) u;
  unsigned theUnit(this->GetNumMembers());
  c->setUnitNumber(theUnit);
  radii.push_back(c->radius);
  // Add the new unit to the group
  UnitGroup<state, action, ConstrainedEnvironment<state, action>>::AddUnit(u);

  basepaths[theUnit]=path;

  // Clear the env constraints
  for (unsigned i(0); i < basepaths.size(); ++i) {
    // Clear the constraints from the environment set
    ClearEnvironmentConstraints(i);
  }

  SetEnvironment(theUnit);

  // Resize the number of paths in the root of the tree
  tree[0].paths.push_back(&basepaths[theUnit]);
  tree[0].wpts.resize(this->GetNumMembers());
  tree[0].wpts[this->GetNumMembers()-1].push_back(0);
  tree[0].wpts[this->GetNumMembers()-1].push_back(path.size()-1);

  // Create new conflict avoidance table instance
  if(comparison::useCAT && this->GetNumMembers() < 2){
    tree[0].cat = conflicttable();
  // We add the optimal path to the root of the tree
    tree[0].cat.insert(*tree[0].paths.back(), GetEnv(theUnit), tree[0].paths.size() - 1);
  }
  //StayAtGoal(0); // Do this every time a unit is added because these updates are taken into consideration by the CAT

  // Set the plan finished to false, as there's new updates
  planFinished = false;

  // Clear up the rest of the tree and clean the open list
  tree.resize(1); // Any time we add a new unit, truncate the tree
  incumbent = 0;
  openList.clear();
  unsigned cost(0);
  for(unsigned i(0); i<basepaths.size(); ++i){
    cost+=GetEnv(i)->GetPathLength(basepaths[i]);
  }
  openList.emplace(0, cost, 0);
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::GetMaxTime(int location, int agent) {

  unsigned maxDuration(1);
  if (disappearAtGoal)
    return 0;

  int i(0);
  // Find max duration of all paths
  for (auto const& n : tree[location].paths) {
    if (agent != i++)
      maxDuration = std::max(maxDuration, n->back().t);
  }
  return maxDuration;
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::StayAtGoal(int location) {

  if (disappearAtGoal)
    return;
  unsigned maxDuration(0.0);

  // Find max duration of all paths
  for (auto const& n : tree[location].paths) {
    maxDuration = std::max(maxDuration, n->back().t);
  }
  if (maxDuration < state::TIME_RESOLUTION_U)
    return;

  // Add wait actions (of 1 second) to goal states less than max
  for (auto& n : tree[location].paths) {
    while (n->back().t < maxDuration - state::TIME_RESOLUTION_U) {
      state x(n->back());
      x.t += state::TIME_RESOLUTION_U;
      n->push_back(x);
    }
  }
}

// Loads conflicts into environements and returns the number of conflicts loaded.
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::LoadConstraintsForNode(int location, int agent)
{
  // Select the unit from the tree with the new constraint
  unsigned numConflicts(0);

  // Reset the constraints in the test-environment
  ClearEnvironmentConstraints(agent);

  // Add all of the constraints in the parents of the current node to the environment
  while (location)
  {
    for (unsigned i(0); i < tree[location].con.units.size(); ++i)
    {
      if (agent == tree[location].con.units[i])
      {
        numConflicts++;
        for (auto const &c : tree[location].con.c[i])
        {
          AddEnvironmentConstraint(c.get(), agent, true);
          if (!quiet)std::cout << "Adding constraint (in accumulation)[" << location << "] " << typeid(*c.get()).name() << ": " << c->start_state << "-->" << c->end_state << " for agent " << agent << "\n";
        }
      }
    }
    location = tree[location].parent;
  } // while (location != 0);
  ProbIdentical<state>::prob = Probable<state>::prob = Params::complete ? numConflicts - 1 : 0;
  return numConflicts;
}

template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::LoadConstraints(std::vector<unsigned> const &agents,unsigned location, std::vector<unsigned> const& costs)
{
  for(auto const& a:agents){
    ClearEnvironmentConstraints(a);
  }
  // This is the set of agents for which constraints are invalid
  //std::set<unsigned> invalid;
  auto loc(location);
  unsigned go(loc+1);
  while(go)
  {
    go=loc;// Let it load constraints for the root, then exit.
    // Agents involved in this node
    if(tree[loc].con.units.size()) // nothing in "units" means no constraints
    for (unsigned i(0); i < tree[loc].con.units.size(); ++i)
    {
      // Agents to add constraints for
      for (auto const &a : agents)
      {
        // This node has constraints for the agent
        if (a == tree[loc].con.units[i])
        {
          // Add all of the constraints
          for (auto const &c : tree[loc].con.c[i])
          {
            bool ok(true);
            if (typeid(*c) == typeid(ContingentIdentical<state>))
            {
              ContingentIdentical<state>const& cc((ContingentIdentical<state>&)*c);
              if(costs[cc.refAgent]<cc.lb || costs[cc.refAgent]>cc.ub){
                ok=false;
              }
            }
            if(ok){
              AddEnvironmentConstraint(c.get(), a, true);
            }
          }
        }
      }
    }
    loc = tree[loc].parent;
  }
  /*{
    bool keep(true);
    for (auto const &v : invalid)
    {
      if (std::find(tree[location].con.units.begin(),
                    tree[location].con.units.end(), v) != tree[location].con.units.end())
      {
        keep = false;
        break;
      }
    }
    for (unsigned i(0); i < tree[location].con.units.size(); ++i)
    {
      if (tree[location].con.costs[i] < costs[tree[location].con.units[i]])
      {
        invalid.insert(tree[location].con.units[i]);
        keep=false;
      }
    }
    if (keep)
    {
      for (unsigned i(0); i < tree[location].con.units.size(); ++i)
      {
        for (auto const &a : agents)
        {
          if (a == tree[location].con.units[i])
          {
            for (auto const &c : tree[location].con.c[i])
              AddEnvironmentConstraint(c.get(), a, true);
          }
        }
      }
    }

    location = tree[location].parent;
  }*/
}

/** Replan a node given a constraint */
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::Replan(
  int location, unsigned theUnit, std::pair<unsigned,unsigned> const& limits, std::vector<unsigned> const& costs)
{
  // Select the unit from the tree with the new constraint
  LoadConstraints({theUnit},location,costs);
  ProbIdentical<state>::cond = Probable<state>::cond = tree[0].paths[theUnit]->size() * tree[0].paths.size() * GetEnv(theUnit)->branchingFactor();
  //if(ProbIdentical<state>::cond<ProbIdentical<state>::prob)
  //ProbIdentical<state>::prob=Probable<state>::prob=Probable<state>::prob-ProbIdentical<state>::cond;
  //else
  //ProbIdentical<state>::prob=Probable<state>::prob=0;
  //std::cout << "cond: " << Probable<state>::cond << " prob: " << Probable<state>::prob << " %: " << float(Probable<state>::prob)/float(Probable<state>::cond) << "\n";

  // Set the environment based on the number of conflicts
  SetEnvironment(theUnit); // This has to happen before calling LoadConstraints

  // Select the unit from the group
  auto c(this->GetUnit(theUnit));

  // Retreive the unit start and goal
  //state start, goal;
  //c->GetStart(start);
  //c->GetGoal(goal);

  // Recalculate the path
  //std::cout << "#conflicts for " << tempLocation << ": " << numConflicts << "\n";
  //GetEnv(theUnit)->setGoal(goal);
  //std::cout << numConflicts << " conflicts " << " using " << GetEnv(theUnit)->name() << " for agent: " << tree[location].con.unit1 << "?="<<c->getUnitNumber()<<"\n";
  //agentEnvs[c->getUnitNumber()]=GetEnv(theUnit);
  //astar.GetPath(GetEnv(theUnit), start, goal, thePath);
  //std::vector<state> thePath(tree[location].paths[theUnit]);
  comparison::openList = astar.GetOpenList();
  comparison::currentEnv = (ConstrainedEnvironment<state, action> *)GetEnv(theUnit);
  comparison::currentAgent = theUnit;
  comparison::CAT = &(tree[location].cat);
  comparison::CAT->set(&tree[location].paths);

  if (comparison::useCAT)
  {
    comparison::CAT->remove(*tree[location].paths[theUnit], GetEnv(theUnit), theUnit);
  }

  unsigned minTime(0);
  // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
  //if (tree[location].con.prevWpt + 1 == tree[location].wpts[theUnit].size() - 1)
  //{
    //minTime = GetMaxTime(location, theUnit); // Take off a 1-second wait action, otherwise paths will grow over and over.
  //}

  if (!quiet)std::cout << "Replan agent " << theUnit << "\n";
  //if(!quiet)std::cout << "re-planning path from " << start << " to " << goal << " on a path of len:" << thePath.size() << " out to time " << minTime <<"\n";
  tree[location].paths[theUnit]->clear();
  ReplanLeg<state, action, comparison, conflicttable, searchalgo, singleHeuristic>(c, astar, *(GetUnit(theUnit)->env), *tree[location].paths[theUnit], tree[location].wpts[theUnit], 0,1, limits.first, theUnit, replanTime, TOTAL_EXPANSIONS);
  //for(int i(0); i<tree[location].paths.size(); ++i)
  //std::cout << "Replanned agent "<<i<<" path " << tree[location].paths[i]->size() << "\n";

  if (killex != INT_MAX && TOTAL_EXPANSIONS > killex)
    processSolution(-CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());

  if (tree[location].paths[theUnit]->size() < 1 && Params::complete)
  {
    ProbIdentical<state>::prob = Probable<state>::prob = 0;
    ReplanLeg<state, action, comparison, conflicttable, searchalgo, singleHeuristic>(c, astar, *(GetUnit(theUnit)->env), *tree[location].paths[theUnit], tree[location].wpts[theUnit], 0,1, limits.first, theUnit, replanTime, TOTAL_EXPANSIONS);
  }
  //DoHAStar(start, goal, thePath);
  //TOTAL_EXPANSIONS += astar.GetNodesExpanded();
  //std::cout << "Replan agent: " << location << " expansions: " << astar.GetNodesExpanded() << "\n";
  if(GetUnit(theUnit)->env->environment->GetPathLength(*tree[location].paths[theUnit]) > limits.second){
    tree[location].paths[theUnit]->resize(0); // Assuming astar gave us the shortest possible path, then if the path is too long, we know it couldn't fit in the interval.
  }

  // Make sure that the current location is satisfiable
  if (tree[location].paths[theUnit]->size() < 1)
  {
    tree[location].satisfiable = false;
  }

  // Add the path back to the tree (new constraint included)
  //tree[location].paths[theUnit].resize(0);
  if (comparison::useCAT)
    comparison::CAT->insert(*tree[location].paths[theUnit], GetEnv(theUnit), theUnit);
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::CountConflicts(
    std::vector<state> const& a, std::vector<int> const& wa,
    std::vector<state> const& b, std::vector<int> const& wb,
    int x, int y){
  CBSTreeNode<state, conflicttable>& location=tree[incumbent];
  // The conflict parameter contains the conflict count so far (conflict.first)
  // and the type of conflict found so far (conflict.second=BOTH_CARDINAL being the highest)

  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax(a.size());
  int ymax(b.size());
  unsigned ccount(0);

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
    int xTime(max(0, min(i, xmax - 2)));
    int yTime(max(0, min(j, ymax - 2)));
    int xNextTime(min(xmax - 1, xTime + 1));
    int yNextTime(min(ymax - 1, yTime + 1));

    if (xTime != pxTime && pwptA + 2 < wa.size() && xTime == wa[pwptA + 1]) {
      ++pwptA;
      pxTime = xTime;
    }
    if (yTime != pyTime && pwptB + 2 < wb.size() && yTime == wb[pwptB + 1]) {
      ++pwptB;
      pyTime = yTime;
    }

    collchecks++;
    if(collisionCheck3D(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y])){
      location.setcct(x, y, xTime, yTime);
      if(verbose)std::cout << "Conflict: agents=(" << x << "," << y << ") @ " << a[xTime] << "-->" << a[xNextTime]<<","<<b[yTime]<<"-->"<<b[yNextTime] << "\n";
      ++ccount;
    }

    // Increment the counters based on the time

    // First we check to see if either is at the end
    // of the path. If so, immediately increment the 
    // other counter.
    if (i == xmax) {
      j++;
      continue;
    } else if (j == ymax) {
      i++;
      continue;
    }

    // Otherwise, we figure out which ends soonest, and
    // we increment that counter.
    if (a[xNextTime].t < b[yNextTime].t) {
      // If the end-time of the x unit segment is before the end-time of the y unit segment
      // we have in increase the x unit but leave the y unit time the same
      i++;
    } else if (a[xNextTime].t == b[yNextTime].t) {
      i++;
      j++;
    } else {
      // Otherwise, the y unit time has to be incremented
      j++;
    }

  } // End time loop
  return ccount;
}


bool isClique(std::vector<unsigned> const& store,
unsigned n,
std::vector<std::vector<bool>> const& graph)
{
  // Run a loop over all the set of edges
  // for the selected vertices
  for (unsigned i(0); i < n - 1; i++)
  {
    for (unsigned j(i + 1); j < n; j++)
    {
      // If any edge is missing it is not a clique
      if (!graph[store[i]][store[j]])
        return false;
    }
  }
  return true;
}

// Function to find all the cliques of size s 
bool findClique(unsigned i, unsigned l,
std::vector<unsigned>& store,
std::vector<std::vector<bool>> const& graph,
std::vector<unsigned> const& d,
unsigned s) 
{ 
	// Check if any vertices from i+1 can be inserted 
	for (unsigned j(i); j < graph.size() - (s - l); j++) 

		// If the degree of the graph is sufficient
    if (d[j] >= s - 1)
    {
      // Add the vertex to store
      store[l-1] = j;
      // If the graph is a clique of size < k
      // then it could lead to clique of size s
      // otherwise it can't
      if(isClique(store, l, graph))
      {
        // If the length of the clique is
        // still less than the desired size
        if (l < s)
        {
          // Recursion to add vertices
          return findClique(j, l + 1, store, graph, d, s);
        }
        else
        {
          return true; // the solution is in "store"
        }
      }
    } 
    return false;
} 

template <typename T>
signed indexOf(T const& v, std::vector<T> const& vec){
  auto s(std::find(vec.begin(),vec.end(),v));
  if(s==vec.end()) return -1;
  return std::distance(vec.begin(),s);
}

/** Find largest conflict group **/
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::FindConflictGroupOneVsAll(
    CBSTreeNode<state, conflicttable> const &location, unsigned x, std::vector<unsigned> &confset, std::vector<unsigned>& stateIndices, bool update)
{
  if (Params::verbose)std::cout << "Checking for conflict groups (one vs all)\n";

  Timer tmr;
  tmr.StartTimer();
  // For each pair of units in the group
  for (unsigned y(0); y < location.paths.size(); ++y)
  {
    if (x == y)
      continue;
    // Augment paths to have same end time if necessary,
    // Either add a wait action of sufficient length or
    // Extend an existing wait action to the necessary length
    if (!disappearAtGoal)
    {
      if (location.paths[x]->back().t < location.paths[y]->back().t)
      {
        if (location.paths[x]->size() > 1 && !location.paths[x]->back().sameLoc(*(location.paths[x]->rbegin() + 1)))
        {
          if (Params::extrinsicconstraints)
          {
            location.paths[x]->push_back(location.paths[x]->back());
          }
          else
          {
            while (location.paths[x]->back().t < location.paths[y]->back().t - GetEnv(x)->WaitTime())
            {
              location.paths[x]->push_back(location.paths[x]->back());
              location.paths[x]->back().t += GetEnv(x)->WaitTime();
            }
          }
        }
        location.paths[x]->back().t = location.paths[y]->back().t;
      }
      else if (location.paths[y]->back().t < location.paths[x]->back().t)
      {
        if (location.paths[y]->size() > 1 && !location.paths[y]->back().sameLoc(*(location.paths[y]->rbegin() + 1)))
        {
          if (Params::extrinsicconstraints)
          {
            location.paths[y]->back().t += GetEnv(y)->WaitTime();
          }
          else
          {
            while (location.paths[y]->back().t < location.paths[x]->back().t - GetEnv(x)->WaitTime())
            {
              location.paths[y]->push_back(location.paths[y]->back());
              location.paths[y]->back().t += GetEnv(y)->WaitTime();
            }
          }
        }
        location.paths[y]->back().t = location.paths[x]->back().t;
      }
    }
    CountConflicts(*location.paths[x], location.wpts[x], *location.paths[y], location.wpts[y], x, y);
  }

  if (update)
    GetConfGroup(location, confset, stateIndices);
  collisionTime += tmr.EndTimer();
  return location.numCollisions();
}

/** Find largest conflict group **/
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::FindConflictGroupAllPairs(
  CBSTreeNode<state, conflicttable> const& location,
  std::vector<unsigned>& confset,
  std::vector<unsigned>& stateIndices)
{
  unsigned total(0);
  if (Params::verbose)std::cout << "Searching for conflict groups (all pairs)\n";
  Timer tmr;
  tmr.StartTimer();

  // For each pair of units in the group
  for (unsigned x(0); x < location.paths.size() - 1; ++x)
  {
    for (unsigned y(x + 1); y < location.paths.size(); ++y)
    {
      // Augment paths to have same end time if necessary,
      // Either add a wait action of sufficient length or
      // Extend an existing wait action to the necessary length
      if (!disappearAtGoal)
      {
        if (location.paths[x]->back().t < location.paths[y]->back().t)
        {
          if (location.paths[x]->size() > 1 && !location.paths[x]->back().sameLoc(*(location.paths[x]->rbegin() + 1)))
          {
            if (Params::extrinsicconstraints)
            {
              location.paths[x]->push_back(location.paths[x]->back());
            }
            else
            {
              while (location.paths[x]->back().t < location.paths[y]->back().t - GetEnv(x)->WaitTime())
              {
                location.paths[x]->push_back(location.paths[x]->back());
                location.paths[x]->back().t += GetEnv(x)->WaitTime();
              }
            }
          }
          location.paths[x]->back().t = location.paths[y]->back().t;
        }
        else if (location.paths[y]->back().t < location.paths[x]->back().t)
        {
          if (location.paths[y]->size() > 1 && !location.paths[y]->back().sameLoc(*(location.paths[y]->rbegin() + 1)))
          {
            if (Params::extrinsicconstraints)
            {
              location.paths[y]->push_back(location.paths[y]->back());
            }
            else
            {
              while (location.paths[y]->back().t < location.paths[x]->back().t - GetEnv(x)->WaitTime())
              {
                location.paths[y]->push_back(location.paths[y]->back());
                location.paths[y]->back().t += GetEnv(y)->WaitTime();
              }
            }
          }
          location.paths[y]->back().t = location.paths[x]->back().t;
        }
      }
      total += CountConflicts(*location.paths[x], location.wpts[x], *location.paths[y], location.wpts[y], x, y);
    }
  }

  if (!total)
  {
    collisionTime += tmr.EndTimer();
    return 0;
  }

  GetConfGroup(location,confset, stateIndices);
  collisionTime += tmr.EndTimer();
  return total;
}

/** Find largest conflict group **/
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::GetConfGroup(
  CBSTreeNode<state, conflicttable> const& location,
  std::vector<unsigned>& confset,
  std::vector<unsigned>& stateIndices)
{

  std::map<std::pair<unsigned,unsigned>,std::vector<std::pair<unsigned,unsigned>>> action2conf;
  unsigned biggest(0);
  for (auto const &pair : location.cct)
  {
    for (auto const &ref : pair.second)
    {
      auto p1(std::make_pair(pair.first.a1, ref.ix1));
      auto p2(std::make_pair(pair.first.a2, ref.ix2));
      action2conf[p1].emplace_back(pair.first.a2, ref.ix2);
      action2conf[p2].emplace_back(pair.first.a1, ref.ix1);
      biggest=std::max(biggest, (unsigned)action2conf[p1].size());
      biggest=std::max(biggest, (unsigned)action2conf[p2].size());
    }
  }
  std::vector<unsigned> hist(biggest+1);
  for(auto const& a:action2conf){
    for(unsigned i(1); i<=a.second.size(); ++i){
      hist[i]++;
    }
  }
  biggest=1;
  for(unsigned i(hist.size()-1);i>1;--i){
    if (hist[i]>=i)
    {
      biggest = i;
      break;
    }
  }
  if (Params::pairwise || biggest == 1)
  {
    confset.resize(2);
    stateIndices.resize(2);
    location.mostConflicting(confset[0], confset[1], stateIndices[0], stateIndices[1]);
    return;
  }
  //std::vector<std::vector<unsigned>> indices(hist.size());
  //indices.reserve(hist.size());
  while (biggest>1)
  {
    std::vector<std::pair<unsigned,unsigned>> v; // set of vs
    v.reserve(action2conf.size());
    std::vector<unsigned> d; // degree of each v in graph
    d.reserve(action2conf.size());
    auto initial(action2conf.begin());
    while(initial != action2conf.end())
    {
      if(initial->second.size() < biggest) {
        ++initial;
        continue;
      }
      // Check if initial action is mutex with others that are of sufficient size
      unsigned count(0);
      for (auto const &ref : initial->second)
      {
        if (action2conf[ref].size() >= biggest)
        {
          ++count;
        }
      }
      if (count < biggest)
      {
        ++initial;
        continue;
      }
      else
      {
        // Add to v list
        v.push_back(initial->first);
        d.push_back(initial->second.size());
      }
      ++initial;
    }
    if(d.empty()){
      biggest--;
      continue;
    }

    std::vector<std::vector<bool>> adjmatrix(v.size(),std::vector<bool>(v.size()));
    bool proceedWithCliquefinding(true);
    for(unsigned i(0); i<v.size(); ++i){
      for(auto const& vv:action2conf[v[i]]){
        auto index(indexOf(vv,v));
        if(index>=0){
          adjmatrix[i][index]=true;
        }
        //auto vi(std::find(v.begin(),v.end(),vv));
        //if(vi!=v.end()){
          //adjmatrix[i][std::distance(v.begin(),vi)]=true;
        //}
      }

      auto cnt(std::count(adjmatrix[i].begin(),adjmatrix[i].end(),true));
      d[i]=cnt;
      if(cnt<biggest){
        // remove this vertex
        v.erase(v.begin()+i);
        d.erase(d.begin()+i);
        adjmatrix.erase(adjmatrix.begin()+i);
        if(v.size()<biggest){
          proceedWithCliquefinding=false;
          break;
        }
        --i;
      }
    }
    if(proceedWithCliquefinding){
      std::vector<unsigned> result(biggest+1);
      if(findClique(0, 1, result, adjmatrix, d, biggest + 1)){
        for(auto const& b:result){
          confset.push_back(v[b].first); // append agent number
          stateIndices.push_back(b); // TODO: Double check this - it should be the state index in the path
        }
        return;
      }else{--biggest;}
    }else{--biggest;}
  }

  confset.resize(2);
  stateIndices.resize(2);
  location.mostConflicting(confset[0], confset[1], stateIndices[0], stateIndices[1]);
}

/** Draw the AIR CBS group */
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::OpenGLDraw(
    const ConstrainedEnvironment<state, action> *ae,
    const SimulationInfo<state, action, ConstrainedEnvironment<state, action>> * sim) const {
      std::vector<float> xcoord(tree.size());
      unsigned maxDepth(0);
      for(int i(0); i<tree.size(); ++i){
        maxDepth=std::max(tree[i].depth,maxDepth);
      }
      std::vector<std::vector<unsigned>> depth(maxDepth+1);
      std::vector<std::vector<unsigned>> child(tree.size());
      depth[0]={0};
      float width(0);
      for(int i(1); i<tree.size(); ++i){
        depth[tree[i].depth].push_back(i);
        child[tree[i].parent].push_back(i);
        width=std::max(width,float(depth[tree[i].depth].size()));
      }
      // Calculate coords
      std::vector<float> xs(tree.size());
      std::vector<float> ys(tree.size());
      for (int i(0); i<depth.size(); ++i)
      {
        for (int k(0); k < depth[i].size(); ++k)
        {
          unsigned n(depth[i][k]);
          float x,y;
          if (n == 0)
          {
            // Draw a tree node
            x=.5;
            y=-.5;
          }
          else
          {
            y=ys[tree[n].parent]+0.2;
            float ix(indexOf(n,child[tree[n].parent]));
            x=xs[tree[n].parent]+(ix* (1/(width/depth[i].size())))/2.0;
          }
            xs[n] = x;
            ys[n] = y;
        }
      }

      //char buf[7];
      for (int i(0); i < tree.size(); ++i)
      {
        if (i)
        {
          // Draw line back to parent
          glColor3f(0, 0, 0);
          glLineWidth(1.0);
          glBegin(GL_LINES);
          glVertex3f(xs[i], ys[i], .001);
          glVertex3f(xs[tree[i].parent], ys[tree[i].parent], .001);
          glEnd();
        }
        glColor3f(1, 1, 1);
        //DrawBox(x,y,0,.1);
        //glColor3f(1, 1, 1);
        DrawRect(xs[i], ys[i], 0, .06, .04);
        glColor3f(0, 0, 0);
        if(tree[i].con.units.empty())
          DrawFmtTextCentered(xs[i], ys[i] - .03, -.01, .04, "%d", i);
        else if(tree[i].con.units.size()==1)
          DrawFmtTextCentered(xs[i], ys[i] - .03, -.01, .04, "%d - %d", i, tree[i].con.units[0]);
        else if(tree[i].con.units.size()==2)
        DrawFmtTextCentered(xs[i], ys[i] - .03, -.01, .04, "%d - %d,%d", i, tree[i].con.units[0], tree[i].con.units[1]);
        else if(tree[i].con.units.size()==3)
        DrawFmtTextCentered(xs[i], ys[i] - .03, -.01, .04, "%d - %d,%d,%d", i, tree[i].con.units[0], tree[i].con.units[1], tree[i].con.units[2]);
        // Add cost vector
        unsigned j(0);
        //std::cout << "FOOOOO " << xs[i] << " " << ys[i] << "\n";
        for (unsigned u : tree[i].con.units)
        {
          auto const &unit(this->GetUnit(u));
          //snprintf(buf, sizeof(buf), "%.1f", tree[i].con.costs[j] / state::TIME_RESOLUTION);

          GLfloat r, g, b;
          unit->GetColor(r, g, b);
          glColor3f(r, g, b);
          //std::cout << "         " << (xs[i]-(.1*(tree[i].con.costs.size()-1)) + .05 * j) << " " << ys[i] << "\n";
          DrawFmtTextCentered(xs[i],
          ys[i]+j*.02-.01,
          -.01,
          0.04,
          "(%.1f,%.1f)",
          tree[i].con.costs[j].first / state::TIME_RESOLUTION,
          tree[i].con.costs[j].second==UINT_MAX?HUGE_VAL:(tree[i].con.costs[j].second / state::TIME_RESOLUTION));
          ++j;
        }
      }
  /*
   GLfloat r, g, b;
   glLineWidth(2.0);
   for (unsigned int x = 0; x < tree[incumbent].paths.size(); x++)
   {
   CBSUnit<state,action,comparison,conflicttable,searchalgo> *unit = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*)this->GetMember(x);
   unit->GetColor(r, g, b);
   ae->SetColor(r, g, b);
   for (unsigned int y = 0; y < tree[incumbent].paths[x]->size(); y++)
   {
   ae->OpenGLDraw(tree[incumbent].paths[x][y]);
   }
   }
   glLineWidth(1.0);
   */
}

#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
