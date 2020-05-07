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
#include "Heuristic.h"
#include "BiClique.h"
#include "Timer.h"
#include "MutexProp.h"
#include "ICTSAlgorithm.h"
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
  static unsigned precheck;
  static bool cct;
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
  static bool topTwo; // Select top two conflicting agents in sub-optimal split
  static bool conditional; // Use conditional constraints to promote completeness
  static bool mutexprop; // Use mutex propagation logic
  static std::vector<unsigned> array;
  static std::vector<unsigned> indices;
  static std::vector<float> ivls;
};

unsigned Params::precheck = 0;
unsigned Params::conn = 1;
bool Params::greedyCT = false;
bool Params::cct = false;
bool Params::skip = false;
bool Params::prioritizeConf = false;
bool Params::crossconstraints = false;
bool Params::identicalconstraints = true;
bool Params::boxconstraints = false;
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
bool Params::topTwo = false;
bool Params::conditional = false;
bool Params::mutexprop = false;
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
  Conflict<state>()
      : unit1(9999999), unit2(9999999), prevWpt(0), replanBoth(false){
  }
  Conflict<state>(Conflict<state> const& from)
      : c(std::move(from.c)), c2(std::move(from.c2)), unit1(from.unit1), unit2(from.unit2), prevWpt(from.prevWpt) {
  }
  Conflict<state>& operator=(Conflict<state> const& from) {
    c=std::move(from.c);
    c2=std::move(from.c2);
    unit1 = from.unit1;
    unit2 = from.unit2;
    prevWpt = from.prevWpt;
    return *this;
  }
  mutable std::vector<std::unique_ptr<Constraint<state>>> c; // constraint representing one agent in the meta-state
  mutable std::vector<std::unique_ptr<Constraint<state>>> c2; // second constraint representing one agent in the meta-state
  unsigned unit1;
  unsigned unit2;
  unsigned prevWpt;
  bool replanBoth;
};

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

template<typename state, typename conflicttable>
struct CBSTreeNode {
  CBSTreeNode()
    : path(nullptr), parent(0), satisfiable(true), cat(), cct(), semi(), cardinal(), path2(nullptr){}
  // Copy ctor takes over memory for path member
  CBSTreeNode(CBSTreeNode<state, conflicttable> const& from)
    : wpts(from.wpts), path(from.path.release()), polygon(from.polygon.release()),
    paths(from.paths), polygons(from.polygons), con(from.con), parent(from.parent),
    satisfiable(from.satisfiable), cat(from.cat), cct(from.cct),
    cardinal(from.cardinal), semi(from.semi), path2(from.path2.release())/*, sweep(from.sweep)*/ {
    }
  CBSTreeNode(CBSTreeNode<state, conflicttable> const& from, Conflict<state> const& c, unsigned p, bool s)
    : wpts(from.wpts), path(new std::vector<state>()),
    polygon(new std::vector<Vector2D>()), paths(from.paths), polygons(from.polygons),
    con(c), parent(p), satisfiable(s), cat(from.cat), cct(from.cct),
    cardinal(from.cardinal), semi(from.semi), path2(from.path2.release())/*,sweep(from.sweep)*/ {
      paths[c.unit1] = path.get();
      if (Params::precheck & (PRE_AABB|PRE_HULL)) {
        polygons[c.unit1] = polygon.get();
      }
      if(Params::cct){
        clearcct(c.unit1); // Clear the cct for this unit so that we can re-count collisions
        //}
        if(Params::prioritizeConf){
          clearcardinal(c.unit1);
          clearsemi(c.unit1);
        }
      }
    }
  bool hasOverlap(unsigned a, unsigned b, float radiusA, float radiusB) const {
    if (Params::precheck == 1) {
      return (polygons[a]->at(0).x <= polygons[b]->at(1).x && polygons[a]->at(1).x >= polygons[b]->at(0).x)
        && (polygons[a]->at(0).y <= polygons[b]->at(1).y && polygons[a]->at(1).y >= polygons[b]->at(0).y);
      //&& (polygons[a]->at(0).z <= polygons[b]->at(1).z && polygons[a]->at(1).z >= polygons[b]->at(0).z);
    } else if (Params::precheck == 2) {
      // Detect polygonal overlap
      return Util::sat<Vector2D>(*polygons[a],*polygons[b],radiusA,radiusB);
    }
    return true; // default
  }

  inline void getCardinalPair(unsigned& a1, unsigned& a2)const{
    for(auto const& e:cardinal){
      a1=e.first.a1;
      a2=e.first.a2;
      return;
    }
  }

  inline void getSemiCardinalPair(unsigned& a1, unsigned& a2)const{
    for(auto const& e:semi){
      a1=e.first.a1;
      a2=e.first.a2;
      return;
    }
  }

  inline void getCollisionPair(unsigned& a1, unsigned& a2)const{
    for(auto const& e:cct){
      a1=e.first.a1;
      a2=e.first.a2;
      return;
    }
  }

  inline bool hasCardinal()const{
    return !cardinal.empty();
  }

  inline bool hasSemiCardinal()const{
    return !semi.empty();
  }

  inline bool hasCollisions()const{
    return !cct.empty();
  }

  inline unsigned numCollisions()const{
    unsigned total(0);
    for(auto const& e:cct){
      total+= e.second.size();
    }
    return total;
  }

  inline void mostConflicting(unsigned& a1, unsigned& a2)const{
    unsigned total1(0);
    for(auto const& e:cct){
      if(e.second.size()>total1){
        a1=e.first.a1;
        a2=e.first.a2;
      }
    }
  }

  inline void topTwo(unsigned& a1, unsigned& a2)const{
    std::vector<unsigned> histogram(paths.size());
    unsigned total1(0);
    for(auto const& e:cct){
      ++histogram[e.first.a1];
      ++histogram[e.first.a2];
      if(total1<histogram[e.first.a1]){
        if(cct.find({a1,a2})!=cct.end()){
          a2=a1;
        }else{
          a2=e.first.a2;
        }
        a1=e.first.a1;
        total1=histogram[e.first.a1];
      }
      if(total1<histogram[e.first.a2]){
        if(cct.find({a1,a2})!=cct.end()){
          a2=a1;
        }else{
          a2=e.first.a1;
        }
        a1=e.first.a2;
        total1=histogram[e.first.a2];
      }
    }
  }

  inline void setcardinal(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2)const{
    cardinal[{a1,a2}].emplace(ix1,ix2);
  }

  inline void unsetcardinal(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2)const{
    cardinal[{a1,a2}].erase({ix1,ix2});
  }

  inline void clearcardinal(unsigned a1)const{
    for(unsigned a2(0);a2<paths.size();++a2){
      auto ix(cardinal.find({a1,a2}));
      if(ix != cardinal.end()){
        cardinal.erase(ix);
      }
    }
  }

  inline void setsemi(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2)const{
    semi[{a1,a2}].emplace(ix1,ix2);
  }

  inline void unsetsemi(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2)const{
    semi[{a1,a2}].erase({ix1,ix2});
  }

  inline void clearsemi(unsigned a1)const{
    for(unsigned a2(0);a2<paths.size();++a2){
      auto ix(semi.find({a1,a2}));
      if(ix != semi.end()){
        semi.erase(ix);
      }
    }
  }

  inline void setcct(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2)const{
    agentpair_t p(a1,a2);
    if(p.a1==a1)
      cct[p].emplace(ix1,ix2);
    else
      cct[p].emplace(ix2,ix1);
  }

  inline void unsetcct(unsigned a1, unsigned a2, unsigned ix1, unsigned ix2)const{
    agentpair_t p(a1,a2);
    if(p.a1==a1)
      cct[p].erase({ix1,ix2});
    else
      cct[p].erase({ix2,ix1});
  }

  inline void clearcct(unsigned a1)const{
    for(unsigned a2(0);a2<paths.size();++a2){
      auto ix(cct.find({a1,a2}));
      if(ix != cct.end()){
        cct.erase(ix);
      }
    }
  }

  std::vector<std::vector<int> > wpts;
  mutable std::unique_ptr<std::vector<state>> path;
  mutable std::unique_ptr<std::vector<Vector2D>> polygon;
  std::vector<std::vector<state>*> paths;
  std::vector<std::vector<Vector2D>*> polygons;
  mutable std::unordered_map<agentpair_t,std::unordered_set<conflict_t,conflict_t>,agentpair_t> cardinal;
  mutable std::unordered_map<agentpair_t,std::unordered_set<conflict_t,conflict_t>,agentpair_t> semi;
  mutable std::unordered_map<agentpair_t,std::unordered_set<conflict_t,conflict_t>,agentpair_t> cct;
  //std::vector<xyztAABB> sweep;
  mutable std::unique_ptr<std::vector<state>> path2;
  Conflict<state> con;
  unsigned int parent;
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
  CBSUnit<state, action, comparison, conflicttable, searchalgo>* GetUnit(unsigned a){
    return (CBSUnit<state, action, comparison, conflicttable, searchalgo> *)this->GetMember(a);
  }
  ConstrainedEnvironment<state,action>* GetEnv(unsigned a){
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
  bool ExpandOneCBSNode();
  void Init();

  std::vector<CBSTreeNode<state, conflicttable> > tree;
  void processSolution(double);
  searchalgo astar;
  Group agents;

private:

  unsigned LoadConstraintsForNode(int location, int agent = -1);
  bool Bypass(int best, std::pair<unsigned, unsigned> const& numConflicts, Conflict<state> const& c1, unsigned otherunit,
      unsigned minTime);
  void Replan(int location, state const& s1, state const& s2, bool a2=false);
  bool IsCardinal(int x, state const&, state const&, int y, state const&, state const&, bool asym=false);
  unsigned HasConflict(std::vector<state> const& a, std::vector<int> const& wa, std::vector<state> const& b,
      std::vector<int> const& wb, int x, int y, std::vector<Conflict<state>> &conflicts,
      std::pair<unsigned, unsigned>& conflict, unsigned& ctype, bool update, bool countall=true);

  void GetBiclique( state const& a1, state const& a2, state const& b1, state const& b2, unsigned x, unsigned y, Conflict<state>& c1, Conflict<state>& c2, bool conditional=false);
  std::pair<unsigned, unsigned> FindHiPriConflictAllPairs(CBSTreeNode<state, conflicttable> const& location,
      std::vector<Conflict<state>> &conflicts, bool update = true);
  std::pair<unsigned, unsigned> FindHiPriConflictOneVsAll( CBSTreeNode<state, conflicttable> const& location,
      unsigned agent,
      std::vector<Conflict<state>> &conflicts, bool update=true);

  unsigned FindFirstConflict(CBSTreeNode<state, conflicttable> const& location, Conflict<state> &c1, Conflict<state> &c2);


  void SetEnvironment(unsigned agent);
  void ClearEnvironmentConstraints(unsigned agent);
  void AddEnvironmentConstraint(Constraint<state>* c, unsigned agent, bool negative);

  double time;

  struct OpenListNode {
    OpenListNode()
        : location(0), cost(0), nc(0), cardinal(false) {
    }
    OpenListNode(uint loc, double c, uint16_t n, bool cardinl=false)
        : location(loc), cost(c), nc(n), cardinal(cardinl) {
    }
    inline bool operator<(OpenListNode const& other)const{
      if (Params::greedyCT){
        return ( cardinal==other.cardinal ? nc == other.nc ? cost<other.cost : nc>other.nc : cardinal>other.cardinal);
      }else{
        return ( cost==other.cost ? (nc > other.nc) : cost > other.cost);
      }
    }

    uint location;
    double cost;
    uint16_t nc;
    bool cardinal;
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
  unsigned bestNode;

  // Algorithm parameters
  bool verify;
  bool nobypass;
  bool ECBSheuristic; // For ECBS
  unsigned killex; // Kill after this many expansions
  bool keeprunning; // Whether to keep running after finding the answer (e.g. for ui)
  int seed;
  static Timer* timer;
  int animate; // Add pauses for animation
  bool quiet = false;
  bool disappearAtGoal = true;
  Solution<state> basepaths;
  Solution<Vector2D> basepolygons;
  static unsigned TOTAL_EXPANSIONS;
  static unsigned collchecks;
  static float collisionTime;
  static float planTime;
  static float replanTime;
  static float bypassplanTime;
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
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
float CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::bypassplanTime=0;


/** AIR CBS UNIT DEFINITIONS */

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSUnit<state, action, comparison, conflicttable, searchalgo>::SetPath(std::vector<state> &p) {
  myPath = p;
  std::reverse(myPath.begin(), myPath.end());
}

template<typename state, typename action, typename comparison, typename conflicttable, class searchalgo>
void CBSUnit<state, action, comparison, conflicttable, searchalgo>::OpenGLDraw(
    const ConstrainedEnvironment<state, action> *ae,
    const SimulationInfo<state, action, ConstrainedEnvironment<state, action>> *si) const {
  GLfloat r, g, b;
  this->GetColor(r, g, b);
  ae->SetColor(r, g, b);

  if (myPath.size() > 1) {
    // Interpolate between the two given the timestep
    state start_t = myPath[myPath.size() - 1];
    state stop_t = myPath[myPath.size() - 2];

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
    : time(0), bestNode(0), planFinished(false), verify(false), nobypass(false), ECBSheuristic(false), killex(INT_MAX), keeprunning(
        false), animate(0), seed(1234567), quiet(true), agents(g){
  //std::cout << "THRESHOLD " << threshold << "\n";

  tree.resize(1);
  tree[0].parent = 0;

  // Sort the environment container by the number of conflicts
  astar.SetVerbose(Params::astarverbose);
  basepaths.resize(MAXNAGENTS);
  basepolygons.resize(MAXNAGENTS);
}

/** Expand a single CBS node */
// Return true while processing
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::ExpandOneCBSNode() {
  //for(auto const& o:openList.getContainer()){
    //std::cout << "tree["<<o.location <<"]="<<o.cost<<","<<o.nc<<"\n";
  //}
  openList.pop();
  if(!quiet)
    std::cout << "Expanding " << bestNode << "\n";
  // There's no reason to expand if the plan is finished.
  if (planFinished)
    return false;

  std::vector<Conflict<state>> conflicts;
  unsigned long last = tree.size();

  std::pair<unsigned,unsigned> numConflicts;
  if((bestNode!=0) && Params::cct){
    numConflicts=FindHiPriConflictOneVsAll(tree[bestNode], tree[bestNode].con.unit1, conflicts);//, !Params::subopt);
    if(tree[bestNode].path2){
      auto numConflicts2(FindHiPriConflictOneVsAll(tree[bestNode], tree[bestNode].con.unit2, conflicts));//, !Params::subopt));
      if(numConflicts2.second>numConflicts.second)numConflicts.second=numConflicts2.second;
    }
    numConflicts.first=tree[bestNode].numCollisions();
  }else{
    numConflicts=FindHiPriConflictAllPairs(tree[bestNode], conflicts);//, !Params::subopt);
  }
  // If no conflicts are found in this node, then the path is done
  if (numConflicts.first == 0) {
    processSolution(CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());
    return false;
  } else {
    //For an unbounded sub-optimal approach, try adding all possible constraints for agent i and j...
    if(false){ // turned off for now
      unsigned a1=0,a2=1;
      if(Params::topTwo){
        tree[bestNode].topTwo(a1,a2);
      }else{
        tree[bestNode].mostConflicting(a1,a2);
      }
      assert(a1!=a2);
      if(a2<a1){ // Swap so a1 is smaller
        unsigned tmp(a1);
        a1=a2;
        a2=tmp;
      }
      conflicts.resize(2);
      auto c(tree[bestNode].cct.find({a1,a2}));
      assert(c!=tree[bestNode].cct.end());
      for(auto const& x:c->second){
        if(conflicts[0].c.size()){
          conflicts[0].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a2]->at(x.ix2),tree[bestNode].paths[a2]->at(x.ix2+1),radii[a1]));
          conflicts[1].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a1]->at(x.ix1),tree[bestNode].paths[a1]->at(x.ix1+1),radii[a2]));
          conflicts[0].unit1=a1;
          conflicts[0].unit2=a2;
          conflicts[1].unit1=a2;
          conflicts[1].unit2=a1;
        }else{
          // Add an optimal and complete constraint for the core constraint
          conflicts[0].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a2]->at(x.ix2),tree[bestNode].paths[a2]->at(x.ix2+1),radii[a2]));
          conflicts[1].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a1]->at(x.ix1),tree[bestNode].paths[a1]->at(x.ix1+1),radii[a1]));
          GetBiclique(tree[bestNode].paths[a1]->at(x.ix1),tree[bestNode].paths[a1]->at(x.ix1+1),tree[bestNode].paths[a2]->at(x.ix2),tree[bestNode].paths[a2]->at(x.ix2+1),a1,a2,conflicts[0],conflicts[1]);
        }
      }
      for(unsigned a(0);a<tree[bestNode].paths.size();++a){
        if(a!=a1 && a!=a2){
          auto ix(tree[bestNode].cct.find({a1,a}));
          if(ix != tree[bestNode].cct.end()){
            conflicts[0].unit1=a1;
            conflicts[0].unit2=a;
            if(a1==ix->first.a1){
              for(auto const& x:ix->second){
                if(Params::conditional && a!=a2){
                  conflicts[0].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a]->at(x.ix2),tree[bestNode].paths[a]->at(x.ix2+1),radii[a]));
                }else if(Params::crossconstraints){
                  conflicts[0].c.emplace_back((Constraint<state>*) new Collision<state>(tree[bestNode].paths[a]->at(x.ix2),tree[bestNode].paths[a]->at(x.ix2+1),radii[a]));
                }else{
                  conflicts[0].c.emplace_back((Constraint<state>*) new Identical<state>(tree[bestNode].paths[a1]->at(x.ix1),tree[bestNode].paths[a1]->at(x.ix1+1)));
                }
                //std::cout << "Adding " << 
              }
            }else{
              for(auto const& x:ix->second){
                if(Params::conditional && a!=a2){
                  conflicts[0].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a]->at(x.ix1),tree[bestNode].paths[a]->at(x.ix1+1),radii[a]));
                }else if(Params::crossconstraints){
                  conflicts[0].c.emplace_back((Constraint<state>*) new Collision<state>(tree[bestNode].paths[a]->at(x.ix1),tree[bestNode].paths[a]->at(x.ix1+1),radii[a]));
                }else{
                  conflicts[0].c.emplace_back((Constraint<state>*) new Identical<state>(tree[bestNode].paths[a1]->at(x.ix2),tree[bestNode].paths[a1]->at(x.ix2+1)));
                }
              }
            }
          }
          ix=tree[bestNode].cct.find({a2,a});
          if(ix != tree[bestNode].cct.end()){
            conflicts[1].unit1=a2;
            conflicts[1].unit2=a;
            if(a2==ix->first.a1){
              for(auto const& x:ix->second){
                if(Params::conditional && a!=a1){
                  conflicts[1].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a]->at(x.ix2),tree[bestNode].paths[a]->at(x.ix2+1),radii[a]));
                }else if(Params::crossconstraints){
                  conflicts[1].c.emplace_back((Constraint<state>*) new Collision<state>(tree[bestNode].paths[a]->at(x.ix2),tree[bestNode].paths[a]->at(x.ix2+1),radii[a]));
                }else{
                  conflicts[1].c.emplace_back((Constraint<state>*) new Identical<state>(tree[bestNode].paths[a2]->at(x.ix1),tree[bestNode].paths[a2]->at(x.ix1+1)));
                }
              }
            }else{
              for(auto const& x:ix->second){
                if(Params::conditional && a!=a1){
                  conflicts[1].c.emplace_back((Constraint<state>*) new Probable<state>(tree[bestNode].paths[a]->at(x.ix1),tree[bestNode].paths[a]->at(x.ix1+1),radii[a]));
                }else if(Params::crossconstraints){
                  conflicts[1].c.emplace_back((Constraint<state>*) new Collision<state>(tree[bestNode].paths[a]->at(x.ix1),tree[bestNode].paths[a]->at(x.ix1+1),radii[a]));
                }else{
                  conflicts[1].c.emplace_back((Constraint<state>*) new Identical<state>(tree[bestNode].paths[a2]->at(x.ix2),tree[bestNode].paths[a2]->at(x.ix2+1)));
                }
              }
            }
          }
        }
      }
    }

    // If the conflict is NON_CARDINAL, try the bypass
    // if semi-cardinal, try bypass on one and create a child from the other
    // if both children are cardinal, create children for both

    // Swap units
    //unsigned tmp(c1.unit1);
    //c1.unit1=c2.unit1;
    //c2.unit1=tmp;
    // Notify the user of the conflict
    if (!quiet){
      std::cout << "TREE " << bestNode << "(" << tree[bestNode].parent << ")\n";
      if(Params::verbose){
        std::cout << " Left child: "<< conflicts[0].unit1 <<"\n";
        for(auto const& c:conflicts[0].c){
          std::cout << "    " << c->start() << "-->" << c->end() << "\n";
        }
        std::cout << " Right child: "<< conflicts[1].unit1 <<"\n";
        for(auto const& c:conflicts[1].c){
          std::cout << "    " << c->start() << "-->" << c->end() << "\n";
        }
      }
    }
    //if(Params::verbose){
    //std::cout << c1.unit1 << ":\n";
    //for(auto const& a:tree[bestNode].paths[c1.unit1]){
    //std::cout << a << "\n";
    //}
    //std::cout << c2.unit1 << ":\n";
    //for(auto const& a:tree[bestNode].paths[c2.unit1]){
    //std::cout << a << "\n";
    //}
    //}
    constraintsz+=conflicts[0].c.size()+conflicts[1].c.size();
    constrainttot++;
    if (animate) {
      for(auto const& c:conflicts[0].c){
        c->OpenGLDraw(GetEnv(0)->GetMap());
      }
      for(auto const& c:conflicts[1].c){
        c->OpenGLDraw(GetEnv(0)->GetMap());
      }
      //usleep(animate * 1000);
    }

    unsigned minTime(0);
    // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
    if(Params::xorconstraints){
      unsigned i(0);
      for(auto const& c:conflicts){
        last = tree.size();
        //tree.resize(last+1);
        tree.emplace_back(tree[bestNode], c, bestNode, true);
        // Pass in the action that should be avoided....
        state s1(Params::extrinsicconstraints?((XOR<state>const*)tree[last].con.c[0].get())->pos_start:tree[last].con.c[0]->start_state);
        state s2(Params::extrinsicconstraints?((XOR<state>const*)tree[last].con.c[0].get())->pos_end:tree[last].con.c[0]->end_state);
        Replan(last,s1,s2);
        ++i;
        unsigned nc1(numConflicts.first);
        double cost = 0;
        for (int y = 0; y < tree[last].paths.size(); y++) {
          if (Params::verbose && y==c.unit1) {
            std::cout << "Agent " << y << ":\n";
            for (auto const& ff : *tree[last].paths[y]) {
              std::cout << ff << "\n";
            }
            std::cout << "cost: " << GetEnv(y)->GetPathLength(*tree[last].paths[y]) << "\n";
          }
          cost += GetEnv(y)->GetPathLength(*tree[last].paths[y]);
        }
        if (Params::verbose) {
          std::cout << "Replanned: " << c.unit1 << " cost: " << cost << " " << nc1 << "\n";
        }
        if(tree[last].con.c2.size()){
          s1=Params::extrinsicconstraints?tree[last].con.c2[0]->start_state:((XOR<state>const*)tree[last].con.c2[0].get())->pos_start;
          s2=Params::extrinsicconstraints?tree[last].con.c2[0]->end_state:((XOR<state>const*)tree[last].con.c2[0].get())->pos_end;
          Replan(last,s1,s2,true);
	  // Clear collision counts for this unit
	  tree[last].clearcct(c.unit2);
	  if(Params::prioritizeConf){
            tree[last].clearcardinal(c.unit2);
            tree[last].clearsemi(c.unit2);
	  }
          unsigned nc1(numConflicts.first);
          double cost = 0;
          for (int y = 0; y < tree[last].paths.size(); y++) {
            if (Params::verbose && y==c.unit2) {
              std::cout << "Agent " << y << ":\n";
              for (auto const& ff : *tree[last].paths[y]) {
                std::cout << ff << "\n";
              }
              std::cout << "cost: " << GetEnv(y)->GetPathLength(*tree[last].paths[y]) << "\n";
            }
            cost += GetEnv(y)->GetPathLength(*tree[last].paths[y]);
          }
          if (Params::verbose) {
            std::cout << "Also replanned: " << c.unit2 << " cost: " << cost << " " << nc1 << "\n";
          }
        }
        openList.emplace(last, cost, nc1, (numConflicts.second & RIGHT_CARDINAL)==RIGHT_CARDINAL);
      }
      if(Params::verbose)std::cout << "New CT NODE: " << bestNode << ">" << last << "\n";
    }else{
      minTime = GetMaxTime(bestNode, conflicts[1].unit1); // Take off a 1-second wait action, otherwise paths will grow over and over.
      if ((numConflicts.second & LEFT_CARDINAL) || !Bypass(bestNode, numConflicts, conflicts[0], conflicts[1].unit1, minTime)) {
        last = tree.size();
        //tree.resize(last+1);
        tree.emplace_back(tree[bestNode], conflicts[0], bestNode, true);
        Replan(last,Params::extrinsicconstraints?conflicts[1].c[0]->start_state:tree[last].con.c[0]->start_state,Params::extrinsicconstraints?conflicts[1].c[0]->end_state:tree[last].con.c[0]->end_state);
        unsigned nc1(numConflicts.first);
        double cost = 0;
        for (int y = 0; y < tree[last].paths.size(); y++) {
          if (Params::verbose && y==tree[last].con.unit1) {
            std::cout << "Agent " << y << ":\n";
            for (auto const& ff : *tree[last].paths[y]) {
              std::cout << ff << "\n";
            }
            std::cout << "cost: " << GetEnv(y)->GetPathLength(*tree[last].paths[y]) << "\n";
          }
          cost += GetEnv(y)->GetPathLength(*tree[last].paths[y]);
        }
        if (Params::verbose) {
          std::cout << "New CT NODE: " << bestNode << ">" << last << " replanned: " << conflicts[0].unit1 << " cost: " << cost << " " << nc1 << "\n";
        }
        openList.emplace(last, cost, nc1, (numConflicts.second & LEFT_CARDINAL)==LEFT_CARDINAL);
      }
      //if(tree[bestNode].con.prevWpt+1==tree[bestNode].wpts[conflicts[1].unit1].size()-1){
      minTime = GetMaxTime(bestNode, conflicts[1].unit1); // Take off a 1-second wait action, otherwise paths will grow over and over.
    //}
      if ((numConflicts.second & RIGHT_CARDINAL) || !Bypass(bestNode, numConflicts, conflicts[1], conflicts[0].unit1, minTime)) {
        last = tree.size();
        //tree.resize(last+1);
        tree.emplace_back(tree[bestNode], conflicts[1], bestNode, true);
        Replan(last,Params::extrinsicconstraints?tree[last-1].con.c[0]->start_state:tree[last].con.c[0]->start_state,Params::extrinsicconstraints?tree[last-1].con.c[0]->end_state:tree[last].con.c[0]->end_state);
        unsigned nc1(numConflicts.first);
        double cost = 0;
        for (int y = 0; y < tree[last].paths.size(); y++) {
          if (Params::verbose && y==tree[last].con.unit1) {
            std::cout << "Agent " << y << ":\n";
            for (auto const& ff : *tree[last].paths[y]) {
              std::cout << ff << "\n";
            }
            std::cout << "cost: " << GetEnv(y)->GetPathLength(*tree[last].paths[y]) << "\n";
          }
          cost += GetEnv(y)->GetPathLength(*tree[last].paths[y]);
        }
        if (Params::verbose) {
          std::cout << "New CT NODE: " << bestNode << ">" << last << " replanned: " << conflicts[1].unit1 << " cost: " << cost << " " << nc1 << "\n";
        }
        openList.emplace(last, cost, nc1, (numConflicts.second & RIGHT_CARDINAL)==RIGHT_CARDINAL);
      }
    }

    // Get the best node from the top of the open list, and remove it from the list
    int count(0);
    do {
      bestNode = openList.top().location;
      if (!tree[bestNode].satisfiable)
        openList.pop();
      if (++count > tree.size())
        assert(!"No solution!?!");
    } while (!tree[bestNode].satisfiable);

    // Set the visible paths for every unit in the node
    if (keeprunning)
      for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++) {
        // Grab the unit
        auto unit(this->GetUnit(x));

        // Prune these paths to the current simulation time
        state current;
        unit->GetLocation(current);
        static std::vector<state> newPath;
        newPath.resize(0);
        newPath.push_back(current); // Add the current simulation node to the new path

        // For everything in the path that's new, add the path back
        for (state xNode : *tree[bestNode].paths[x]) {
          if (current.t < xNode.t) {
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
      if (!quiet)
        std::cout << "Agent " << x << "(" << GetEnv(x)->GetPathLength(*tree[bestNode].paths[x]) << "): " << "\n";
      unsigned wpt(0);
      signed ix(0);
      for (auto &a : *tree[bestNode].paths[x]) {
        //std::cout << a << " " << wpt << " " << unit->GetWaypoint(wpt) << "\n";
        if (ix++ == tree[bestNode].wpts[x][wpt]) {
          if (!quiet)
            std::cout << " *" << a << "\n";
          if (wpt < tree[bestNode].wpts[x].size() - 1)
            wpt++;
        } else {
          if (!quiet)
            std::cout << "  " << a << "\n";
        }
      }
    } else {
      if (!quiet)
        std::cout << "Agent " << x << ": " << "NO Path Found.\n";
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
  if(!quiet){
    fflush(stdout);
    std::cout
    << "elapsed,planTime,replanTime,bypassplanTime,collisionTime,expansions,CATcollchecks,collchecks,collisions,cost,actions,maxCSet,meanCSet\n";
    if (verify && elapsed > 0)
      std::cout << (valid ? "VALID" : "INVALID") << std::endl;
    if (elapsed < 0) {
      //std::cout << seed<<":FAILED\n";
      std::cout << seed << ":" << elapsed * (-1.0) << ",";
    } else {
      std::cout << seed << ":" << elapsed << ",";
    }
    std::cout << planTime << ",";
    std::cout << replanTime << ",";
    std::cout << bypassplanTime << ",";
    std::cout << collisionTime << ",";
    std::cout << TOTAL_EXPANSIONS << ",";
    std::cout << comparison::collchecks << ",";
    std::cout << collchecks << ",";
    std::cout << tree.size() << ",";
    std::cout << cost / state::TIME_RESOLUTION_D << ",";
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
  //ProbIdentical<state>::cond=Probable<state>::cond=total/tree[0].paths.size();
  /*if(Params::precheck==PRE_SAP){
    for(int i(0); i<tree[0].basepaths.size(); ++i){
      addAABBs<state,xyztAABB>(tree[0].basepaths[i],tree[0].sweep,i);
    }
    std::sort(tree[0].sweep.begin(),tree[0].sweep.end(),
        [](xyztAABB const& a, xyztAABB const& b) -> bool {
        return a.lowerBound.x < b.lowerBound.x; // sort by x
        });
  }*/
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
  basepolygons.resize(this->GetNumMembers());

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
  if (Params::precheck) {
    tree[0].polygons.resize(this->GetNumMembers());
    tree[0].polygons.back() = &basepolygons[this->GetNumMembers()-1];
  }
  tree[0].wpts.resize(this->GetNumMembers());
  //agentEnvs.resize(this->GetNumMembers());

  // Recalculate the optimum path for the root of the tree
  //std::cout << "AddUnit "<<(theUnit) << " getting path." << std::endl;
  //std::cout << "Search using " << GetEnv(theUnit)->name() << "\n";
  //agentEnvs[c->getUnitNumber()]=GetEnv(theUnit);
  comparison::CAT = &(tree[0].cat);
  comparison::CAT->set(&tree[0].paths);
  GetFullPath<state, action, comparison, conflicttable, searchalgo, singleHeuristic>(c, astar, *(GetUnit(theUnit)->env),
      *tree[0].paths.back(), tree[0].wpts.back(), 1, theUnit, replanTime, TOTAL_EXPANSIONS);
  assert(tree[0].paths.back()->size());
  if(Params::precheck==PRE_AABB){
    computeAABB(*tree[0].polygons.back(),*tree[0].paths.back());
  }else if(Params::precheck==PRE_HULL){
    Util::convexHull<state>(*tree[0].paths.back(),*tree[0].polygons.back());
  }
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
  bestNode = 0;
  openList.clear();
  openList.emplace(0, 0, 0, false);
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
  static std::vector<Vector2D> poly;
  poly.resize(0);
  if(Params::precheck==PRE_AABB){
    computeAABB(poly,path);
  }else if(Params::precheck==PRE_HULL){
    Util::convexHull<state>(path,poly);
  }
  basepolygons[theUnit]=(poly);

  // Clear the env constraints
  for (unsigned i(0); i < basepolygons.size(); ++i) {
    // Clear the constraints from the environment set
    ClearEnvironmentConstraints(i);
  }

  SetEnvironment(theUnit);

  // Resize the number of paths in the root of the tree
  tree[0].paths.push_back(&basepaths[theUnit]);
  if (Params::precheck) {
    tree[0].polygons.push_back(&basepolygons[theUnit]);
  }
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
  bestNode = 0;
  openList.clear();
  openList.emplace(0, 0, 0, false);
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
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::LoadConstraintsForNode(int location, int agent) {
  // Select the unit from the tree with the new constraint
  agent=(agent < 0 ? tree[location].con.unit1 : agent);
  unsigned numConflicts(0);

  // Reset the constraints in the test-environment
  ClearEnvironmentConstraints(agent);

  // Add all of the constraints in the parents of the current node to the environment
  while (location != 0) {
    if (agent == tree[location].con.unit1) {
      numConflicts++;
      for (auto const& c : tree[location].con.c) {
        AddEnvironmentConstraint(c.get(), agent, true);
      if (!quiet)std::cout << "Adding constraint (in accumulation)["<<location<<"] " << typeid(*c.get()).name() << ": " << c->start_state << "-->"
            << c->end_state << " for agent " << agent << "\n";
      }
    }else if(Params::xorconstraints && agent == tree[location].con.unit2){
      if(tree[location].con.c2.size()){
        AddEnvironmentConstraint(tree[location].con.c2[0].get(), agent, true);

      if (!quiet)
        std::cout << "Adding -constraint (in accumulation)" << tree[location].con.c[0].get()->start_state << "-->"
            << tree[location].con.c2[0].get()->end_state << " for agent " << agent << "\n";
      }else{
        AddEnvironmentConstraint(tree[location].con.c[0].get(), agent, false);
        if (!quiet){
          if(Params::extrinsicconstraints){
            std::cout << "Adding +constraint (in accumulation)"
              << ((XOR<state> const*)tree[location].con.c[0].get())->start_state << "-->"
              << ((XOR<state> const*)tree[location].con.c[0].get())->end_state
              << " for agent " << agent << "\n";
	  }else{
            std::cout << "Adding +constraint (in accumulation)"
              << ((XOR<state> const*)tree[location].con.c[0].get())->pos_start << "-->"
              << ((XOR<state> const*)tree[location].con.c[0].get())->pos_end
              << " for agent " << agent << "\n";
          }
        }
      }
    }
    location = tree[location].parent;
  } // while (location != 0);
  ProbIdentical<state>::prob=Probable<state>::prob=Params::complete?numConflicts-1:0;
  return numConflicts;
}

// Attempts a bypass around the conflict using an alternate optimal path
// Returns whether the bypass was effective
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::Bypass(int best,
    std::pair<unsigned, unsigned> const& numConflicts, Conflict<state> const& c1, unsigned otherunit, unsigned minTime) {
  if (nobypass)
    return false;
  unsigned theUnit(c1.unit1);
  LoadConstraintsForNode(best, c1.unit1);
  for(auto const& c:c1.c)
    AddEnvironmentConstraint(c.get(), c1.unit1, true); // Add this constraint

  //std::cout << "Attempt to find a bypass.\n";

  bool success(false);
  static std::vector<Conflict<state>> confl;
  confl.resize(0);
  std::vector<state>* oldPath(tree[best].paths[theUnit]);
  static std::vector<int> newWpts;
  newWpts.resize(0);
  newWpts.insert(newWpts.begin(),tree[best].wpts[theUnit].begin(),tree[best].wpts[theUnit].end());
  // Re-perform the search with the same constraints (since the start and goal are the same)
  auto c(this->GetUnit(c1.unit1));

  // Never use conflict avoidance tree for bypass
  bool orig(comparison::useCAT);
  comparison::useCAT = false;

  state start(c->GetWaypoint(c1.prevWpt));
  state goal(c->GetWaypoint(c1.prevWpt + 1));
  // Preserve proper start time
  start.t = (*oldPath)[newWpts[c1.prevWpt]].t;
  // Cost of the previous path
  double cost(GetEnv(theUnit)->GetPathLength(*oldPath));
  GetEnv(theUnit)->setGoal(goal);
  static std::vector<state> path;
  path.resize(0);

  Conflict<state> t1, t2; // Temp variables
  // Perform search for the leg
  if (Params::verbose)
    std::cout << "Bypass for unit " << theUnit << " on:\n";
  if (Params::verbose)
    for (auto const& a : *oldPath) {
      std::cout << a << "\n";
    }
  if (Params::verbose)
    std::cout << cost << " cost\n";
  if (Params::verbose)
    std::cout << openList.top().nc << " conflicts\n";
  unsigned pnum(0);
  unsigned nc1(openList.top().nc);
  // Initialize A*, etc.
  Timer tmr;
  tmr.StartTimer();
  SetEnvironment(theUnit);
  astar.GetPath(GetEnv(theUnit), start, goal, path, minTime); // Get the path with the new constraint
  bypassplanTime += tmr.EndTimer();
  if (path.size() == 0) {
    return false;
  }
  // This construction takes over the pointer to c1.c
  CBSTreeNode<state, conflicttable> newNode(tree[best], c1, best, true);
  std::vector<state>* newPath(newNode.path.get());
  newPath->insert(newPath->end(), oldPath->begin(), oldPath->end()); // Initialize with the old path
  MergeLeg<state, action, comparison, conflicttable, searchalgo>(path, *newPath, newWpts, c1.prevWpt, c1.prevWpt + 1,
      minTime);
  if (fleq(GetEnv(theUnit)->GetPathLength(*newPath), cost)) {
    do {
      pnum++;
      if (path.size() == 0)
        continue;
      MergeLeg<state, action, comparison, conflicttable, searchalgo>(path, *newPath, newWpts, c1.prevWpt, c1.prevWpt + 1,
          minTime);
      newNode.paths[theUnit] = newPath;
      newNode.wpts[theUnit] = newWpts;

      if (Params::verbose)
        for (auto const& a : *newPath) {
          std::cout << a << "\n";
        }

      // TODO do full conflict count here
      std::pair<unsigned,unsigned> pconf;
      //if(Params::precheck==PRE_SAP){
        // Update the sweep list while counting collisions at the same time
        //if(bestNode&&Params::cct){
          //pconf=FindHiPriConflictOneVsAllSAP(tree[bestNode], c3, c4, false);
          //pconf.first=tree[bestNode].numCollisions();
        //}else{
          //pconf=FindHiPriConflictAllPairsSAP(tree[bestNode], c3, c4, false);
        //}
      //}else{
        if(bestNode&&Params::cct){
          pconf=FindHiPriConflictOneVsAll(tree[bestNode], tree[bestNode].con.unit1, confl, false);
	  if(tree[bestNode].path2){
            auto numConflicts2(FindHiPriConflictOneVsAll(tree[bestNode], tree[bestNode].con.unit2, confl, false));
	  }
          pconf.first=tree[bestNode].numCollisions();
        }else{
          pconf=FindHiPriConflictAllPairs(tree[bestNode], confl, false);
        }
      //}
      if (Params::verbose)
        std::cout << "Path number " << pnum << "\n";
      if (Params::verbose)
        for (auto const& a : *newPath) {
          std::cout << a << "\n";
        }
      if (Params::verbose)
        std::cout << pconf.first << " conflicts\n";
      if (nc1 > pconf.first) { // Is this bypass helpful?
        nc1 = pconf.first;
        success = true;
      }
      if (pconf.first == 0) {
        if (Params::verbose) {
          std::cout << "BYPASS -- solution\n";
        }
        processSolution(CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());
        break;
      }
    } while (fleq(astar.GetNextPath(GetEnv(theUnit), start, goal, path, minTime), cost));
  }
  TOTAL_EXPANSIONS += astar.GetNodesExpanded();
  if (killex != INT_MAX && TOTAL_EXPANSIONS > killex)
    processSolution(-CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());

  if (!success) {
    // Give back the pointers to c1
    c1.c.clear();
    for(auto& c:newNode.con.c)
      c1.c.emplace_back(c.release());
    return false;
  }
  // Add CT node with the "best" bypass
  unsigned last = tree.size();
  tree.push_back(newNode);
  cost = 0;
  for (int y = 0; y < tree[last].paths.size(); y++) {
    if (Params::verbose) {
      std::cout << "Agent " << y << ":\n";
      for (auto const& ff : *tree[last].paths[y]) {
        std::cout << ff << "\n";
      }
      std::cout << "cost: " << GetEnv(theUnit)->GetPathLength(*tree[last].paths[y]) << "\n";
    }
    cost += GetEnv(theUnit)->GetPathLength(*tree[last].paths[y]);
  }
  if (Params::verbose) {
    std::cout << "New BYPASS NODE: " << last << " replanned: " << theUnit << " cost: " << cost << " " << nc1 << "\n";
  }
  openList.emplace(last, cost, nc1, false);

  comparison::useCAT = orig;

  // Make sure that the current location is satisfiable
  if (newPath->size() == 0 && !(tree[best].paths[theUnit]->front() == tree[best].paths[theUnit]->back())) {
    return false;
  }

  return success;
}

/** Replan a node given a constraint */
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::Replan(int location, state const& s1, state const& s2, bool a2) {
  // Select the unit from the tree with the new constraint
    unsigned theUnit(tree[location].con.unit1);
    unsigned numConflicts(LoadConstraintsForNode(location, theUnit));
      ProbIdentical<state>::cond=Probable<state>::cond=tree[0].paths[theUnit]->size()*tree[0].paths.size()*GetEnv(theUnit)->branchingFactor();
      //if(ProbIdentical<state>::cond<ProbIdentical<state>::prob)
        //ProbIdentical<state>::prob=Probable<state>::prob=Probable<state>::prob-ProbIdentical<state>::cond;
      //else
        //ProbIdentical<state>::prob=Probable<state>::prob=0;
    //std::cout << "cond: " << Probable<state>::cond << " prob: " << Probable<state>::prob << " %: " << float(Probable<state>::prob)/float(Probable<state>::cond) << "\n";

    // We are building path2 - allocate memory for it
    if(a2){
      tree[location].path2.reset(new std::vector<state>());
      tree[location].paths[theUnit] = tree[location].path2.get();
    }


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
    comparison::currentEnv = (ConstrainedEnvironment<state, action>*) GetEnv(theUnit);
    comparison::currentAgent = theUnit;
    comparison::CAT = &(tree[location].cat);
    comparison::CAT->set(&tree[location].paths);

    if (comparison::useCAT) {
      comparison::CAT->remove(*tree[location].paths[theUnit], GetEnv(theUnit), theUnit);
    }

    unsigned minTime(0);
    // If this is the last waypoint, the plan needs to extend so that the agent sits at the final goal
    if (tree[location].con.prevWpt + 1 == tree[location].wpts[theUnit].size() - 1) {
      minTime = GetMaxTime(location, theUnit); // Take off a 1-second wait action, otherwise paths will grow over and over.
    }

    if(!quiet)std::cout << "Replan agent " << theUnit << "\n";
    //if(!quiet)std::cout << "re-planning path from " << start << " to " << goal << " on a path of len:" << thePath.size() << " out to time " << minTime <<"\n";
    ReplanLeg<state,action,comparison,conflicttable,searchalgo,singleHeuristic>(c, astar, *(GetUnit(theUnit)->env), *tree[location].paths[theUnit], tree[location].wpts[theUnit], tree[location].con.prevWpt, tree[location].con.prevWpt+1,minTime,theUnit,replanTime,TOTAL_EXPANSIONS);
    //for(int i(0); i<tree[location].paths.size(); ++i)
    //std::cout << "Replanned agent "<<i<<" path " << tree[location].paths[i]->size() << "\n";

    if(!quiet){
      bool found(false);
      for(unsigned i(1); i<tree[location].paths[theUnit]->size(); ++i) {
        if(tree[location].paths[theUnit]->at(i-1)==s1 && tree[location].paths[theUnit]->at(i)==s2){
          found=true;
          break;
        }
      }
      if(found)std::cout << "Action " << s1 << "-->" << s2 << " was not eliminated!\n";
    }

    if (killex != INT_MAX && TOTAL_EXPANSIONS > killex)
      processSolution(-CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::timer->EndTimer());

    if(tree[location].paths[theUnit]->size() < 1 && Params::complete){
      ProbIdentical<state>::prob=Probable<state>::prob=0;
      ReplanLeg<state,action,comparison,conflicttable,searchalgo,singleHeuristic>(c, astar, *(GetUnit(theUnit)->env), *tree[location].paths[theUnit], tree[location].wpts[theUnit], tree[location].con.prevWpt, tree[location].con.prevWpt+1,minTime,theUnit,replanTime,TOTAL_EXPANSIONS);
    }
    //DoHAStar(start, goal, thePath);
    //TOTAL_EXPANSIONS += astar.GetNodesExpanded();
    //std::cout << "Replan agent: " << location << " expansions: " << astar.GetNodesExpanded() << "\n";

    // Make sure that the current location is satisfiable
    if (tree[location].paths[theUnit]->size() < 1) {
      tree[location].satisfiable = false;
    }else{
      if(Params::precheck==PRE_AABB){
        computeAABB(*tree[location].polygons[theUnit],*tree[location].paths[theUnit]);
      }else if(Params::precheck==PRE_HULL){
        Util::convexHull<state>(*tree[location].paths[theUnit],*tree[location].polygons[theUnit]);
      }
    }

    // Add the path back to the tree (new constraint included)
    //tree[location].paths[theUnit].resize(0);
    if (comparison::useCAT)
      comparison::CAT->insert(*tree[location].paths[theUnit], GetEnv(theUnit), theUnit);

    /*for(int i(0); i<thePath.size(); ++i) {
      tree[location].paths[theUnit].push_back(thePath[i]);
      }*/
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
bool CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::IsCardinal(int x, state const& ax1, state const& ax2, int y, state const& bx1, state const& bx2, bool asym){
  CBSTreeNode<state, conflicttable>& location=tree[bestNode];
  std::vector<std::unique_ptr<Constraint<state>>> constraints;
  const unsigned diam((radii[x]+radii[y])*state::TIME_RESOLUTION_D);
  if(Params::vc){
    if(ax2.sameLoc(bx2) && abs(ax2.t-bx2.t)<diam){ // Vertex collision
      constraints.emplace_back((Constraint<state>*) new EndVertex<state>(ax2,radii[x]));
    }else if(ax1.sameLoc(bx1) && abs(ax1.t-bx1.t)<diam){
      constraints.emplace_back((Constraint<state>*) new StartVertex<state>(ax1,radii[x]));
    }
  }
  if(!constraints.size()){
    if (Params::extrinsicconstraints) {
      state b2(bx2);
      if(bx1.sameLoc(bx2)){
        b2.t=ax2.t;//+GetEnv(x)->WaitTime();
      }
      if(Params::boxconstraints){
        constraints.emplace_back((Box<state>*) new Box<state>(bx1,b2));
      }else if(Params::crossconstraints){
        if(asym){
          constraints.emplace_back((Constraint<state>*) new Identical<state>(ax1,ax2));
        }else{
          constraints.emplace_back((Constraint<state>*) new Collision<state>(bx1,b2,radii[x]));
        }
      }else if(Params::overlapconstraints){
        constraints.emplace_back((Overlap<state>*) new Overlap<state>(bx1,b2,radii[x]));
      }else if(Params::pyramidconstraints){
        constraints.emplace_back((Constraint<state>*) new Pyramid<state>(bx1,b2,ax1.t,radii[x]));
      }
    } else {
      if(Params::identicalconstraints){
        constraints.emplace_back((Constraint<state>*) new Identical<state>(ax1,ax2));
      }else if(Params::timerangeconstraints){
        state a1(ax1);
        state a2(ax2);
        state b1(bx1);
        state b2(bx2);
        auto intvl(getForbiddenInterval(a1,a2,a1.t,a2.t,radii[x],b1,b2,b1.t,b2.t,radii[y]));
        a2.t-=intvl.second*state::TIME_RESOLUTION_U-a1.t;
        a1.t-=intvl.first*state::TIME_RESOLUTION_U-a1.t;
        //b2.t-=intvl.second*state::TIME_RESOLUTION_U-b1.t;
        //b1.t-=intvl.first*state::TIME_RESOLUTION_U-b1.t;
        //if(a1.t<0)a1.t=TOLERANCE;
        //if(b1.t<0)b1.t=TOLERANCE;
        assert(a1.t<a2.t);
        //assert(b1.t<b2.t);
        constraints.emplace_back((Constraint<state>*) new TimeRange<state>(a1,a2));
      }else if(Params::mutualconstraints){
        // Mutually conflicting sets.
        state as[64];
        state bs[64];
        //LoadConstraintsForNode(bestNode,x);
        unsigned na(GetEnv(x)->GetSuccessors(ax1,as));
        //LoadConstraintsForNode(bestNode,y);
        unsigned nb(GetEnv(y)->GetSuccessors(bx1,bs));
        if(na==0){as[na++]=ax2;}
        if(nb==0){bs[nb++]=bx2;}

        // Determine the mutually conflicting set...
        static std::vector<std::vector<unsigned>> fwd;
        fwd.resize(0);
        fwd.resize(1,std::vector<unsigned>(1)); // The 0th element is the collsion between a1,b1.
        fwd.reserve(na);
        static std::vector<std::vector<unsigned>> rwd;
        rwd.resize(0);
        rwd.resize(1,std::vector<unsigned>(1));
        rwd.reserve(nb);
        unsigned amap[64];
        unsigned bmap[64];
        static std::vector<unsigned> armap;
        armap.resize(1,1);
        armap.reserve(na);
        static std::vector<unsigned> brmap;
        brmap.resize(1,1);
        brmap.reserve(nb);
        static std::pair<unsigned,unsigned> conf(0,0);
        for(unsigned a(0); a<na; ++a){
          //std::cout << ax1 << "<->" << as[a] << " " << bx1 << "<->" << bx2 << " => "; 
          if(collisionCheck3D(ax1, as[a], bx1, bx2, radii[x],radii[y])){
          //if(collisionCheck3DAPriori(ax1, as[a], bx1, bx2, radii[x],radii[y]))
            //std::cout << "CRASH\n";
            if(ax2==as[a]){
              amap[a]=0;
              armap[0]=a;
            }else{
              amap[a]=fwd.size();
              armap.push_back(a);
              fwd.push_back(std::vector<unsigned>(1));
              rwd[0].push_back(amap[a]);
            }
          }
          //else{std::cout << "NO CRASH\n";}
        }
        for(unsigned b(0); b<nb; ++b){
          //std::cout << ax1 << "<->" << ax2 << " " << bx1 << "<->" << bs[b] << " => "; 
          if(collisionCheck3D(ax1, ax2, bx1, bs[b], radii[x],radii[y])){
          //if(collisionCheck3DAPriori(ax1, ax2, bx1, bs[b], radii[x],radii[y]))
            //std::cout << "CRASH\n";
            if(bx2==bs[b]){
              bmap[b]=0;
              brmap[0]=b;
            }else{
              bmap[b]=rwd.size();
              brmap.push_back(b);
              rwd.push_back(std::vector<unsigned>(1));
              fwd[0].push_back(bmap[b]);
            }
          }
          //else{std::cout << "NO CRASH\n";}
        }
        for(unsigned i(1); i<armap.size(); ++i){
          unsigned a(armap[i]);
          for(unsigned j(1); j<brmap.size(); ++j){
            unsigned b(brmap[j]);
            //std::cout << ax1 << "<->" << as[a] << " " << bx1 << "<->" << bs[b] << " => "; 
            if(collisionCheck3D(ax1, as[a], bx1, bs[b], radii[x],radii[y])){
            //if(collisionCheck3DAPriori(ax1, as[a], bx1, bs[b], radii[x],radii[y]))
              //std::cout << "CRASH: ["<<amap[a]<<"]="<<bmap[b] <<"\n";
              fwd[amap[a]].push_back(bmap[b]);
              rwd[bmap[b]].push_back(amap[a]);
            }
            //else{std::cout << "NO CRASH\n";}
          }
        }
        //std::cout << "\n";
        static std::vector<unsigned> left;
        left.resize(0);
        left.reserve(fwd.size());
        static std::vector<unsigned> right;
        right.resize(0);
        right.reserve(rwd.size());
        if(fwd.size()<=rwd.size()){
          BiClique::findBiClique(fwd,rwd,conf,left,right);
        }else{
          BiClique::findBiClique(rwd,fwd,{conf.second,conf.first},right,left);
        }
        for(auto const& m:left){
          constraints.emplace_back((Constraint<state>*) new Identical<state>(ax1,as[armap[m]]));
        }
      }
    }
  }
  LoadConstraintsForNode(bestNode,x);
  for(auto const& constraint:constraints){
    GetEnv(x)->AddConstraint(constraint.get());
  }
  astar.SetHeuristic(GetHeuristic(x));

  double origcost(GetEnv(x)->GetPathLength(*location.paths[x]));

  // Select the unit from the group
  auto c(this->GetUnit(x));

/*
  comparison::currentEnv = (ConstrainedEnvironment<state, action>*) GetEnv(x);
  comparison::currentAgent = x;

  if (comparison::useCAT) {
    comparison::CAT->remove(*location.paths[x], GetEnv(x), x);
  }
*/
  static std::vector<state> thePath;
  thePath.resize(0);
  GetFullPath<state,action,comparison,conflicttable,searchalgo,singleHeuristic>(c, astar, *(GetUnit(x)->env), thePath, location.wpts[x], location.paths[y]->back().t, x, replanTime, TOTAL_EXPANSIONS);

  double newcost(GetEnv(x)->GetPathLength(thePath));
  //for(auto const& constraint:constraints){
    //GetEnv(x)->constraints.remove(constraint->start().t,constraint->end().t,constraint.get());
  //}
  return !fequal(origcost,newcost);
}

template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::GetBiclique( state const& a1, state const& a2, state const& b1, state const& b2, unsigned x, unsigned y, Conflict<state>& c1, Conflict<state>& c2, bool conditional){
  static state as[64];
  static state bs[64];
  static bool bc[64];
  memset(&bc,0,sizeof(bool)*64);
  // 0th element is the cardinal pair of actions
  unsigned ai(0);
  as[ai++]=a2;
  unsigned bi(0);
  bs[bi++]=b2;

  //Assume all agents have the same bf
  const unsigned bf(GetEnv(x)->branchingFactor());

  // Determine the mutually conflicting set...
  static std::vector<std::vector<unsigned>> fwd(bf);
  for(auto&f:fwd){
    f.resize(0);
  }
  fwd[0].push_back(0); // The 0th element is the collsion between a1,b1.
  static std::vector<std::vector<unsigned>> rwd(bf);
  for(auto&f:rwd){
    f.resize(0);
  }
  rwd[0].push_back(0); // The 0th element is the collsion between a1,b1.
  unsigned adir(1);
  int i(1);
  unsigned bdir(-1);
  int j(1);
  //Loop through actions of a
  while(i<bf/2){
    // Set current element if it is a valid move
    if(GetEnv(x)->fetch(a1,a2,i*adir,as[ai])){
      //Does it collide with the core action of b?
      //if(collisionCheck3DAPriori(a1, as[ai], b1, b2, radii[x],radii[y]))
      if(collisionCheck3D(a1, as[ai], b1, b2, radii[x],radii[y])){
        fwd[ai].push_back(0);
        rwd[0].push_back(ai);
        //std::cout << "a"<<ai << "collides with core\n";
        // Loop through actions of b
        bi=1;
        j=1;
        while(j<bf/2){
          // Set current element if it is a valid move
          if(GetEnv(y)->fetch(b1,b2,j*bdir,bs[bi])){
            //Does it collide with the core action of a?
            //if(!bc[bi] && collisionCheck3DAPriori(a1, a2, b1, bs[bi], radii[x],radii[y]))
            if(!bc[bi] && collisionCheck3D(a1, a2, b1, bs[bi], radii[x],radii[y])){
              rwd[bi].push_back(0);
              fwd[0].push_back(bi);
            }
            bc[bi]=1;
            if(rwd[bi].size()){
              //std::cout << "b"<<bi << "collides with core\n";
              //if(collisionCheck3DAPriori(a1, as[ai], b1, bs[bi], radii[x],radii[y]))
              if(collisionCheck3D(a1, as[ai], b1, bs[bi], radii[x],radii[y])){
                //std::cout << "a"<<ai << " collides with b"<<bi << "\n";
                rwd[bi].push_back(ai);
                fwd[ai].push_back(bi);
                bi++;
              }else{ // No need to sweep
                break;
              }
            }else{ // No collision with core
              break;
            }
          }
          ++j;
        }
        ai++;
      }else{ // No collision with core
        break;
      }
    }
    ++i;
  }
  // Sweep the other direction
  adir=-1;
  i = j = 1;
  bdir=1;
  //Loop through actions of a
  while(i<bf/2-1){
    // Set current element if it is a valid move
    if(GetEnv(x)->fetch(a1,a2,i*adir,as[ai])){
      //Does it collide with the core action of b?
      //if(collisionCheck3DAPriori(a1, as[ai], b1, b2, radii[x],radii[y]))
      if(collisionCheck3D(a1, as[ai], b1, b2, radii[x],radii[y])){
        fwd[ai].push_back(0);
        rwd[0].push_back(ai);
        //std::cout << "a"<<ai << "collides with core\n";
        // Loop through actions of b
        bi=1;
        j=1;
        while(j<bf/2-1){
          // Set current element if it is a valid move
          if(GetEnv(y)->fetch(b1,b2,j*bdir,bs[bi])){
            //Does it collide with the core action of a?
            //if(!bc[bi] && collisionCheck3DAPriori(a1, a2, b1, bs[bi], radii[x],radii[y]))
            if(!bc[bi] && collisionCheck3D(a1, a2, b1, bs[bi], radii[x],radii[y])){
              rwd[bi].push_back(0);
              fwd[0].push_back(bi);
            }
            bc[bi]=1;
            if(rwd[bi].size()){
              //std::cout << "b"<<bi << "collides with core\n";
              //if(collisionCheck3DAPriori(a1, as[ai], b1, bs[bi], radii[x],radii[y]))
              if(collisionCheck3D(a1, as[ai], b1, bs[bi], radii[x],radii[y])){
                //std::cout << "a"<<ai << " collides with b"<<bi << "\n";
                rwd[bi].push_back(ai);
                fwd[ai].push_back(bi);
                bi++;
              }else{ // No need to sweep
                break;
              }
            }else{ // No collision with core
              break;
            }
          }
          ++j;
        }
        ai++;
      }else{ // No collision with core
        break;
      }
    }
    ++i;
  }
  //std::cout << "\n";
  static std::vector<unsigned> left;
  left.resize(0);
  left.reserve(rwd[0].size());
  static std::vector<unsigned> right;
  right.resize(0);
  right.reserve(fwd[0].size());
  std::pair<unsigned,unsigned> conf(0,0);
  if(rwd[0].size()<=fwd[0].size()){
    BiClique::findBiClique(fwd,rwd,conf,left,right);
  }else{
    BiClique::findBiClique(rwd,fwd,{conf.second,conf.first},right,left);
  }
  //std::cout << "sz:" << (left.size()+right.size())-std::max(left.size(),right.size())-1 << "\n";
  if(conditional){
    for(auto const& m:left){
      c1.c.emplace_back((Constraint<state>*) new ProbIdentical<state>(a1,as[m]));
    }
    for(auto const& m:right){
      c2.c.emplace_back((Constraint<state>*) new ProbIdentical<state>(b1,bs[m]));
    }
  }else{
    for(auto const& m:left){
      c1.c.emplace_back((Constraint<state>*) new Identical<state>(a1,as[m]));
    }
    for(auto const& m:right){
      c2.c.emplace_back((Constraint<state>*) new Identical<state>(b1,bs[m]));
    }
  }
  c1.unit1=x;
  c1.unit2=y;
  c2.unit1=y;
  c2.unit2=x;
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
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
unsigned CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::HasConflict(
    std::vector<state> const& a, std::vector<int> const& wa,
    std::vector<state> const& b, std::vector<int> const& wb,
    int x, int y, std::vector<Conflict<state>>& conflicts,
    std::pair<unsigned, unsigned>& conflict, unsigned& ctype,
    bool update, bool countall){
  CBSTreeNode<state, conflicttable>& location=tree[bestNode];
  // The conflict parameter contains the conflict count so far (conflict.first)
  // and the type of conflict found so far (conflict.second=BOTH_CARDINAL being the highest)

  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax(a.size());
  int ymax(b.size());
  unsigned orig(conflict.first); // Save off the original conflict count

  //if(Params::verbose)std::cout << "Checking for conflicts between: "<<x << " and "<<y<<" ranging from:" << xmax <<"," << ymax << "\n";

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
    if(Params::skip){
      unsigned onedDist=min(abs(a[i].x-b[j].x),abs(a[i].y-b[j].y)/(2*Params::conn));
      if(onedDist>2){
        i+=onedDist-1;
        j+=onedDist-1;
        if(disappearAtGoal){
          if(i>=xmax || j>=ymax) break;
        }else{
          if(i>=xmax && j>=ymax) break;
          if(i>=xmax){i=xmax-1;}
          if(j>=ymax){j=ymax-1;}
        }
        while(a[i].t>b[min(j+1,ymax-1)].t){
          --i;
        }
        while(b[j].t>a[min(i+1,xmax-1)].t){
          --j;
        }
      }
    }
    // I and J hold the current step in the path we are comparing. We need 
    // to check if the current I and J have a conflict, and if they do, then
    // we have to deal with it.
    // Figure out which indices we're comparing
    int xTime(max(0, min(i, xmax - 2)));
    int yTime(max(0, min(j, ymax - 2)));
    int xNextTime(min(xmax - 1, xTime + 1));
    int yNextTime(min(ymax - 1, yTime + 1));

    // Check if we're looking directly at a waypoint.
    // Increment so that we know we've passed it.
    //std::cout << "if(xTime != pxTime && A->GetWaypoint(pwptA+1)==a[xTime]){++pwptA; pxTime=xTime;}\n";
    //std::cout << " " << xTime << " " << pxTime << " " << pwptA;std::cout << " " << A->GetWaypoint(pwptA+1) << " " << a[xTime] << "==?" << (A->GetWaypoint(pwptA+1)==a[xTime]) <<  "\n";
    //std::cout << "if(yTime != pyTime && B->GetWaypoint(pwptB+1)==b[yTime]){++pwptB; pyTime=yTime;}\n";
    //std::cout << " " << yTime << " " << pyTime << " " << pwptB;std::cout << " " << B->GetWaypoint(pwptB+1) << " " << b[yTime] << "==?" << (B->GetWaypoint(pwptB+1)==b[yTime]) <<  "\n";
    if (xTime != pxTime && pwptA + 2 < wa.size() && xTime == wa[pwptA + 1]) {
      ++pwptA;
      pxTime = xTime;
    }
    if (yTime != pyTime && pwptB + 2 < wb.size() && yTime == wb[pwptB + 1]) {
      ++pwptB;
      pyTime = yTime;
    }

    //if(Params::verbose)std::cout << "Looking at positions " << xTime <<":"<<a[xTime].t << "," << j<<":"<<b[yTime].t << std::endl;

    // Check the point constraints
    //Constraint<state> x_c(a[xTime]);
    //state y_c =b[yTime];

    //state const& aGoal(a[wa[pwptA + 1]]);
    //state const& bGoal(b[wb[pwptB + 1]]);
    collchecks++;
    //if(collisionCheck3DAPriori(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y]) !=
        //collisionCheck3D(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y])){
      //bool ap=collisionCheck3DAPriori(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y]);
      //bool cc=collisionCheck3D(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y]);
      //std::cout << "bad\n";
    //}
    //if(Params::boxconstraints?Box<state>(a[xTime], a[xNextTime]).ConflictsWith(b[yTime], b[yNextTime]):collisionCheck3DAPriori(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y])){
    //if(Params::boxconstraints?Box<state>(a[xTime], a[xNextTime]).ConflictsWith(b[yTime], b[yNextTime]):collisionCheck3D(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y],radii[x],radii[y]))
    if(collisionCheck3D(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x],radii[y])){
      ++conflict.first;
      if(Params::cct){
        location.setcct(x,y,xTime,yTime);
      }
      if(!update && !countall){
        //std::cout << "Exiting conf check loop (!countall && !update) "<<x<<","<<y<<" - \n";
        break;} // We don't care about anything except whether there is at least one conflict
      if(Params::verbose){
        std::cout << conflict.first << " conflicts; #"
          << x << ":" << a[xTime] << "-->" << a[xNextTime]
          << " #" << y << ":"
          << b[yTime] << "-->" << b[yNextTime] << "\n";
        collisionCheck3D(a[xTime], a[xNextTime], b[yTime], b[yNextTime], radii[x], radii[y]);
      }
      if(update) { // Keep updating until we find a both-cardinal conflict
        // Determine conflict type
        auto& conf(conflict.second);
        //unsigned conf(NO_CONFLICT);
        //std::cout << "CONFLICT: " <<x<<","<<y<<"\n";

        if(Params::prioritizeConf){
          // Prepare for re-planning the paths
          if(!quiet) std::cout << "Cardinal check\n";
          if(bestNode)LoadConstraintsForNode(bestNode);
          //comparison::openList = astar.GetOpenList();
          comparison::CAT = nullptr;//&(location.cat);
          //comparison::CAT->set(&location.paths);
          // Left is cardinal?
            state a1(a[xTime]);
            state a2(a[xNextTime]);
            state b1(b[yTime]);
            state b2(b[yNextTime]);
            //a1.t=ctime*state::TIME_RESOLUTION_U;
            //b1.t=ctime*state::TIME_RESOLUTION_U;
            if(a1.sameLoc(a2)){
              a2.t=b2.t+GetEnv(x)->WaitTime();
            }
            if(b1.sameLoc(b2)){
              b2.t=a2.t+GetEnv(x)->WaitTime();
            }
          if(IsCardinal(x,a1,a2,y,b1,b2)){
            conf |= LEFT_CARDINAL;
            if(Params::verbose)std::cout << "LEFT_CARDINAL: " <<x<<","<<y<<"\n";
          }
          // Right is cardinal?
          if(IsCardinal(y,b1,b2,x,a1,a2,Params::asym)){
            conf |= RIGHT_CARDINAL;
            if(Params::verbose)std::cout << "RIGHT_CARDINAL: " <<x<<","<<y<<"\n";
          }
          if(conf>=BOTH_CARDINAL){
            location.setcardinal(x,y,xTime,yTime);
          }else if(conf&BOTH_CARDINAL){
            location.setsemi(x,y,xTime,yTime);
          }
        }else{conf=BOTH_CARDINAL;}
        // Have we increased from non-cardinal to semi-cardinal or both-cardinal?
        if (NO_CONFLICT == ctype || ((ctype <= NON_CARDINAL) && conf) || BOTH_CARDINAL == conf) {
          if(conf==BOTH_CARDINAL && !Params::overload)update=false; // We won't need to add any more constraints after this
          ctype = conf + 1;

          const unsigned diam((radii[x]+radii[y])*state::TIME_RESOLUTION_D);
          bool vc(false);
          if(Params::vc){
            state const& a1(a[xTime]);
            state const& a2(a[xNextTime]);
            state const& b1(b[yTime]);
            state const& b2(b[yNextTime]);
            if(a2.sameLoc(b2) && abs(a2.t-b2.t)<diam){ // Vertex collision
              vc=true;
//std::cout << "vc!\n";
              conflicts.resize(2);
              Conflict<state>& c1=conflicts[0];
              Conflict<state>& c2=conflicts[1];
              if(!Params::overload){
                c1.c.clear();
                c2.c.clear();
              }
              c1.prevWpt = pwptA;
              c2.prevWpt = pwptB;
              if(Params::extrinsicconstraints){
                c1.unit1 = y;
                c1.unit2 = x;
                c2.unit1 = x;
                c2.unit2 = y;
                c1.c.emplace_back((Constraint<state>*) new Vertex<state>(b2));
                c2.c.emplace_back((Constraint<state>*) new Vertex<state>(a2));
              }else{
                c1.unit1 = x;
                c1.unit2 = y;
                c2.unit1 = y;
                c2.unit2 = x;
                c1.c.emplace_back((Constraint<state>*) new Vertex<state>(a2));
                c2.c.emplace_back((Constraint<state>*) new Vertex<state>(b2));
              }
            }else if(a1.sameLoc(b1) && abs(a1.t-b1.t)<diam){
              vc=true;
//std::cout << "vc!\n";
              conflicts.resize(2);
              Conflict<state>& c1=conflicts[0];
              Conflict<state>& c2=conflicts[1];
              if(!Params::overload){
                c1.c.clear();
                c2.c.clear();
              }
              c1.prevWpt = pwptA;
              c2.prevWpt = pwptB;
              if(Params::extrinsicconstraints){
                c1.unit1 = y;
                c1.unit2 = x;
                c2.unit1 = x;
                c2.unit2 = y;
                c1.c.emplace_back((Constraint<state>*) new Vertex<state>(b1));
                c2.c.emplace_back((Constraint<state>*) new Vertex<state>(a1));
              }else{
                c1.unit1 = x;
                c1.unit2 = y;
                c2.unit1 = y;
                c2.unit2 = x;
                c1.c.emplace_back((Constraint<state>*) new Vertex<state>(a1));
                c2.c.emplace_back((Constraint<state>*) new Vertex<state>(b1));
              }
            }
          }
          if(!vc){
            if(Params::xorconstraints){
              conflicts.resize(3);
              Conflict<state>& c1=conflicts[0];
              Conflict<state>& c2=conflicts[1];
              Conflict<state>& c3=conflicts[2];
              state const& a1(a[xTime]);
              state const& a2(a[xNextTime]);
              state const& b1(b[yTime]);
              state const& b2(b[yNextTime]);
              c1.c.clear();
              c2.c.clear();
              c3.c.clear();
              c1.c2.clear();
              c2.c2.clear();
              c3.c2.clear();
              if(Params::extrinsicconstraints) {
                c1.c.emplace_back((Constraint<state>*) new XORCollision<state>(a1,a2,b1,b2,false));
                c2.c.emplace_back((Constraint<state>*) new XORCollision<state>(b1,b2,a1,a2,false));
                c3.c.emplace_back((Constraint<state>*) new Identical<state>(a1,a2));
                c3.c2.emplace_back((Constraint<state>*) new Identical<state>(b1,b2));
                c1.unit1 = y;
                c1.unit2 = x;
                c2.unit1 = x;
                c2.unit2 = y;
                c3.unit1 = x;
                c3.unit2 = y;
                c1.prevWpt = pwptA;
                c2.prevWpt = pwptB;
                c3.prevWpt = pwptA;
              }else{
                c1.c.emplace_back((Constraint<state>*) new XORIdentical<state>(a1,a2,b1,b2,false));
                c2.c.emplace_back((Constraint<state>*) new XORIdentical<state>(b1,b2,a1,a2,false));
                c3.c.emplace_back((Constraint<state>*) new Identical<state>(a1,a2));
                c3.c2.emplace_back((Constraint<state>*) new Identical<state>(b1,b2));
                c1.unit1 = x;
                c1.unit2 = y;
                c2.unit1 = y;
                c2.unit2 = x;
                c3.unit1 = x;
                c3.unit2 = y;
                c1.prevWpt = pwptA;
                c2.prevWpt = pwptB;
                c3.prevWpt = pwptA;
              }
            }else{
              conflicts.resize(2);
              Conflict<state>& c1=conflicts[0];
              Conflict<state>& c2=conflicts[1];
              if(!Params::overload){
                c1.c.clear();
                c2.c.clear();
              }
              if(Params::extrinsicconstraints) {
                state const& a1(a[xTime]);
                state a2(a[xNextTime]);
                state const& b1(b[yTime]);
                state b2(b[yNextTime]);
                //a1.t=ctime*state::TIME_RESOLUTION_U;
                //b1.t=ctime*state::TIME_RESOLUTION_U;
                /*if(a1.sameLoc(a2)){
                  a2.t=b2.t;//+GetEnv(x)->WaitTime();
                }
                if(b1.sameLoc(b2)){
                  b2.t=a2.t;//+GetEnv(x)->WaitTime();
                }*/
                if(Params::boxconstraints){
                  c1.c.emplace_back((Box<state>*) new Box<state>(a1, a2));
                  c2.c.emplace_back((Box<state>*) new Box<state>(b1, b2));
                }else if(Params::crossconstraints){
                  if(Params::asym && c1.c.size()==0){ // size=0 only when first in series or optimal...
                    // Detect which side will have the largest set of nodes in a 1xn biclique
                    state s;
                    unsigned left(0);
                    unsigned right(0);
                    //Assume all agents have the same bf
                    const unsigned bf(GetEnv(x)->branchingFactor());

                    unsigned dir(1);
                    int i(1);
                    //Loop through actions of s clockwise
                    while(i<bf/2){
                      // Set current element if it is a valid move
                      if(GetEnv(x)->fetch(a1,a2,i*dir,s)){
                        //Does it collide with the core action of b?
                        //if(collisionCheck3DAPriori(a1, s, b1, b2, radii[x], radii[y]))
                        if(collisionCheck3D(a1, s, b1, b2, radii[x], radii[y])){
                          ++left;
                        }else{ // No collision with core
                          break;
                        }
                      }
                      ++i;
                    }
                    // Sweep the other direction
                    dir=-1;
                    i = 1;
                    //Loop through actions of s
                    while(i<bf/2-1){
                      // Set current element if it is a valid move
                      if(GetEnv(x)->fetch(a1,a2,i*dir,s)){
                        //Does it collide with the core action of b?
                        //if(collisionCheck3DAPriori(a1, s, b1, b2, radii[x], radii[y]))
                        if(collisionCheck3D(a1, s, b1, b2, radii[x], radii[y])){
                          ++left;
                        }else{ // No collision with core
                          break;
                        }
                      }
                      ++i;
                    }
                    // Loop through actions of b
                    i=1;
                    while(i<bf/2 && right<=left){
                      // Set current element if it is a valid move
                      if(GetEnv(y)->fetch(b1,b2,i*dir,s)){
                        //Does it collide with the core action of a?
                        //if(collisionCheck3DAPriori(a1, a2, b1, s, radii[x], radii[y]))
                        if(collisionCheck3D(a1, a2, b1, s, radii[x], radii[y])){
                          right++;
                        }else{ // No collision with core
                          break;
                        }
                      }
                      ++i;
                    }
                    // Loop through actions of b
                    i=1;
                    dir=1;
                    while(i<bf/2-1 && right<=left){
                      // Set current element if it is a valid move
                      if(GetEnv(y)->fetch(b1,b2,i*dir,s)){
                        //Does it collide with the core action of a?
                        //if(collisionCheck3DAPriori(a1, a2, b1, s, radii[x], radii[y]))
                        if(collisionCheck3D(a1, a2, b1, s, radii[x], radii[y])){
                          right++;
                        }else{ // No collision with core
                          break;
                        }
                      }
                      ++i;
                    }
                    float at(a1.t/state::TIME_RESOLUTION_D);
                    float a2t(a2.t/state::TIME_RESOLUTION_D);
                    float bt(b1.t/state::TIME_RESOLUTION_D);
                    float b2t(b2.t/state::TIME_RESOLUTION_D);
                    float diam(radii[x]+radii[y]);
                    if(left>right){
                      if(Params::mutualtimerange){
                        float tdiff(at-bt);
                        // Compute the correct time range...
                        // Start by setting the start/stop time range to be the forbidden interval for the core actions
                        auto intvl(getForbiddenInterval(a1,a2,at,a2t,radii[x],b1,b2,bt,b2t,radii[y]));
                        if(intvl.first==-std::numeric_limits<float>::infinity()){
                          intvl.first=std::min(tdiff,-diam);
                        }
                        if(intvl.second==std::numeric_limits<float>::infinity()){
                          intvl.second=std::max(tdiff,diam);
                        }
                        float t1(intvl.first);
                        float t2(intvl.second);

                        // Now narrow the interval such that all conflicting actions' intervals overlap
                        dir=1;
                        i=1;
                        //Loop through actions of s clockwise
                        while(i<bf/2){
                          if(GetEnv(x)->fetch(a1,a2,i*dir,s)){
                            //Does the interval overlap with the composite interval?
                            intvl = getForbiddenInterval(a1,s,at,s.t/state::TIME_RESOLUTION_D,radii[x],b1,b2,bt,b2t,radii[y]);
                            // Done if no overlap with current range, or no overlap with start delay
                            if(intvl.second < t1 || intvl.first > t2 || intvl.first>tdiff || intvl.second<tdiff ){
                              break;
                            }
                            t1=std::max(t1,intvl.first);
                            t2=std::min(t2,intvl.second);
                          }
                          ++i;
                        }
                        // Sweep the other direction
                        dir=-1;
                        i = 1;
                        //Loop through actions of s
                        while(i<bf/2-1){
                          // Set current element if it is a valid move
                          if(GetEnv(x)->fetch(a1,a2,i*dir,s)){
                            //Does the interval overlap with the composite interval?
                            intvl = getForbiddenInterval(a1,s,at,s.t/state::TIME_RESOLUTION_D,radii[x],b1,b2,bt,b2t,radii[y]);
                            // Done if no overlap with current range, or no overlap with start delay
                            if(intvl.second < t1 || intvl.first > t2 || intvl.first>tdiff || intvl.second<tdiff ){
                              break;
                            }
                            t1=std::max(t1,intvl.first);
                            t2=std::min(t2,intvl.second);
                          }
                          ++i;
                        }
                        // Set relative to current time
                        state ar1(a1);
                        state ar2(a2);
                        if(t2>t1){
                          ar1.t=std::max(1.0,b1.t+t1*state::TIME_RESOLUTION_D);
                          ar2.t=std::max(1.0,b1.t+t2*state::TIME_RESOLUTION_D);
                        }
                        c1.c.emplace_back((Constraint<state>*) new Collision<state>(a1, a2,radii[x]));
if(Params::verbose)std::cout << "Adding time range constraint for" << a1 << "-->" << a2 << ": " << ar1 << "-->" << ar2 << "\n";
                        c2.c.emplace_back((Constraint<state>*) new TimeRange<state>(ar1,ar2));
                      }else{
                        c1.c.emplace_back((Constraint<state>*) new Collision<state>(a1, a2,radii[x]));
                        c2.c.emplace_back((Constraint<state>*) new Identical<state>(a1, a2));
                      }
                    }else{
                      if(Params::mutualtimerange){
                        float tdiff(bt-at);
                        // Compute the correct time range...
                        // Start by setting the start/stop time range to be the forbidden interval for the core actions
                        auto intvl(getForbiddenInterval(b1,b2,bt,b2t,radii[x],a1,a2,at,a2t,radii[y]));
                        if(intvl.first==-std::numeric_limits<float>::infinity()){
                          intvl.first=std::min(tdiff,-diam);
                        }
                        if(intvl.second==std::numeric_limits<float>::infinity()){
                          intvl.second=std::max(tdiff,diam);
                        }
                        float t1(intvl.first);
                        float t2(intvl.second);

                        // Now narrow the interval such that all conflicting actions' intervals overlap
                        dir=1;
                        i=1;
                        //Loop through actions of s clockwise
                        while(i<bf/2){
                          if(GetEnv(y)->fetch(b1,b2,i*dir,s)){
                            //Does the interval overlap with the composite interval?
                            intvl = getForbiddenInterval(b1,s,bt,s.t/state::TIME_RESOLUTION_D,radii[x],a1,a2,at,a2t,radii[y]);
                            // Done if no overlap with current range, or no overlap with start delay
                            if(intvl.second < t1 || intvl.first > t2 || intvl.first>tdiff || intvl.second<tdiff ){
                              break;
                            }
                            t1=std::max(t1,intvl.first);
                            t2=std::min(t2,intvl.second);
                          }
                          ++i;
                        }
                        // Sweep the other direction
                        dir=-1;
                        i = 1;
                        //Loop through actions of s
                        while(i<bf/2-1){
                          // Set current element if it is a valid move
                          if(GetEnv(y)->fetch(b1,b2,i*dir,s)){
                            //Does the interval overlap with the composite interval?
                            intvl = getForbiddenInterval(b1,s,bt,s.t/state::TIME_RESOLUTION_D,radii[x],a1,a2,at,a2t,radii[y]);
                            // Done if no overlap with current range, or no overlap with start delay
                            if(intvl.second < t1 || intvl.first > t2 || intvl.first>tdiff || intvl.second<tdiff ){
                              break;
                            }
                            t1=std::max(t1,intvl.first);
                            t2=std::min(t2,intvl.second);
                          }
                          ++i;
                        }
                        // Set relative to current time
                        state br1(b1);
                        state br2(b2);
                        if(t2>t1){
                          br1.t=std::max(0.0,a1.t+t1*state::TIME_RESOLUTION_D);
                          br2.t=std::max(0.0,a1.t+t2*state::TIME_RESOLUTION_D);
                        }
if(Params::verbose)std::cout << "Adding time range constraint for" << b1 << "-->" << b2 << ": " << br1 << "-->" << br2 << "\n";
                        c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(br1, br2));
                        c2.c.emplace_back((Constraint<state>*) new Collision<state>(b1, b2,radii[x]));
                      }else{
                        c1.c.emplace_back((Constraint<state>*) new Identical<state>(b1, b2));
                        c2.c.emplace_back((Constraint<state>*) new Collision<state>(b1, b2,radii[y]));
                      }
                    }
                  }else{
                    if(Params::overload && c1.c.size()){ // Not the first one...
                      c1.c.emplace_back((Constraint<state>*) new Probable<state>(a1, a2, radii[x]));
                      c2.c.emplace_back((Constraint<state>*) new Probable<state>(b1, b2, radii[y]));
                    }else{
                      if(Params::complete){
                        //c1.c.emplace_back((Constraint<state>*) new Identical<state>(b1, b2));
                        c2.c.emplace_back((Constraint<state>*) new Identical<state>(a1, a2));
                        c1.c.emplace_back((Constraint<state>*) new Collision<state>(a1, a2,radii[x]));
                        c2.c.emplace_back((Constraint<state>*) new Probable<state>(b1, b2, radii[y]));
                      }else{
                        c1.c.emplace_back((Constraint<state>*) new Collision<state>(a1, a2,radii[x]));
                        c2.c.emplace_back((Constraint<state>*) new Collision<state>(b1, b2,radii[y]));
                      }
                    }
                  }
                }else if(Params::overlapconstraints){
                  c1.c.emplace_back((Overlap<state>*) new Overlap<state>(a1,a2));
                  c2.c.emplace_back((Overlap<state>*) new Overlap<state>(b1,b2));
                }else if(Params::pyramidconstraints){
                  c1.c.emplace_back((Constraint<state>*) new Pyramid<state>(a1, a2, b1.t, radii[x]));
                  c2.c.emplace_back((Constraint<state>*) new Pyramid<state>(b1, b2, a1.t, radii[y]));
                }
                c1.unit1 = y;
                c1.unit2 = x;
                c2.unit1 = x;
                c2.unit2 = y;
                c1.prevWpt = pwptB;
                c2.prevWpt = pwptA;
              } else {
                if(Params::timerangeconstraints){
                  state a1(a[xTime]);
                  state a2(a[xNextTime]);
                  state b1(b[yTime]);
                  state b2(b[yNextTime]);
                  float durA((a2.t-a1.t)/state::TIME_RESOLUTION_D);
                  float durB((b2.t-b1.t)/state::TIME_RESOLUTION_D);
                  //std::cout << "Collision at: " << x << ": " << a1 << "-->" << a2 << ", " << y << ": " << b1 << " " << b2 << "\n";
                  auto intvl(getForbiddenInterval(a1,a2,0,durA,radii[x],b1,b2,0,durB,radii[y]));
                  //auto intvl(getForbiddenInterval(a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,radii[x],b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,radii[y]));
                  //std::cout << "the interval for " << a1 << "-->" << a2 << "," << b1 << "-->" << b2 << ": " << intvl.first << "," << intvl.second << "\n";
                  a2.t=b1.t+intvl.second*state::TIME_RESOLUTION_D;
                  b2.t=a1.t-intvl.first*state::TIME_RESOLUTION_D;
                  auto tmp(b1.t);
                  b1.t=a1.t-intvl.second*state::TIME_RESOLUTION_D;
                  a1.t=tmp+intvl.first*state::TIME_RESOLUTION_D;
assert(a1.t<=a[xTime].t && a2.t >= a[xTime].t);
assert(b1.t<=b[yTime].t && b2.t >= b[yTime].t);
                  c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(a1, a2));
                  c2.c.emplace_back((Constraint<state>*) new TimeRange<state>(b1, b2));
                } else if (Params::mutexprop) {
                  // Set up a bunch of stuff
                  static std::vector<Node<state> *> toDelete;
                  toDelete.clear();
                  static MultiState<state> root;
                  static MultiEdge<state> start;
                  static std::vector<DAG<state>> dags;
                  root.resize(2);
                  start.resize(2);
                  dags.resize(2);
                  static std::vector<EdgeSet<state>> terminals(2);
                  static std::vector<EdgeSet<state>> mutexes(2);
                  for (auto &a : terminals) { a.clear(); }
                  for (auto &a : mutexes) { a.clear(); }
                  auto cost1(GetEnv(x)->GetPathLength(a));
                  auto cost2(GetEnv(y)->GetPathLength(b));
                  uint32_t max1(cost1);
                  uint32_t max2(cost2);
                  auto origCost1(cost1);
                  auto origCost2(cost2);
                  auto minCost1(cost1 - 1);
                  auto minCost2(cost2 - 1);
                  uint32_t best1(INT_MAX);
                  uint32_t best2(INT_MAX);
                  auto u1(this->GetUnit(x));
                  auto u2(this->GetUnit(y));
                  std::vector<state> goals = {u1->GetWaypoint(1), u2->GetWaypoint(1)};
                  std::vector<ConstrainedEnvironment<state, action> *> env(2);
                  env[0] = GetEnv(x);
                  env[1] = GetEnv(y);
                  std::vector<float> radi = {radii[x], radii[y]};

                  bool hasSolution(false);
                  unsigned increment(state::TIME_RESOLUTION_U);
                  unsigned epsilon(1);
                  best1 = best2 = INT_MAX;
                  // Re-initialize DAGs
                  root[0] = root[1] = nullptr;
                  dags[0].clear();
                  dags[1].clear();
                  static std::vector<std::vector<uint32_t>> somecosts(4);
                  somecosts.clear();
                  somecosts.resize(1);
                  bool blocked(false);
                      std::vector<EnvironmentContainer<state, action> *> ec(2);
                      ec[0] = GetUnit(x)->env;
                      ec[1] = GetUnit(y)->env;
                      std::vector<state> starts = {u1->GetWaypoint(0), u2->GetWaypoint(0)};
                      ICTSAlgorithm<state, action> ictsalgo(radi);
                      ictsalgo.SetVerbose(true);
                      location.satisfiable=ictsalgo.GetSolutionCosts(ec, starts, goals, somecosts);
                      if(!location.satisfiable){
                        state const &a1(a[xTime]);
                        state const &a2(a[xNextTime]);
                        c1.c.clear();
                        c2.c.clear();
                        c1.c.emplace_back((Constraint<state> *)new Collision<state>(a1, a2, radii[x]));
                        c2.c.emplace_back((Constraint<state> *)new Identical<state>(a1, a2));
                        ctype=BOTH_CARDINAL;
                        return 9999999;

                      }
                      if (somecosts.size() == 1 && somecosts[0][0] == origCost1 && somecosts[0][1] == origCost2) {
                        // Do a 1x1 Biclique
                        state const &a1(a[xTime]);
                        state const &a2(a[xNextTime]);
                        c1.c.clear();
                        c2.c.clear();
                        c1.c.emplace_back((Constraint<state> *)new Collision<state>(a1, a2, radii[x]));
                        // TODO Change this to a time-range constraint
                        c2.c.emplace_back((Constraint<state> *)new Identical<state>(a1, a2));
                        break;
                      } else {
                      unsigned i(0);
                      for (auto const &costs : somecosts) {
                        // Get MDDs and mutexes again with new costs
                        best1 = best2 = INT_MAX;
                        // Re-initialize DAGs
                        root[0] = root[1] = nullptr;
                        dags[0].clear();
                        dags[1].clear();
                        cost1 = costs[0]; //==origCost1?origCost1:*costs[0]-epsilon;  // Minus epsilon so that we can guarantee a cardinal conflict
                        minCost1 = cost1 - increment;
                        cost2 = costs[1]; //==origCost2?origCost2:*costs[1]-epsilon;
                        minCost2 = cost2 - increment;
                        terminals[0].clear();
                        terminals[1].clear();
                        mutexes[0].clear();
                        mutexes[1].clear();
                        // Create MDDs
                        while (!getMDD(u1->GetWaypoint(0), u1->GetWaypoint(1),
                                       dags[0], root[0], terminals[0], minCost1, cost1,
                                       best1, GetEnv(x), blocked)) {
                          if (blocked)
                            break;
                          minCost1 = cost1;
                          cost1 += increment;
                          root[0] = nullptr;
                          dags[0].clear();
                          terminals[0].clear();
                        }
                        std::cout << "MDD1:\n"
                                  << root[0] << "\n";
                        while (!getMDD(u2->GetWaypoint(0), u2->GetWaypoint(1),
                                       dags[1], root[1], terminals[1], minCost2, cost2,
                                       best2, GetEnv(y), blocked)) {
                          if (blocked)
                            break;
                          minCost2 = cost2;
                          cost2 += increment;
                          root[1] = nullptr;
                          dags[1].clear();
                          terminals[1].clear();
                        }
                        std::cout << "MDD2:\n"
                                  << root[1] << "\n";
                        start[0] = {root[0], root[0]};
                        start[1] = {root[1], root[1]};
                        hasSolution = getMutexes(start, goals, env, toDelete, // actions, edges,
                                                 terminals, mutexes,
                                                 radi, disappearAtGoal);
                        std::cout << "Solution found: " << hasSolution << "\n";
                        for (auto &d : toDelete) {
                          delete d;
                        }
                        // Add min-cost constraints (forces paths to have minimum cost).
                        state dummy(a.back());
                        dummy.t = costs[0];
                        c1.c.emplace_back((Constraint<state> *)new MinCost<state>(dummy));
                        dummy = b.back();
                        dummy.t = costs[1];
                        c2.c.emplace_back((Constraint<state> *)new MinCost<state>(dummy));
                        //assert(hasSolution); // Because we did a check for these costs already, there MUST be a solution at this cost
                        // Set cardinal type
                        // Remove redundancies: Any edge which has all parents mutexed can be removed.
                        for (int i(0); i < mutexes.size(); ++i) {
                          auto edge(mutexes[i].rbegin());
                          while (edge != mutexes[i].rend()) {
                            if (edge->first->parents.size()) {
                              bool all(true);
                              // Check that all parents of this edge are in the mutexed list
                              for (auto const &p : edge->first->parents) {
                                bool found(false);
                                for (auto k : mutexes[i]) {
                                  if (*edge == k)
                                    continue;
                                  if (k.first == p && k.second == edge->first) {
                                    found = true;
                                    break;
                                  }
                                }
                                if (!found) {
                                  all = false;
                                  break;
                                }
                              }
                              if (all) {
                                // convert to regular iterator
                                edge = Util::make_reverse_iter(mutexes[i].erase((++edge).base())--);
                              } else {
                                ++edge;
                              }
                            } else {
                              ++edge;
                            }
                          }
                        }

                          if(conflicts.size()<=i){conflicts.resize(i+1);}
                          max1 = std::max(max1,somecosts[i][0]);
                          max2 = std::max(max2,somecosts[i][1]);
                          conflicts[i].replanBoth=true;
                          for (auto const &l : mutexes[0]) {
                            conflicts[i].c.emplace_back((Constraint<state> *)new Identical<state>(l.first->n, l.second->n));
                            std::cout << "Add constraint to env 0: " << l.first->n << "-->" << l.second->n << "\n";
                            //env[0]->AddConstraint(constraints[0].back().get());
                          }
                          for (auto const &l : mutexes[1]) {
                            conflicts[i].c.emplace_back((Constraint<state> *)new Identical<state>(l.first->n, l.second->n));
                            std::cout << "Add constraint to env 1: " << l.first->n << "-->" << l.second->n << "\n";
                            //env[1]->AddConstraint(constraints[1].back().get());
                          }
                        ++i;
                      }
                      conf = NON_CARDINAL;
                      if (max1 > origCost1) {
                        conf |= LEFT_CARDINAL;
                      }
                      if (max2 > origCost2) {
                        conf |= RIGHT_CARDINAL;
                      }
                      }
                  /*} else {
                    // Too many constraints have blocked us from finding the goal
                    conflicts.resize(2);
                    Conflict<state> &c1 = conflicts[0];
                    Conflict<state> &c2 = conflicts[1];
                    state const &a1(a[xTime]);
                    state const &a2(a[xNextTime]);
                    c1.c.emplace_back((Constraint<state> *)new Collision<state>(a1, a2, radii[x]));
                    // TODO Change this to a time-range constraint
                    c2.c.emplace_back((Constraint<state> *)new Identical<state>(a1, a2));
                  }*/
                } else if (Params::mutualconstraints) {
                  //GetBiclique(a[xTime],a[xNextTime],b[yTime],b[yNextTime],x,y,c1,c2);
                  state const& a1(a[xTime]);
                  state const& a2(a[xNextTime]);
                  state const& b1(b[yTime]);
                  state const& b2(b[yNextTime]);
                  state c;
                  static state as[64];
                  static state bs[64];
                  static bool bc[64];
                  memset(&bc,0,sizeof(bool)*64);
                  // 0th element is the cardinal pair of actions
                  unsigned ai(0);
                  as[ai++]=a2;
                  unsigned bi(0);
                  bs[bi++]=b2;

                  //Assume all agents ahve the same bf
                  const unsigned bf(GetEnv(x)->branchingFactor());

                  // Determine the mutually conflicting set...
                  static std::vector<std::vector<unsigned>> fwd(bf);
                  for(auto&f:fwd){
                    f.resize(0);
                  }
                  fwd[0].push_back(0); // The 0th element is the collsion between a1,b1.
                  static std::vector<std::vector<unsigned>> rwd(bf);
                  for(auto&f:rwd){
                    f.resize(0);
                  }
                  rwd[0].push_back(0); // The 0th element is the collsion between a1,b1.
                  unsigned adir(1);
                  int i(1);
                  unsigned bdir(-1);
                  int j(1);
                  //Loop through actions of a
                  while(i<bf/2){
                    // Set current element if it is a valid move
                    if(GetEnv(x)->fetch(a1,a2,i*adir,as[ai])){
                      //Does it collide with the core action of b?
                      //if(collisionCheck3DAPriori(a1, as[ai], b1, b2, radii[x]))
                      if(collisionCheck3D(a1, as[ai], b1, b2, radii[x], radii[y])){
                        fwd[ai].push_back(0);
                        rwd[0].push_back(ai);
                        //std::cout << "a"<<ai << "collides with core\n";
                        // Loop through actions of b
                        bi=1;
                        j=1;
                        while(j<bf/2){
                          // Set current element if it is a valid move
                          if(GetEnv(y)->fetch(b1,b2,j*bdir,bs[bi])){
                            //Does it collide with the core action of a?
                            //if(!bc[bi] && collisionCheck3DAPriori(a1, a2, b1, bs[bi], radii[x], radii[y]))
                            if(!bc[bi] && collisionCheck3D(a1, a2, b1, bs[bi], radii[x], radii[y])){
                              rwd[bi].push_back(0);
                              fwd[0].push_back(bi);
                            }
                            bc[bi]=1;
                            if(rwd[bi].size()){
                              //std::cout << "b"<<bi << "collides with core\n";
                              //if(collisionCheck3DAPriori(a1, as[ai], b1, bs[bi], radii[x], radii[y]))
                              if(collisionCheck3D(a1, as[ai], b1, bs[bi], radii[x], radii[y])){
                                //std::cout << "a"<<ai << " collides with b"<<bi << "\n";
                                rwd[bi].push_back(ai);
                                fwd[ai].push_back(bi);
                                bi++;
                              }else{ // No need to sweep
                                break;
                              }
                            }else{ // No collision with core
                              break;
                            }
                          }
                          ++j;
                        }
                        ai++;
                      }else{ // No collision with core
                        break;
                      }
                    }
                    ++i;
                  }
                  // Sweep the other direction
                  adir=-1;
                  i = j = 1;
                  bdir=1;
                  //Loop through actions of a
                  while(i<bf/2-1){
                    // Set current element if it is a valid move
                    if(GetEnv(x)->fetch(a1,a2,i*adir,as[ai])){
                      //Does it collide with the core action of b?
                      //if(collisionCheck3DAPriori(a1, as[ai], b1, b2, radii[x], radii[y]))
                      if(collisionCheck3D(a1, as[ai], b1, b2, radii[x], radii[y])){
                        fwd[ai].push_back(0);
                        rwd[0].push_back(ai);
                        //std::cout << "a"<<ai << "collides with core\n";
                        // Loop through actions of b
                        bi=1;
                        j=1;
                        while(j<bf/2-1){
                          // Set current element if it is a valid move
                          if(GetEnv(y)->fetch(b1,b2,j*bdir,bs[bi])){
                            //Does it collide with the core action of a?
                            //if(!bc[bi] && collisionCheck3DAPriori(a1, a2, b1, bs[bi], radii[x], radii[y]))
                            if(!bc[bi] && collisionCheck3D(a1, a2, b1, bs[bi], radii[x], radii[y])){
                              rwd[bi].push_back(0);
                              fwd[0].push_back(bi);
                            }
                            bc[bi]=1;
                            if(rwd[bi].size()){
                              //std::cout << "b"<<bi << "collides with core\n";
                              //if(collisionCheck3DAPriori(a1, as[ai], b1, bs[bi], radii[x], radii[y]))
                              if(collisionCheck3D(a1, as[ai], b1, bs[bi], radii[x], radii[y])){
                                //std::cout << "a"<<ai << " collides with b"<<bi << "\n";
                                rwd[bi].push_back(ai);
                                fwd[ai].push_back(bi);
                                bi++;
                              }else{ // No need to sweep
                                break;
                              }
                            }else{ // No collision with core
                              break;
                            }
                          }
                          ++j;
                        }
                        ai++;
                      }else{ // No collision with core
                        break;
                      }
                    }
                    ++i;
                  }
                  //std::cout << "\n";
                  static std::vector<unsigned> left;
                  left.resize(0);
                  left.reserve(rwd[0].size());
                  static std::vector<unsigned> right;
                  right.resize(0);
                  right.reserve(fwd[0].size());
                  std::pair<unsigned,unsigned> conf(0,0);
                  if(rwd[0].size()<=fwd[0].size()){
                    BiClique::findBiClique(fwd,rwd,conf,left,right);
                  }else{
                    BiClique::findBiClique(rwd,fwd,{conf.second,conf.first},right,left);
                  }
                  //std::cout << "sz:" << (left.size()+right.size())-std::max(left.size(),right.size())-1 << "\n";
                  for(auto const& m:left){
                    c1.c.emplace_back((Constraint<state>*) new Identical<state>(a1,as[m]));
                  }
                  for(auto const& m:right){
                    c2.c.emplace_back((Constraint<state>*) new Identical<state>(b1,bs[m]));
                  }
                }else if(Params::mutualtimerange){
//std::cout << "mxntime\n";
                  state const& a1(a[xTime]);
                  state const& a2(a[xNextTime]);
                  state const& b1(b[yTime]);
                  state const& b2(b[yNextTime]);
                  const unsigned bf(GetEnv(x)->branchingFactor());
                  //auto moveA(moveNum(a1,a2,0,bf));
                  //auto moveB(moveNum(b1,b2,0,bf));
                  static std::vector<unsigned> left;
                  left.resize(0);
                  static std::vector<unsigned> right;
                  right.resize(0);
                  static std::vector<std::pair<float,float>> livls;
                  livls.resize(0);
                  static std::vector<std::pair<float,float>> rivls;
                  rivls.resize(0);
                  if(Params::apriori){
                    if(Params::extended){
                      signed d(getDist(bf));
                      auto span(d*2+1);
                      getExtendedAreaVertexAnnotatedBiclique(a1,a2,b1,b2,
                                                 a1.t/state::TIME_RESOLUTION_D,
                                                 a2.t/state::TIME_RESOLUTION_D,
                                                 b1.t/state::TIME_RESOLUTION_D,
                                                 b2.t/state::TIME_RESOLUTION_D,
                                                 Params::array, Params::indices,
                                                 Params::ivls, left, right, livls, rivls,
                                                 bf,span*span,radii[x], radii[y]);
                      /*std::vector<unsigned> left1;
                        std::vector<unsigned> right1;
                        std::vector<std::pair<float,float>> livls1;
                        std::vector<std::pair<float,float>> rivls1;
                        getExtendedAreaVertexAnnotatedBiclique(a1,a2,b1,b2,
                        a1.t/state::TIME_RESOLUTION_D,
                        a2.t/state::TIME_RESOLUTION_D,
                        b1.t/state::TIME_RESOLUTION_D,
                        b2.t/state::TIME_RESOLUTION_D,
                        left1, right1, livls1, rivls1,
                        bf,radii[x],radii[y]);
                       */
                      if(left.size()==1){
                        c1.c.emplace_back((Constraint<state>*) new Identical<state>(a1, a2));
                        if(right.size()==1){
                          c2.c.emplace_back((Constraint<state>*) new Identical<state>(b1, b2));
                        }else{
                          c2.c.emplace_back((Constraint<state>*) new Collision<state>(a1, a2,radii[x]));
                        }
                      }else if(right.size()==1){
                          c2.c.emplace_back((Constraint<state>*) new Identical<state>(b1, b2));
                          c1.c.emplace_back((Constraint<state>*) new Collision<state>(b1, b2,radii[y]));
                      }else{

                        bool swap=false, ortho=false, y=false;
                        locationIndex(a1,b1,swap,ortho,y,bf); // Get rotation params from reverse action
                        state dest;
                        bool found(false);
                        unsigned p,q;
                        signed xx,yy;
                        for(unsigned i(0); i<left.size(); ++i){
                          p=left[i]/bf;
                          q=left[i]%bf;
                          xx=p%span;
                          yy=p/span;
                          state src(a1);
                          src.x+=xx-d; // relative to a1
                          if(signed(src.x)<0)continue;
                          src.y+=yy-d; // relative to a1
                          if(signed(src.y)<0)continue;

                          auto move(invertMirroredMove(q,swap,ortho,y,bf));
                          //if(std::find(left1.begin(),left1.end(),move)==left1.end()){
                          //assert(!"Integrity of biclique is bad");
                          //}
                          fetch(src,move,dest,bf);
                          //src.x=a1.x;
                          //src.y=a1.y;
                          src.t=std::max(0.0,b1.t+floor(livls[i].first*state::TIME_RESOLUTION_D));
                          dest.t=std::max(0.0,b1.t+ceil(livls[i].second*state::TIME_RESOLUTION_D));
                          if(src.t>a1.t)src.t=a1.t;
                          if(dest.t<=src.t)dest.t=src.t+1;
                          //if(dest.t==src.t)dest.t++;
                          //assert(src.t<=a1.t && dest.t>=a1.t);
                          //if(moveA==move)found=true;
                          if(src.sameLoc(a1) && dest.sameLoc(a2)) found=true;
                          c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        // In case left was empty...
                        if(left.empty()){
                          std::cout << "BICLIQUE: " << left.size() << " " << right.size() << "\n";
                          assert(false&&"left side of biclique was empty");
                          auto intvl(getForbiddenInterval(a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,
                                                          radii[x],b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,radii[y]));
                          // Done if no overlap with current range, or no overlap with start delay
                          auto src(a1);
                          src.t=std::max(0.0,b1.t+floor(intvl.first*state::TIME_RESOLUTION_D));
                          dest=a2;
                          dest.t=std::max(0.0,b1.t+ceil(intvl.second*state::TIME_RESOLUTION_D));
                          if(src.t>a1.t)src.t=a1.t;
                          if(dest.t<=src.t)dest.t=src.t+1;
                          if(src.sameLoc(a1) && dest.sameLoc(a2)) found=true;
                          c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        assert(found&&"Core action A was not added!");
                        found=false;
                        // Do the same for the other side of biclique...
                        //swap=false; ortho=false; y=false;
                        //locationIndex(b2,b1,swap,ortho,y,bf); // Get rotation params from reverse action
                        for(unsigned i(0); i<right.size(); ++i){
                          p=right[i]/bf;
                          q=right[i]%bf;
                          xx=p%span;
                          yy=p/span;
                          state src(b1);
                          src.x+=xx-d; // relative to b1
                          if(signed(src.x)<0)continue;
                          src.y+=yy-d; // relative to b1
                          if(signed(src.y)<0)continue;
                          //auto move(getMirroredMove(right[i],swap,ortho,y,bf));
                          auto move(invertMirroredMove(q,swap,ortho,y,bf));
                          //if(std::find(right1.begin(),right1.end(),move)==right1.end()){
                          //assert(!"Integrity of biclique is bad");
                          //}
                          fetch(src,move,dest,bf);
                          //src.x=b1.x;
                          //src.y=b1.y;
                          // Yes, a1.t is correct (delays are relative to a1)
                          src.t=std::max(0.0,a1.t+floor(rivls[i].first*state::TIME_RESOLUTION_D));
                          dest.t=std::max(0.0,a1.t+ceil(rivls[i].second*state::TIME_RESOLUTION_D));
                          if(src.t>b1.t)src.t=b1.t;
                          if(dest.t<=src.t)dest.t=src.t+1;
                          //assert(src.t<=b1.t && dest.t>=b1.t);
                          //if(moveB==move)found=true;
                          if(src.sameLoc(b1) && dest.sameLoc(b2)) found=true;
                          c2.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        if(right.empty()){
                          std::cout << "BICLIQUE: " << left.size() << " " << right.size() << "\n";
                          assert(false&&"right side of biclique was empty");
                          auto intvl(getForbiddenInterval(b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,
                                                          radii[x],a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,radii[y]));
                          // Done if no overlap with current range, or no overlap with start delay
                          auto src(b1);
                          src.t=std::max(0.0,a1.t+floor(intvl.first*state::TIME_RESOLUTION_D));
                          dest=b2;
                          dest.t=std::max(0.0,a1.t+ceil(intvl.second*state::TIME_RESOLUTION_D));
                          if(src.t>b1.t)src.t=b1.t;
                          if(dest.t<=src.t)dest.t=src.t+1;
                          if(src.sameLoc(b1) && dest.sameLoc(b2)) found=true;
                          c2.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        assert(found&&"Core action B was not added!");
                      }
                    }else{
                      getVertexAnnotatedBiclique(a1,a2,b1,b2,
                                                 a1.t/state::TIME_RESOLUTION_D,
                                                 a2.t/state::TIME_RESOLUTION_D,
                                                 b1.t/state::TIME_RESOLUTION_D,
                                                 b2.t/state::TIME_RESOLUTION_D,
                                                 Params::array, Params::indices,
                                                 Params::ivls, left, right, livls, rivls,
                                                 bf,radii[x],radii[y]);
                      /*std::vector<unsigned> left1;
                        std::vector<unsigned> right1;
                        std::vector<std::pair<float,float>> livls1;
                        std::vector<std::pair<float,float>> rivls1;
                        getVertexAnnotatedBiclique(a1,a2,b1,b2,
                        a1.t/state::TIME_RESOLUTION_D,
                        a2.t/state::TIME_RESOLUTION_D,
                        b1.t/state::TIME_RESOLUTION_D,
                        b2.t/state::TIME_RESOLUTION_D,
                        left1, right1, livls1, rivls1,
                        bf,radii[x],radii[y]);
                       */
                      if(left.size()==1){
                        c1.c.emplace_back((Constraint<state>*) new Identical<state>(a1, a2));
                        if(right.size()==1){
                          c2.c.emplace_back((Constraint<state>*) new Identical<state>(b1, b2));
                        }else{
                          c2.c.emplace_back((Constraint<state>*) new Collision<state>(a1, a2,radii[x]));
                        }
                      }else if(right.size()==1){
                        c2.c.emplace_back((Constraint<state>*) new Identical<state>(b1, b2));
                        c1.c.emplace_back((Constraint<state>*) new Collision<state>(b1, b2,radii[y]));
                      }else{
                        bool swap=false, ortho=false, y=false;
                        locationIndex(a1,b1,swap,ortho,y,bf); // Get rotation params from reverse action
                        state src(a1);
                        state dest;
                        //bool found(false);
                        for(unsigned i(0); i<left.size(); ++i){
                          auto move(invertMirroredMove(left[i],swap,ortho,y,bf));
                          //if(std::find(left1.begin(),left1.end(),move)==left1.end()){
                          //assert(!"Integrity of biclique is bad");
                          //}
                          fetch(a1,move,dest,bf);
                          //src.x=a1.x;
                          //src.y=a1.y;
                          src.t=std::max(0.0,b1.t+floor(livls[i].first*state::TIME_RESOLUTION_D));
                          dest.t=std::max(0.0,b1.t+ceil(livls[i].second*state::TIME_RESOLUTION_D));
                          if(src.t>a1.t)src.t=a1.t;
                          if(dest.t<=src.t)dest.t=src.t+1;
                          //if(dest.t==src.t)dest.t++;
                          //assert(src.t<=a1.t && dest.t>=a1.t);
                          //if(moveA==move)found=true;
                          c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        // In case left was empty...
                        if(left.empty()){
                          auto intvl(getForbiddenInterval(a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,
                                                          radii[x],b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,radii[y]));
                          // Done if no overlap with current range, or no overlap with start delay
                          src.t=std::max(0.0,b1.t+floor(intvl.first*state::TIME_RESOLUTION_D));
                          dest=a2;
                          dest.t=std::max(0.0,b1.t+ceil(intvl.second*state::TIME_RESOLUTION_D));
                          c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        //assert(found&&"Core action A was not added!");
                        //found=false;
                        // Do the same for the other side of biclique...
                        //swap=false; ortho=false; y=false;
                        //locationIndex(b2,b1,swap,ortho,y,bf); // Get rotation params from reverse action
                        src=b1;
                        for(unsigned i(0); i<right.size(); ++i){
                          //auto move(getMirroredMove(right[i],swap,ortho,y,bf));
                          auto move(invertMirroredMove(right[i],swap,ortho,y,bf));
                          //if(std::find(right1.begin(),right1.end(),move)==right1.end()){
                          //assert(!"Integrity of biclique is bad");
                          //}
                          fetch(b1,move,dest,bf);
                          //src.x=b1.x;
                          //src.y=b1.y;
                          // Yes, a1.t is correct (delays are relative to a1)
                          src.t=std::max(0.0,a1.t+floor(rivls[i].first*state::TIME_RESOLUTION_D));
                          dest.t=std::max(0.0,a1.t+ceil(rivls[i].second*state::TIME_RESOLUTION_D));
                          if(src.t>b1.t)src.t=b1.t;
                          if(dest.t<=src.t)dest.t=src.t+1;
                          //assert(src.t<=b1.t && dest.t>=b1.t);
                          //if(moveB==move)found=true;
                          c2.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        if(right.empty()){
                          auto intvl(getForbiddenInterval(b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,
                                                          radii[x],a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,radii[y]));
                          // Done if no overlap with current range, or no overlap with start delay
                          src.t=std::max(0.0,a1.t+floor(intvl.first*state::TIME_RESOLUTION_D));
                          dest=b2;
                          dest.t=std::max(0.0,a1.t+ceil(intvl.second*state::TIME_RESOLUTION_D));
                          c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                        }
                        //assert(found&&"Core action B was not added!");
                      }
                    }
                  }else{
                    getVertexAnnotatedBiclique(a1,a2,b1,b2,
                                               a1.t/state::TIME_RESOLUTION_D,
                                               a2.t/state::TIME_RESOLUTION_D,
                                               b1.t/state::TIME_RESOLUTION_D,
                                               b2.t/state::TIME_RESOLUTION_D,
                                               left, right, livls, rivls,
                                               bf,radii[x],radii[y]);
                    state src;
                    state dest;
                    for(unsigned i(0); i<left.size(); ++i){
                      fetch(a1,left[i],dest,bf);
                      src.x=a1.x;
                      src.y=a1.y;
                      src.t=std::max(0.0,b1.t+floor(livls[i].first*state::TIME_RESOLUTION_D));
                      dest.t=std::max(0.0,b1.t+ceil(livls[i].second*state::TIME_RESOLUTION_D));
                      if(src.t>a1.t)src.t=a1.t;
                      if(dest.t<=src.t)dest.t=src.t+1;
                      c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                    }
                    if(left.empty()){
                      auto intvl(getForbiddenInterval(a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,
                            radii[x],b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,radii[y]));
                      // Done if no overlap with current range, or no overlap with start delay
                      src.t=std::max(0.0,b1.t+floor(intvl.first*state::TIME_RESOLUTION_D));
                      dest=a2;
                      dest.t=std::max(0.0,b1.t+ceil(intvl.second*state::TIME_RESOLUTION_D));
                      c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                    }
                    for(unsigned i(0); i<right.size(); ++i){
                      fetch(b1,right[i],dest,bf);
                      src.x=b1.x;
                      src.y=b1.y;
                      // Yes, a1.t is correct (delays are relative to a1)
                      src.t=std::max(0.0,a1.t+floor(rivls[i].first*state::TIME_RESOLUTION_D));
                      dest.t=std::max(0.0,a1.t+ceil(rivls[i].second*state::TIME_RESOLUTION_D));
                      if(src.t>a1.t)src.t=a1.t;
                      if(dest.t<=src.t)dest.t=src.t+1;
                      c2.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                    }
                    if(right.empty()){
                      auto intvl(getForbiddenInterval(b1,b2,b1.t/state::TIME_RESOLUTION_D,b2.t/state::TIME_RESOLUTION_D,
                            radii[x],a1,a2,a1.t/state::TIME_RESOLUTION_D,a2.t/state::TIME_RESOLUTION_D,radii[y]));
                      // Done if no overlap with current range, or no overlap with start delay
                      src.t=std::max(0.0,a1.t+floor(intvl.first*state::TIME_RESOLUTION_D));
                      dest=b2;
                      dest.t=std::max(0.0,a1.t+ceil(intvl.second*state::TIME_RESOLUTION_D));
                      c1.c.emplace_back((Constraint<state>*) new TimeRange<state>(src, dest));
                    }
                  }
                }else{
                  c1.c.emplace_back((Constraint<state>*) new Identical<state>(a[xTime], a[xNextTime]));
                  c2.c.emplace_back((Constraint<state>*) new Identical<state>(b[yTime], b[yNextTime]));
                }
                c1.unit1 = x;
                c1.unit2 = y;
                c2.unit1 = y;
                c2.unit2 = x;
                c1.prevWpt = pwptA;
                c2.prevWpt = pwptB;
              }
            }
          }
          if(!countall && conf == BOTH_CARDINAL){
            //std::cout << "Exiting conf check loop (!countall && conf == BOTH_CARDINAL) "<<x<<","<<y<<" - \n";
            break; // don't count any more, we don't care how many conflicts there are in total
          }
        }
      }else{
        //std::cout << "Already found a cardinal conflict - "<<x<<","<<y<<" - no check for cardinality being performed...\n";
      }
      assert(conflicts[0].c.size() && conflicts[1].c.size());
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
  return conflict.first - orig;
}


/*template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
std::pair<unsigned, unsigned> CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::FindHiPriConflictAllPairsSAP(CBSTreeNode<state, conflicttable>& location, Conflict<state> &c1, Conflict<state> &c2, bool update, bool countall){
  std::pair<unsigned, unsigned> conflict(0,0);
  unsigned& ctype(conflict.second);
  std::vector<xyztAABB> path;
  path.reserve(location.paths[location.con.unit1]->size());
  addAABBs(*location.paths[location.con.unit1],path,location.con.unit1);
  unsigned a(1);
  unsigned b(0);
  std::vector<xyztAABB const*> active;
  active.push_back(&*location.sweep.begin());

  bool add(true);
  while(a<location.sweep.size()){
    if(location.sweep[a].agent==path[b].agent){ // Erase the old path point
      location.sweep.erase(location.sweep.begin()+a); // erase and skip
      continue;
    }else if(b<path.size() && path[b].upperBound.x<location.sweep[a].lowerBound.x){
      ++b;// Prepare to insert the path point
      add=true;
      continue;
    }else if(add && b<path.size() && path[b].lowerBound.x<location.sweep[a].lowerBound.x){
      location.sweep.insert(location.sweep.begin()+a,path[b]); // insert the path point now
      add=false;
    }else{
      // Check against everything in the active list
      for(auto c(active.begin()); c!=active.end(); ++c){
        if(
            location.sweep[a].upperBound.t>=(*c)->lowerBound.t &&
            location.sweep[a].lowerBound.t<=(*c)->upperBound.t &&
            location.sweep[a].lowerBound.y<=(*c)->upperBound.y &&
            location.sweep[a].upperBound.y>=(*c)->lowerBound.y){
          // Decide if this is a cardinal conflict
          if(Params::boxconstraints?Box<state>(location.sweep[a].start, location.sweep[a].end).ConflictsWith(location.sweep[b].start,location.sweep[b].end):collisionCheck3D(location.sweep[a].start, location.sweep[a].end,location.sweep[b].start,location.sweep[b].end, radii[x],radii[y])){
            ++conflict.first;
            if(!update && !countall){return conflict;} // We don't care about anything except whether there is at least one conflict
            if (Params::verbose)
              std::cout << conflict.first << " conflicts; #" << location.sweep[a].agent << ":" << location.sweep[a].start << "-->" << location.sweep[a].end << " #" << location.sweep[b].agent << ":"
                << location.sweep[b].start << "-->" << location.sweep[b].end << "\n";
            if (update && (BOTH_CARDINAL != (ctype & BOTH_CARDINAL))) { // Keep updating until we find a both-cardinal conflict
              // Determine conflict type
              // If there are other legal successors with succ.f()=child.f(), this is non-cardinal
              unsigned conf(NO_CONFLICT); // Left is cardinal?

              // Prepare for re-planning the paths
              if(bestNode)LoadConstraintsForNode(bestNode);
              comparison::openList = astar.GetOpenList();
              comparison::CAT = &(location.cat);
              comparison::CAT->set(&location.paths);
              // Left is cardinal?
              if(IsCardinal(location.sweep[a].agent, location.sweep[a].start, location.sweep[a].end, location.sweep[b].agent, location.sweep[b].start, location.sweep[b].end)){
                conf |= LEFT_CARDINAL;
              }
              // Right is cardinal?
              if(IsCardinal(location.sweep[b].agent, location.sweep[b].start, location.sweep[b].end, location.sweep[a].agent, location.sweep[a].start, location.sweep[a].end)){
                conf |= RIGHT_CARDINAL;
              }
              // Have we increased from non-cardinal to semi-cardinal or both-cardinal?
              if (NO_CONFLICT == ctype || ((ctype <= NON_CARDINAL) && conf) || BOTH_CARDINAL == conf) {
                ctype = conf + 1;

                if (Params::crossconstraints) {
                  state a1(location.sweep[a].start);
                  state a2(location.sweep[a].end);
                  state b1(location.sweep[b].start);
                  state b2(location.sweep[b].end);
                  //a1.t=ctime*state::TIME_RESOLUTION_U;
                  //b1.t=ctime*state::TIME_RESOLUTION_U;
                  if(a1.sameLoc(a2)){
                    a2.t=b2.t;//+GetEnv(x)->WaitTime();
                  }
                  if(b1.sameLoc(b2)){
                    b2.t=a2.t;//+GetEnv(x)->WaitTime();
                  }
                  if(Params::boxconstraints){
                    c1.c.reset((Box<state>*) new Box<state>(a1, a2));
                    c2.c.reset((Box<state>*) new Box<state>(b1, b2));
                  }else{
                    c1.c.reset((Constraint<state>*) new Collision<state>(a1, a2));
                    c2.c.reset((Constraint<state>*) new Collision<state>(b1, b2));
                  }
                  c1.unit1 = location.sweep[b].agent;
                  c1.unit2 = location.sweep[a].agent;
                  c2.unit1 = location.sweep[a].agent;
                  c2.unit2 = location.sweep[b].agent;
                  c1.prevWpt = 0;
                  c2.prevWpt = 0;
                } else {
                  c1.c.reset((Constraint<state>*) new Identical<state>(location.sweep[a].start, location.sweep[a].end));
                  c2.c.reset((Constraint<state>*) new Identical<state>(location.sweep[b].start, location.sweep[b].end));
                  c1.unit1 = location.sweep[a].agent;
                  c1.unit2 = location.sweep[b].agent;
                  c2.unit1 = location.sweep[b].agent;
                  c2.unit2 = location.sweep[a].agent;
                  c1.prevWpt = 0;
                  c2.prevWpt = 0;
                }
                if(!countall && conf == BOTH_CARDINAL){
                  return conflict; // don't count any more, we don't care how many conflicts there are in total
                }
              }
            }
          }
        }
        ++c;
      }
      active.push_back(&location.sweep[a]);
      ++a;
    }
  }
  while(a<=location.sweep.size() && b<path.size()){
    location.sweep.push_back(path[b]);
  }
}*/

/*template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
std::pair<unsigned, unsigned> CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::FindHiPriConflictOneVsAllSAP(CBSTreeNode<state, conflicttable>& location, Conflict<state> &c1, Conflict<state> &c2, bool update, bool countall){
  std::pair<unsigned, unsigned> conflict;
  unsigned& ctype(conflict.second);
  std::vector<xyztAABB> path;
  path.reserve(location.paths[location.con.unit1]->size());
  addAABBs(*location.paths[location.con.unit1],path,location.con.unit1);
  unsigned a(0);
  unsigned b(0);
  while(a<location.sweep.size() && b<path.size() &&
        location.sweep[a].lowerBound.x<path[b].lowerBound.x){
    if(location.sweep[a].agent==path[b].agent){
      location.sweep.erase(location.sweep.begin()+a); // erase and skip
    }else{
    ++a;
    }
  }
  bool add(true);
  while(a<location.sweep.size() && b<path.size()){
    if(location.sweep[a].agent==path[b].agent){
      location.sweep.erase(location.sweep.begin()+a); // erase and skip
      continue;
    }else if(path[b].upperBound.x<location.sweep[a].lowerBound.x){
      ++b;
      add=true;
      continue;
    }else if(add && path[b].lowerBound.x<location.sweep[a].lowerBound.x){
      location.sweep.insert(location.sweep.begin()+a,path[b]);
      add=false;
    }else if(
        location.sweep[a].upperBound.t>=path[b].lowerBound.t &&
        location.sweep[a].lowerBound.t<=path[b].upperBound.t &&
        location.sweep[a].lowerBound.y<=path[b].upperBound.y &&
        location.sweep[a].upperBound.y>=path[b].lowerBound.y){
      // Decide if this is a cardinal conflict
      if(Params::boxconstraints?Box<state>(location.sweep[a].start, location.sweep[a].end).ConflictsWith(location.sweep[b].start,location.sweep[b].end):collisionCheck3D(location.sweep[a].start, location.sweep[a].end,location.sweep[b].start,location.sweep[b].end, radii[x],radii[y])){
        ++conflict.first;
        if(!update && !countall){break;} // We don't care about anything except whether there is at least one conflict
        if (Params::verbose)
          std::cout << conflict.first << " conflicts; #" << location.sweep[a].agent << ":" << location.sweep[a].start << "-->" << location.sweep[a].end << " #" << location.sweep[b].agent << ":"
            << location.sweep[b].start << "-->" << location.sweep[b].end << "\n";
        if (update && (BOTH_CARDINAL != (ctype & BOTH_CARDINAL))) { // Keep updating until we find a both-cardinal conflict
          // Determine conflict type
          // If there are other legal successors with succ.f()=child.f(), this is non-cardinal
          unsigned conf(NO_CONFLICT); // Left is cardinal?

          // Prepare for re-planning the paths
          if(bestNode)LoadConstraintsForNode(bestNode);
          comparison::openList = astar.GetOpenList();
          comparison::CAT = &(location.cat);
          comparison::CAT->set(&location.paths);
          // Left is cardinal?
          if(IsCardinal(location.sweep[a].agent, location.sweep[a].start, location.sweep[a].end, location.sweep[b].agent, location.sweep[b].start, location.sweep[b].end)){
            conf |= LEFT_CARDINAL;
          }
          // Right is cardinal?
          if(IsCardinal(location.sweep[b].agent, location.sweep[b].start, location.sweep[b].end, location.sweep[a].agent, location.sweep[a].start, location.sweep[a].end)){
            conf |= RIGHT_CARDINAL;
          }
          // Have we increased from non-cardinal to semi-cardinal or both-cardinal?
          if (NO_CONFLICT == ctype || ((ctype <= NON_CARDINAL) && conf) || BOTH_CARDINAL == conf) {
            ctype = conf + 1;

            if (Params::crossconstraints) {
              state a1(location.sweep[a].start);
              state a2(location.sweep[a].end);
              state b1(location.sweep[b].start);
              state b2(location.sweep[b].end);
              //a1.t=ctime*state::TIME_RESOLUTION_U;
              //b1.t=ctime*state::TIME_RESOLUTION_U;
              if(a1.sameLoc(a2)){
                a2.t=b2.t;//+GetEnv(x)->WaitTime();
              }
              if(b1.sameLoc(b2)){
                b2.t=a2.t;//+GetEnv(x)->WaitTime();
              }
              if(Params::boxconstraints){
                c1.c.reset((Box<state>*) new Box<state>(a1, a2));
                c2.c.reset((Box<state>*) new Box<state>(b1, b2));
              }else{
                c1.c.reset((Constraint<state>*) new Collision<state>(a1, a2));
                c2.c.reset((Constraint<state>*) new Collision<state>(b1, b2));
              }
              c1.unit1 = location.sweep[b].agent;
              c1.unit2 = location.sweep[a].agent;
              c2.unit1 = location.sweep[a].agent;
              c2.unit2 = location.sweep[b].agent;
              c1.prevWpt = 0;
              c2.prevWpt = 0;
            } else {
              c1.c.reset((Constraint<state>*) new Identical<state>(location.sweep[a].start, location.sweep[a].end));
              c2.c.reset((Constraint<state>*) new Identical<state>(location.sweep[b].start, location.sweep[b].end));
              c1.unit1 = location.sweep[a].agent;
              c1.unit2 = location.sweep[b].agent;
              c2.unit1 = location.sweep[b].agent;
              c2.unit2 = location.sweep[a].agent;
              c1.prevWpt = 0;
              c2.prevWpt = 0;
            }
            if(!countall && conf == BOTH_CARDINAL){
              break; // don't count any more, we don't care how many conflicts there are in total
            }
          }
        }
      }
    }
    ++a;
  }
  while(a==location.sweep.size() && b<path.size()){
    location.sweep.push_back(path[b]);
  }
}*/

/** Find the highest priority conflict **/
// By definition, we can't call this function unless we are using the CCT
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
std::pair<unsigned, unsigned> CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::FindHiPriConflictOneVsAll(
    CBSTreeNode<state, conflicttable> const& location, unsigned x, std::vector<Conflict<state>> &conflicts, bool update) {
  if (Params::verbose)
    std::cout << "Checking for conflicts (one vs all)\n";
  // prefer cardinal conflicts
  std::pair<unsigned, unsigned> best;

  Timer tmr;
  tmr.StartTimer();
  unsigned ctype(NO_CONFLICT);
    // For each pair of units in the group
    for (unsigned y(0); y < location.paths.size(); ++y)
    {
      unsigned previous(best.second);
      if (x == y)
        continue;
      // This call will update "best" with the number of conflicts and
      // with the *most* cardinal conflicts
      if (location.hasOverlap(x, y, radii[x], radii[y]))
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
        if (HasConflict(*location.paths[x], location.wpts[x], *location.paths[y], location.wpts[y], x, y,
                        conflicts, best, (Params::prioritizeConf ? ctype : best.second), update, true))
        {
          // Make sure that the conflict counted is the one being returned
          if (best.second > previous)
          {
            if (Params::xorconstraints)
            {
              if (Params::extrinsicconstraints)
              {
                conflicts[0].unit1 = x;
                conflicts[0].unit2 = y;
                conflicts[1].unit1 = y;
                conflicts[1].unit2 = x;
              }
              else
              {
                conflicts[0].unit1 = y;
                conflicts[0].unit2 = x;
                conflicts[1].unit1 = x;
                conflicts[1].unit2 = y;
              }
              conflicts[2].unit1 = x;
              conflicts[2].unit2 = y;
            }
            else
            {
              if (Params::extrinsicconstraints)
              {
                conflicts[0].unit1 = x;
                conflicts[0].unit2 = y;
                conflicts[1].unit1 = y;
                conflicts[1].unit2 = x;
              }
              else
              {
                conflicts[0].unit1 = y;
                conflicts[0].unit2 = x;
                conflicts[1].unit1 = x;
                conflicts[1].unit2 = y;
              }
            }
          }
          best.second = std::max(ctype, best.second);
          if (best.second == BOTH_CARDINAL)
          {
            update = false;
          } // Now that we've found a both cardinal conflict, no sense updating
        }
        }
    }
  collisionTime += tmr.EndTimer();
  if(!best.first && location.hasCollisions()){ // Are there any collisions left?
    unsigned x(0);
    unsigned y(0);
    if(Params::prioritizeConf){
      if(location.hasCardinal()){
        location.getCardinalPair(x,y);
    //std::cout << "selected C" << x << ","<<y<<"\n";
      }else if(location.hasSemiCardinal()){
        location.getSemiCardinalPair(x,y);
    //std::cout << "selected S" << x << ","<<y<<"\n";
      }else{
        location.getCollisionPair(x,y); // Select an arbitrary pair of colliding agents.
    //std::cout << "selected L" << x << ","<<y<<"\n";
      }
    }else{
      location.getCollisionPair(x,y); // Select an arbitrary pair of colliding agents.
    }
    //std::cout << "selected " << x << ","<<y<<"\n";
    assert(HasConflict(*location.paths[x], location.wpts[x], *location.paths[y], location.wpts[y], x, y,
          conflicts, best, best.second, true, true));
  }
  
  return best;
}

/** Find the highest priority conflict **/
template <typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
std::pair<unsigned, unsigned> CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::FindHiPriConflictAllPairs(
    CBSTreeNode<state, conflicttable> const &location, std::vector<Conflict<state>> &conflicts, bool update)
{
  if (Params::verbose)
    std::cout << "Checking for conflicts (all pairs)\n";
  // prefer cardinal conflicts
  std::pair<std::pair<unsigned, unsigned>, std::vector<Conflict<state>>> best;

  Timer tmr;
  tmr.StartTimer();
  unsigned ctype(NO_CONFLICT);
  // For each pair of units in the group
  for (unsigned x(0); x < location.paths.size() - 1; ++x)
  {
    for (unsigned y(x + 1); y < location.paths.size(); ++y)
    {
      unsigned previous(best.first.second);
      // This call will update "best" with the number of conflicts and
      // with the *most* cardinal conflicts
      if (location.hasOverlap(x, y, radii[x], radii[y]))
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
        if (HasConflict(*location.paths[x], location.wpts[x], *location.paths[y], location.wpts[y], x, y,
                        //best.second.first, best.second.second, best.first, best.first.second, update))
                        best.second, best.first, (Params::prioritizeConf ? ctype : best.first.second), update, true))
        {
          // Make sure that the conflict counted is the one being returned
          if (best.first.second > previous)
          {
            if (Params::xorconstraints)
            {
              best.second[0].unit1 = y;
              best.second[0].unit2 = x;
              best.second[1].unit1 = x;
              best.second[1].unit2 = y;
              best.second[2].unit1 = x;
              best.second[2].unit2 = y;
            }
            else
            {
              if (Params::extrinsicconstraints)
              {
                best.second[0].unit1 = y;
                best.second[0].unit2 = x;
                best.second[1].unit1 = x;
                best.second[1].unit2 = y;
              }
              else
              {
                best.second[0].unit1 = x;
                best.second[0].unit2 = y;
                best.second[1].unit1 = y;
                best.second[1].unit2 = x;
              }
            }
            if (best.first.second == BOTH_CARDINAL)
            {
              update = false;
            }
          }
          best.first.second = std::max(ctype, best.first.second);
        }
      }
    }
  }

  collisionTime += tmr.EndTimer();
  if (update && best.first.first)
  {
    conflicts = best.second;
  }
  return best.first;
}

/** Draw the AIR CBS group */
template<typename state, typename action, typename comparison, typename conflicttable, class singleHeuristic, class searchalgo>
void CBSGroup<state, action, comparison, conflicttable, singleHeuristic, searchalgo>::OpenGLDraw(
    const ConstrainedEnvironment<state, action> *ae,
    const SimulationInfo<state, action, ConstrainedEnvironment<state, action>> * sim) const {
  /*
   GLfloat r, g, b;
   glLineWidth(2.0);
   for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
   {
   CBSUnit<state,action,comparison,conflicttable,searchalgo> *unit = (CBSUnit<state,action,comparison,conflicttable,searchalgo>*)this->GetMember(x);
   unit->GetColor(r, g, b);
   ae->SetColor(r, g, b);
   for (unsigned int y = 0; y < tree[bestNode].paths[x]->size(); y++)
   {
   ae->OpenGLDraw(tree[bestNode].paths[x][y]);
   }
   }
   glLineWidth(1.0);
   */
}

#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
