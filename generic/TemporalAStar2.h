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

#ifndef TemporalAStar_H
#define TemporalAStar_H

#include <iostream>
#include <limits.h> 
#include "FPUtil.h"
#include <ext/hash_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"

#include <algorithm> // for vector reverse

#include "GenericSearchAlgorithm.h"
//static double lastF = 0;

#define BRANCHING_FACTOR 100
template <class state>
struct TemporalAStarCompare {
	bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
	{
		if (fequal(i1.g+i1.h, i2.g+i2.h))
		{
			return (fless(i1.g, i2.g));
		}
		return (fgreater(i1.g+i1.h, i2.g+i2.h));
	}
};

/**
 * A templated version of A*, based on HOG genericAStar
 * This version makes sure that the goal state has at least time >= minTime
 */
template <class state, class action, class environment, class openList=AStarOpenClosed<state, TemporalAStarCompare<state>>>
class TemporalAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	TemporalAStar():env(0),totalExternalNodesExpanded(nullptr),externalExpansionLimit(INT_MAX),stopAfterGoal(true),verbose(false),weight(1),theHeuristic(0),noncritical(false),SuccessorFunc(&environment::GetSuccessors),ActionFunc(&environment::GetAction),GCostFunc(&environment::GCost){ResetNodeCount();}
	virtual ~TemporalAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, unsigned minTime=0);
        void GetPaths(environment *_env, const state& from, const state& to, std::vector<std::vector<state>> &paths, double window=1.0, double bestf=-1.0, unsigned minTime=0);
	double GetNextPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, unsigned minTime=0);
	void GetPath(environment *, const state& , const state& , std::vector<action> &);
        inline openList* GetOpenList(){return &openClosedList;}
	
	openList openClosedList;
	//AStarOpenClosed<state, AStarCompare<state> > openClosedList;
	//BucketOpenClosed<state, AStarCompare<state> > openClosedList;
	state goal, start;
        bool noncritical;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath, unsigned minTime=0);
	bool DoSingleSearchStep(std::vector<state> &thePath, unsigned minTime=0);
	void AddAdditionalStartState(state const& newState);
	void AddAdditionalStartState(state const& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedData<state> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetGoalSteps(bool v) { goalSteps=v; }

	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void SetEnvironment(environment *e) {env = e;}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	inline void SetVerbose(bool val) { verbose = val; }
	inline bool GetVerbose() { return verbose; }
	
	void OpenGLDraw() const;
	void Draw() const;
	std::string SVGDraw() const;
	
	void SetWeight(double w) {weight = w;}
        void SetGCostFunc(void (environment::*gf)(const state&, const state&) const){GCostFunc=gf;}
        void SetHCostFunc(void (environment::*hf)(const state&, const state&) const){HCostFunc=hf;}
        void SetSuccessorFunc(unsigned (environment::*sf)(const state&, state*) const){SuccessorFunc=sf;}
        void SetActionFunc(action (environment::*af)(const state&, const state&) const){ActionFunc=af;}
        void SetExternalExpansionsPtr(uint* ptr){totalExternalNodesExpanded=ptr;}
        void SetExternalExpansionLimit(uint limit){externalExpansionLimit=limit;}//std::cout << "Expansion limit set to: " << limit << "\n";}
private:
	uint64_t nodesTouched, nodesExpanded;
//	bool GetNextNode(state &next);
//	//state Node();
//	void UpdateClosedNode(environment *env, state& currOpenNode, state& neighbor);
//	void UpdateWeight(environment *env, state& currOpenNode, state& neighbor);
//	void AddToOpenList(environment *env, state& currOpenNode, state& neighbor);
	
	state neighbors[BRANCHING_FACTOR];
	uint64_t neighborID[BRANCHING_FACTOR];
	double edgeCosts[BRANCHING_FACTOR];
	double hCosts[BRANCHING_FACTOR];
	dataLocation neighborLoc[BRANCHING_FACTOR];
	environment *env;
        uint* totalExternalNodesExpanded;
        uint externalExpansionLimit;
	bool stopAfterGoal;
        bool verbose;
	
	double weight; 
	bool goalSteps;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;
        double (environment::*HCostFunc)(const state&, const state&) const;
        double (environment::*GCostFunc)(const state&, const state&) const;
        unsigned (environment::*SuccessorFunc)(const state&, state*) const;
        action (environment::*ActionFunc)(const state&, const state&) const;
        double timeStep;
        double estimate;
};

//static const bool verbose = false;

/**
 * Return the name of the algorithm. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, class openList>
const char *TemporalAStar<state,action,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "TemporalAStar[]");
	return name;
}

/**
 * Perform an A* search between two states.  
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath, unsigned minTime)
{
  if (theHeuristic == 0) theHeuristic = _env;
  // If the cost limit is provided, use it.
  estimate=to.t?to.t:DBL_MAX;

  if (!InitializeSearch(_env, from, to, thePath,minTime))
  {	
    return;
  }
  while (!DoSingleSearchStep(thePath,minTime))
  {
  }
}

/*
 * Get a set of paths in range of optimality
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state,action,environment,openList>::GetPaths(environment *_env, const state& from, const state& to, std::vector<std::vector<state>> &paths, double window, double bestf, unsigned minTime)
{
  double nextbestf(0);
  static std::vector<state> thePath;
  if(openClosedList.OpenSize() == 0){
    thePath.resize(0);
    GetPath(_env,from,to,thePath,minTime);
    GetClosedListGCost(thePath.back(),bestf);
    paths.push_back(thePath);
  }else if(bestf<0){
    uint64_t key(openClosedList.Peek());
    bestf=openClosedList.Lookup(key).g+openClosedList.Lookup(key).h;
  }

  while(fgeq(bestf+window,nextbestf)){
    thePath.resize(0);
    do{
      uint64_t key(openClosedList.Peek());
      nextbestf=openClosedList.Lookup(key).g+openClosedList.Lookup(key).h;
    }while(fgeq(bestf+window,nextbestf)&&!DoSingleSearchStep(thePath,minTime));
    if(thePath.size())paths.push_back(thePath);
  }
}

/**
 * Retrieve the next path found in the OPEN list
 */
template <class state, class action, class environment, class openList>
double TemporalAStar<state,action,environment,openList>::GetNextPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, unsigned minTime)
{
  if(openClosedList.OpenSize() == 0){
    GetPath(env,from,to,thePath,minTime);
    double val(0.0);
    GetClosedListGCost(thePath.back(),val);
    return val;
  }else{
    thePath.resize(0);
    double f(0.0);
    do{
      uint64_t key(openClosedList.Peek());
      f=openClosedList.Lookup(key).g+openClosedList.Lookup(key).h;
    }while(!DoSingleSearchStep(thePath,minTime));
    return f;
  }
}

template <class state, class action, class environment, class openList>
void TemporalAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
  estimate=to.t?to.t:DBL_MAX;
  std::vector<state> thePath;
  if (!InitializeSearch(_env, from, to, thePath))
  {
    return;
  }
  path.resize(0);
  while (!DoSingleSearchStep(thePath))
  {
  }
  for (int x = 0; x < thePath.size()-1; x++)
  {
    path.push_back((_env->*ActionFunc)(thePath[x], thePath[x+1]));
  }
}


/**
 * Initialize the A* search
 */
template <class state, class action, class environment, class openList>
bool TemporalAStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath, unsigned minTime)
{
	thePath.resize(0);
	env = _env;
	//	closedList.clear();
	//	openQueue.reset();
	//	assert(openQueue.size() == 0);
	//	assert(closedList.size() == 0);
	openClosedList.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	
	/*if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
                thePath.push_back(start);
		return false;
	}*/
	
	openClosedList.AddOpenNode(start, env->GetStateHash(start), 0, weight*theHeuristic->HCost(start, goal));
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state,action,environment,openList>::AddAdditionalStartState(state const& newState)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state,action,environment,openList>::AddAdditionalStartState(state const& newState, double cost)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
}

/**
 * Expand a single node. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param thePath will contain an optimal path from start to goal if the 
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 */
template <class state, class action, class environment, class openList>
bool TemporalAStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath, unsigned minTime)
{
  if(openClosedList.OpenSize() == 0)
  {
    //thePath.resize(0); // no path found!
    //closedList.clear();
    return true;
  }
  uint64_t nodeid = openClosedList.Close();
  //	if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
  //	{ lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
  //		//printf("Updated limit to %f\n", lastF);
  //	}
  if(!openClosedList.Lookup(nodeid).reopened)
    uniqueNodesExpanded++;

  nodesExpanded++;
  // Limit expansions WRT to an external counter (e.g. high-level search)
  if(totalExternalNodesExpanded){
    (*totalExternalNodesExpanded)++; // Increment external counter
    if(*totalExternalNodesExpanded>externalExpansionLimit){
      thePath.resize(0);
      return true;
    }
  }

  double G(openClosedList.Lookup(nodeid).g);
  //double H(openClosedList.Lookup(nodeid).h);
  // Check for fcost > optCost and return early if we think there is no other solution

  if(verbose)std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:" << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
  unsigned nSuccessors((env->*SuccessorFunc)(openClosedList.Lookup(nodeid).data, neighbors));
  if(stopAfterGoal){
    if(env->GoalTest(openClosedList.Lookup(nodeid).data, goal)){
      if(openClosedList.Lookup(nodeid).data.t>=minTime)
      {
        ExtractPathToStartFromID(nodeid, thePath);
        // Path is backwards - reverse
        reverse(thePath.begin(), thePath.end()); 
        return true;
      }else{
        // Need an action with a good time.
        state n=openClosedList.Lookup(nodeid).data;
        n.t=minTime;
        // Returns 0 if no violation, otherwise the minimum safe time (minus epsilon)
        n.t=env->ViolatesConstraint(openClosedList.Lookup(nodeid).data,n);
        if(!n.t){
          n.t=minTime;
          if(!goalSteps){
            bool found(false);
            for(unsigned x(0); x<nSuccessors; ++x){
              if(n==neighbors[x]){
                found=true;
                break;
              }
            }
            if(!found){
              neighbors[nSuccessors++]=n;
            }
          }
        }else{
          //n.t-=env->WaitTime();
          if(!goalSteps
              && fgreater(n.t,openClosedList.Lookup(nodeid).data.t)){
            bool found(false);
            for(unsigned x(0); x<nSuccessors; ++x){
              if(n==neighbors[x]){
                found=true;
                break;
              }
            }
            if(!found){
              neighbors[nSuccessors++]=n;
            }
          }
        }
      }
    }else if(goal.t && openClosedList.Lookup(nodeid).data.t > goal.t){
      return false;
    }
  }


  //std::cout << openClosedList.Lookup(nodeid).data << "("<<G<<"+"<<H<<")="<<(G+H)<<", "<<neighbors.size()<<" succ.\n";
  // 1. load all the children
  for (unsigned int x = 0; x < nSuccessors; x++)
  {
    uint64_t theID;
    neighborLoc[x]=openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID);
    neighborID[x]=theID;
    edgeCosts[x]=(env->GoalTest(openClosedList.Lookup(nodeid).data,goal)&&env->GoalTest(neighbors[x],goal))?0:(env->*GCostFunc)(openClosedList.Lookup(nodeid).data, neighbors[x]);

    double h=DBL_MAX;
    if(neighborLoc[x] != kNotFound)
      h=openClosedList.Lookup(theID).h;
    else
      h=theHeuristic->HCost(neighbors[x], goal);
    hCosts[x]=h;

  }

  // iterate again updating costs and writing out to memory
  for (int x = 0; x < nSuccessors; x++)
  {
    nodesTouched++;
    //double edgeCost;
    //		std::cout << "Checking neighbor: " << neighbors[x] << "\n";

    switch (neighborLoc[x])
    {
      case kClosedList:
        if(verbose)std::cout << "Closed\n";
        //edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
        //				std::cout << "Already closed\n";
        break;
      case kOpenList:
        //edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
        if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
        {
          openClosedList.Lookup(neighborID[x]).parentID = nodeid;
          openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
          // This line isn't normally needed, but in some state spaces we might have
          // equality but different meta information, so we need to make sure that the
          // meta information is also copied, since this is the most generic A* implementation
          openClosedList.Lookup(neighborID[x]).data = neighbors[x];
          openClosedList.KeyChanged(neighborID[x]);
          if(verbose)std::cout << " Reducing cost to " << openClosedList.Lookup(nodeid).g+edgeCosts[x] << "\n";
          // TODO: unify the KeyChanged calls.
        }
        break;
      case kNotFound:
        if(fleq(G+edgeCosts[x]+weight*hCosts[x],estimate)){
          if(verbose)std::cout << "Add node ("<<std::hex<<env->GetStateHash(neighbors[x])<<std::dec<<") to open " << neighbors[x] << (G+edgeCosts[x]) << "+" << (weight*hCosts[x]) << "=" << (G+edgeCosts[x]+weight*hCosts[x]) << "\n";
          openClosedList.AddOpenNode(neighbors[x],
              env->GetStateHash(neighbors[x]),
              G+edgeCosts[x],
              weight*hCosts[x],
              nodeid);
        }
    }
  }
  if(!stopAfterGoal && openClosedList.OpenSize() == 0)
  {
    // We have reached the end of the search.
    // Return the last state
    thePath.push_back(openClosedList.Lookup(nodeid).data);
  }
  return false;
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue). 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment, class openList>
state TemporalAStar<state, action,environment,openList>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}

/**
 * Get the path from a goal state to the start state 
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment,class openList>
void TemporalAStar<state, action,environment,openList>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action,class environment,class openList>
const state &TemporalAStar<state, action,environment,openList>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

/**
 * A function that prints the number of states in the closed list and open
 * queue. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state, action,environment,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by TemporalAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 * 
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment, class openList>
int TemporalAStar<state, action,environment,openList>::GetMemoryUsage()
{
	return openClosedList.size();
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 * 
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * the states
 */
template <class state, class action, class environment, class openList>
bool TemporalAStar<state, action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool TemporalAStar<state, action,environment,openList>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool TemporalAStar<state, action,environment,openList>::GetClosedItem(const state &s, AStarOpenClosedData<state> &result)
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = openClosedList.Lookat(theID);
		return true;
	}
	return false;

}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 * 
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state, action,environment,openList>::OpenGLDraw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
//	double minf = 1e9, maxf = 0;
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
//	for (unsigned int x = 0; x < openClosedList.size(); x++)
//	{
//		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
//		double f = data.g+data.h;
//		if (f > maxf)
//			maxf = f;
//		if (f < minf)
//			minf = f;
//	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kOpenList) 
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kClosedList)
		{
//			if (top != -1)
//			{
//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
//			}
//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
//			}
			env->OpenGLDraw(data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->OpenGLDraw(goal);
}

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 7/12/16
 *
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state, action,environment,openList>::Draw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(data.data);
		}
		else if (data.where == kClosedList)
		{
			//			if (top != -1)
			//			{
			//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
			//			}
			//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			//			}
			env->Draw(data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(goal);
}

template <class state, class action, class environment, class openList>
std::string TemporalAStar<state, action,environment,openList>::SVGDraw() const
{
	std::string s;
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return s;
	uint64_t top = -1;
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			s+=env->SVGDraw(data.data);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			s+=env->SVGDraw(data.data);
		}
	}
	return s;
}


#endif
