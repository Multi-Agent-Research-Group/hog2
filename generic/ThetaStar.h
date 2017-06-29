/**
 * @file ThetaStar.h
 * @package hog2
 * @brief A templated version of the original HOG's genericAstar.h
 * @author Thayne Walker
 * SearchEnvironment
 *
 * This file is part of HOG2.
 * HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
 * HOG2: http://code.google.com/p/hog2/
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
 * along with HOG2; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef ThetaStar_H
#define ThetaStar_H

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
// this is defined in stdint.h, but it doesn't always get defined correctly
// even when __STDC_CONSTANT_MACROS is defined before including stdint.h
// because stdint might be included elsewhere first...
#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#include "FPUtil.h"
#include <ext/hash_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"

#include <algorithm> // for vector reverse

#include "GenericSearchAlgorithm.h"

template <class state>
struct ThetaStarCompare {
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
 */
template <class state, class action, class environment>
class ThetaStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	ThetaStar() { ResetNodeCount(); theHeuristic=nullptr; env = nullptr; stopAfterGoal = true; weight=1; reopenNodes = false; }
	virtual ~ThetaStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	
	AStarOpenClosed<state, ThetaStarCompare<state>, AStarOpenClosedData<state> > openClosedList;
	//BucketOpenClosed<state, ThetaStarCompare<state>, AStarOpenClosedData<xyLoc> > openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	//closedList_iterator GetClosedListIter() const;
	//	void GetClosedListIter(closedList_iterator);
	//	bool ClosedListIterNext(closedList_iterator& it, state& next) const;
	//bool GetClosedListGCost(state &val, double &gCost) const;
	bool GetClosedListGCost(const state &val, double &gCost) const;
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedData<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	void OpenGLDraw() const;
	
	void SetWeight(double w) {weight = w;}
private:
	std::vector<state> succ;
        std::pair<uint64_t,double> SetVertex(AStarOpenClosedData<xyLoc> const& p);
        std::pair<uint64_t,double> ComputeCost(AStarOpenClosedData<xyLoc> const& p, state& c, double oldg);
	uint64_t nodesTouched, nodesExpanded;
	environment *env;
	bool stopAfterGoal;
	
	double weight; 
	
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;
};

//static const bool verbose = false;

/**
 * Return the name of the algorithm. 
 * @author Thayne Walker
 * @date 06/27/17
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment>
const char *ThetaStar<state,action,environment>::GetName()
{
	static char name[32];
	sprintf(name, "ThetaStar[]");
	return name;
}

/**
 * Perform an A* search between two states.  
 * @author Thayne Walker
 * @date 06/27/17
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @param thePath A vector of states which will contain an optimal path 
 * between from and to when the function returns, if one exists. 
 */
template <class state, class action, class environment>
void ThetaStar<state,action,environment>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	//discardcount=0;
  	if (!InitializeSearch(_env, from, to, thePath))
  	{	
  		return;
  	}
  	while (!DoSingleSearchStep(thePath)) {}
}

/**
 * Initialize the A* search 
 * @author Thayne Walker	
 * @date 06/27/17
 * 
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @return TRUE if initialization was successful, FALSE otherwise
 */
template <class state, class action, class environment>
bool ThetaStar<state,action,environment>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if(theHeuristic==nullptr)theHeuristic = _env;
	thePath.resize(0);
	//if (useRadius)
	//std::cout<<"Using radius\n";
	env = _env;
	//	closedList.clear();
	//	openQueue.reset();
	//	assert(openQueue.size() == 0);
	//	assert(closedList.size() == 0);
	openClosedList.Reset();
	//openClosedList.Print();
	ResetNodeCount();
	start = from;
	goal = to;
	
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	
	openClosedList.AddOpenNode(start, env->GetStateHash(start), 0, weight*theHeuristic->HCost(start, goal));

        // Set the start state to be its own parent
        uint64_t id;
	openClosedList.Lookup(env->GetStateHash(start),id);
        openClosedList.Lookup(id).parentID=id;
	//openClosedList.Print();
	
	return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Thayne Walker
 * @date 06/27/17
 */
template <class state, class action, class environment>
void ThetaStar<state,action,environment>::AddAdditionalStartState(state& newState)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Thayne Walker
 * @date 06/27/17
 */
template <class state, class action, class environment>
void ThetaStar<state,action,environment>::AddAdditionalStartState(state& newState, double cost)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), cost, weight*theHeuristic->HCost(start, goal));
}

/**
 * Expand a single node. 
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @param thePath will contain an optimal path from start to goal if the 
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 */
template <class state, class action, class environment>
bool ThetaStar<state,action,environment>::DoSingleSearchStep(std::vector<state> &thePath){
  if (openClosedList.OpenSize() == 0)
  {
    thePath.resize(0); // no path found!
    //closedList.clear();
    return true;
  }
  uint64_t nodeid = openClosedList.Close();
  AStarOpenClosedData<state>& openNode = openClosedList.Lookup(nodeid);
  const state& currOpenNode = openNode.data;

  if (!openNode.reopened)
    uniqueNodesExpanded++;
  nodesExpanded++;

  {
    auto update(SetVertex(openNode));
    if(update.second>=0){
      openNode.parentID=update.first;
      openNode.g=update.second;
      //std::cout << "Reset parent of " << currOpenNode << " to " << openNode.parentID <<" " << update.second << "\n";
      //openClosedList.Print();
    }else{
      //std::cout << "Leave parent of " << currOpenNode << " as " << openClosedList.Lookup(openNode.parentID).data <<"\n";
    }
  }

  if ((stopAfterGoal) && (env->GoalTest(currOpenNode, goal)))
  {
    ExtractPathToStartFromID(nodeid, thePath);
    // Path is backwards - reverse
    reverse(thePath.begin(), thePath.end()); 
    return true;
  }

  env->GetSuccessors(currOpenNode, succ);
  double fCost = openNode.h+openNode.g;

  nodesTouched++;
  uint64_t theID;

  for (unsigned int x = 0; x < succ.size(); x++)
  {
    double edgeCost(env->GCost(currOpenNode, succ[x]));

    switch (openClosedList.Lookup(env->GetStateHash(succ[x]), theID))
    {
      case kClosedList:
        if (reopenNodes)
        {
          // do something here...
        }
        break;
      case kOpenList:
        {
          //edgeCost = env->GCost(openNode.data, neighbors[x]);
          auto update(ComputeCost(openNode,succ[x],openClosedList.Lookup(theID).g));
          //if (fless(openNode.g+edgeCost, openClosedList.Lookup(theID).g))
          if(update.second>=0 && fless(update.second, openClosedList.Lookup(theID).g))
          {
            //openClosedList.Lookup(theID).reopened = true;
            openClosedList.Lookup(theID).parentID = update.first;
            openClosedList.Lookup(theID).g = update.second;
            //std::cout << "Update " << succ[x] << " to p=" << openClosedList.Lookup(update.first).data << " " << update.second << "\n";
            openClosedList.KeyChanged(theID);
            //openClosedList.Print();
          }
        }
        break;
      case kNotFound:
        {
          auto update(ComputeCost(openNode,succ[x],9999999));
          //std::cout << "Create " << succ[x] << " with p=" << openClosedList.Lookup(update.first).data << " " << update.second << "\n";
          openClosedList.AddOpenNode(succ[x],
              env->GetStateHash(succ[x]),
              update.second,
              weight*theHeuristic->HCost(succ[x], goal),
              update.first);
          //openClosedList.Print();
        }
    }
  }
  return false;
}

template <class state, class action, class environment>
std::pair<uint64_t,double> ThetaStar<state, action,environment>::SetVertex(AStarOpenClosedData<xyLoc> const& p){
  AStarOpenClosedData<xyLoc>& pp(openClosedList.Lookup(p.parentID));
  if(!env->LineOfSight(pp.data,p.data)){
    //std::cout << "No LOS: " << pp.data << " " << p.data << "\n";
    uint64_t best(0);
    bool found(false);
    double bestg(9999999);
    std::vector<state> neighbors;
    env->GetSuccessors(p.data,neighbors);
    uint64_t id(0);
    for(auto const& n:neighbors){
      if(p.data == n) continue;
      auto loc(openClosedList.Lookup(env->GetStateHash(n),id));
      if(kClosedList==loc){ // Have we seen this before?
        double g(openClosedList.Lookat(id).g+env->GCost(p.data,n));
        if(fless(g,bestg)){
         bestg=g;
         best=id;
         //std::cout << "Reset parent of " << p.data << " from " << pp.data << " to " << n << " " << bestg << "\n";
         found=true;
        }
      }
    }
    assert(found && "best is zero .. this is not possible!");
    return {best,bestg};
  }else{
    return {0,-1.0};
  }
}

template <class state, class action, class environment>
std::pair<uint64_t,double> ThetaStar<state, action,environment>::ComputeCost(AStarOpenClosedData<xyLoc> const& p, state& c, double oldg){
  AStarOpenClosedData<xyLoc>& pp(openClosedList.Lookup(p.parentID));
  double newg(pp.g+env->GCost(pp.data,c));
  if(fless(newg,oldg)){
      return {p.parentID,newg};
  }
  return {0,-1.0};
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue). 
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment>
state ThetaStar<state, action,environment>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}


/**
 * Get the path from a goal state to the start state 
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment>
void ThetaStar<state, action,environment>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
{
  do {
    thePath.push_back(openClosedList.Lookup(node).data);
    node = openClosedList.Lookup(node).parentID;
  } while (openClosedList.Lookup(node).parentID != node);
  thePath.push_back(openClosedList.Lookup(node).data);
}

/**
 * A function that prints the number of states in the closed list and open
 * queue. 
 * @author Thayne Walker
 * @date 06/27/17
 */
template <class state, class action, class environment>
void ThetaStar<state, action,environment>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by PEAStar
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment>
int ThetaStar<state, action,environment>::GetMemoryUsage()
{
	return openClosedList.size();
}

/**
 * Get state from the closed list
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * more states
 */
template <class state, class action, class environment>
bool ThetaStar<state, action,environment>::GetClosedListGCost(const state &val, double &gCost) const
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

/**
 * Draw the open/closed list
 * @author Thayne Walker
 * @date 06/27/17
 * 
 */
template <class state, class action, class environment>
void ThetaStar<state, action,environment>::OpenGLDraw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	if (openClosedList.OpenSize() > 0)
		top = openClosedList.Peek();
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const AStarOpenClosedData<state> &data = openClosedList.Lookat(x);
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
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
}

#endif
