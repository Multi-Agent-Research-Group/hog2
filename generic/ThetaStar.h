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

struct vals{
  uint64_t p;
  double t;
  double g;
};

/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment, class openList = AStarOpenClosed<state, ThetaStarCompare<state>>>
class ThetaStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	ThetaStar():totalExternalNodesExpanded(nullptr),externalExpansionLimit(INT_MAX),verbose(false),noncritical(false),theHeuristic(nullptr),env(nullptr),stopAfterGoal(true),weight(1),reopenNodes(false),SuccessorFunc(&environment::GetSuccessors),ActionFunc(&environment::GetAction),GCostFunc(&environment::GCost){ResetNodeCount();}
	virtual ~ThetaStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, double minTime=0);
	
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	
        inline openList* GetOpenList(){return &openClosedList;}
	openList openClosedList;
	//BucketOpenClosed<state, ThetaStarCompare<state>, AStarOpenClosedData<state> > openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath, double minTime=0);
	bool DoSingleSearchStep(std::vector<state> &thePath, double minTime);
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
        void SetGCostFunc(void (environment::*gf)(const state&, const state&) const){GCostFunc=gf;}
        void SetHCostFunc(void (environment::*hf)(const state&, const state&) const){HCostFunc=hf;}
        void SetSuccessorFunc(void (environment::*sf)(const state&, std::vector<state>&) const){SuccessorFunc=sf;}
        void SetActionFunc(action (environment::*af)(const state&, const state&) const){ActionFunc=af;}
        void SetVerbose(bool v){verbose=v;}
        bool noncritical;
        void SetExternalExpansionsPtr(uint* ptr){totalExternalNodesExpanded=ptr;}
        void SetExternalExpansionLimit(uint limit){externalExpansionLimit=limit;}
private:
        uint* totalExternalNodesExpanded;
        uint externalExpansionLimit;
        bool verbose;
	std::vector<state> succ;
        vals ComputeCost(AStarOpenClosedData<state> const& p, state& c, double newg, double oldg);
	uint64_t nodesTouched, nodesExpanded;
	environment *env;
	bool stopAfterGoal;
	
	double weight; 
	
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	Heuristic<state> *theHeuristic;
        double (environment::*HCostFunc)(const state&, const state&) const;
        double (environment::*GCostFunc)(const state&, const state&) const;
        void (environment::*SuccessorFunc)(const state&, std::vector<state>&) const;
        action (environment::*ActionFunc)(const state&, const state&) const;
};

//static const bool verbose = false;

/**
 * Return the name of the algorithm. 
 * @author Thayne Walker
 * @date 06/27/17
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, class openList>
const char *ThetaStar<state,action,environment,openList>::GetName()
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
template <class state, class action, class environment, class openList>
void ThetaStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath, double minTime)
{
	//discardcount=0;
  	if (!InitializeSearch(_env, from, to, thePath))
  	{	
  		return;
  	}
  	while (!DoSingleSearchStep(thePath,minTime)) {}
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
template <class state, class action, class environment, class openList>
bool ThetaStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath, double minTime)
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
	
	if (env->GoalTest(from, to) && (stopAfterGoal) && to.t > minTime) //assumes that from and to are valid states
	{
                thePath.push_back(from);
                thePath.push_back(to);
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
template <class state, class action, class environment, class openList>
void ThetaStar<state,action,environment,openList>::AddAdditionalStartState(state& newState)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Thayne Walker
 * @date 06/27/17
 */
template <class state, class action, class environment, class openList>
void ThetaStar<state,action,environment,openList>::AddAdditionalStartState(state& newState, double cost)
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
template <class state, class action, class environment, class openList>
bool ThetaStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath, double minTime){
  if (openClosedList.OpenSize() == 0)
  {
    thePath.resize(0); // no path found!
    //closedList.clear();
    return true;
  }
  uint64_t nodeid = openClosedList.Close();
  AStarOpenClosedData<state> openNode(openClosedList.Lookup(nodeid));

  if (!openNode.reopened)
    uniqueNodesExpanded++;
  nodesExpanded++;
  if(totalExternalNodesExpanded){
    (*totalExternalNodesExpanded)++; // Increment external counter
    if(*totalExternalNodesExpanded>externalExpansionLimit){
      thePath.resize(0);
      return true;
    }
  }

  if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)) && openClosedList.Lookup(nodeid).data.t > minTime)
  {
    ExtractPathToStartFromID(nodeid, thePath);
    // Path is backwards - reverse
    reverse(thePath.begin(), thePath.end()); 
    if(thePath.size() == 0&&verbose)std::cout<<"No answer\n";
    float total(0.0);
    state* p(&thePath[0]);
    bool first=true;
    for(auto& n:thePath){
      total+=n.t-p->t;
      if(!first&&!((n.t >= total - .001) && (n.t <= total+.001)))std::cout << "Time is bad: ("<<total<<")" << *p << "-->" << n << "\n";
      p=&n;
      first=false;
    }
    return true;
  }

  if(verbose)std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data<<std::hex<<"("<<env->GetStateHash(openClosedList.Lookup(nodeid).data)<<")"<<std::dec << "(parent)"<<openClosedList.Lookup(openNode.parentID).data <<" with f:" << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
  succ.resize(0);
  (env->*SuccessorFunc)(openClosedList.Lookup(nodeid).data, succ);
  //double fCost = openNode.h+openNode.g;

  nodesTouched++;
  uint64_t theID;

  for (unsigned int x = 0; x < succ.size(); x++)
  {
    double edgeCost((env->*GCostFunc)(openClosedList.Lookup(nodeid).data, succ[x]));

    if(verbose)std::cout << "  Lookup successor (" <<std::hex<<env->GetStateHash(succ[x])<<std::dec<<") in open " << succ[x]<<"\n";
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
          //edgeCost = (env->*GCostFunc)(openNode.data, neighbors[x]);
          auto update(ComputeCost(openNode,succ[x],openNode.g+edgeCost,openClosedList.Lookup(theID).g));
          //if (fless(openNode.g+edgeCost, openClosedList.Lookup(theID).g))
          if(update.g>=0){
            auto newNode(succ[x]);
            newNode.t=update.t;
            //newNode.h=Util::heading<1024>(openNode.data.x,openNode.data.y,succ[x].x,succ[x].y);
            if(verbose)std::cout << "  Update " << succ[x] << " to ";
            uint64_t newid;
            if(openClosedList.Lookup(env->GetStateHash(newNode), newid)==kNotFound){
              // This may happen if re-parented
              if(verbose)std::cout << newNode << " p=" << openClosedList.Lookup(update.p).data << " " << update.g << "\n";
              openClosedList.AddOpenNode(newNode,
                  env->GetStateHash(newNode),
                  update.g,
                  weight*theHeuristic->HCost(newNode, goal),
                  update.p);
            }else{if(verbose)std::cout << "  Discarded (new open update already seen)\n";}
            //}else if(update.p!=openClosedList.Lookup(theID).parentID){
              // This might not ever happen since a new parent necessarily means a reduction in g-cost
            //assert(!"This shouldn't happen since a new parent necessarily means a reduction in g-cost");
            //openClosedList.Lookup(theID).parentID = update.p;
            //openClosedList.Lookup(theID).g = update.g;
              //openClosedList.KeyChanged(theID);
            //}
          }
        }
        break;
      case kNotFound:
        {
          if(openClosedList.Lookup(nodeid).data.sameLoc(succ[x])){
            // This is a wait action
              if(verbose)std::cout << "  Add wait action ("<<std::hex<<env->GetStateHash(succ[x])<<std::dec<<") to open " << succ[x] << openNode.g+edgeCost << "+" << (weight*theHeuristic->HCost(succ[x], goal)) << "=" << (openNode.g+edgeCost+weight*theHeuristic->HCost(succ[x], goal)) << "\n";
            openClosedList.AddOpenNode(succ[x],
                env->GetStateHash(succ[x]),
                openNode.g+edgeCost,
                weight*theHeuristic->HCost(succ[x], goal),
                nodeid);
          }else{
            auto update(ComputeCost(openNode,succ[x],openNode.g+edgeCost,9999999));
            succ[x].t=update.t;
            //succ[x].h=Util::heading<1024>(openNode.data.x,openNode.data.y,succ[x].x,succ[x].y);
            // If time was updated, check if it is still not in open
            if(openClosedList.Lookup(env->GetStateHash(succ[x]), theID)==kNotFound){
              //if(verbose)std::cout << "Create " << succ[x] << " with p=" << openClosedList.Lookup(update.p).data << " " << update.g << "\n";
              if(verbose)std::cout << "  Add node ("<<std::hex<<env->GetStateHash(succ[x])<<std::dec<<") to open " << succ[x] << update.g << "+" << (weight*theHeuristic->HCost(succ[x], goal)) << "=" << (update.g+weight*theHeuristic->HCost(succ[x], goal)) << "\n";
              openClosedList.AddOpenNode(succ[x],
                  env->GetStateHash(succ[x]),
                  update.g,
                  weight*theHeuristic->HCost(succ[x], goal),
                  update.p);
              //openClosedList.Print();
            }else{if(verbose)std::cout << "  Discarded (already seen "<<std::hex<<env->GetStateHash(succ[x])<<std::dec<<")\n";}
          }
        }
    }
    //usleep(10000);
    //OpenGLDraw();
  }
  return false;
}

template <class state, class action, class environment, class openList>
vals ThetaStar<state,action,environment,openList>::ComputeCost(AStarOpenClosedData<state> const& p, state& c, double newg, double oldg){
  uint64_t pid;
  openClosedList.Lookup(env->GetStateHash(p.data),pid);
  AStarOpenClosedData<state>& pp(openClosedList.Lookup(p.parentID));
  // Special cases for waiting actions
  double newt(c.t);
  if(pp.data.sameLoc(c))return{pid,newt,newg}; // parent of parent is same as self
  if(p.data.sameLoc(c))return{pid,newt,newg}; // parent is same as self
  if(p.data.sameLoc(pp.data))return{pid,newt,newg}; // parent is same as self

  if(env->LineOfSight(pp.data,c)){
    if(verbose)std::cout << "  LOS " << pp.data << "-->" << c << "\n";
    newg=pp.g+(env->*GCostFunc)(pp.data,c);
    if(fless(newg,oldg)){
      if(verbose)std::cout << "  Change parent of " << c << " to " << pp.data << " from " << p.data <<" " << newg << "\n";
      newt = pp.data.t+Util::distance(pp.data.x,pp.data.y,c.x,c.y);
      return {p.parentID,newt,newg};
    }
  }else if(fless(newg,oldg)){
    if(verbose)std::cout << "  NO LOS " << pp.data << "-->" << c << "\n";
    if(verbose)std::cout << "  Change gcost of " << c << " with parent " << p.data<<"("<<p.openLocation<<")" <<" to " << newg << "\n";
    return {pid,newt,newg};
  }
  if(verbose)std::cout << "  NO LOS " << pp.data << "-->" << c << "\n";
  if(verbose)std::cout << "  Leave parent of " << c << " as " << p.data <<" " << newg << "\n";
  return {0,0.0,-1.0};
}

/**
 * Returns the next state on the open list (but doesn't pop it off the queue). 
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment, class openList>
state ThetaStar<state,action,environment,openList>::CheckNextNode()
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
template <class state, class action,class environment, class openList>
void ThetaStar<state,action,environment,openList>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
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
template <class state, class action, class environment, class openList>
void ThetaStar<state,action,environment,openList>::PrintStats()
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
template <class state, class action, class environment, class openList>
int ThetaStar<state,action,environment,openList>::GetMemoryUsage()
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
template <class state, class action, class environment, class openList>
bool ThetaStar<state,action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
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
template <class state, class action, class environment, class openList>
void ThetaStar<state,action,environment,openList>::OpenGLDraw() const
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
