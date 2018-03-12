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
template <class BB, class environment, class openList=AStarOpenClosed<typename BB::State, TemporalAStarCompare<typename BB::State>>>
class TemporalAStar : public GenericSearchAlgorithm<typename BB::State,environment> {
public:
        typedef openList OpenList;
	TemporalAStar():env(0),totalExternalNodesExpanded(nullptr),externalExpansionLimit(INT_MAX),radius(4.0),stopAfterGoal(true),doPartialExpansion(false),verbose(false),weight(1),radEnv(0),reopenNodes(false),theHeuristic(0),directed(false),noncritical(false),SuccessorFunc(&environment::GetSuccessors),ActionFunc(&environment::GetAction),GCostFunc(&environment::GCost){ResetNodeCount();}
	virtual ~TemporalAStar() {}
        template<typename SVal>
	void GetPath(environment *env, const typename BB::State& from, const typename BB::State& to, std::vector<SVal> &thePath, unsigned minTime=0);
        template<typename SVal>
        void GetPaths(environment *_env, const typename BB::State& from, const typename BB::State& to, std::vector<std::vector<SVal>> &paths, double window=1.0, double bestf=-1.0, unsigned minTime=0);
        template<typename SVal>
	double GetNextPath(environment *env, const typename BB::State& from, const typename BB::State& to, std::vector<SVal> &thePath, unsigned minTime=0);
        inline openList* GetOpenList(){return &openClosedList;}
	
	openList openClosedList;
	//AStarOpenClosed<BB, AStarCompare<typename BB::State> > openClosedList;
	//BucketOpenClosed<BB, AStarCompare<typename BB::State> > openClosedList;
	typename BB::State goal, start;
        bool noncritical;
	
        template<typename SVal>
	bool InitializeSearch(environment *env, const typename BB::State& from, const typename BB::State& to, std::vector<SVal> &thePath, unsigned minTime=0);
        template<typename SVal>
	bool DoSingleSearchStep(std::vector<SVal> &thePath, unsigned minTime=0);
	void AddAdditionalStartState(typename BB::State const& newState);
	void AddAdditionalStartState(typename BB::State const& newState, double cost);
	
	typename BB::State CheckNextNode();
	void ExtractPathToStart(typename BB::State &node, std::vector<typename BB::State> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStart(typename BB::State &node, std::vector<BB> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<typename BB::State> &thePath);
	void ExtractPathToStartFromID(uint64_t node, std::vector<BB> &thePath);
	const typename BB::State &GetParent(const typename BB::State &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
	int GetMemoryUsage();
	
	bool GetClosedListGCost(const typename BB::State &val, double &gCost) const;
	bool GetOpenListGCost(const typename BB::State &val, double &gCost) const;
	bool GetClosedItem(const typename BB::State &s, AStarOpenClosedData<typename BB::State> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedData<typename BB::State> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedData<typename BB::State> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const typename BB::State &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const typename BB::State &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	void SetDirected(bool d) { directed = d; }
	
	void SetHeuristic(Heuristic<typename BB::State> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void SetRadius(double rad) {radius = rad;}
	double GetRadius() { return radius; }
	
	void SetRadiusEnvironment(environment *e) {radEnv = e;}
	void SetEnvironment(environment *e) {env = e;}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	inline void SetDoPartialExpansion(bool val) { doPartialExpansion = val; }
	inline bool GetDoPartialExpansion() { return doPartialExpansion; }
	
	inline void SetVerbose(bool val) { verbose = val; }
	inline bool GetVerbose() { return verbose; }
	
	void OpenGLDraw() const;
	void Draw() const;
	std::string SVGDraw() const;
	
	void SetWeight(double w) {weight = w;}
        void SetGCostFunc(void (environment::*gf)(const typename BB::State&, const typename BB::State&) const){GCostFunc=gf;}
        void SetHCostFunc(void (environment::*hf)(const typename BB::State&, const typename BB::State&) const){HCostFunc=hf;}
        void SetSuccessorFunc(void (environment::*sf)(const typename BB::State&, std::vector<typename BB::State>&) const){SuccessorFunc=sf;}
        void SetExternalExpansionsPtr(uint* ptr){totalExternalNodesExpanded=ptr;}
        void SetExternalExpansionLimit(uint limit){externalExpansionLimit=limit;}// std::cout << "Expansion limit set to: " << limit << "\n";}
private:
	uint64_t nodesTouched, nodesExpanded;
//	bool GetNextNode(typename BB::State &next);
//	//typename BB::State Node();
//	void UpdateClosedNode(environment *env, typename BB::State& currOpenNode, typename BB::State& neighbor);
//	void UpdateWeight(environment *env, typename BB::State& currOpenNode, typename BB::State& neighbor);
//	void AddToOpenList(environment *env, typename BB::State& currOpenNode, typename BB::State& neighbor);
	
	std::vector<typename BB::State> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<double> hCosts;
	std::vector<dataLocation> neighborLoc;
	environment *env;
        uint* totalExternalNodesExpanded;
        uint externalExpansionLimit;
	bool stopAfterGoal;
	bool doPartialExpansion;
        bool verbose;
	
	double radius; // how far around do we consider other agents?
	double weight; 
	bool directed;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<typename BB::State> *theHeuristic;
        double (environment::*HCostFunc)(const typename BB::State&, const typename BB::State&) const;
        double (environment::*GCostFunc)(const typename BB::State&, const typename BB::State&) const;
        void (environment::*SuccessorFunc)(const typename BB::State&, std::vector<typename BB::State>&) const;
        double timeStep;
};

//static const bool verbose = false;

/**
 * Return the name of the algorithm. 
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class BB, class environment, class openList>
const char *TemporalAStar<BB,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "TemporalAStar[]");
	return name;
}

/**
 * Perform an A* search between two states.  
 */
template <class BB, class environment, class openList>
template <class SVal>
void TemporalAStar<BB,environment,openList>::GetPath(environment *_env, const typename BB::State& from, const typename BB::State& to, std::vector<SVal> &thePath, unsigned minTime)
{
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
template <class BB, class environment, class openList>
template <class SVal>
void TemporalAStar<BB,environment,openList>::GetPaths(environment *_env, const typename BB::State& from, const typename BB::State& to, std::vector<std::vector<SVal>> &paths, double window, double bestf, unsigned minTime)
{
  double nextbestf(0);
  if(openClosedList.OpenSize() == 0){
    std::vector<SVal> thePath;
    GetPath(_env,from,to,thePath,minTime);
    GetClosedListGCost(thePath.back(),bestf);
    paths.push_back(thePath);
  }else if(bestf<0){
    uint64_t key(openClosedList.Peek());
    bestf=openClosedList.Lookup(key).g+openClosedList.Lookup(key).h;
  }

  while(fgeq(bestf+window,nextbestf)){
    std::vector<SVal> thePath;
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
template <class BB, class environment, class openList>
template <class SVal>
double TemporalAStar<BB,environment,openList>::GetNextPath(environment *env, const typename BB::State& from, const typename BB::State& to, std::vector<SVal> &thePath, unsigned minTime)
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


/**
 * Initialize the A* search
 */
template <class BB, class environment, class openList>
template <class SVal>
bool TemporalAStar<BB,environment,openList>::InitializeSearch(environment *_env, const typename BB::State& from, const typename BB::State& to, std::vector<SVal> &thePath, unsigned minTime)
{
	//lastF = 0;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	if (!radEnv)
		radEnv = _env;
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
template <class BB, class environment, class openList>
void TemporalAStar<BB,environment,openList>::AddAdditionalStartState(typename BB::State const& newState)
{
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), 0, weight*theHeuristic->HCost(start, goal));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class BB, class environment, class openList>
void TemporalAStar<BB,environment,openList>::AddAdditionalStartState(typename BB::State const& newState, double cost)
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
template <class BB, class environment, class openList>
template <class SVal>
bool TemporalAStar<BB,environment,openList>::DoSingleSearchStep(std::vector<SVal> &thePath, unsigned minTime)
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
	if (!openClosedList.Lookup(nodeid).reopened)
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
        double H(openClosedList.Lookup(nodeid).h);
        // Check for fcost > optCost and return early if we think there is no other solution

 	neighbors.resize(0);
	edgeCosts.resize(0);
	hCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);

	if(verbose)std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:" << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
 	(env->*SuccessorFunc)(openClosedList.Lookup(nodeid).data, neighbors);
        if((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal))){
          if(openClosedList.Lookup(nodeid).data.t>=minTime)
          {
            ExtractPathToStartFromID(nodeid, thePath);
            // Path is backwards - reverse
            reverse(thePath.begin(), thePath.end()); 
            return true;
          }else{
            // Need an action with a good time.
            typename BB::State n=openClosedList.Lookup(nodeid).data;
            n.t=minTime;
            // Returns 0 if no violation, otherwise the minimum safe time (minus epsilon)
            n.t=env->ViolatesConstraint(openClosedList.Lookup(nodeid).data,n);
            if(!n.t){
                n.t=minTime;
              if(std::find(neighbors.begin(),neighbors.end(),n)==neighbors.end()){
                neighbors.insert(neighbors.begin(),n);
              }
            }else{
              //n.t-=env->WaitTime();
              if(fgreater(n.t,openClosedList.Lookup(nodeid).data.t)
                  && std::find(neighbors.begin(),neighbors.end(),n)==neighbors.end()){
                neighbors.insert(neighbors.begin(),n);
              }
            }
          }
        }
	
	
        //std::cout << openClosedList.Lookup(nodeid).data << "("<<G<<"+"<<H<<")="<<(G+H)<<", "<<neighbors.size()<<" succ.\n";
	double bestH = 0;
	double lowHC = DBL_MAX;
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
		neighborID.push_back(theID);
		edgeCosts.push_back((env->GoalTest(openClosedList.Lookup(nodeid).data,goal)&&env->GoalTest(neighbors[x],goal))?0:(env->*GCostFunc)(openClosedList.Lookup(nodeid).data, neighbors[x]));

                double g(edgeCosts.back()+G);
                double h=DBL_MAX;
                if(neighborLoc.back() != kNotFound)
                  h=openClosedList.Lookup(theID).h;
                else
                  h=theHeuristic->HCost(neighbors[x], goal);
                hCosts.push_back(h);

                //std::cout << "  "<<neighbors[x]<<"("<<g<<"+"<<h<<")="<<(g+h)<<"\n";
                // PEA*
                // Find the lowest f-cost of the children
                if(doPartialExpansion && fgreater(g+h,G+H)) {
                  // New H for current node
                  lowHC = std::min(lowHC, h+edgeCosts.back());
                  //std::cout << "New H: " << lowHC << "\n";
                }

	}
	
        // PEA*
        // Replace the current node fcost with the "best" f-cost of it's children
        // and put it back in the open list
        if(doPartialExpansion && fless(H,lowHC)){
          openClosedList.Lookup(nodeid).h = lowHC;
          openClosedList.Reopen(nodeid);
          //std::cout << "ReOpened " << openClosedList.Lookup(nodeid).data << " with H="<<lowHC<<"\n";
        }
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
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
				if (reopenNodes)
				{
					if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
					{
						openClosedList.Lookup(neighborID[x]).parentID = nodeid;
						openClosedList.Lookup(neighborID[x]).g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
						openClosedList.Reopen(neighborID[x]);
						// This line isn't normally needed, but in some state spaces we might have
						// equality but different meta information, so we need to make sure that the
						// meta information is also copied, since this is the most generic A* implementation
						openClosedList.Lookup(neighborID[x]).data = neighbors[x];
					}
				}
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
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
//					std::cout << " adding to open ";
//					std::cout << double(theHeuristic->HCost(neighbors[x], goal)+openClosedList.Lookup(nodeid).g+edgeCosts[x]);
//					std::cout << " \n";
					if(doPartialExpansion) {
                                          // PEA*
                                          // Only add children that have the same f-cost as the parent
                                          double h(hCosts[x]);
                                          double g(G+edgeCosts[x]);
                                          if(fequal(G+H,g+h)){
                                            //std::cout << "  OPEN-->"<<neighbors[x]<<"("<<g<<"+"<<h<<")="<<(g+h)<<"\n";
                                            openClosedList.AddOpenNode(neighbors[x],
                                                env->GetStateHash(neighbors[x]),
                                                g,
                                                weight*h,
                                                nodeid);
                                          }
                                          //else
                                            //std::cout << "  ignore "<<neighbors[x]<<"("<<g<<"+"<<h<<")="<<(g+h)<<"\n";
                                        } else {
                                          if(verbose)std::cout << "Add node ("<<std::hex<<env->GetStateHash(neighbors[x])<<std::dec<<") to open " << neighbors[x] << (G+edgeCosts[x]) << "+" << (weight*hCosts[x]) << "=" << (G+edgeCosts[x]+weight*hCosts[x]) << "\n";
                                          openClosedList.AddOpenNode(neighbors[x],
                                              env->GetStateHash(neighbors[x]),
                                              G+edgeCosts[x],
                                              weight*hCosts[x],
                                              nodeid);
                                        }
//					if (loc == -1)
//					{ // duplicate edges
//						neighborLoc[x] = kOpenList;
//						x--;
//					}
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
template <class BB, class environment, class openList>
typename BB::State TemporalAStar<BB,environment,openList>::CheckNextNode()
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
template <class BB, class environment,class openList>
void TemporalAStar<BB,environment,openList>::ExtractPathToStartFromID(uint64_t node,
    std::vector<typename BB::State> &thePath)
{
  do {
    thePath.push_back(openClosedList.Lookup(node).data);
    node = openClosedList.Lookup(node).parentID;
  } while (openClosedList.Lookup(node).parentID != node);
  thePath.push_back(openClosedList.Lookup(node).data);
}

template <class BB, class environment,class openList>
void TemporalAStar<BB,environment,openList>::ExtractPathToStartFromID(uint64_t node,
    std::vector<BB> &thePath) {
  typename BB::State const& parent(openClosedList.Lookup(node).data);
  while(parent.parentID!=node){
    node = openClosedList.Lookup(node).parentID;
    BB const& val(openClosedList.Lookup(node).data);
    thePath.emplace_back(val,parent,agent); // This is backwards because we are traversing from g to s
  }
}

template <class BB, class environment,class openList>
const typename BB::State &TemporalAStar<BB,environment,openList>::GetParent(const typename BB::State &s)
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
template <class BB, class environment, class openList>
void TemporalAStar<BB,environment,openList>::PrintStats()
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
template <class BB, class environment, class openList>
int TemporalAStar<BB,environment,openList>::GetMemoryUsage()
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
template <class BB, class environment, class openList>
bool TemporalAStar<BB,environment,openList>::GetClosedListGCost(const typename BB::State &val, double &gCost) const
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

template <class BB, class environment, class openList>
bool TemporalAStar<BB,environment,openList>::GetOpenListGCost(const typename BB::State &val, double &gCost) const
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

template <class BB, class environment, class openList>
bool TemporalAStar<BB,environment,openList>::GetClosedItem(const typename BB::State &s, AStarOpenClosedData<typename BB::State> &result)
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
template <class BB, class environment, class openList>
void TemporalAStar<BB,environment,openList>::OpenGLDraw() const
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
//		const AStarOpenClosedData<typename BB::State> &data = openClosedList.Lookat(x);
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
template <class BB, class environment, class openList>
void TemporalAStar<BB,environment,openList>::Draw() const
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

template <class BB, class environment, class openList>
std::string TemporalAStar<BB,environment,openList>::SVGDraw() const
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
