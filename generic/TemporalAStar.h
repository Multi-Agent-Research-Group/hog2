/**
 * @file TemporalAStar.h
 * @package hog2
 * @brief Returns a path of minimum time length - requires states to have a field "t" for time
 * (for agents who stay at their goal and may need to move back out)
 * @author Thayne Walker
 * @date 9/11/2017
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
template <class state, class action, class environment, class openList=AStarOpenClosed<state, TemporalAStarCompare<state>>>
class TemporalAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	TemporalAStar():env(0),totalExternalNodesExpanded(nullptr),externalExpansionLimit(INT_MAX),useBPMX(0),radius(4.0),stopAfterGoal(true),doPartialExpansion(false),verbose(false),weight(1),useRadius(false),useOccupancyInfo(false),radEnv(0),reopenNodes(false),theHeuristic(0),directed(false),noncritical(false),SuccessorFunc(&environment::GetSuccessors),ActionFunc(&environment::GetAction),GCostFunc(&environment::GCost){ResetNodeCount();}
	virtual ~TemporalAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, double minTime=0.0);
        void GetPaths(environment *_env, const state& from, const state& to, std::vector<std::vector<state>> &paths, double window=1.0, double bestf=-1.0, double minTime=0.0);
	double GetNextPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, double minTime=0.0);
	void GetPath(environment *, const state& , const state& , std::vector<action> &);
        inline openList* GetOpenList(){return &openClosedList;}
	
	openList openClosedList;
	//AStarOpenClosed<state, AStarCompare<state> > openClosedList;
	//BucketOpenClosed<state, AStarCompare<state> > openClosedList;
	state goal, start;
        bool noncritical;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath, double minTime=0.0);
	bool DoSingleSearchStep(std::vector<state> &thePath, double minTime=0.0);
	void AddAdditionalStartState(state const& newState);
	void AddAdditionalStartState(state const& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{ uint64_t theID; openClosedList.Lookup(env->GetStateHash(node), theID); ExtractPathToStartFromID(theID, thePath); }
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	void DoAbstractSearch(){useOccupancyInfo = false; useRadius = false;}
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
	
	void SetUseBPMX(int depth) { useBPMX = depth; if (depth) reopenNodes = true; }
	int GetUsingBPMX() { return useBPMX; }

	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	void SetDirected(bool d) { directed = d; }
	
	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
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
	
	void FullBPMX(uint64_t nodeID, int distance);
	
	void OpenGLDraw() const;
	void Draw() const;
	std::string SVGDraw() const;
	
	void SetWeight(double w) {weight = w;}
        void SetGCostFunc(void (environment::*gf)(const state&, const state&) const){GCostFunc=gf;}
        void SetHCostFunc(void (environment::*hf)(const state&, const state&) const){HCostFunc=hf;}
        void SetSuccessorFunc(void (environment::*sf)(const state&, std::vector<state>&) const){SuccessorFunc=sf;}
        void SetActionFunc(action (environment::*af)(const state&, const state&) const){ActionFunc=af;}
        void SetExternalExpansionsPtr(uint* ptr){totalExternalNodesExpanded=ptr;}
        void SetExternalExpansionLimit(uint limit){externalExpansionLimit=limit;}// std::cout << "Expansion limit set to: " << limit << "\n";}
private:
	uint64_t nodesTouched, nodesExpanded;
//	bool GetNextNode(state &next);
//	//state Node();
//	void UpdateClosedNode(environment *env, state& currOpenNode, state& neighbor);
//	void UpdateWeight(environment *env, state& currOpenNode, state& neighbor);
//	void AddToOpenList(environment *env, state& currOpenNode, state& neighbor);
	
	std::vector<state> neighbors;
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
	bool useOccupancyInfo;// = false;
	bool useRadius;// = false;
	int useBPMX;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<state> *theHeuristic;
        double (environment::*HCostFunc)(const state&, const state&) const;
        double (environment::*GCostFunc)(const state&, const state&) const;
        void (environment::*SuccessorFunc)(const state&, std::vector<state>&) const;
        action (environment::*ActionFunc)(const state&, const state&) const;
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
void TemporalAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath, double minTime)
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
template <class state, class action, class environment, class openList>
void TemporalAStar<state,action,environment,openList>::GetPaths(environment *_env, const state& from, const state& to, std::vector<std::vector<state>> &paths, double window, double bestf, double minTime)
{
  double nextbestf(0);
  if(openClosedList.OpenSize() == 0){
    std::vector<state> thePath;
    GetPath(_env,from,to,thePath,minTime);
    GetClosedListGCost(thePath.back(),bestf);
    paths.push_back(thePath);
  }else if(bestf<0){
    uint64_t key(openClosedList.Peek());
    bestf=openClosedList.Lookup(key).g+openClosedList.Lookup(key).h;
  }

  while(fgeq(bestf+window,nextbestf)){
    std::vector<state> thePath;
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
double TemporalAStar<state,action,environment,openList>::GetNextPath(environment *env, const state& from, const state& to, std::vector<state> &thePath, double minTime)
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
bool TemporalAStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath, double minTime)
{
	//lastF = 0;
	
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	//if (useRadius)
	//std::cout<<"Using radius\n";
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
bool TemporalAStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath, double minTime)
{
// Special hack... Don't consider paths that take too many expansions
if(this->nodesExpanded>1000 && this->noncritical){
  thePath.resize(0);
  noncritical=false;
  return true;
}
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

 	(env->*SuccessorFunc)(openClosedList.Lookup(nodeid).data, neighbors);
        if((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal))){
          if(fgeq(openClosedList.Lookup(nodeid).data.t,minTime))
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
              neighbors.insert(neighbors.begin(),n);
            }else{
              n.t-=1.0;
              if(fgreater(n.t,openClosedList.Lookup(nodeid).data.t)){
                neighbors.insert(neighbors.begin(),n);
              }
            }
          }
        }
	
	if(verbose)std::cout << "Expanding: " << openClosedList.Lookup(nodeid).data << " with f:" << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
	
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

		if (useBPMX)
		{
			if (neighborLoc.back() != kNotFound)
			{
				if (!directed)
					bestH = std::max(bestH, openClosedList.Lookup(theID).h-edgeCosts.back());
				lowHC = std::min(lowHC, openClosedList.Lookup(theID).h+edgeCosts.back());
			}
			else {
				if (!directed)
					bestH = std::max(bestH, h-edgeCosts.back());
				lowHC = std::min(lowHC, h+edgeCosts.back());
			}
		}
	}
	
	if (useBPMX) // propagate best child to parent
	{
		if (!directed)
			openClosedList.Lookup(nodeid).h = std::max(openClosedList.Lookup(nodeid).h, bestH);
		openClosedList.Lookup(nodeid).h = std::max(openClosedList.Lookup(nodeid).h, lowHC);
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
				if (useBPMX) // propagate parent to child - do this before potentially re-opening
				{
					if (fless(openClosedList.Lookup(neighborID[x]).h, bestH-edgeCosts[x]))
					{
						openClosedList.Lookup(neighborID[x]).h = bestH-edgeCosts[x]; 
						if (useBPMX > 1) FullBPMX(neighborID[x], useBPMX-1);
					}
				}
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
				else {
//					std::cout << " no cheaper \n";
				}
				if (useBPMX) // propagate best child to parent
				{
					if (fgreater(bestH-edgeCosts[x], openClosedList.Lookup(neighborID[x]).h))
					{
						openClosedList.Lookup(neighborID[x]).h = std::max(openClosedList.Lookup(neighborID[x]).h, bestH-edgeCosts[x]); 
						openClosedList.KeyChanged(neighborID[x]);
					}
				}
				break;
			case kNotFound:
				// node is occupied; just mark it closed
				if (useRadius && useOccupancyInfo && env->GetOccupancyInfo() && radEnv && (radEnv->HCost(start, neighbors[x]) < radius) &&(env->GetOccupancyInfo()->GetStateOccupied(neighbors[x])) && ((!(radEnv->GoalTest(neighbors[x], goal)))))
				{
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
					openClosedList.AddClosedNode(neighbors[x],
												 env->GetStateHash(neighbors[x]),
												 openClosedList.Lookup(nodeid).g+edgeCosts[x],
												 std::max(hCosts[x], openClosedList.Lookup(nodeid).h-edgeCosts[x]),
												 nodeid);
				}
				else { // add node to open list
					//double edgeCost = env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]);
//					std::cout << " adding to open ";
//					std::cout << double(theHeuristic->HCost(neighbors[x], goal)+openClosedList.Lookup(nodeid).g+edgeCosts[x]);
//					std::cout << " \n";
					if (useBPMX)
					{
						openClosedList.AddOpenNode(neighbors[x],
												   env->GetStateHash(neighbors[x]),
												   openClosedList.Lookup(nodeid).g+edgeCosts[x],
												   std::max(weight*hCosts[x], openClosedList.Lookup(nodeid).h-edgeCosts[x]),
												   nodeid);
					} else if(doPartialExpansion) {
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
 * Perform a full bpmx propagation
 * @author Nathan Sturtevant
 * @date 6/9/9
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment, class openList>
void TemporalAStar<state, action,environment,openList>::FullBPMX(uint64_t nodeID, int distance)
{
	if (distance <= 0)
		return;
	
	nodesExpanded++;
	std::vector<state> succ;
 	(env->*SuccessorFunc)(openClosedList.Lookup(nodeID).data, succ);
	double parentH = openClosedList.Lookup(nodeID).h;
	
	// load all the children and push parent heuristic value to children
	for (unsigned int x = 0; x < succ.size(); x++)
	{
		uint64_t theID;
		dataLocation loc = openClosedList.Lookup(env->GetStateHash(succ[x]), theID);
		double edgeCost = (env->*GCostFunc)(openClosedList.Lookup(nodeID).data, succ[x]);
		double newHCost = parentH-edgeCost;
		
		switch (loc)
		{
			case kClosedList:
			{
				if (fgreater(newHCost, openClosedList.Lookup(theID).h))
				{
					openClosedList.Lookup(theID).h = newHCost;
					FullBPMX(theID, distance-1);
				}
			}
			case kOpenList:
			{
				if (fgreater(newHCost, openClosedList.Lookup(theID).h))
				{
					openClosedList.Lookup(theID).h = newHCost;
					openClosedList.KeyChanged(theID);
				}
			}
			case kNotFound: break;
		}
	}
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
