/**
 * @file NAMOAStar.h
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

#ifndef NAMOAStar_H
#define NAMOAStar_H

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
// this is defined in stdint.h, but it doesn't always get defined correctly
// even when __STDC_CONSTANT_MACROS is defined before including stdint.h
// because stdint might be included elsewhere first...
#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#include <strings.h>
#include "FPUtil.h"
//#include <ext/hash_map>
#include "AStarOpenClosed.h"
#include "BucketOpenClosed.h"
//#include "SearchEnvironment.h" // for the SearchEnvironment class
#include "float.h"

#include <algorithm> // for vector reverse
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "GenericSearchAlgorithm.h"
#include "MultiObjective.h"

template <unsigned dim>
static std::ostream& operator<<(std::ostream& ss, cost<dim> const& v){
  ss << "<" << v.value[0];
  for(int i(1); i<dim; ++i){
    ss<<","<<v.value[i];
  }
  ss << ">";
  return ss;
}

template<typename state, typename environ, unsigned dim>
class NAMOAOpenClosedData {
public:
        NAMOAOpenClosedData() {}
        NAMOAOpenClosedData(state const& theData, cost<dim> gc, cost<dim> hc, uint64_t parent, uint64_t openLoc, dataLocation location)
        :data(theData), g(gc), h(hc), parentID(parent), openLocation(openLoc), reopened(false), visited(false), where(location){}
        NAMOAOpenClosedData(state const& theData, float dummy1, float dummy2, uint64_t parent, uint64_t openLoc, dataLocation location)
        :data(theData), parentID(parent), openLocation(openLoc), reopened(false), visited(false), where(location){}
        state data;
        cost<dim> g;
        cost<dim> h;
        uint64_t parentID; // For compatibility and "scalarized" multi-objective search; this is not used for lexicographic or regular dominance search
        // sets of parents and their non-dominated cost vectors reaching this node
	std::unordered_map<cost<dim>,std::unordered_set<uint64_t>> open;
	std::unordered_map<uint64_t,std::unordered_set<cost<dim>>> closed; // Note: this map is equivalient to COSTS (in the literature) for any goal node
        uint64_t openLocation;
        bool reopened;
        bool visited;
        dataLocation where;
};


template <class state, typename environ, unsigned dim>
struct NAMOAStarCompare {
  bool operator()(const NAMOAOpenClosedData<state,environ,dim> &i1, const NAMOAOpenClosedData<state,environ,dim> &i2) const
  {
    // Lexicographical comparison...
    for(int i(0); i<dim; ++i){
      if (fequal(i1.g.value[i]+i1.h.value[i], i2.g.value[i]+i2.h.value[i]))
      {
        // I don't think we care about tie breaking
        // at this level since we are returning a pareto-optimal set
        //if(!fequal(i1.g.value[i], i2.g.value[i]))
          //return (fless(i1.g.value[i], i2.g.value[i]));
      }else{
        return (fgreater(i1.g.value[i]+i1.h.value[i], i2.g.value[i]+i2.h.value[i]));
      }
    }
    // All F-costs are equal...
    // Tie break on values without considering hcost
    for(int i(0); i<dim; ++i){
      if(!fequal(i1.g.value[i], i2.g.value[i]))
      {
        return (fless(i1.g.value[i], i2.g.value[i]));
      }
    }
    return true; // They are equivalent. Just choose one.
  }
};

/**
 * A templated version of A*, based on HOG genericAStar
 */
template <class state, class action, class environment, unsigned dim, class openList = AStarOpenClosed<state, NAMOAStarCompare<state,environment,dim>, NAMOAOpenClosedData<state,environment,dim>>>
class NAMOAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	NAMOAStar():totalExternalNodesExpanded(nullptr),externalExpansionLimit(INT_MAX),verbose(false),noncritical(false),env(nullptr),stopAfterGoal(true),weight(1),reopenNodes(false),SuccessorFunc(&environment::GetSuccessors),ActionFunc(&environment::GetAction),GCostFunc(&environment::GCostVector),HCostFunc(&environment::HCostVector){ResetNodeCount();}
	virtual ~NAMOAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPaths(environment *env, const state& from, const state& to, std::vector<std::vector<state>>& paths);
	
	void GetPath(environment *, const state& , const state& , std::vector<action> & ) { assert(false); };
	
        inline openList* GetOpenList(){return &openClosedList;}
	openList openClosedList;

	//BucketOpenClosed<state, NAMOAStarCompare<state>, NAMOAOpenClosedData<state> > openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<std::vector<state>>& paths);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	void AddAdditionalStartState(state const& newState);
	void AddAdditionalStartState(state const& newState, cost<dim> const& cost);
	
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
	//bool GetClosedListGCost(const state &val, double &gCost) const;
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const NAMOAOpenClosedData<state,environment,dim> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const NAMOAOpenClosedData<state,environment,dim> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
	
	void SetFullSet(bool val) { fullSet = val; }
	bool GetFullSet() { return fullSet; }
	
	void SetLazyFiltering(bool val) { lazyFiltering = val; }
	bool GetLazyFiltering() { return lazyFiltering; }
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
	
	void OpenGLDraw() const;
	
	void SetWeight(double w) {weight = w;}
        void SetGCostFunc(std::vector<float> (environment::*gf)(const state&, const state&) const){GCostFunc=gf;}
        void SetHCostFunc(std::vector<float> (environment::*hf)(const state&, const state&) const){HCostFunc=hf;}
        void SetSuccessorFunc(void (environment::*sf)(const state&, std::vector<state>&) const){SuccessorFunc=sf;}
        void SetActionFunc(action (environment::*af)(const state&, const state&) const){ActionFunc=af;}
        void SetVerbose(bool v){verbose=v;}
        bool noncritical;
        void SetExternalExpansionsPtr(uint* ptr){totalExternalNodesExpanded=ptr;}
        void SetExternalExpansionLimit(uint limit){externalExpansionLimit=limit;}
        void SetHeuristic(Heuristic<state>*){}//dummy}
private:
        uint* totalExternalNodesExpanded;
        uint externalExpansionLimit;
        bool verbose;
	std::vector<state> succ;
        //std::pair<uint64_t,double> ComputeCost(NAMOAOpenClosedData<state,environment,dim> const& p, state& c, double newg, double oldg);
	uint64_t nodesTouched, nodesExpanded;
	environment *env;
	bool stopAfterGoal;
	bool fullSet; // Whether to return the first pareto-optimal path or "all" pareto-optimal paths
	bool lazyFiltering; // Whether to apply lazy filtering enhancement
	
	double weight; 
	
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
        std::vector<float> (environment::*HCostFunc)(const state&, const state&) const;
        std::vector<float> (environment::*GCostFunc)(const state&, const state&) const;
        void (environment::*SuccessorFunc)(const state&, std::vector<state>&) const;
        action (environment::*ActionFunc)(const state&, const state&) const;
	std::unordered_set<uint64_t> costs; // Set of goal nodes
};

//static const bool verbose = false;

/**
 * Return the name of the algorithm. 
 * @author Thayne Walker
 * @date 06/27/17
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, unsigned dim, class openList>
const char *NAMOAStar<state,action,environment,dim,openList>::GetName()
{
	static char name[32];
	sprintf(name, "NAMOAStar[]");
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
template <class state, class action, class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
        fullSet=false;
	//discardcount=0;
  	if (!InitializeSearch(_env, from, to, thePath))
  	{	
  		return;
  	}
  	while (!DoSingleSearchStep(thePath)) {}
}

template <class state, class action, class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::GetPaths(environment *_env, const state& from, const state& to, std::vector<std::vector<state>>& paths)
{
  fullSet=true;
  //discardcount=0;
  if (!InitializeSearch(_env, from, to, paths))
  {	
    uint64_t id;
    openClosedList.Lookup(env->GetStateHash(from),id);
    return;
  }
  std::vector<state> dummy;
  while (!DoSingleSearchStep(dummy)) {}

  uint64_t id;
  if(kNotFound!=openClosedList.Lookup(env->GetStateHash(to),id)){
    return &openClosedList.Lookup(id);
  }else{
    return nullptr;
  }
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
template <class state, class action, class environment, unsigned dim, class openList>
bool NAMOAStar<state,action,environment,dim,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
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
                thePath.push_back(from);
                thePath.push_back(to);
		return false;
	}
	
        NAMOAOpenClosedData<state,environment,dim> data(start,cost<dim>(),(env->*HCostFunc)(start, goal),kTAStarNoNode,0,kOpenList);
	openClosedList.AddOpenNode(data, env->GetStateHash(start));

        // Set the start state to be its own parent
        uint64_t id;
	openClosedList.Lookup(env->GetStateHash(start),id);
        //openClosedList.Lookup(id).parentID=id;
	//openClosedList.Print();
	
	return true;
}

template <class state, class action, class environment, unsigned dim, class openList>
bool NAMOAStar<state,action,environment,dim,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<std::vector<state>>& paths)
{
  env = _env;
  openClosedList.Reset();
  ResetNodeCount();
  start = from;
  goal = to;

  NAMOAOpenClosedData<state,environment,dim> data(start,cost<dim>(),(env->*HCostFunc)(start, goal),kTAStarNoNode,0,kOpenList);
  uint64_t id(openClosedList.AddOpenNode(data, env->GetStateHash(start)));
  data.open[data.g].insert(id);

  if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
  {
    //std::vector<state> tmp = {from};
    paths.push_back({from});
    return false;
  }

  return true;
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Thayne Walker
 * @date 06/27/17
 */
template <class state, class action, class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::AddAdditionalStartState(state const& newState)
{
        NAMOAOpenClosedData<state,environment,dim> data(newState,cost<dim>(),(env->*HCostFunc)(newState, goal),kTAStarNoNode,0,kOpenList);
	openClosedList.AddOpenNode(data, env->GetStateHash(newState));
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Thayne Walker
 * @date 06/27/17
 */
template <class state, class action, class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::AddAdditionalStartState(state const& newState, cost<dim> const& gcost)
{
        NAMOAOpenClosedData<state,environment,dim> data(newState,gcost,(env->*HCostFunc)(newState, goal),kTAStarNoNode,0,kOpenList);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState));
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
template <class state, class action, class environment, unsigned dim, class openList>
bool NAMOAStar<state,action,environment,dim,openList>::DoSingleSearchStep(std::vector<state> &thePath){
  if (openClosedList.OpenSize() == 0)
  {
    thePath.resize(0); // no path found!
    //closedList.clear();
    return true;
  }
  uint64_t nodeid(openClosedList.Close());
  auto gcost(openClosedList.Lookup(nodeid).g);

  // PATH SELECTION
  if(fullSet){
    // Move from Gopen to Gclosed
    for(auto const& v:openClosedList.Lookup(nodeid).open[gcost]){
      openClosedList.Lookup(nodeid).closed[v].insert(gcost);
    }
    openClosedList.Lookup(nodeid).open.erase(gcost);
  }

  // Lazy filtering is not good if you want to grab solutions of cost greater than C*,
  // for example, when you need solutions from various plateaus.
  if(lazyFiltering){
    // Check if any goal node has a dominating vector
    // There is no good way around this for pure dominance objectives :(
    for(uint64_t id: costs){
      auto const& gnode(openClosedList.Lookup(id));
      for(auto const& p:gnode.closed){
        for(auto const& cstar:p.second){
          if(cstar<gcost) return false;
        }
      }
    }
  }

  if (!openClosedList.Lookup(nodeid).reopened)
    uniqueNodesExpanded++;
  nodesExpanded++;

  if(totalExternalNodesExpanded){
    (*totalExternalNodesExpanded)++; // Increment external counter
    if(*totalExternalNodesExpanded>externalExpansionLimit){
      thePath.resize(0);
      return true;
    }
  }
  if(stopAfterGoal && env->GoalTest(openClosedList.Lookup(nodeid).data, goal)){
    // SOLUTION RECORDING
    if(fullSet){
      openClosedList.Lookup(nodeid).closed[openClosedList.Lookup(nodeid).parentID].insert(gcost);
      costs.insert(nodeid);
      return false;
    }else{
      ExtractPathToStartFromID(nodeid, thePath);
      // Path is backwards - reverse
      reverse(thePath.begin(), thePath.end()); 
      if(thePath.size() == 0&&verbose)std::cout<<"No answer\n";
      /*std::vector<float> t(dim);
        cost<dim> total(t);
        state* p(&thePath[0]);
        for(auto& n:thePath){
        total+=(env->*GCostFunc)(*p,n);
      //if(!((n.t >= total - .001) && (n.t <= total+.001)))std::cout << "Time is bad: ("<<total<<")" << *p << "-->" << n << "\n";
      p=&n;
      }*/
      return true;
    }
  }

  if(verbose)std::cout << "Expanding: "
    << openClosedList.Lookup(nodeid).data<<std::hex<<"("<<env->GetStateHash(openClosedList.Lookup(nodeid).data)<<")"<<std::dec
    << " with f:" << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;

  succ.resize(0);
  (env->*SuccessorFunc)(openClosedList.Lookup(nodeid).data, succ);
  //double fCost = openClosedList.Lookup(nodeid).h+openClosedList.Lookup(nodeid).g;

  nodesTouched++;
  uint64_t theID;

  for(unsigned x(0); x < succ.size(); x++)
  {
    // Note: If we were performing regular filtering, we would skip a successor if it has a dominated f-cost,
    // however, lazy filtering is better (above)
    cost<dim> newg(openClosedList.Lookup(nodeid).g+(env->*GCostFunc)(openClosedList.Lookup(nodeid).data, succ[x]));

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
          // Does this node dominate anything in Gopen?
          // If so, erase it from Gopen
          bool dominates(newg<openClosedList.Lookup(theID).g);
          if(fullSet&&!dominates){
            for(auto theG(openClosedList.Lookup(theID).open.cbegin()); theG!=openClosedList.Lookup(theID).open.cend(); /* no increment*/){
              if(newg<theG->first){
                dominates=true;
                openClosedList.Lookup(theID).open.erase(theG++);
              }else{
                ++theG;
              }
            }
          }
          if(dominates){
            if(verbose)std::cout << "Update node ("<<std::hex<<env->GetStateHash(succ[x])<<std::dec
              << ") " << succ[x]
                << " change G from " << openClosedList.Lookup(theID).g
                << " to " << newg << "\n";
            openClosedList.Lookup(theID).parentID = nodeid;
            openClosedList.Lookup(theID).g = newg;
            openClosedList.Lookup(theID).data = succ[x];
            openClosedList.KeyChanged(theID);
            if(fullSet)
              openClosedList.Lookup(theID).open[newg].insert(nodeid);
          }else if(fullSet){ // Does anything dominate this vector?
            bool dominated(false);
            for(auto const& theG:openClosedList.Lookup(theID).open){
              if(theG.first<newg){
                dominated=true;
                break;
              }
            }
            if(!dominated){
              // Neither dominates... This is a valid pareto-optimal path
              openClosedList.Lookup(theID).open[newg].insert(nodeid);
            }
          }
        }
        break;
      case kNotFound:
        {
          
          cost<dim> hcost((env->*HCostFunc)(succ[x],goal));
          if(verbose)std::cout << "Add node ("<<std::hex<<env->GetStateHash(succ[x])<<std::dec
            << ") to open " << succ[x]
              << newg << "+" << hcost
              << "=" << (newg+hcost) << "\n";
          NAMOAOpenClosedData<state,environment,dim> data(succ[x],
              newg,hcost,nodeid,0,kOpenList);
          if(fullSet)
            data.open[newg].insert(nodeid);

          openClosedList.AddOpenNode(data, env->GetStateHash(succ[x]));
        }
    }
    //usleep(10000);
    //OpenGLDraw();
  }
  return false;
}

/*template <class state, class action, class environment, unsigned dim, class openList>
std::pair<uint64_t,double> NAMOAStar<state,action,environment,dim,openList>::ComputeCost(NAMOAOpenClosedData<state,environment,dim> const& p, state& c, double newg, double oldg){
  uint64_t pid;
  openClosedList.Lookup(env->GetStateHash(p.data),pid);
  NAMOAOpenClosedData<state,environment,dim>& pp(openClosedList.Lookup(p.parentID));
  // Special cases for waiting actions
  if(pp.data.sameLoc(c))return{pid,newg}; // parent of parent is same as self
  if(p.data.sameLoc(c))return{pid,newg}; // parent is same as self
  if(p.data.sameLoc(pp.data))return{pid,newg}; // parent is same as self

  if(env->LineOfSight(pp.data,c)){
    if(verbose)std::cout << "  LOS " << pp.data << "-->" << c << "\n";
    newg=pp.g+(env->*GCostFunc)(pp.data,c);
    if(fless(newg,oldg)){
      if(verbose)std::cout << "  Change parent of " << c << " to " << pp.data << " from " << p.data <<" " << newg << "\n";
      return {p.parentID,newg};
    }
  }else if(fless(newg,oldg)){
    if(verbose)std::cout << "  NO LOS " << pp.data << "-->" << c << "\n";
    if(verbose)std::cout << "  Change gcost of " << c << " with parent " << p.data<<"("<<p.openLocation<<")" <<" to " << newg << "\n";
    return {pid,newg};
  }
  if(verbose)std::cout << "  NO LOS " << pp.data << "-->" << c << "\n";
  if(verbose)std::cout << "  Leave parent of " << c << " as " << p.data <<" " << newg << "\n";
  return {0,-1.0};
}
*/

/**
 * Returns the next state on the open list (but doesn't pop it off the queue). 
 * @author Thayne Walker
 * @date 06/27/17
 * 
 * @return The first state in the open list. 
 */
template <class state, class action, class environment, unsigned dim, class openList>
state NAMOAStar<state,action,environment,dim,openList>::CheckNextNode()
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
template <class state, class action,class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
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
template <class state, class action, class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::PrintStats()
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
template <class state, class action, class environment, unsigned dim, class openList>
int NAMOAStar<state,action,environment,dim,openList>::GetMemoryUsage()
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
/*template <class state, class action, class environment, unsigned dim, class openList>
bool NAMOAStar<state,action,environment,dim,openList>::GetClosedListGCost(const state &val, double &gCost) const
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
*/

/**
 * Draw the open/closed list
 * @author Thayne Walker
 * @date 06/27/17
 * 
 */
template <class state, class action, class environment, unsigned dim, class openList>
void NAMOAStar<state,action,environment,dim,openList>::OpenGLDraw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	if (openClosedList.OpenSize() > 0)
		top = openClosedList.Peek();
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const NAMOAOpenClosedData<state,environment,dim> &data = openClosedList.Lookat(x);
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

template <unsigned dim>
typename cost<dim>::CompareType cost<dim>::compareType=cost<dim>::LEXICOGRAPHIC;
template <unsigned dim>
float cost<dim>::goals[dim]={-1.0f};
template <unsigned dim>
float cost<dim>::weights[dim]={1.0};


#endif
