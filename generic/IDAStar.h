/*
 *  IDAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef IDASTAR_H
#define IDASTAR_H

#include <iostream>
#include <functional>
#include <unordered_map>
#include "FPUtil.h"
#include "vectorCache.h"

//#define DO_LOGGING

template <class state, class action, class environment>
class IDAStar {
public:
	IDAStar():incumbentcost(9999999999.9){ useHashTable = usePathMax = false; storedHeuristic = false;}
	virtual ~IDAStar() {}
	void GetPath(environment *env, state const& from, state const& to, std::vector<state> &thePath);
	void GetPath(environment *env, state from, state to, std::vector<action> &thePath);

	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void SetUseBDPathMax(bool val) { usePathMax = val; }
	void SetHeuristic(Heuristic<state> *heur) { heuristic = heur; if (heur != 0) storedHeuristic = true;}
private:
        std::vector<state> incumbent;
        double best;
        double incumbentcost;
	unsigned long long nodesExpanded, nodesTouched;
	
	double DoIteration(environment *env,
					   state parent, state currState,
					   std::vector<state> &thePath, double bound, double g,
					   double maxH);
	double DoIteration(environment *env,
					   action forbiddenAction, state &currState,
					   std::vector<action> &thePath, double bound, double g,
					   double maxH, double parentH);
	void PrintGHistogram()
	{
//		uint64_t early = 0, late = 0;
//		for (int x = 0; x < gCostHistogram.size(); x++)
//		{
//			printf("%d\t%llu\n", x, gCostHistogram[x]);
//			if (x*2 > gCostHistogram.size()-1)
//				late += gCostHistogram[x];
//			else
//				early += gCostHistogram[x];
//		}
//		if (late < early)
//			printf("Strong heuristic - Expect MM > A*\n");
//		else
//			printf("Weak heuristic - Expect MM >= MM0.\n");
	}
	void UpdateNextBound(double currBound, double fCost);
	state goal;
	double nextBound;
	bool usePathMax;
	bool useHashTable;
	vectorCache<action> actCache;
	bool storedHeuristic;
	Heuristic<state> *heuristic;
	std::vector<uint64_t> gCostHistogram;
	std::map<uint32_t,uint64_t> fCostHistogram;
        std::unordered_map<std::string,bool> transTable;

#ifdef DO_LOGGING
public:
	std::function<void (state, int)> func;
#endif
};

template <class state, class action, class environment>
void IDAStar<state, action, environment>::GetPath(environment *env,
						 state const& from, state const& to,
						 std::vector<state> &thePath)
{
	if (!storedHeuristic)
		heuristic = env;
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	//UpdateNextBound(0, heuristic->HCost(from, to));
	fCostHistogram[uint32_t(heuristic->HCost(from, to)*1000)]=1;
	goal = to;
	thePath.push_back(from);
	while (true) //thePath.size() == 0)
	{
                unsigned total(0);
                auto cost(fCostHistogram.begin());
                auto pBound(nextBound);

                best=0;
                while((nextBound-pBound)<.99 && cost!=fCostHistogram.end()){
                  total+=cost->second;
                  nextBound=cost->first/1000.0;
                  if(!best && fless(pBound,nextBound)){
                    best=nextBound;
                  }
                  ++cost;
                }
		//nodeTable.clear();
		fCostHistogram.clear();
		gCostHistogram.clear();
                transTable.clear();
		gCostHistogram.resize(nextBound+1);
		printf("Starting iteration with bound %f best=%f\n", nextBound,best);
		if (DoIteration(env, from, from, thePath, nextBound, 0, 0) == 0)
			break;
                if(incumbent.size()){
                  thePath=incumbent;
                  break;
                }
		PrintGHistogram();
	}
	PrintGHistogram();
}

template <class state, class action, class environment>
void IDAStar<state, action, environment>::GetPath(environment *env,
									 state from, state to,
									 std::vector<action> &thePath)
{
	if (!storedHeuristic)
		heuristic = env;
	nextBound = 0;
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);

	if (env->GoalTest(from, to))
		return;

	double rootH = heuristic->HCost(from, to);
	UpdateNextBound(0, rootH);
	goal = to;
	std::vector<action> act;
	env->GetActions(from, act);
	while (thePath.size() == 0)
	{
		//nodeTable.clear();
		gCostHistogram.clear();
		gCostHistogram.resize(nextBound+1);
		printf("Starting iteration with bound %f; %llu expanded, %llu generated\n", nextBound, nodesExpanded, nodesTouched);
		fflush(stdout);
		DoIteration(env, act[0], from, thePath, nextBound, 0, 0, rootH);
		PrintGHistogram();
	}
}

template <class state, class action, class environment>
double IDAStar<state, action, environment>::DoIteration(environment *env,
    state parent, state currState,
    std::vector<state> &thePath, double bound, double g,
    double maxH)
{
        // Compute hash for transposition table
        std::string hash(currState.size()*sizeof(uint64_t),1);
        int k(0);
        for(auto v:currState){
          uint64_t h1(env->GetStateHash(v));
          uint8_t c[sizeof(uint64_t)];
          memcpy(c,&h1,sizeof(uint64_t));
          for(unsigned j(0); j<sizeof(uint64_t); ++j){
            hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
          }
          ++k;
        }

        //if(verbose)std::cout << "saw " << s << " hash ";
        //if(verbose)for(unsigned int i(0); i<hash.size(); ++i){
        //std::cout << (unsigned)hash[i]<<" ";
        //}
        //if(verbose)std::cout <<"\n";
        if(transTable.find(hash)!=transTable.end()){
          //std::cout << "AGAIN!\n";
          return 999999999.9;
          //if(!transTable[hash]){return false;}
        }

        // path max
        //if (usePathMax && fless(h, maxH))
        //h = maxH;
        //std::cout << "EVAL " << currState << "\n";
        if (env->GoalTest(currState, goal))
		return 0;
		
	std::vector<state> neighbors;
	env->GetSuccessors(currState, neighbors);
	nodesTouched += neighbors.size();
	nodesExpanded++;
	gCostHistogram[g]++;
        //std::cout << "Neightbors:\n";
        //for(auto const& n:neighbors){
          //std::cout << n << "\n";
        //}

	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (neighbors[x] == parent)
			continue;
                double h = heuristic->HCost(neighbors[x], goal);
		double edgeCost = env->GCost(currState, neighbors[x]);
                double f(g+edgeCost+h);
                if(fgreater(f, bound)){
                  fCostHistogram[uint32_t(f*1000)]++;
                  //std::cout << "Ignore " << neighbors[x] << " " << g+edgeCost+h << "\n";
                  continue;
                }
                //std::cout << "Investigate " << neighbors[x] << " " << g+edgeCost+h << "\n";
		thePath.push_back(neighbors[x]);
		DoIteration(env, currState, neighbors[x], thePath, bound, g+edgeCost, maxH - edgeCost);
                if (env->GoalTest(thePath.back(), goal)){
                  //std::cout << "Found goal:\n";
                  //for(auto const& r:thePath){
                    //std::cout << r << "\n";
                  //}
                  if(fless(f,incumbentcost)){
                    incumbent=thePath;
                    incumbentcost=f;
                    std::cout << "incumbent: " << incumbentcost << "\n";
                  }
                  // Return immediately if no better answer is possible
                  if(fleq(f,best+1/1000.)){
                    return 0;
                  }
                }
                transTable[hash]=false;
		thePath.pop_back();
		// pathmax
		//if (usePathMax && fgreater(childH-edgeCost, h))
		//{
//			nodeTable[currState] = g;//+h
			//h = childH-edgeCost;
			//if (fgreater(g+h, bound))
			//{
				//UpdateNextBound(bound, g+h);
				//return h;
			//}
		//}
	}
	return 1;
}

template <class state, class action, class environment>
double IDAStar<state, action, environment>::DoIteration(environment *env,
										   action forbiddenAction, state &currState,
										   std::vector<action> &thePath, double bound, double g,
										   double maxH, double parentH)
{
	double h = heuristic->HCost(currState, goal);//, parentH); // TODO: restore code that uses parent h-cost
	parentH = h;
	// path max
	if (usePathMax && fless(h, maxH))
		h = maxH;
	if (fgreater(g+h, bound))
	{
		UpdateNextBound(bound, g+h);
		//printf("Stopping at (%d, %d). g=%f h=%f\n", currState>>16, currState&0xFFFF, g, h);
		return h;
	}
	// must do this after we check the f-cost bound
	if (env->GoalTest(currState, goal))
		return -1; // found goal
	
	std::vector<action> &actions = *actCache.getItem();
	env->GetActions(currState, actions);
	nodesTouched += actions.size();
	nodesExpanded++;
	gCostHistogram[g]++;
	int depth = (int)thePath.size();
#ifdef t
	func(currState, depth);
#endif
	
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		if ((depth != 0) && (actions[x] == forbiddenAction))
			continue;

		thePath.push_back(actions[x]);
		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		action a = actions[x];
		env->InvertAction(a);

		double childH = DoIteration(env, a, currState, thePath, bound,
									g+edgeCost, maxH - edgeCost, parentH);
		env->UndoAction(currState, actions[x]);
		if (fequal(childH, -1)) // found goal
		{
			actCache.returnItem(&actions);
			return -1;
		}

		thePath.pop_back();

		// pathmax
		if (usePathMax && fgreater(childH-edgeCost, h))
		{
			//			nodeTable[currState] = g;//+h
			h = childH-edgeCost;
			if (fgreater(g+h, bound))
			{
				UpdateNextBound(bound, g+h);
				actCache.returnItem(&actions);
				return h;
			}
		}
	}
	actCache.returnItem(&actions);
	return h;
}


template <class state, class action, class environment>
void IDAStar<state, action, environment>::UpdateNextBound(double currBound, double fCost)
{
	if (!fgreater(nextBound, currBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
	else if (fgreater(fCost, currBound) && fless(fCost, nextBound))
	{
		nextBound = fCost;
		//printf("Updating next bound to %f\n", nextBound);
	}
}


#endif
