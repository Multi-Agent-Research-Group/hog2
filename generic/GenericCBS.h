//
//  GenericCBSUnits.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__GenericCBSUnits__
#define __hog2_glut__GenericCBSUnits__

#include <iostream>
#include <limits> 
#include <algorithm>
#include <map>

#include <queue>
#include <functional>
#include <vector>

#include "Unit.h"
#include "UnitGroup.h"
#include "ConstrainedEnvironment.h"
#include "TemplateAStar.h"
#include "BFS.h"
#include "Heuristic.h"

template<class State>
struct TimedState : public State {
	TimedState(State loc, float time) : State(loc), t(time) {}
	TimedState() : State(), t(0) {}
	float t;
}

template<class State, class Action>
class GenericCBSUnit : public Unit<TimedState<State>, Action, ConstrainedEnvironment<TimedState<State>, Action>> {
public:
	GenericCBSUnit(const TimedState<State> &s, const TimedState<State> &g) :start(s), goal(g), current(s) {}

	const char *GetName() { 
		return "GenericCBSUnit"; 
	}

	virtual bool MakeMove(Environment *, OccupancyInterface<TimedState<State>,Action> *, SimulationInfo<TimedState<State>,Action,Environment> *, Action& a)
	{
		if (myPath.size() > 1 && si->GetSimulationTime() > myPath[myPath.size()-2].t)
		{
			a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
			myPath.pop_back();
			return true;
		}
		return false;
	}

	virtual void UpdateLocation(Environment *, TimedState<State> &newLoc, bool success, SimulationInfo<TimedState<State>,Action,Environment> *)
	{ 
		if (success) current = newLoc; else assert(!"CBS Unit: Movement failed"); 
	}
	
	virtual void GetLocation(TimedState<State> &l) 
	{ 
		l = current; 
	}

	virtual void OpenGLDraw(const Environment * env, const SimulationInfo<TimedState<State>,Action,Environment> * si) const
	{
		GLfloat r, g, b;
		GetColor(r, g, b);
		env->SetColor(r, g, b);
		env->OpenGLDraw(this->current);
	}

	virtual void GetGoal(TimedState<State> &s)
	{ 
		s = goal; 
	}

	virtual void GetStart(TimedState<State> &s) 
	{
		s = start;
	}
	
	virtual void SetPath(std::vector<TimedState<State>> &p)
	{
		myPath = p;
		std::reverse(myPath.begin(), myPath.end());
	}
	
    virtual void UpdateGoal(TimedState<State> &start, TimedState<State> &goal)
    {
		start = s;
		goal = g;
    }

    inline std::vector<TimedState<State>> const& GetPath()const{return myPath;}

protected:
	TimedState<State> start, goal, current;
	std::vector<TimedState<State>> myPath;
};

template<class State>
struct Conflict {
	Constraint<TimedState<State>> c;
	int unit1;
};

template<class State>
struct GenericCBSTreeNode {
	GenericCBSTreeNode():parent(0),satisfiable(true){}
	std::vector< std::vector<TimedState<State> > > paths;
	Conflict<TimedState<State> > con;
	unsigned int parent;
	bool satisfiable;
};

template<class State, class Action>
struct EnvironmentContainer {
	EnvironmentContainer() : name("NULL ENV"), environment(0), heuristic(0), conflict_cutoff(0), astar_weight(0.0f) {}
	EnvironmentContainer(std::string n, ConstrainedEnvironment<TimedState<State>, Action>* e, Heuristic<TimedState<State>>* h, uint32_t conf, float a) : name(n), environment(e), heuristic(h), conflict_cutoff(conf), astar_weight(a) {}
	ConstrainedEnvironment<TimedState<State>, Action>* environment;
	Heuristic<TimedState<State>>* heuristic;
	uint64_t conflict_cutoff;
	float astar_weight;
	std::string name;
};


template<class State, class Action, class UnitType>
class GenericCBSGroup : public UnitGroup<TimedState<State>, Action, ConstrainedEnvironment<TimedState<State>, Action>>
{
public:
	GenericCBSGroup(ConstrainedEnvironment<TimedState<State>, Action> *me, ConstrainedEnvironment<TimedState<State>, Action>* simple, unsigned threshold) : time(0), bestNode(0), planFinished(false)
	{
		tree.resize(1);
		tree[0].parent = 0;
		// Construct a simple environment container
		EnvironmentContainer<TimedState<State>, Action> e_simple("Simple", simple, new ManhattanHeuristic<TimedState<State>>(), 0, 1.2);
		this->environments.push_back(e_simple);
		// Construct a complex environment container
		EnvironmentContainer<TimedState<State>, Action> e_complex("Complex", me, new OctileDistanceHeuristic<TimedState<State>>(), threshold, 1.8);
		this->environments.push_back(e_complex);
		// Sort the environment container by the number of conflicts
		std::sort(this->environments.begin(), this->environments.end(), 
			[](const EnvironmentContainer<TimedState<State>, Action>& a, const EnvironmentContainer<TimedState<State>, Action>& b) -> bool 
				{
					return a.conflict_cutoff < b.conflict_cutoff;
				}
		);
		// Set the current environment to that with 0 conflicts
	    SetEnvironment(0);
	}


	bool MakeMove(Unit<TimedState<State>, Ac, ConstrainedEnvironment<TimedState<State>, Action>> *u, ConstrainedEnvironment<TimedState<State>, Action> *e, SimulationInfo<TimedState<State>,Action,ConstrainedEnvironment<TimedState<State>, Action>> *si, Action& a)
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

	void UpdateLocation(Unit<TimedState<State>, Action, ConstrainedEnvironment<TimedState<State>, Action>> *u, ConstrainedEnvironment<TimedState<State>, Action> *e, TimedState<State> &loc, bool success, SimulationInfo<TimedState<State>,Action,ConstrainedEnvironment<TimedState<State>, Action>> *si)
	{
		u->UpdateLocation(e, loc, success, si);
	}

	void AddUnit(Unit<TimedState<State>, Action, ConstrainedEnvironment<TimedState<State>, Action>> *u)
	{
		// Add the new unit to the group, and construct an AirCBSUnit
		UnitGroup::AddUnit(u);
		UnitType *c = (UnitType*)u;

		// Clear the constraints from the environment set
		ClearEnvironmentConstraints();

		// Setup the TimedState<State> and goal in the graph
		TimedState<State> start, goal;
		c->GetStart(start);
		c->GetGoal(goal);

		// Resize the number of paths in the root of the tree
		tree[0].paths.resize(GetNumMembers());

		// Recalculate the optimum path for the root of the tree
		astar.GetPath(currentEnvironment->environment, start, goal, thePath);

		// We add the optimal path to the root of the tree
		for (unsigned int x = 0; x < thePath.size(); x++)
		{
			tree[0].paths.back().push_back(thePath[x]);
		}
		
		// Set the plan finished to false, as there's new updates
		planFinished = false;

		// Clear up the rest of the tree and clean the open list
		tree.resize(1);
		while(!openList.empty()) openList.pop();
		openList.push(OpenListNode(0, 0));
	}

	void UpdateUnitGoal(Unit<TimedState<State>, Action, ConstrainedEnvironment<TimedState<State>, Action>> *u, TimedState<State> newGoal)
	{
		// Clear the tree and the open list
		tree.resize(1);
		while(!openList.empty()) openList.pop();

		// Resize the number of paths in the root of the tree
		tree[0].paths.resize(GetNumMembers());

		// Clear the constraints from the environment set
		ClearEnvironmentConstraints();

		// Update the start for all of the units
		for (int x = 0; x < GetNumMembers(); x++)
		{
			// Get the unit
			UnitType *c = (UnitType*)GetMember(x);

			// Obtain the unit's current location and current goal
			TimedState<State> current, goal;
			c->GetLocation(current);
			c->GetGoal(goal);

			// Update the start of that unit to be their current location and the goal to be the new goal
			c->UpdateGoal(current, (GetMember(x) != u ? goal : newGoal));

			// Replan the unit's optimal path
			astar.GetPath(currentEnvironment->environment, current, goal, thePath);

			// Add the optimal path to the root of the tree
			for (unsigned int i = 0; i < thePath.size(); i++)
			{
				tree[0].paths[x].push_back(thePath[i]);
			}
		}

		// Add the root of the node to the open list
		openList.push(OpenListNode(0, 0));

		// Clean up the root node
		tree[0].parent = 0;
		tree[0].satisfiable = true;
		bestNode = 0;

		// Set if the plan is finished to false
		planFinished = false;
	}

	void OpenGLDraw(const ConstrainedEnvironment<TimedState<State>, Action> *, const SimulationInfo<TimedState<State>,Action,ConstrainedEnvironment<TimedState<State>, Action>> *)  const {}
	double getTime() {return time;}
	bool donePlanning() {return planFinished;}

private:    

	void ExpandOneCBSNode()
	{
		// There's no reason to expand if the plan is finished.
		if (planFinished)
			return;

		Conflict<TimedState<State>> c1, c2;
		bool found = FindFirstConflict(bestNode, c1, c2);

		// If not conflicts are found in this node, then the path is done
		if (!found)
		{
			planFinished = true;
			
			// For every unit in the node
			for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
			{
				// Grab the unit
				UnitType *unit = (UnitType*) GetMember(x);
				
				// Prune these paths to the current simulation time
				TimedState<State> current;
				unit->GetLocation(current);
				std::vector<TimedState<State>> newPath;
				newPath.push_back(current); // Add the current simulation node to the new path

				// For everything in the path that's new, add the path back
				bool seen_current = false;
				for (TimedState<State> xNode : tree[bestNode].paths[x]) {
					if (seen_current) {
						newPath.push_back(xNode);
					} else if (current == xNode){
						seen_current = true;
					}
				}

				// Update the actual unit path
				unit->SetPath(newPath);
			}
			return;
		} 

		// Otherwise, we need to deal with the conflicts
		else {
			// Add two nodes to the tree for each of the children
			unsigned long last = tree.size();
			tree.resize(last+2);
			
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

			// Add the new nodes to the open list
			double cost = 0;
			for (int y = 0; y < tree[last].paths.size(); y++)
					cost += currentEnvironment->environment->GetPathLength(tree[last].paths[y]);
			OpenListNode l1(last, cost);
			openList.push(l1);

			cost = 0;
			for (int y = 0; y < tree[last+1].paths.size(); y++)
					cost += currentEnvironment->environment->GetPathLength(tree[last+1].paths[y]);
			OpenListNode l2(last, cost);
			openList.push(l2);

			// Get the best node from the top of the open list, and remove it from the list
			bestNode = openList.top().location;
			openList.pop();	
		}
	}
	void Replan(int location)
	{
		// Select the unit from the tree with the new constraint
		int theUnit = tree[location].con.unit1;

		// Reset the constraints in the test-environment
		ClearEnvironmentConstraints();

		// Add all of the constraints in the parents of the current node to the environment
		int tempLocation = location;
	    unsigned numConflicts = 0;
	    do {
			if (theUnit == tree[tempLocation].con.unit1)
	        {
	          numConflicts++;
	          AddEnvironmentConstraint(tree[tempLocation].con.c);     
	        }
			tempLocation = tree[tempLocation].parent;
		} while (tempLocation != 0);
	    
	    // Set the environment based on the number of conflicts
	    SetEnvironment(numConflicts);

		// Select the air unit from the group
		UnitType *c = (UnitType*)GetMember(theUnit);
		
		// Retreive the unit start and goal
		TimedState<State> start, goal;
		c->GetStart(start);
		c->GetGoal(goal);

		// Recalculate the path
		astar.GetPath(currentEnvironment->environment, start, goal, thePath);

		// Make sure that the current location is satisfiable
		if (thePath.size() == 0 && !(goal == start)) {
			tree[location].satisfiable = false;
		}

		// Add the path back to the tree (new constraint included)
		tree[location].paths[theUnit].resize(0);
		for (unsigned int x = 0; x < thePath.size(); x++)
		{
			tree[location].paths[theUnit].push_back(thePath[x]);
		}
	}

	bool FindFirstConflict(int location, Conflict<TimedState<State>> &c1, Conflict<TimedState<State>> &c2)
	{
		// For each pair of units in the group
		for (int x = 0; x < GetNumMembers(); x++)
		{
			for (int y = x+1; y < GetNumMembers(); y++)
			{

				// Unit 1 path is tree[location].paths[x]
				// Unit 2 path is tree[location].paths[y]

				// To check for conflicts, we loop through the timed actions, and check 
				// each bit to see if a constraint is violated
				int xmax = tree[location].paths[x].size();
				int ymax = tree[location].paths[y].size();

				//std::cout << "Checking for conflicts between: "<<x << " and "<<y<<" ranging from:" << xmax <<"," << ymax <<"\n";

				for (int i = 0, j = 0; j < ymax && i < xmax;) // If we've reached the end of one of the paths, then time is up and 
																// no more conflicts could occur
				{
					// I and J hold the current step in the path we are comparing. We need 
					// to check if the current I and J have a conflict, and if they do, then
					// we have to deal with it, if not, then we don't.
					
					// Figure out which indices we're comparing
					int xTime = max(0, min(i, xmax-1));
					int yTime = max(0, min(j, ymax-1));

					// Check the point constraints
					Constraint<TimedState<State>> x_c(tree[location].paths[x][xTime]);
					TimedState<State> y_c = tree[location].paths[y][yTime];

					// Check the vertex conflict
					if (x_c.ConflictsWith(y_c))
					{
						c1.c = x_c;
						c2.c = y_c;

						c1.unit1 = x;
						c2.unit1 = y;
						return true;
					}

					// Check the edge conflicts
					Constraint<TimedState<State>> x_e_c(tree[location].paths[x][xTime], tree[location].paths[x][min(xmax-1, xTime+1)]);
					Constraint<TimedState<State>> y_e_c(tree[location].paths[y][yTime], tree[location].paths[y][min(ymax-1, yTime+1)]);
					
					if (x_e_c.ConflictsWith(y_e_c))
					{
						c1.c = x_e_c;
						c2.c = y_e_c;

						c1.unit1 = x;
						c2.unit1 = y;
						return true;
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
					if (tree[location].paths[x][min(xmax-1, xTime+1)].t < tree[location].paths[y][min(ymax-1, yTime+1)].t)
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
			} // End Unit 2 loop
		} // End Unit 1 Loop
		}
		
	bool planFinished;

	/* Code for dealing with multiple environments */
	std::vector<EnvironmentContainer<TimedState<State>, Action>> environments;
	EnvironmentContainer<TimedState<State>, Action>* currentEnvironment;

	void SetEnvironment(unsigned)
	{
		for (int i = 0; i < this->environments.size(); i++) {
			if (numConflicts >= environments[i].conflict_cutoff) {
				currentEnvironment = &(environments[i]);
			} else {
				break;
			}
		}

		astar.SetHeuristic(currentEnvironment->heuristic);
		astar.SetWeight(currentEnvironment->astar_weight);
	}
    void ClearEnvironmentConstraints()
    {
    	for (EnvironmentContainer env : this->environments) {
			env.environment->ClearConstraints();
		}
    }
    void AddEnvironmentConstraint(Constraint<TimedState<State>> c)
    {
    	for (EnvironmentContainer env : this->environments) {
			env.environment->AddConstraint(c);
		}
    }

	std::vector<GenericCBSTreeNode<TimedState<State>>> tree;
	std::vector<TimedState<State>> thePath;
	TemplateAStar<TimedState<State>, Action, ConstrainedEnvironment<TimedState<State>, Action>> astar;

	double time;

	unsigned int bestNode;
	struct OpenListNode {
		OpenListNode() : location(0), cost(0) {}
		OpenListNode(uint loc, double c) : location(loc), cost(c) {}
		uint location;
		double cost;	
	};
	struct OpenListNodeCompare {
		bool operator() (const OpenListNode& left, const OpenListNode& right) {
			return left.cost > right.cost;
		}
	};

	std::priority_queue<GenericCBSGroup::OpenListNode, std::vector<GenericCBSGroup::OpenListNode>, AirCBSGroup::OpenListNodeCompare> openList;
};


#endif /* defined(__hog2_glut__AirplaneCBSUnits__) */
