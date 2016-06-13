//
//  AirCBSUnits.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneCBSUnits.h"

/** AIR CBS UNIT DEFINITIONS */

bool AirCBSUnit::MakeMove(AirplaneEnvironment *ae, OccupancyInterface<airplaneState,airplaneAction> *,
							 SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *, airplaneAction& a)
{
	if (myPath.size() > 1)
	{
		std::cout << "Moved from " << myPath[myPath.size()-1] << " to " << myPath[myPath.size()-2] << std::endl;
		a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
		myPath.pop_back();
		return true;
	}
	return false;
}
	
void AirCBSUnit::SetPath(std::vector<airplaneState> &p)
{
	myPath = p;
	std::reverse(myPath.begin(), myPath.end());
}

void AirCBSUnit::OpenGLDraw(const AirplaneEnvironment *ae, 
							const SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *) const
{
	GLfloat r, g, b;
	GetColor(r, g, b);
	ae->SetColor(r, g, b);
	ae->OpenGLDraw(current);
}

/** CBS GROUP DEFINITIONS */

AirCBSGroup::AirCBSGroup(AirplaneEnvironment *ae)
{
	std::cout << "Constructed an AirCBSGroup" << std::endl;
	this->ae = ae;
	this->a2e = new AirplaneConstrainedEnvironment(ae);
	planFinished = false;
	time = 0;
	tree.resize(1);
	tree[0].parent = 0;
	bestNode = 0;
	// Removed the overlay from the Map-2D example, will have to 
	// add back a similar concept
}

/** Make a move on the airplane environment for a given unit */
std::vector<bool> AirCBSGroup::MakeMoveAllUnits(AirplaneEnvironment *e, SimulationInfo<airplaneState, airplaneAction, AirplaneEnvironment> *si, std::vector<airplaneAction> &a)
{
	std::vector<bool> ret;
	a.resize(0);

	if (planFinished && si->GetSimulationTime() > time)
	{
		time += 1;
		for (unsigned int x = 0; x < GetNumMembers(); x++)
		{
			std::cout << "Moving member:" << x << std::endl;
			Unit<airplaneState, airplaneAction, AirplaneEnvironment> *unit = GetMember(x);
			airplaneAction act;
			bool val = unit->MakeMove(e, 0, si, act);
			ret.push_back(val);
			a.push_back(act);
		}
	}
	else if ((si->GetSimulationTime() - time) < 0.0001) {
		for (unsigned int x = 0; x < GetNumMembers(); x++)
			ret.push_back(false);
		return ret;
	}
	else 
	{
		time = si->GetSimulationTime();
		ExpandOneCBSNode();
	}
}


bool AirCBSGroup::MakeMove(Unit<airplaneState, airplaneAction, AirplaneEnvironment> *u, AirplaneEnvironment *e,
							 SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *si, airplaneAction& a)
{
	
	if (planFinished && si->GetSimulationTime() > time)
	{
		time += 1;
		return u->MakeMove(e, 0, si, a);
	}
	else if ((si->GetSimulationTime() - time) < 0.0001)
	{
		return false;
	}
	else {
		time = si->GetSimulationTime();
		// expand 1 CBS node
		ExpandOneCBSNode();
	}
	return false;
}

/** Expand a single CBS node */
void AirCBSGroup::ExpandOneCBSNode()
{
	airConflict c1, c2;
	bool found = FindFirstConflict(bestNode, c1, c2);

	// If not conflicts are found in this node, then the path is done
	if (!found)
	{
		std::cout << "Finished the plan..." << std::endl;
		planFinished = true;
		for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
		{
			AirCBSUnit *unit = (AirCBSUnit*) GetMember(x);
			unit->SetPath(tree[bestNode].paths[x]);
			for (airplaneState p: tree[bestNode].paths[x])
			{
				std::cout << p << " ";		
			}std::cout << std::endl;
		}
		return;
	}
	
	// Otherwise, we add two nodes to the tree for each of the children
	unsigned long last = tree.size();
	tree.resize(last+2);
	
	// The fist node contains the conflict c1
	tree[last] = tree[bestNode];
	tree[last].con = c1;
	tree[last].parent = bestNode;

	// The second node constains the conflict c2
	tree[last+1] = tree[bestNode];
	tree[last+1].con = c2;
	tree[last+1].parent = bestNode;
	
	// We set the best-node to closed, as we have already expanded it
	tree[bestNode].closed = true;
	
	// We now replan in the tree
	Replan(last);
	Replan(last+1);
	
	// Finally, we select the best cost node from the search tree
	// There's a huge waste of time doing this with a linear search
	// we should be using a priority queue - but I really just want this
	// to work first
	double bestCost = std::numeric_limits<double>::max();
	double cost = 0;
	for (unsigned int x = 0; x < tree.size(); x++)
	{
		if (tree[x].closed)
			continue;
		cost = 0;
		for (int y = 0; y < tree[x].paths.size(); y++)
			cost += ae->GetPathLength(tree[x].paths[y]);
		if (cost < bestCost)
		{
			bestNode = x;
			bestCost = cost;
		}
	}
	std::cout << "New best node " << bestNode << std::endl;
}

/** Update the location oof a unit */
void AirCBSGroup::UpdateLocation(Unit<airplaneState, airplaneAction, AirplaneEnvironment> *u, AirplaneEnvironment *e, airplaneState &loc, 
									bool success, SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *si)
{
	u->UpdateLocation(e, loc, success, si);
}

/** Add a new unit with a new start and goal state to the CBS group */
void AirCBSGroup::AddUnit(Unit<airplaneState, airplaneAction, AirplaneEnvironment> *u)
{
	// Add the new unit to the group, and construct an AirCBSUnit
	UnitGroup::AddUnit(u);
	AirCBSUnit *c = (AirCBSUnit*)u;

	// Clear the constraints from the constrained-environment
	a2e->ClearConstraints();

	// Setup the state and goal in the graph
	airtimeState start, goal;
	c->GetStart(start.l);
	c->GetGoal(goal.l);
	start.t = 0;
	goal.t = 0; // Maybe we will change this in the future

	// Resize the number of paths in the root of the tree
	tree[0].paths.resize(GetNumMembers());

	// Recalculate the optimum path for the root of the tree
	astar.GetPath(a2e, start, goal, thePath);

	// We add the optimal path to the root of the tree
	for (unsigned int x = 0; x < thePath.size(); x++)
	{
		tree[0].paths.back().push_back(thePath[x].l);
	}
}

/** Replan a state given a constraint */
void AirCBSGroup::Replan(int location)
{
	// Select the unit from the tree with the new constraint
	int theUnit = tree[location].con.unit1;

	// Reset the constraints in the test-environment
	a2e->ClearConstraints();

	// Add all of the constraints in the parents of the current
	// node to the environment
	int tempLocation = location;
	while (tempLocation != 0)
	{
		if (theUnit == tree[tempLocation].con.unit1)
			a2e->AddConstraint(tree[tempLocation].con.c);
		tempLocation = tree[tempLocation].parent;
		//TODO: Find constraints on the goals of the agents (need heading and time)
	}

	// Select the air unit from the group
	AirCBSUnit *c = (AirCBSUnit*)GetMember(theUnit);
	airtimeState start, goal;
	c->GetStart(start.l);
	c->GetGoal(goal.l);
	start.t = 0;
	goal.t = 1; // TODO: find the true goal time

	// Recalculate the path
	astar.GetPath(a2e, start, goal, thePath);

	// Add it back to the tree (new constraint included)
	tree[location].paths[theUnit].resize(0);
	for (unsigned int x = 0; x < thePath.size(); x++)
	{
		tree[location].paths[theUnit].push_back(thePath[x].l);
	}
}

/** Find the first place that there is a conflict in the tree */
bool AirCBSGroup::FindFirstConflict(int location, airConflict &c1, airConflict &c2)
{

	// Setup some temporary variables (which we will use instead of memory accesses)
	airplaneState a1;
	airplaneState a2;
	airplaneState a1z;
	airplaneState a2z;
	airplaneState a1z1;
	airplaneState a2z1;


	// For each pair of units in the group
	for (int x = 0; x < GetNumMembers(); x++)
	{
		for (int y = x+1; y < GetNumMembers(); y++)
		{

			// Calculate the last time that either will be in the tree
			// that is, take the max of the two path sizes for the units
			// at the given location in the tree
			int maxLength = std::max(tree[location].paths[x].size(), tree[location].paths[y].size());

			// Check each position in the paths for conflicts
			for (int z = 0; z+1 < maxLength; z++) // There may be an off-by-one error here. Not sure. Will have to check.
			{
				// Here we check the goal conflict - in the future if this is an 
				// airstrip, then we will remove the goal, and just continue if 
				// the shortest path length is reached.
				int a1time = min(z, tree[location].paths[x].size()-1);
				int a2time = min(z, tree[location].paths[y].size()-1);

				a1 = tree[location].paths[x][a1time];
				a2 = tree[location].paths[y][a2time];

				a1z = tree[location].paths[x][z];
				a2z = tree[location].paths[y][z];
				a1z1 = tree[location].paths[x][z+1];
				a2z1 = tree[location].paths[y][z+1];

				// We do a compare where heading doesn't matter
				if (a1.x == a2.x && a1.y == a2.y && a1.height == a2.height)
				{
					// Update the conflicting units
					c1.unit1 = x;
					c2.unit1 = y;

					// Update the conflicting states
					c1.c.loc.l = tree[location].paths[x][a1time];
					c2.c.loc.l = tree[location].paths[y][a2time];

					// Update the conflicting time
					c1.c.loc.t = z;
					c2.c.loc.t = z;

					// Update the conflicting direction (Not properly implemented)
					c1.c.dir = 0;
					c2.c.dir = 0;

					// Inform the user of the conflict
					std::cout << "State conflict found between " << x << " and " << y << " at " << c1.c.loc.l << std::endl;
					return true;
				}
				// If we check to see if the time is too big, that is, if we've compared the last element
				// in one of the paths
				else if (z >= tree[location].paths[x].size() ||
					z >= tree[location].paths[y].size())
					continue;

				// Otherwise, we check to see if there is an edge conflict between the two elements
				else if ( (a1z.x == a2z1.x && a1z.y == a2z1.y && a1z.height == a2z1.height ) &&
					(a1z1.x == a2z.x && a1z1.y == a2z.y && a1z1.height == a2z.height ))
				{
					// Set the units in the conflict
					c1.unit1 = x;
					c2.unit1 = y;

					// Set the tree locations in the conflict
					c1.c.loc.l = tree[location].paths[x][z];
					c2.c.loc.l = tree[location].paths[y][z];

					//Set the time in the conflict
					c1.c.loc.t = z;
					c2.c.loc.t = z;

					// Get the actions that lead to the problem
					c1.c.dir = ae->GetAction(tree[location].paths[x][z], tree[location].paths[y][z]);
					c2.c.dir = ae->GetAction(tree[location].paths[y][z], tree[location].paths[x][z]);
					
					std::cout << "Edge conflict found between " << x << " and " << y << " at " << c1.c.loc.l << " and " << c2.c.loc.l << std::endl;

					return true;
				}
			}
		}
	}
	
	return false;
}

/** Draw the AIR CBS group */
void AirCBSGroup::OpenGLDraw(const AirplaneEnvironment *ae, const SimulationInfo<airplaneState,airplaneAction,AirplaneEnvironment> *)  const
{
	GLfloat r, g, b;
	glLineWidth(2.0);
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		AirCBSUnit *unit = (AirCBSUnit*)GetMember(x);
		unit->GetColor(r, g, b);
		a2e->SetColor(0, 1, 0);
		for (unsigned int y = 0; y+1 < tree[bestNode].paths[x].size(); y++)
		{
			airtimeState a(tree[bestNode].paths[x][y], y);
			airtimeState b(tree[bestNode].paths[x][y+1], y+1);
			a2e->GLDrawLine(a, b);
		}
	}
	glLineWidth(1.0);
}

