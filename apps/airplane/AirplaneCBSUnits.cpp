//
//  AirCBSUnits.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneCBSUnits.h"

/** AIR CBS UNIT DEFINITIONS */

bool AirCBSUnit::MakeMove(AirplaneConstrainedEnvironment *ae, OccupancyInterface<airtimeState,airplaneAction> *,
							 SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> * si, airplaneAction& a)
{
	if (myPath.size() > 1 && si->GetSimulationTime() > myPath[myPath.size()-2].t)
	{

		std::cout << "Moved from " << myPath[myPath.size()-1] << " to " << myPath[myPath.size()-2] << std::endl;
		a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
		myPath.pop_back();
		return true;
	}
	return false;
}
	
void AirCBSUnit::SetPath(std::vector<airtimeState> &p)
{
	myPath = p;
	std::reverse(myPath.begin(), myPath.end());
}

void AirCBSUnit::OpenGLDraw(const AirplaneConstrainedEnvironment *ae, 
							const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si) const
{
	GLfloat r, g, b;
	GetColor(r, g, b);
	ae->SetColor(r, g, b);

	if (myPath.size() > 1) {
		// Interpolate between the two given the timestep
		airtimeState start_t = myPath[myPath.size()-1];
		airtimeState stop_t = myPath[myPath.size()-2];

		if (si->GetSimulationTime() <= stop_t.t && si->GetSimulationTime() >= start_t.t)
		{
			float perc = (stop_t.t - si->GetSimulationTime())/(stop_t.t - start_t.t);
			ae->OpenGLDraw(stop_t, start_t, perc);
			airConstraint c(stop_t, start_t);
			c.OpenGLDraw();
		} else {
			ae->OpenGLDraw(current);
			airConstraint c(current);
			c.OpenGLDraw();
		}
	} else {
		ae->OpenGLDraw(current);
		airConstraint c(current);
		c.OpenGLDraw();
	}
}

/** CBS GROUP DEFINITIONS */

AirCBSGroup::AirCBSGroup(AirplaneConstrainedEnvironment *ae)
{
	std::cout << "Constructed an AirCBSGroup" << std::endl;
	this->ae = ae;
	planFinished = false;
	time = 0;
	tree.resize(1);
	tree[0].parent = 0;
	bestNode = 0;
}


bool AirCBSGroup::MakeMove(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e,
							 SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si, airplaneAction& a)
{
	if (planFinished && si->GetSimulationTime() > time)
	{
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

	// There's no reason to expand if the plan is finished.
	if (planFinished)
		return;

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

			// Print the finished path for the node
			for (airtimeState p: tree[bestNode].paths[x])
			{
				std::cout << p << " ";		
			}
			std::cout << std::endl;
		}
		return;
	}
    std::cout << "Conflict found between unit " << c1.unit1 << " and unit " << c2.unit1 << " @time: " << c2.c.start_state.t << std::endl;
	
	// Otherwise, we add two nodes to the tree for each of the children
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
                //std::cout << "Checking: " << tree[x] << "\n";
		if (tree[x].closed || !tree[x].satisfiable)
			continue;
		cost = 0;
		for (int y = 0; y < tree[x].paths.size(); y++)
			cost += ae->GetPathLength(tree[x].paths[y]);
//std::cout << "Node " << x << " COST " << cost << "\n";
		if (cost < bestCost)
		{
			bestNode = x;
			bestCost = cost;
		}
	}

	std::cout << "New best node " << bestNode << " out of " << tree.size() << " nodes" << std::endl;
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		AirCBSUnit *unit = (AirCBSUnit*) GetMember(x);
		unit->SetPath(tree[bestNode].paths[x]);
	}
}

/** Update the location of a unit */
void AirCBSGroup::UpdateLocation(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, airtimeState &loc, 
									bool success, SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si)
{
	u->UpdateLocation(e, loc, success, si);
}

/** Add a new unit with a new start and goal state to the CBS group */
void AirCBSGroup::AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u)
{
	// Add the new unit to the group, and construct an AirCBSUnit
	UnitGroup::AddUnit(u);
	AirCBSUnit *c = (AirCBSUnit*)u;

	// Clear the constraints from the constrained-environment
	ae->ClearConstraints();

	// Setup the state and goal in the graph
	airtimeState start, goal;
	c->GetStart(start);
	c->GetGoal(goal);

	// Resize the number of paths in the root of the tree
	tree[0].paths.resize(GetNumMembers());

	// Recalculate the optimum path for the root of the tree
	astar.GetPath(ae, start, goal, thePath);


	// We add the optimal path to the root of the tree
	for (unsigned int x = 0; x < thePath.size(); x++)
	{
		tree[0].paths.back().push_back(thePath[x]);
	}
	
	// Set the plan finished to false, as there's new updates
	planFinished = false;
}

/** Replan a state given a constraint */
void AirCBSGroup::Replan(int location)
{
	// Select the unit from the tree with the new constraint
	int theUnit = tree[location].con.unit1;

	// Reset the constraints in the test-environment
	ae->ClearConstraints();

	// Add all of the constraints in the parents of the current
	// node to the environment
	int tempLocation = location;
	
    do {
		if (theUnit == tree[tempLocation].con.unit1)
	    {
	      ae->AddConstraint(tree[tempLocation].con.c);
	      //std::cout << "Add Constraint: " << tree[tempLocation].con.c << "\n";
	    }
		tempLocation = tree[tempLocation].parent;
		//TODO: Find constraints on the goals of the agents (need heading and time)
	} while (tempLocation != 0);

	// Select the air unit from the group
	AirCBSUnit *c = (AirCBSUnit*)GetMember(theUnit);
	airtimeState start, goal;
	c->GetStart(start);
	c->GetGoal(goal);

	// Recalculate the path
	astar.GetPath(ae, start, goal, thePath);

	if (thePath.size() == 0 && !(goal == start))
		tree[location].satisfiable = false;


	// Add it back to the tree (new constraint included)
	tree[location].paths[theUnit].resize(0);
	for (unsigned int x = 0; x < thePath.size(); x++)
	{
		tree[location].paths[theUnit].push_back(thePath[x]);
	}
}

/** Find the first place that there is a conflict in the tree */
bool AirCBSGroup::FindFirstConflict(int location, airConflict &c1, airConflict &c2)
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
				//std::cout << "Checking for conflict at: "<<xTime << ","<<yTime<<"\n";

				// Check the point constraints
				airConstraint x_c(tree[location].paths[x][xTime]);
				airtimeState y_c=tree[location].paths[y][yTime];

				if (x_c.ConflictsWith(y_c))
				{
					c1.c = x_c;
					c2.c = y_c;

					c1.unit1 = x;
					c2.unit1 = y;

					//std::cout << "Found vertex conflict\n";
					return true;
				}

				// Check the edge constraints
				{
					airConstraint x_e_c(tree[location].paths[x][xTime], tree[location].paths[x][min(xmax-1, xTime+1)]);
					airConstraint y_e_c(tree[location].paths[y][yTime], tree[location].paths[y][min(ymax-1, yTime+1)]);

					if (x_e_c.ConflictsWith(y_e_c))
					{
						c1.c = x_e_c;
						c2.c = y_e_c;

						c1.unit1 = x;
						c2.unit1 = y;

						//std::cout << "Found edge conflict\n";
						return true;
					}
				}


				// Increment the counters
				
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
	
	return false;
}

/** Draw the AIR CBS group */
void AirCBSGroup::OpenGLDraw(const AirplaneConstrainedEnvironment *ae, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> * sim)  const
{
	/*
	GLfloat r, g, b;
	glLineWidth(2.0);
	for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
	{
		AirCBSUnit *unit = (AirCBSUnit*)GetMember(x);
		unit->GetColor(r, g, b);
		ae->SetColor(r, g, b);
		for (unsigned int y = 0; y < tree[bestNode].paths[x].size(); y++)
		{
			ae->OpenGLDraw(tree[bestNode].paths[x][y]);
		}
	}
	glLineWidth(1.0);
	*/
}

