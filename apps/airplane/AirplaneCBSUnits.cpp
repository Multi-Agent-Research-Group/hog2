//
//  AirCBSUnits.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneCBSUnits.h"


//----------------------------------------------------------------------------------------------------------------------------------------------//

/** AIR CBS UNIT DEFINITIONS */

bool AirCBSUnit::MakeMove(AirplaneConstrainedEnvironment *ae, OccupancyInterface<airtimeState,airplaneAction> *,
							 SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> * si, airplaneAction& a)
{
	if (myPath.size() > 1 && si->GetSimulationTime() > myPath[myPath.size()-2].t)
	{

		//std::cout << "Moved from " << myPath[myPath.size()-1] << " to " << myPath[myPath.size()-2] << std::endl;
		a = ae->GetAction(myPath[myPath.size()-1], myPath[myPath.size()-2]);
		myPath.pop_back();
		return true;
	} else if (myPath.size() <= 1) {
		if (current.landed || current.type == AirplaneType::QUAD)
		{
			return false;
		} else {

			// With a random probability - either land or keep flying around.
			if (rand()%5 == 0) {
				// Replan the node to a landing location
				airplaneState land(18, 23, 0, 0, 0, true);
				airtimeState newGoal(land, 0);
				AirCBSGroup* g = (AirCBSGroup*) this->GetUnitGroup();
				//g->UpdateUnitGoal(this, newGoal);			
				g->UpdateSingleUnitPath(this, newGoal);
			} else {
				// Replan the node to a random location
				airplaneState rs(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
				airtimeState newGoal(rs, 0);
				AirCBSGroup* g = (AirCBSGroup*) this->GetUnitGroup();
				//g->UpdateUnitGoal(this, newGoal);
				g->UpdateSingleUnitPath(this, newGoal);
			}
		}
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
			glColor3f(1, 0, 0);
			c.OpenGLDraw();
		} else {		
			ae->OpenGLDraw(current);
			glColor3f(1, 0, 0);
			airConstraint c(current);
			c.OpenGLDraw();
		}
	} else {
		if (current.landed)
		      return;
		ae->OpenGLDraw(current);
		airConstraint c(current);
		glColor3f(1, 0, 0);
		c.OpenGLDraw();
	}
}

void AirCBSUnit::UpdateGoal(airtimeState &s, airtimeState &g)
{
	start = s;
	goal = g;
}

//----------------------------------------------------------------------------------------------------------------------------------------------//

/** CBS GROUP DEFINITIONS */

AirCBSGroup::AirCBSGroup(AirplaneConstrainedEnvironment *complex, AirplaneConstrainedEnvironment* simple,
  unsigned threshold, bool u_r, bool u_w) : time(0), bestNode(0), planFinished(false), use_restricted(u_r), use_waiting(u_w)
{

	tree.resize(1);
	tree[0].parent = 0;
    
	// Construct a simple environment container
	//EnvironmentContainer e_simple("Simple", simple, new ManhattanHeuristic<airtimeState>(), 0, 1.2);
	EnvironmentContainer e_simple("Simple", simple, 0, 0, 1.2);
	this->environments.push_back(e_simple);
	simple->SetTicketAuthority(&ticketAuthority);

	// Construct a complex environment container
	//EnvironmentContainer e_complex("Complex", complex, new OctileDistanceHeuristic<airtimeState>(), threshold, 1.8);
	EnvironmentContainer e_complex("Complex", complex, 0, threshold, 2.5);
	this->environments.push_back(e_complex);
	complex->SetTicketAuthority(&ticketAuthority);

	// Sort the environment container by the number of conflicts
	std::sort(this->environments.begin(), this->environments.end(), 
		[](const EnvironmentContainer& a, const EnvironmentContainer& b) -> bool 
			{
				return a.conflict_cutoff < b.conflict_cutoff;
			}
	);

	// Add some restricted airspace
	if (use_restricted) 
	{
		airplaneState l1(10,15,0,0,0);
		airplaneState l2(26, 31, 12, 0, 0);
		ticketAuthority.RegisterAirspace(airtimeState(l1, FLT_MIN), airtimeState(l2,FLT_MAX), 3);
	}

	// Set the current environment to that with 0 conflicts
    SetEnvironment(0);

}


bool AirCBSGroup::MakeMove(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e,
							 SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si, airplaneAction& a)
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

/** Expand a single CBS node */
void AirCBSGroup::ExpandOneCBSNode()
{
	// There's no reason to expand if the plan is finished.
	if (planFinished)
		return;

	std::cout << "Looking for conflicts..." << std::endl;
	airConflict c1, c2;
	bool found = FindFirstConflict(bestNode, c1, c2);

	// If not conflicts are found in this node, then the path is done
	if (!found)
	{
		std::cout << "None found. Finished the plan using " << TOTAL_EXPANSIONS << " expansions." << std::endl;
		TOTAL_EXPANSIONS = 0;
		planFinished = true;
		
		// For every unit in the node
		for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
		{
			// Grab the unit
			AirCBSUnit *unit = (AirCBSUnit*) GetMember(x);
			
			// Prune these paths to the current simulation time
			airtimeState current;
			unit->GetLocation(current);
			std::vector<airtimeState> newPath;
			newPath.push_back(current); // Add the current simulation node to the new path

			// For everything in the path that's new, add the path back
			for (airtimeState xNode : tree[bestNode].paths[x]) {
				if (current.t < xNode.t - 0.0001) {
					newPath.push_back(xNode);
				}
			}

			// Update the actual unit path
			unit->SetPath(newPath);
            std::cout << "Agent " << x << ": " << "\n";
            for(auto &a: tree[bestNode].paths[x])
              std::cout << "  " << a << "\n";
		}
	} 

	// Otherwise, we need to deal with the conflicts
	else {

		// Notify the user of the conflict
		std::cout << "Conflict found between unit " << c1.unit1 << " and unit " << c2.unit1 << " @:" << c2.c.start_state <<  " and " << c1.c.start_state << std::endl;
	
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


		// Set the visible paths for every unit in the node
		for (unsigned int x = 0; x < tree[bestNode].paths.size(); x++)
		{
			// Grab the unit
			AirCBSUnit *unit = (AirCBSUnit*) GetMember(x);
			
			// Prune these paths to the current simulation time
			airtimeState current;
			unit->GetLocation(current);
			std::vector<airtimeState> newPath;
			newPath.push_back(current); // Add the current simulation node to the new path

			// For everything in the path that's new, add the path back
			for (airtimeState xNode : tree[bestNode].paths[x]) {
				if (current.t < xNode.t - 0.0001) {
					newPath.push_back(xNode);
				}
			}

			// Update the actual unit path
			unit->SetPath(newPath);
		}
		
	}
}

/** Update the location of a unit */
void AirCBSGroup::UpdateLocation(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, airtimeState &loc, 
									bool success, SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si)
{
	u->UpdateLocation(e, loc, success, si);
}

void AirCBSGroup::SetEnvironment(unsigned numConflicts){
	for (int i = 0; i < this->environments.size(); i++) {
		if (numConflicts >= environments[i].conflict_cutoff) {
std::cout << "Setting to env# " << i << " b/c >= " << environments[i].conflict_cutoff<<"\n";
			currentEnvironment = &(environments[i]);
		} else {
			break;
		}
	}

	astar.SetHeuristic(currentEnvironment->heuristic);
	astar.SetWeight(currentEnvironment->astar_weight);
}

void AirCBSGroup::ClearEnvironmentConstraints(){
	for (EnvironmentContainer env : this->environments) {
		env.environment->ClearConstraints();
	}
}


void AirCBSGroup::AddEnvironmentConstraint(airConstraint  c){
	for (EnvironmentContainer env : this->environments) {
		env.environment->AddConstraint(c);
	}
}

/** Add a new unit with a new start and goal state to the CBS group */
void AirCBSGroup::AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u)
{
	// Add the new unit to the group, and construct an AirCBSUnit
	UnitGroup::AddUnit(u);
	AirCBSUnit *c = (AirCBSUnit*)u;

	// Clear the constraints from the environment set
	ClearEnvironmentConstraints();

	// Setup the state and goal in the graph
	airtimeState start, goal;
	c->GetStart(start);
	c->GetGoal(goal);

	// Resize the number of paths in the root of the tree
	tree[0].paths.resize(GetNumMembers());

	// Recalculate the optimum path for the root of the tree
	//std::cout << "AddUnit "<<(GetNumMembers()-1) << " getting path." << std::endl;
	astar.GetPath(currentEnvironment->environment, start, goal, thePath);
    std::cout << "AddUnit agent: " << (GetNumMembers()-1) << " expansions: " << astar.GetNodesExpanded() << "\n";
    TOTAL_EXPANSIONS += astar.GetNodesExpanded();

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

void AirCBSGroup::UpdateUnitGoal(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, airtimeState newGoal)
{

	std::cout << "Replanning units..." << std::endl;

	//std::cout << "Clearing open list..." << std::endl;
	// Clear the tree and the open list
	tree.resize(1);
	while(!openList.empty()) openList.pop();

	//std::cout << "Resizing the main tree..." << std::endl;
	// Resize the number of paths in the root of the tree
	tree[0].paths.resize(GetNumMembers());

	//std::cout << "Clear the environmental constraints" << std::endl;
	// Clear the constraints from the environment set
	ClearEnvironmentConstraints();


	//std::cout << "Beginning to update members..." << std::endl;
	// Update the start for all of the units
	for (int x = 0; x < GetNumMembers(); x++)
	{

		//std::cout << "Updating member " << (x + 1) << " of " << GetNumMembers() << std::endl;

		// Get the unit
		AirCBSUnit *c = (AirCBSUnit*)GetMember(x);
                if(c!=u) continue;

		// Obtain the unit's current location and current goal
		//airtimeState current = *(tree[0].paths[x].rbegin());
		airtimeState current, goal;
		if (c->GetPath().size() > 1) {
			current = c->GetPath()[c->GetPath().size()-2];
		} else {
			c->GetLocation(current);
		}
		c->GetGoal(goal);

		// Update the start of that unit to be their current location and the goal to be the new goal
		c->UpdateGoal(current, (GetMember(x) != u ? goal : newGoal));


		//std::cout << "Planning optimal path from " << current << " to " << goal << std::endl;
		// Replan the unit's optimal path
		//astar.GetPath(currentEnvironment->environment, current, goal, thePath);
		DoHAStar(current, goal, thePath);
		TOTAL_EXPANSIONS += astar.GetNodesExpanded();

		//std::cout << "Got optimal path" << std::endl;
		// Add the optimal path to the root of the tree
		tree[0].paths[x].resize(0);
		if (c->GetPath().size() > 1){
			airtimeState cur;
			c->GetLocation(cur);
			tree[0].paths[x].push_back(cur);
		}
		for (unsigned int i = 0; i < thePath.size(); i++)
		{
			tree[0].paths[x].push_back(thePath[i]);
		}

		for(auto &a : tree[0].paths[x]){
			std::cout << " --  " << a << "\n";
		}
	}

	//std::cout << "Adding the root to the open list" << std::endl;

	// Add the root of the node to the open list
	openList.push(OpenListNode(0, 0));

	// Clean up the root node
	tree[0].parent = 0;
	tree[0].satisfiable = true;
	bestNode = 0;

	// Set if the plan is finished to false
	planFinished = false;
}

/** Update Unit Path */
void AirCBSGroup::UpdateSingleUnitPath(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, airtimeState newGoal)
{

	std::cout << "Replanning Single Unit Path..." << std::endl;

	//std::cout << "Clear the environmental constraints" << std::endl;
	// Clear the constraints from the environment set
	ClearEnvironmentConstraints();

	// Add constraints for the rest of the units' pre-planned paths
	for (int x = 0; x < GetNumMembers(); x++) {
		if (GetMember(x) != u) {
			AirCBSUnit *c = (AirCBSUnit*)GetMember(x);

			// Loop over the pre-planned path
			for (int i = 0; i < c->GetPath().size(); i++) {
				// Add a vertex constraint. If you're landed, you don't collide with
				// anything
				if (!(c->GetPath()[i].landed)) {
					airConstraint c1(c->GetPath()[i]);
					AddEnvironmentConstraint(c1);
				}

				// If we can add an edge constraint (because we know where we're going...)
				if (i +1 < c->GetPath().size()) {
					// Add edge consraints if the plane is landing, taking off, or 
					// just in the air. If both states stay landed, no edge constraint
					if (!(c->GetPath()[i].landed && !c->GetPath()[i+1].landed) ||
						!(c->GetPath()[i].landed && c->GetPath()[i+1].landed) || 
						(c->GetPath()[i].landed && !c->GetPath()[i+1].landed)
						) 
					{
						airConstraint c2(c->GetPath()[i], c->GetPath()[i+1]);
						AddEnvironmentConstraint(c2);
					}
				}
			} /* End loop over pre-planned path */
		} /* End if */
	} /* End for */

	// Issue tickets for restricted airspace
	ticketAuthority.UnissueAllTickets();

    for (int i = 0; i < GetNumMembers(); i++) {
    	if (GetMember(i) != u) {
    		AirCBSUnit *c = (AirCBSUnit*)GetMember(i);
    		// Issue tickets for the other paths
    		std::vector<airtimeState> mPath(c->GetPath());
    		ticketAuthority.IssueTicketsForPath(mPath, i);
    		std::reverse(mPath.begin(), mPath.end());
    		c->SetPath(mPath);
    	}
    }

	// Get the unit location
	AirCBSUnit *c = (AirCBSUnit*)u;
	airtimeState current = c->GetPath()[0];

	// Plan a new path for the unit
	std::cout << "Going from " << current << " to " << newGoal << std::endl;
	//astar.GetPath(currentEnvironment->environment, current, newGoal, thePath);
	DoHAStar(current, newGoal, thePath);
	std::cout << "Finished replanning with " << astar.GetNodesExpanded() << " expansions." << std::endl;

	std::cout << "New Path: ";
	for (auto &a : thePath)
		std::cout << a << " ";
	std::cout << std::endl;
	
	// Update the Unit Path
	c->SetPath(thePath);
}

/** Replan a node given a constraint */
void AirCBSGroup::Replan(int location)
{
	// Select the unit from the tree with the new constraint
	int theUnit = tree[location].con.unit1;

	// Reset the constraints in the test-environment
	ClearEnvironmentConstraints();

	// Add all of the constraints in the parents of the current node to the environment
	int tempLocation = location;
    unsigned numConflicts = 0;
    std::vector<airConstraint> constraints;

    do {
      if (theUnit == tree[tempLocation].con.unit1)
      {
        numConflicts++;
        AddEnvironmentConstraint(tree[tempLocation].con.c);
        // Reissue tickets for the conflicting unit
        constraints.push_back(tree[tempLocation].con.c);
      }
      tempLocation = tree[tempLocation].parent;
    } while (tempLocation != 0);

    // Unissue all tickets and re-issue for non-conflicting units
    ticketAuthority.UnissueAllTickets();

    for (int i = 0; i < tree[tempLocation].paths.size(); i++) {
    	if (theUnit != i) {
    		// Issue tickets for the other paths
    		ticketAuthority.IssueTicketsForPathIgnoringCheck(tree[tempLocation].paths[i], i);
    	}
    }

    // Set the environment based on the number of conflicts
    SetEnvironment(numConflicts);

	// Select the air unit from the group
	AirCBSUnit *c = (AirCBSUnit*)GetMember(theUnit);
	
	// Retreive the unit start and goal
	airtimeState start, goal;
	c->GetStart(start);
	c->GetGoal(goal);

	// Recalculate the path
	//std::cout << "#conflicts for " << tempLocation << ": " << numConflicts << "\n";
	//astar.GetPath(currentEnvironment->environment, start, goal, thePath);
	DoHAStar(start, goal, thePath);
	TOTAL_EXPANSIONS += astar.GetNodesExpanded();
	//std::cout << "Replan agent: " << location << " expansions: " << astar.GetNodesExpanded() << "\n";

	// Check path for conflict assertions (Bad idea to enable this while running)
	//for (uint x = 0; x < thePath.size(); x++) {
	//	for (uint i = 0; i < constraints.size(); i++) {
	//		if (constraints[i].ConflictsWith(thePath[x]) || x < (thePath.size() -1 ) && constraints[i].ConflictsWith(thePath[x], thePath[x+1]))
	//			assert(false && "Error with constrained environment");
	//	}
	//}

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

	// Issue tickets on the path
	ticketAuthority.IssueTicketsForPath(tree[location].paths[theUnit], theUnit);
}

/** Find the first place that there is a conflict in the tree */
bool AirCBSGroup::FindFirstConflict(int location, airConflict &c1, airConflict &c2)
{

	// Check for ticket conflicts while we're searching for normal conflicts
	ticketAuthority.UnissueAllTickets();
	for (int x = 0; x < GetNumMembers(); x++) {
		ticketAuthority.IssueTicketsForPathIgnoringCheck(tree[location].paths[x], x);
    }


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
				//std::cout << "Looking at positions " << i << "," << j << std::endl;
				
				// Figure out which indices we're comparing
				int xTime = max(0, min(i, xmax-1));
				int yTime = max(0, min(j, ymax-1));

				// Check the point constraints
				airConstraint x_c(tree[location].paths[x][xTime]);
				airtimeState y_c =tree[location].paths[y][yTime];

				
				// Deal with landing conflicts, we dont't conflict if one of the planes stays landed at
				// the entire time
				if (!(tree[location].paths[x][xTime].landed && tree[location].paths[x][min(xTime + 1, xmax-1)].landed || 
					tree[location].paths[y][yTime].landed && tree[location].paths[y][min(yTime + 1, xmax-1)].landed)) 
				{
					// There are 4 states landed->landed landed->not_landed not_landed->landed and not_landed->not_landed. we
					// need to check edge conflicts on all except the landed->landed states, which we do above. If one plane
					// stays landed the whole time, then there's no edge-conflict -> but if they don't stay landed the whole time
					// then there's obviously a conflict, because they both have to be doing something fishy.
					

					// Check the vertex conflict
					if (x_c.ConflictsWith(y_c))
					{
						c1.c = x_c;
						c2.c = y_c;

						c1.unit1 = y;
						c2.unit1 = x;
						return true;
					}

					// Check the edge conflicts
					airConstraint x_e_c(tree[location].paths[x][xTime], tree[location].paths[x][min(xmax-1, xTime+1)]);
					airConstraint y_e_c(tree[location].paths[y][yTime], tree[location].paths[y][min(ymax-1, yTime+1)]);
					
					if (x_e_c.ConflictsWith(y_e_c))
					{
						c1.c = x_e_c;
						c2.c = y_e_c;

						c1.unit1 = y;
						c2.unit1 = x;
						return true;
					}
				}

				// Check if these states have issued and conflicting tickets
				for (RAirspaceTicket* tx : tree[location].paths[x][xTime].current_tickets) {
					for (RAirspaceTicket* ty : tree[location].paths[y][yTime].current_tickets) {
						// We're looking for tickets with the same issuing airspace at the same time
						if (tx->issuing_airspace == ty->issuing_airspace && max(ty->GetLowPoint(), tx->GetLowPoint()) <= min(ty->GetHighPoint(), tx->GetHighPoint()) + 0.001) {
							// There might be a conflict. Check the capacity at this time
							if (tx->issuing_airspace->GetNumUniqueTicketsAtTime(tx->GetLowPoint(), tx->GetHighPoint()) > tx->issuing_airspace->max_capacity ||
								ty->issuing_airspace->GetNumUniqueTicketsAtTime(ty->GetLowPoint(), ty->GetHighPoint()) > ty->issuing_airspace->max_capacity) {
								
								// We're over capacity in a space
								c1.c = tx->issuing_airspace->governed_area;
								c2.c = ty->issuing_airspace->governed_area;

								c1.unit1 = y;
								c2.unit1 = x;

								return true;
							}
						}
					}	
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
	
	return false;
}

void AirCBSGroup::DoHAStar(airtimeState& start, airtimeState& goal, std::vector<airtimeState>& thePath) 
{
	unsigned x = 1;
	unsigned i = 5000;
	std::vector<airtimeState> tpb;
	while (!HAStarHelper(start, goal, thePath, x, i))
	{
		i *= 2;
		if (this->use_waiting) {
			if (i > 10000) {
				std::cout << "Pushing back the start...." << std::endl;
				SetEnvironment(100000);
				tpb.push_back(start);
				currentEnvironment->environment->ApplyAction(start, airplaneAction(kWait,0,0));
				SetEnvironment(0);
			}
		}
	}
	if (this->use_waiting) {
		for (airtimeState x : tpb) 
			std::cout << "\t" << x << std::endl;
		thePath.insert(thePath.begin(), tpb.begin(), tpb.end());
	}
}

bool AirCBSGroup::HAStarHelper(airtimeState& start, airtimeState& goal, std::vector<airtimeState>& thePath, unsigned& envConflicts, unsigned& conflicts) 
{
	thePath.resize(0);
	SetEnvironment(envConflicts);
	if (!astar.InitializeSearch(currentEnvironment->environment, start, goal, thePath))
		return false;

	while (!astar.DoSingleSearchStep(thePath))
	{
		if (astar.GetNodesExpanded() > conflicts)
		{
			envConflicts = conflicts;
			return false;
		}
	}
	return true;

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

