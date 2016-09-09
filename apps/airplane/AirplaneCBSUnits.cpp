//
//  AirCBSUnits.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneCBSUnits.h"

extern bool heuristic;

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
	} else if (false){//myPath.size() <= 1) { // Don't plan a next goal...
		if (current.landed || current.type == AirplaneType::QUAD)
		{
			return false;
		} else {
      return false;
			// With a random probability - either land or keep flying around.
			// if (rand()%5 == 0) {
			// 	// Replan the node to a landing location
			// 	airplaneState land(18, 23, 0, 0, 0, true);
			// 	airtimeState newGoal(land, 0);
			// 	AirCBSGroup* g = (AirCBSGroup*) this->GetUnitGroup();
			// 	//g->UpdateUnitGoal(this, newGoal);			
			// 	g->UpdateSingleUnitPath(this, newGoal);
			// } else {
			// 	// Replan the node to a random location
			// 	airplaneState rs(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
			// 	airtimeState newGoal(rs, 0);
			// 	AirCBSGroup* g = (AirCBSGroup*) this->GetUnitGroup();
			// 	//g->UpdateUnitGoal(this, newGoal);
			// 	g->UpdateSingleUnitPath(this, newGoal);
			// }
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

AirCBSGroup::AirCBSGroup(AirplaneConstrainedEnvironment* cv, bool u_r, bool u_w, bool no_bypass) : time(0), bestNode(0), planFinished(false), use_restricted(u_r), use_waiting(u_w), nobypass(no_bypass)
{
  tree.resize(1);
  tree[0].parent = 0;
  cenv = cv;
  env = new AirplaneMultiAgentEnvironment(cv);
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

void AirCBSGroup::processSolution()
{
  std::cout << "Finished the plan using " << TOTAL_EXPANSIONS << " expansions.\n";
  std::cout << "Time elapsed: " << timer->EndTimer() << "\n";
  std::cout << "Total conflicts: " << tree.size() << std::endl;
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
    //std::cout << "Agent " << x << ": " << "\n";
    //for(auto &a: tree[bestNode].paths[x])
      //std::cout << "  " << a << "\n";
  }
}

/** Expand a single CBS node */
void AirCBSGroup::ExpandOneCBSNode(bool gui)
{
  // There's no reason to expand if the plan is finished.
  if (planFinished)
    return;
  if(timer == 0){
    timer = new Timer();
    timer->StartTimer();
  }
  if(fgeq(timer->EndTimer(), killtime))
  {
    std::cout << "FAILED\n";
    std::cout << "Finished with failure using " << TOTAL_EXPANSIONS << " expansions.\n";
    std::cout << "Time elapsed: " << killtime << "\n";
    std::cout << "Total conflicts: " << tree.size() << std::endl;
    exit(0);
  }

  //std::cout << "Looking for conflicts..." << std::endl;
  airMetaConflict c1, c2;
  unsigned long last = tree.size();

  unsigned numConflicts(FindFirstConflict(tree[bestNode], c1, c2));
  // If not conflicts are found in this node, then the path is done
  if (numConflicts==0)
  {
    processSolution();
    if(!gui)exit(0);
  } 
  // Otherwise, we need to deal with the conflicts
  else
  {
    for (int i = 0; i < activeMetaAgents.size(); i++) {
      for (int j = 0; j < activeMetaAgents.size(); j++) {
        if (metaAgentConflictMatrix.at(i).at(j) > 5) {
          // Merge I and J
          for (unsigned x : activeMetaAgents.at(j).units) {
            activeMetaAgents.at(i).units.push_back(x);
          }
          // Remove J from the active list
          activeMetaAgents.erase(activeMetaAgents.begin() + j);
          // Remove J from the conflict matrix
          for (int x = 0; x < metaAgentConflictMatrix.size(); x++) {
          metaAgentConflictMatrix.at(x).erase(metaAgentConflictMatrix.at(x).begin() + j);
          }
          metaAgentConflictMatrix.erase(metaAgentConflictMatrix.begin() + j);

          // Reset the search
          // Clear up the rest of the tree and clean the open list
          tree.resize(1);
          while(!openList.empty()) openList.pop();
          openList.push(OpenListNode(0, 0, 0));

          // Re-Plan the first node
          for (unsigned a = 0; a < activeMetaAgents.size(); a++) {
            // Build the MultiAgentState
            MultiAgentState start;
            MultiAgentState goal;
            for (unsigned x = 0; x < activeMetaAgents.at(a).units.size(); x++) {
              // Select the air unit from the group
              AirCBSUnit *c = (AirCBSUnit*)GetMember(activeMetaAgents.at(a).units.at(x));
              // Retreive the unit start and goal
              airtimeState s, g;
              c->GetStart(s);
              c->GetGoal(g);
              start.push_back(s);
              start.push_back(g);
            }

            std::vector<MultiAgentState> tp; 
            env->setGoal(goal);
            astar.GetPath(env, start, goal, tp);

            TOTAL_EXPANSIONS += astar.GetNodesExpanded();
            tree[0].paths.resize(GetNumMembers());

            for (unsigned k = 0; k < activeMetaAgents.at(a).units.size(); k++) {
              // Add the path back to the tree (new constraint included)
              tree[0].paths[activeMetaAgents.at(a).units.at(k)].resize(0);
              thePath = tp.at(a);
              for (unsigned int l = 0; l < thePath.size(); l++)
              {
                tree[0].paths[activeMetaAgents.at(a).units.at(k)].push_back(thePath[l]);
              }
            }
          }
          // Finished merging - return from the unit
          
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
          std::cout << "Merged MAs " << i << " and " << j << std::endl;
          return;
        }
      }
    }




    // Notify the user of the conflict
    std::cout << "Conflict found between unit " << c1.unit1 << " and unit " << c2.unit1 << std::endl;

    {
      last = tree.size();
      // Add two nodes to the tree for each of the children
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
      unsigned nc1(0);
      unsigned nc2(0);
      //unsigned nc1(FindFirstConflict(tree[last], c1, c2));
      //unsigned nc2(FindFirstConflict(tree[last+1], c1, c2));


      // Add the new nodes to the open list
      double cost = 0;
      for (int y = 0; y < tree[last].paths.size(); y++)
        cost += cenv->GetPathLength(tree[last].paths[y]);
      OpenListNode l1(last, cost, nc1);
      openList.push(l1);

      cost = 0;
      for (int y = 0; y < tree[last+1].paths.size(); y++)
        cost += cenv->GetPathLength(tree[last+1].paths[y]);
      OpenListNode l2(last+1, cost, nc2);
      openList.push(l2);
    }

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

/** Add a new unit with a new start and goal state to the CBS group */
void AirCBSGroup::AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u)
{
  // Add the new unit to the group, and construct an AirCBSUnit
	AirCBSUnit *c = (AirCBSUnit*)u;
  c->setUnitNumber(GetNumMembers());
	UnitGroup::AddUnit(u);

  // Add the unit into a new active meta-agent
  this->activeMetaAgents.push_back(AirMetaAgent(GetNumMembers()-1));
  unitToMetaAgentMap[GetNumMembers()-1] = this->activeMetaAgents.size()-1;

  // Add the new meta-agent to the conflict matrix
  metaAgentConflictMatrix.push_back(std::vector<unsigned>());
  metaAgentConflictMatrix.back().resize(this->activeMetaAgents.size());
  for (unsigned i = 0; i < this->activeMetaAgents.size(); i++) {
    metaAgentConflictMatrix.at(i).push_back(0);
    metaAgentConflictMatrix.back().at(i) = 0;
  }

  // Re-Plan with the new meta-agent
	// Clear the constraints from the environment set
	cenv->ClearConstraints();

	// Setup the state and goal in the graph
	airtimeState start, goal;
	c->GetStart(start);
	c->GetGoal(goal);

  MultiAgentState s;
  MultiAgentState g;
  s.push_back(start);
  g.push_back(goal);


	// Resize the number of paths in the root of the tree
	tree[0].paths.resize(GetNumMembers());

	// Recalculate the optimum path for the root of the tree
  std::vector<MultiAgentState> tp; 
  env->setGoal(g);
	astar.GetPath(env, s, g, tp);

  //std::cout << "AddUnit agent: " << (GetNumMembers()-1) << " expansions: " << astar.GetNodesExpanded() << "\n";
  TOTAL_EXPANSIONS += astar.GetNodesExpanded();
  thePath = tp.at(0);

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
	openList.push(OpenListNode(0, 0, 0));
}

/** Replan a node given a constraint */
void AirCBSGroup::Replan(int location)
{
  // Select the meta-agent from the tree with the new constraint
  unsigned theMA = tree[location].con.unit1;

  // Build the MultiAgentState
  MultiAgentState start;
  MultiAgentState goal;
  for (unsigned x = 0; x < activeMetaAgents.at(theMA).units.size(); x++) {
    // Select the air unit from the group
    AirCBSUnit *c = (AirCBSUnit*)GetMember(activeMetaAgents.at(theMA).units.at(x));
    // Retreive the unit start and goal
    airtimeState s, g;
    c->GetStart(s);
    c->GetGoal(g);
    start.push_back(s);
    start.push_back(g);
  }

  // Add all of the constraints
  cenv->ClearConstraints();
  int loc = location;
  while (loc != 0) {
    if (tree[loc].con.unit1 == theMA) {
      for (airConstraint c : tree[loc].con.c) {
        cenv->AddConstraint(c);
      }
    }
    loc = tree[loc].parent;
  }


  // Recalculate the path
  // Recalculate the optimum path for the root of the tree
  std::vector<MultiAgentState> tp; 
  env->setGoal(goal);
  astar.GetPath(env, start, goal, tp);

  //std::cout << "AddUnit agent: " << (GetNumMembers()-1) << " expansions: " << astar.GetNodesExpanded() << "\n";
  TOTAL_EXPANSIONS += astar.GetNodesExpanded();

  for (unsigned x = 0; x < activeMetaAgents.at(theMA).units.size(); x++) {
    // Add the path back to the tree (new constraint included)
    tree[location].paths[activeMetaAgents.at(theMA).units.at(x)].resize(0);
    thePath = tp.at(x);
    for (unsigned int l = 0; l < thePath.size(); l++)
    {
      tree[location].paths[activeMetaAgents.at(theMA).units.at(x)].push_back(thePath[l]);
    }
  }

}

unsigned AirCBSGroup::HasConflict(std::vector<airtimeState> const& a, std::vector<airtimeState> const& b, int x, int y, airMetaConflict &c1, airMetaConflict &c2, bool update, bool verbose)
{
  unsigned numConflicts(0);
  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax = a.size();
  int ymax = b.size();

  if(verbose)std::cout << "Checking for conflicts between: "<<x << " and "<<y<<" ranging from:" << xmax <<"," << ymax << " update: " << update << "\n";

  for (int i = 0, j = 0; j < ymax && i < xmax;) // If we've reached the end of one of the paths, then time is up and 
    // no more conflicts could occur
  {
    // I and J hold the current step in the path we are comparing. We need 
    // to check if the current I and J have a conflict, and if they do, then
    // we have to deal with it, if not, then we don't.

    // Figure out which indices we're comparing
    int xTime = max(0, min(i, xmax-1));
    int yTime = max(0, min(j, ymax-1));

    if(verbose)std::cout << "Looking at positions " << xTime <<":"<<a[xTime].t << "," << j<<":"<<b[yTime].t << std::endl;

    // Check the point constraints
    airConstraint x_c(a[xTime]);
    airtimeState y_c =b[yTime];


    // Deal with landing conflicts, we dont't conflict if one of the planes stays landed at
    // the entire time
    if (!(a[xTime].landed && a[min(xTime + 1, xmax-1)].landed || 
          b[yTime].landed && b[min(yTime + 1, xmax-1)].landed)) 
    {
      // There are 4 states landed->landed landed->not_landed not_landed->landed and not_landed->not_landed. we
      // need to check edge conflicts on all except the landed->landed states, which we do above. If one plane
      // stays landed the whole time, then there's no edge-conflict -> but if they don't stay landed the whole time
      // then there's obviously a conflict, because they both have to be doing something fishy.


      // Check the vertex conflict
      if (x_c.ConflictsWith(y_c) && ++numConflicts && update)
      {
        c1.c.push_back(x_c);
        c2.c.push_back(y_c);

        update = false;
        return 1;
      }

      // Check the edge conflicts
      airConstraint x_e_c(a[xTime], a[min(xmax-1, xTime+1)]);
      airConstraint y_e_c(b[yTime], b[min(ymax-1, yTime+1)]);

      if (x_e_c.ConflictsWith(y_e_c) && ++numConflicts && update)
      {
        c1.c.push_back(x_e_c);
        c2.c.push_back(y_e_c);

        update = false;
        return 1;
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
    if (a[min(xmax-1, xTime+1)].t < b[min(ymax-1, yTime+1)].t)
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
  return numConflicts;
}

/** Find the first place that there is a conflict in the tree */
unsigned AirCBSGroup::FindFirstConflict(AirCBSTreeNode const& location, airMetaConflict &c1, airMetaConflict &c2)
{

  unsigned numConflicts(0);

  // We loop over meta-agent conflicts
  std::vector<std::pair<airMetaConflict, airMetaConflict>> conflicts;

  // For each pair of units in the group
  for (int a = 0; a < activeMetaAgents.size(); a++)
  {
    for (int b = a+1; b < activeMetaAgents.size(); b++)
    {
      unsigned maInternalConflicts(0);

      airMetaConflict u1c;
      airMetaConflict u2c;
      u1c.unit1 = a;
      u2c.unit1 = b;

      // Check to see if, for each of the agents in each meta-agent there is a conflict
      for (int x = 0; x < activeMetaAgents.at(a).units.size(); x++) {
        for (int y = 0; y < activeMetaAgents.at(b).units.size(); y++) {
          maInternalConflicts += HasConflict(location.paths[x],location.paths[y],x,y,u1c,u2c,maInternalConflicts==0,false);
        }
      }

      metaAgentConflictMatrix.at(a).at(b) = maInternalConflicts;
      numConflicts += maInternalConflicts;

      if (maInternalConflicts != 0) {
        conflicts.push_back(std::make_pair(u1c, u2c));
      }

    }
  }

  if (conflicts.size() > 0) {
    c1 = std::get<0>(conflicts.back());
    c2 = std::get<1>(conflicts.back());
  }

  return numConflicts;
}

/** Draw the AIR CBS group */
void AirCBSGroup::OpenGLDraw(const AirplaneConstrainedEnvironment *ae, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> * sim)  const
{
	
}

