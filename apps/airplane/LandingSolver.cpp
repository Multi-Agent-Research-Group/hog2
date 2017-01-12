//
//  AirCBSUnits.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "LandingSolver.h"

extern bool gui;
extern bool heuristic;
extern int seed;
extern Timer* timer;


/** LANDING GROUP DEFINITIONS */

AirLandingGroup::AirLandingGroup(std::vector<EnvironmentContainer> const& environs) : time(0), planFinished(false) {
  environments = environs;

  // Sort the environment container by the number of conflicts
  std::sort(this->environments.begin(), this->environments.end(), 
      [](const EnvironmentContainer& a, const EnvironmentContainer& b) -> bool 
      {
      return a.conflict_cutoff < b.conflict_cutoff;
      }
      );

  // Set the current environment to the most complex environment
  SetEnvironment(this->environments.size()-1);
  this->final_plan = AirLandingPlan();

}

bool AirLandingGroup::MakeMove(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e,
    SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si, airplaneAction& a)
{
  std::cout << "Making move...." << std::endl;
  if (planFinished && si->GetSimulationTime() > time)
  {
    std::cout << "plan is done... moving" << std::endl;
    return u->MakeMove(e,0,si,a);
  }
  else if ((si->GetSimulationTime() - time) < 0.0001)
  {
    std::cout << "no time differential" << std::endl;
    return false;
  }
  else {
    std::cout << "doing planning step..." << std::endl;
    time = si->GetSimulationTime();
    DoPlanningStep();
  }
  return false;
}

void AirLandingGroup::processSolution(double elapsed)
{

  double cost(0.0);
  unsigned total(0);

  // For every unit in the plan
  for (unsigned int x = 0; x < final_plan.paths.size(); x++)
  {
    cost += currentEnvironment->environment->GetPathLength(final_plan.paths[x]);
    total += final_plan.paths[x].size();
    
    // Grab the unit
    AirCBSUnit *unit = (AirCBSUnit*) GetMember(x);

    // Prune these paths to the current simulation time
    airtimeState current;
    unit->GetLocation(current);
    std::vector<airtimeState> newPath;
    newPath.push_back(current); // Add the current simulation node to the new path

    // For everything in the path that's new, add the path back
    for (airtimeState xNode : final_plan.paths[x]) {
      if (current.t < xNode.t - 0.0001) {
        newPath.push_back(xNode);
      }
    }

    // Update the actual unit path
    unit->SetPath(newPath);
  }

  // Mark the plan as finished
  planFinished = true;
  
  
  if(elapsed<0){
    std::cout << "FAILED\n";
    std::cout << "Finished with failure using " << TOTAL_EXPANSIONS << " expansions.\n";
    std::cout << seed<<":Time elapsed: " << elapsed*(-1.0) << "\n";
  }else{
    std::cout << "Finished the plan using " << TOTAL_EXPANSIONS << " expansions.\n";
    std::cout << seed<<":Time elapsed: " << elapsed << "\n";
  }

  
  std::cout << seed<<":Solution cost: " << cost << "\n"; 
  std::cout << seed<<":solution length: " << total << std::endl;
  if(!gui)exit(0);
}

/** Do a planning step */
void AirLandingGroup::DoPlanningStep(bool gui)
{
  // There's no reason to expand if the plan is finished.
  if (planFinished){
    std::cout << "Plan is done already!" << std::endl;
    return;
  }

  std::cout << "Looking for conflicts..." << std::endl;
  airConflict c1, c2;

  unsigned numConflicts(FindFirstConflict(final_plan, c1, c2));
  // If not conflicts are found in this node, then the path is done
  if (numConflicts==0) {  
      std::cout << "No conflicts found!"  << std::endl;
      processSolution(timer->EndTimer()); 
  } else {   
    // Notify the user of the conflict
    std::cout << "Conflict found between unit " << c1.unit1 << " and unit " << c2.unit1 << std::endl;

    // Push back one of the agents
    if (rand() % 2) {
        final_plan.waits[c1.unit1] += 1;
    } else {
        final_plan.waits[c2.unit1] += 1;
    }

    // Set the visible paths for every unit in the node
    for (unsigned int x = 0; x < final_plan.paths.size(); x++)
    {
      // Grab the unit
      AirCBSUnit *unit = (AirCBSUnit*) GetMember(x);

      // Get the path from start to goal for the unit
      airtimeState start;
      unit->GetStart(start);
      std::vector<airtimeState> newPath;

      // Add the current simulation node to the new path
      newPath.push_back(start); 

      // Add the waits
      for (unsigned i = 0; i < final_plan.waits[x]; i++){
          airtimeState new_state(newPath.back());
          currentEnvironment->environment->ApplyAction(new_state, airplaneAction(kWait, 0, 0));
          newPath.push_back(new_state);
      }

      // Generate the final optimal path and add it
      std::vector<airtimeState> thePath;
      astar.GetPath(currentEnvironment->environment, newPath.back(), final_plan.goals[x], thePath);
      newPath.insert(newPath.end(), thePath.begin(), thePath.end());

      final_plan.paths[x] = newPath;

      // Prune these paths to the current simulation time
      std::vector<airtimeState> unitPath;
      airtimeState current;
      unit->GetLocation(current);
      for (airtimeState xNode : newPath) {
        if (current.t < xNode.t - 0.0001) {
          unitPath.push_back(xNode);
        }
      }

      // Update the actual unit path
      unit->SetPath(unitPath);
    }

  }
}

/** Update the location of a unit */
void AirLandingGroup::UpdateLocation(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, airtimeState &loc, 
    bool success, SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si)
{
  u->UpdateLocation(e, loc, success, si);
}

void AirLandingGroup::SetEnvironment(unsigned i){
  currentEnvironment = &(environments[i]);
  astar.SetHeuristic(currentEnvironment->heuristic);
  astar.SetWeight(currentEnvironment->astar_weight);
}

/** Add a new unit with a new start and goal state to the CBS group */
void AirLandingGroup::AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u)
{
  AirCBSUnit *c = (AirCBSUnit*)u;
  c->setUnitNumber(GetNumMembers());
  // Add the new unit to the group, and construct an AirCBSUnit
  UnitGroup::AddUnit(u);

  // Resize the number of paths in the root of the tree
  final_plan.paths.resize(GetNumMembers());
  final_plan.goals.resize(GetNumMembers());
  final_plan.waits.resize(GetNumMembers());
  
  // Calculate the optimum path 
  std::cout << "LS: Calculating Optimal Path..." << std::endl;
  airtimeState start, goal;
  c->GetGoal(goal);
  c->GetStart(start);
  std::vector<airtimeState> thePath;
  astar.GetPath(currentEnvironment->environment, start, goal, thePath);
  std::cout << "LS: Finished Calculating Optimal Path. (Length " << thePath.size() << ")" << std::endl;
  
  
  // Initialze the plan
  final_plan.paths[final_plan.paths.size()-1] = thePath;
  final_plan.goals[final_plan.paths.size()-1] = goal;
  final_plan.waits[final_plan.paths.size()-1] = 0;
      
  // Set the plan finished to false, as there's new updates
  planFinished = false;
}

unsigned AirLandingGroup::HasConflict(std::vector<airtimeState> const& a, std::vector<airtimeState> const& b, int x, int y, airConflict &c1, airConflict &c2, bool update, bool verbose)
{
  unsigned numConflicts(0);
  // To check for conflicts, we loop through the timed actions, and check 
  // each bit to see if a constraint is violated
  int xmax = a.size();
  int ymax = b.size();

  // Pull the units from the graph
  AirCBSUnit* A = (AirCBSUnit*) GetMember(x);
  AirCBSUnit* B = (AirCBSUnit*) GetMember(y);
  
  int pxTime(-1);
  int pyTime(-1);

  // If we've reached the end of one of the paths, then time is up and no more conflicts could occur
  for (int i = 0, j = 0; j < ymax && i < xmax;) 
  {
    // I and J hold the current step in the path we are comparing. We need 
    // to check if the current I and J have a conflict, and if they do, then
    // we have to deal with it, if not, then we don't.

    // Figure out which indices we're comparing
    int xTime = max(0, min(i, xmax-1));
    int yTime = max(0, min(j, ymax-1));

    // Check the point constraints
    airConstraint x_c(a[xTime]);
    airtimeState y_c =b[yTime];


    // Deal with landing conflicts, we don't conflict if one of the planes stays landed at
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
        c1.c = x_c;
        c2.c = y_c;

        c1.unit1 = y;
        c2.unit1 = x;

        update = false;
        return 1;
      }

      // Check the edge conflicts
      airConstraint x_e_c(a[xTime], a[min(xmax-1, xTime+1)]);
      airConstraint y_e_c(b[yTime], b[min(ymax-1, yTime+1)]);

      if (x_e_c.ConflictsWith(y_e_c) && ++numConflicts && update)
      {
        c1.c = x_e_c;
        c2.c = y_e_c;

        c1.unit1 = y;
        c2.unit1 = x;

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
unsigned AirLandingGroup::FindFirstConflict(AirLandingPlan const& location, airConflict &c1, airConflict &c2)
{
  unsigned numConflicts(0);

  // For each pair of units in the group... We may want to return early
  for (int x = 0; x < GetNumMembers(); x++)
  {
    for (int y = x+1; y < GetNumMembers(); y++)
    {
      numConflicts += HasConflict(location.paths[x],location.paths[y],x,y,c1,c2,numConflicts==0);
      if(!highsort&&numConflicts) return numConflicts;
    }
  }
  return numConflicts;
}


/** Draw the AIR LANDING group */
void AirLandingGroup::OpenGLDraw(const AirplaneConstrainedEnvironment *ae, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> * sim)  const
{	
    /*
	GLfloat r, g, b;
	glLineWidth(2.0);
	for (unsigned int x = 0; x < final_plan.paths.size(); x++)
	{
		AirCBSUnit *unit = (AirCBSUnit*)GetMember(x);
		unit->GetColor(r, g, b);
		ae->SetColor(r, g, b);
		for (unsigned int y = 0; y < final_plan.paths[x].size(); y++)
		{
			ae->OpenGLDraw(final_plan.paths[x][y]);
		}
	}
	glLineWidth(1.0);
    */
}


