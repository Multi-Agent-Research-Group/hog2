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
#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "MACBSUnits.h"
#include "NonUnitTimeCAT.h"
#include "UnitTimeCAT.h"
#include "ICTSAlgorithm.h"
#include "MapPerfectHeuristic.h"
#include "Map3dPerfectHeuristic.h"
//#include "AirplaneConstrained.h"
#include "Grid3DConstrainedEnvironment.h"
#include "Utilities.h"
#include <sstream>

std::string mapdir("/hog2/benchmarks/maps");
extern double agentRadius;
bool greedyCT = false; // use greedy heuristic at the high-level
bool ECBSheuristic = false; // use ECBS heuristic at low-level
bool randomalg = false; // Randomize tiebreaking
bool useCAT = false; // Use conflict avoidance table
bool verify = false;
bool mouseTracking;
unsigned killtime(300); // Kill after some number of seconds
unsigned killmem(1024); // 1GB
unsigned killex(INT_MAX); // Kill after some number of expansions
bool disappearAtGoal(false);
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
int width = 64;
int length = 64;
int height = 0;
bool recording = false; // Record frames
bool quiet(false);
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double frameIncrement = 1.0/1000.0;
std::string mapfile;
std::string dtedfile;
unsigned mergeThreshold(5);
std::vector<std::vector<xyztLoc> > waypoints;
Solution<xyztLoc> solution;
//std::vector<SoftConstraint<xytLoc> > sconstraints;
#define NUMBER_CANONICAL_STATES 10

char const* envnames[11] = {"fourconnected","fiveconnected","eightconnected","nineconnected","twentyfourconnected","twentyfiveconnected","fortyeightconnected","fortynineconnected","3dcardinal","3done","3dtwo"};
int cutoffs[10] = {0,9999,9999,9999,9999,9999,9999,9999,9999,9999}; // for each env
double weights[10] = {1,1,1,1,1,1,1,1,1,1}; // for each env
std::vector<std::vector<EnvironmentContainer<xyztLoc,t3DDirection>>> environs;
std::vector<std::vector<EnvData>> envdata;
int seed = clock();
int num_agents = 0;
int minsubgoals(1);
int maxsubgoals(1);
int wait = xyztLoc::TIME_RESOLUTION_U;
bool nobypass = false;
bool noid = false;
bool paused = false;

Grid3DConstrainedEnvironment *ace = 0;
typedef UnitSimulation<xyztLoc, t3DDirection, ConstrainedEnvironment<xyztLoc,t3DDirection>> UnitSim;
UnitSim *sim = 0;
//typedef CBSUnit<xyztLoc,t3DDirection,UnitTieBreaking3D<xyztLoc,t3DDirection>,UnitTimeCAT<xyztLoc,t3DDirection>> MACBSUnit;
//typedef CBSGroup<xyztLoc,t3DDirection,UnitTieBreaking3D<xyztLoc,t3DDirection>,UnitTimeCAT<xyztLoc,t3DDirection>,ICTSAlgorithm<xyztLoc,t3DDirection>> MACBSGroup;
typedef CBSUnit<xyztLoc,t3DDirection,TieBreaking3D<xyztLoc,t3DDirection>,NonUnitTimeCAT<xyztLoc,t3DDirection>> MACBSUnit;
typedef CBSGroup<xyztLoc,t3DDirection,TieBreaking3D<xyztLoc,t3DDirection>,NonUnitTimeCAT<xyztLoc,t3DDirection>,ICTSAlgorithm<xyztLoc,t3DDirection>,Map3dPerfectHeuristic<xyztLoc,t3DDirection>> MACBSGroup;
std::unordered_map<unsigned,MACBSGroup*> groups;
std::vector<unsigned> rgroups;
std::vector<MACBSUnit*> units;

template<>
double NonUnitTimeCAT<xyztLoc, t3DDirection>::bucketWidth=xyztLoc::TIME_RESOLUTION_D;

bool gui=true;
int animate(0);
uint64_t maxcost(0);
void InitHeadless();
bool detectIndependence();

void processSolution(double elapsed){
  double cost(0.0);
  unsigned total(0);
  // For every unit in the node
  bool valid(true);
  for (unsigned int x = 0; x < solution.size(); x++) {
    cost += environs[0][0].environment->GetPathLength(solution[x]);
    total += solution[x].size();

    if(!quiet){
      if(solution[x].size()) {
        std::cout << "Agent " << x << " (" << environs[0][0].environment->GetPathLength(solution[x]) << "): " << "\n";
        for (auto &a : solution[x]) {
          std::cout << "  " << a << "\n";
        }
      } else {
        std::cout << "Agent " << x << ": " << "NO Path Found.\n";
      }
    }
    // Only verify the solution if the run didn't time out
    if (verify && elapsed > 0) {
      for (unsigned y = x + 1; y < solution.size(); y++) {
        auto ap(solution[x].begin());
        auto a(ap + 1);
        auto bp(solution[y].begin());
        auto b(bp + 1);
        while (a != solution[x].end() && b != solution[y].end()) {
          if (collisionCheck3D(*ap, *a, *bp, *b, agentRadius)){
            valid = false;
            std::cout << "ERROR: Solution invalid; collision at: " << x << ":" << *ap << "-->" << *a << ", " << y << ":"
              << *bp << "-->" << *b << std::endl;
          }
          if (a->t < b->t) {
            ++a;
            ++ap;
          } else if (a->t > b->t) {
            ++b;
            ++bp;
          } else {
            ++a;
            ++b;
            ++ap;
            ++bp;
          }
        }
      }
    }
  }
  unsigned nodes(0);
  for(auto const& g:groups){
    nodes+=g.second->tree.size();
  }
  fflush(stdout);
  std::cout
    << "elapsed,planTime,replanTime,bypassplanTime,maplanTime,collisionTime,expansions,CATcollchecks,collchecks,collisions,cost,actions,maxCSet,meanCSet\n";
  if (verify && elapsed > 0)
    std::cout << (valid ? "VALID" : "INVALID") << std::endl;
  if (elapsed < 0) {
    //std::cout << seed<<":FAILED\n";
    std::cout << seed << ":" << elapsed * (-1.0) << ",";
  } else {
    std::cout << seed << ":" << elapsed << ",";
  }
  std::cout << MACBSGroup::planTime << ",";
  std::cout << MACBSGroup::replanTime << ",";
  std::cout << MACBSGroup::bypassplanTime << ",";
  std::cout << MACBSGroup::maplanTime << ",";
  std::cout << MACBSGroup::collisionTime << ",";
  std::cout << MACBSGroup::TOTAL_EXPANSIONS << ",";
  std::cout << TieBreaking3D<xyztLoc,t3DDirection>::collchecks << ",";
  std::cout << MACBSGroup::collchecks << ",";
  std::cout << nodes << ",";
  std::cout << cost / xyztLoc::TIME_RESOLUTION_D << ",";
  std::cout << total << ",";
  std::cout << MACBSGroup::constraintsz/std::max(1ul,MACBSGroup::constrainttot)<< std::endl;
  if (!gui)
    exit(elapsed>0?0:1);
}

int main(int argc, char* argv[])
{
  //load3DCollisionTable();
  InstallHandlers();
  Params::precheck=0; // No precheck (default)
  ProcessCommandLineArgs(argc, argv);
  Util::setmemlimit(killmem);

  if(gui)
  {
    RunHOGGUI(0, 0);
  }
  else
  {
    InitHeadless();
    //std::cout << solution;
    while((groups.size()==1&&!groups.begin()->second->donePlanning())||!detectIndependence()){
      if(Params::verbose)
        std::cout << "There are " << groups.size() << " groups" << std::endl;
      for(auto& group:groups){
        if(!group.second->donePlanning()){
          if(Params::verbose)std::cout << "Independent group " << group.first << " being replanned:\n";
          while(group.second->ExpandOneCBSNode()){ }
          unsigned k(0);
          for(auto const& a:group.second->agents){
            if(Params::verbose)std::cout << "  " << a << "\n";
            solution[a]=*group.second->tree[group.second->bestNode].paths[k++];
          }
        }else{
          if(Params::verbose)std::cout << "Independent group " << group.first << " NOT being replanned\n";
          if(Params::verbose)for(auto const& a:group.second->agents){
            std::cout << "  " << a << "\n";
          }
        }
      }
    }
    processSolution(MACBSGroup::timer->EndTimer());
    //if(Params::verbose)
    //for(int i(0);i<solution.size();++i){
    //std::cout << "path for agent " << i << ":\n";
    //for(auto const& n: solution[i])
    //std::cout << n << "\n";
    //}
  }
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
  SetNumPorts(id, 1);

  //	unitSims.resize(id+1);
  //	unitSims[id] = new DirectionSimulation(new Directional2DEnvironment(map, kVehicle));
  //	unitSims[id]->SetStepType(kRealTime);
  //	unitSims[id]->GetStats()->EnablePrintOutput(true);
  //	unitSims[id]->GetStats()->AddIncludeFilter("gCost");
  //	unitSims[id]->GetStats()->AddIncludeFilter("nodesExpanded");
  //	dp = new DirectionalPlanner(quad);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
  InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
  InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
  InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
  InstallKeyboardHandler(MyDisplayHandler, "Speed Up Simulation", "Speed Up simulation execution.", kNoModifier, '=');
  InstallKeyboardHandler(MyDisplayHandler, "Slow Down Simulation", "Slow Down simulation execution.", kNoModifier, '-');
  InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
  InstallKeyboardHandler(MyDisplayHandler, "Record", "Toggle recording.", kNoModifier, 'r');
  InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
  InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
  InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
  InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

  InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
  InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
  InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
  InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

  InstallCommandLineHandler(MyCLHandler, "-wait", "-wait <time units>", "The duration of wait actions");
  InstallCommandLineHandler(MyCLHandler, "-dimensions", "-dimensions width,length,height", "Set the length,width and height of the environment (max 65K,65K,1024).");
  InstallCommandLineHandler(MyCLHandler, "-nagents", "-nagents <number>", "Select the number of agents.");
  InstallCommandLineHandler(MyCLHandler, "-nsubgoals", "-nsubgoals <number>,<number>", "Select the min,max number of subgoals per agent.");
  InstallCommandLineHandler(MyCLHandler, "-seed", "-seed <number>", "Seed for random number generator (defaults to clock)");
  InstallCommandLineHandler(MyCLHandler, "-nobypass", "-nobypass", "Turn off bypass option");
  InstallCommandLineHandler(MyCLHandler, "-noid", "-noid", "Turn off independence detection");
  InstallCommandLineHandler(MyCLHandler, "-record", "-record", "Record frames");
  InstallCommandLineHandler(MyCLHandler, "-cutoffs", "-cutoffs <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Number of conflicts to tolerate before switching to less constrained layer of environment. Environments are ordered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
  InstallCommandLineHandler(MyCLHandler, "-weights", "-weights <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Weight to apply to the low-level search for each environment entered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
  InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
  InstallCommandLineHandler(MyCLHandler, "-cfgfile", "-cfgfile", "Load MAPF configuration from file");
  InstallCommandLineHandler(MyCLHandler, "-envfile", "-envfile", "Load environment settings per agent");
  InstallCommandLineHandler(MyCLHandler, "-constraints", "-constraints", "Load constraints from file");
  InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime", "Kill after this many seconds");
  InstallCommandLineHandler(MyCLHandler, "-killmem", "-killmem", "Kill after this many seconds");
  InstallCommandLineHandler(MyCLHandler, "-killex", "-killex", "Kill after this many expansions");
  InstallCommandLineHandler(MyCLHandler, "-mapfile", "-mapfile", "Map file to use");
  InstallCommandLineHandler(MyCLHandler, "-dtedfile", "-dtedfile", "Map file to use");
  InstallCommandLineHandler(MyCLHandler, "-scenfile", "-scenfile", "Scenario file to use");
  InstallCommandLineHandler(MyCLHandler, "-mergeThreshold", "-mergeThreshold", "Number of conflicts to tolerate between meta-agents before merging");
  InstallCommandLineHandler(MyCLHandler, "-radius", "-radius <value>", "Radius in units of agent");
  InstallCommandLineHandler(MyCLHandler, "-resolution", "-resolution <value>", "Inverse resolution of time/cost");
  InstallCommandLineHandler(MyCLHandler, "-disappear", "-disappear", "Agents disappear at goal");
  InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
  InstallCommandLineHandler(MyCLHandler, "-verbose", "-verbose", "Turn on verbose output");
  InstallCommandLineHandler(MyCLHandler, "-vc", "-vc", "Turn on vertex collisions");
  InstallCommandLineHandler(MyCLHandler, "-asym", "-asym", "Turn on asymmetric pairings");
  InstallCommandLineHandler(MyCLHandler, "-astarverbose", "-astarverbose", "Turn on verbose output for A*");
  InstallCommandLineHandler(MyCLHandler, "-quiet", "-quiet", "Extreme minimal output");
  InstallCommandLineHandler(MyCLHandler, "-cat", "-cat", "Use Conflict Avoidance Table (CAT)");
  InstallCommandLineHandler(MyCLHandler, "-animate", "-animate <usecs>", "Animate CBS search");
  InstallCommandLineHandler(MyCLHandler, "-verify", "-verify", "Verify results");
  InstallCommandLineHandler(MyCLHandler, "-toptwo", "-toptwo", "Choose top two conflict counts for agents at split");
  InstallCommandLineHandler(MyCLHandler, "-complete", "-complete", "Ensure completeness when using suboptimal switch");
  InstallCommandLineHandler(MyCLHandler, "-overload", "-overload", "Add additional constriants during conflict check");
  InstallCommandLineHandler(MyCLHandler, "-suboptimal", "-suboptimal", "Sub-optimal answers");
  InstallCommandLineHandler(MyCLHandler, "-random", "-random", "Randomize conflict resolution order");
  InstallCommandLineHandler(MyCLHandler, "-greedyCT", "-greedyCT", "Greedy sort high-level search by number of conflicts (GCBS)");
  InstallCommandLineHandler(MyCLHandler, "-xor", "-xor", "Use XOR constraints");
  InstallCommandLineHandler(MyCLHandler, "-ctype", "-ctype", "Constraint type: \n\t1: identical constraints\n\t2: Pyramid constraints\n\t3: Collision constraints (sub-optimal)\n\t4: Time range constraints (sub-optimal)\n\t5: Mutual Conflict set constraints\n\t6: Overlap Constraints\n\t7:Box constraints (sub-optimal)\n\t8:1xn TimeRange\n\t9:nxm TimeRange");
  InstallCommandLineHandler(MyCLHandler, "-pc", "-pc", "prioritize conflicts");
  InstallCommandLineHandler(MyCLHandler, "-apriori", "-apriori", "Use a-priori computed conflict detection");
  InstallCommandLineHandler(MyCLHandler, "-cct", "-cct", "Conflict count table");
  InstallCommandLineHandler(MyCLHandler, "-uniqcost", "-uniqcost <value>", "Use randomized unique costs up to <value>");
  InstallCommandLineHandler(MyCLHandler, "-skip", "-skip", "Ship-ahead logic");
  InstallCommandLineHandler(MyCLHandler, "-precheck", "-precheck", "Pre-check for broadphase collision 0(default)=No precheck, 1=AABB precheck, 2=Convex hull intersection check, 3=Sweep and prune");
  InstallCommandLineHandler(MyCLHandler, "-ECBSheuristic", "-ECBSheuristic", "Use heuristic in low-level search");

  InstallWindowHandler(MyWindowHandler);

  InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
  if (eType == kWindowDestroyed)
  {
    printf("Window %ld destroyed\n", windowID);
    RemoveFrameHandler(MyFrameHandler, windowID, 0);
  }
  else if (eType == kWindowCreated)
  {
    glClearColor(0.6, 0.8, 1.0, 1.0);
    printf("Window %ld created\n", windowID);
    InstallFrameHandler(MyFrameHandler, windowID, 0);
    InitSim();
    CreateSimulation(windowID);
  }
}

void InitGroupParams(MACBSGroup* group){
  Params::greedyCT=greedyCT;
  group->disappearAtGoal=disappearAtGoal;
  group->seed=seed;
  group->keeprunning=gui;
  group->animate=animate;
  group->killex=killex;
  group->mergeThreshold=mergeThreshold;
  group->ECBSheuristic=ECBSheuristic;
  group->nobypass=nobypass;
  group->verify=verify;
  group->quiet=quiet;
}
void fillWaypoints(){
  if(waypoints.size()<num_agents){
    // Adding random waypoints
    std::vector<xyztLoc> s;
    unsigned r(maxsubgoals-minsubgoals);
    int numsubgoals(minsubgoals+1);
    if(r>0){
      numsubgoals = rand()%(maxsubgoals-minsubgoals)+minsubgoals+1;
    }
    if(Params::verbose)std::cout << "Agent " << waypoints.size() << " add " << numsubgoals << " subgoals\n";

    for(int n(0); n<numsubgoals; ++n){
      bool conflict(true);
      while(conflict){
        conflict=false;
        xyztLoc start(rand() % width, rand() % length, rand() % height,0u);
        if(!ace->GetMap()->IsTraversable(start.x,start.y,start.z,Map3D::air)){conflict=true;continue;}
        for (int j = 0; j < waypoints.size()-1; j++)
        {
          if(waypoints[j].size()>n)
          {
            xyztLoc a(waypoints[j][n]);
            // Make sure that no subgoals at similar times have a conflict
            Collision<xyztLoc> x_c(a,a,agentRadius);
            if(x_c.ConflictsWith(start,start)){conflict=true;break;}
            if(a==start){conflict=true;break;}
          }
          /*xytLoc a(start,1.0);
            xytLoc b(a);
            b.x++;
            if(conflict=ace->ViolatesConstraint(a,b)){break;}*/
        }
        if(!conflict) s.push_back(start);
      }
    }
    waypoints.push_back(s);
  }
  for(auto w(waypoints.begin()+1); w!=waypoints.end();/*++w*/){
    if(*(w-1) == *w)
      waypoints.erase(w);
    else
      ++w;
  }
}


void InitHeadless(){
  ace=(Grid3DConstrainedEnvironment*)environs[0].rbegin()->environment;
  UnitTieBreaking3D<xyztLoc,t3DDirection>::randomalg=randomalg;
  UnitTieBreaking3D<xyztLoc,t3DDirection>::useCAT=useCAT;
  TieBreaking3D<xyztLoc,t3DDirection>::randomalg=randomalg;
  TieBreaking3D<xyztLoc,t3DDirection>::useCAT=useCAT;

  if(gui){
    sim = new UnitSim(ace);
    sim->SetStepType(kLockStep);
    sim->SetLogStats(false);
  }
  groups.reserve(num_agents);
  solution.resize(num_agents);
  units.reserve(num_agents);

  if(noid){
    groups[0]=new MACBSGroup(environs,{});
    MACBSGroup* group=groups[0];
    group->agents.resize(num_agents);
    std::iota(group->agents.begin(),group->agents.end(),0); // Add agents 0, 1, ...
    rgroups.resize(1);
    rgroups[0]=0;
    if(!gui){
      Timer::Timeout func(std::bind(processSolution, std::placeholders::_1));
      MACBSGroup::timer->StartTimeout(std::chrono::seconds(killtime),func);
    }
    InitGroupParams(group);

    if(gui){
      sim->AddUnitGroup(group);
    }
    fillWaypoints();

    float softEff(.9);
    for(int i(0); i<num_agents; ++i){
      if(!quiet){
        std::cout << "Set unit " << i << " subgoals: ";
        for(auto &a: waypoints[i])
          std::cout << a << " ";
        std::cout << std::endl;
      }
      units.push_back(new MACBSUnit(waypoints[i],softEff));
      units[i]->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
      group->AddUnit(units[i]); // Add to the group
      if(Params::verbose)std::cout << "initial path for agent " << i << ":\n";
      if(Params::verbose)
        for(auto const& n: *group->tree[0].paths[i])
          std::cout << n << "\n";
      if(gui){sim->AddUnit(units[i]);} // Add to the group
      solution[i]=group->basepaths[0];
    }
    group->Init();
  }else{
    rgroups.resize(num_agents);
    for(uint16_t i(0); i<num_agents; ++i){
      std::vector<std::vector<EnvironmentContainer<xyztLoc,t3DDirection>>> environ={environs[i]};
      if(Params::verbose)std::cout << "Creating group " << i << "\n";
      groups[i]=new MACBSGroup(environ,{i});
      MACBSGroup* group=groups[i];
      rgroups[i]=i;
      if(i==0 && !gui){
        Timer::Timeout func(std::bind(&processSolution, std::placeholders::_1));
        MACBSGroup::timer->StartTimeout(std::chrono::seconds(killtime),func);
      }
      InitGroupParams(group);
      if(gui){
        sim->AddUnitGroup(group);
      }

      fillWaypoints();

      if(!quiet){
        std::cout << "Set unit " << i << " subgoals: ";
        for(auto &a: waypoints[i])
          std::cout << a << " ";
        std::cout << std::endl;
      }
      float softEff(.9);
      units.push_back(new MACBSUnit(waypoints[i],softEff));
      units[i]->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
      group->AddUnit(units[i]); // Add to the group
      if(Params::verbose)std::cout << "initial path for agent " << i << ":\n";
      if(Params::verbose)
        for(auto const& n: *group->tree[0].paths[0])
          std::cout << n << "\n";
      if(gui){sim->AddUnit(units[i]);} // Add to the group

      //assert(false && "Exit early");
      group->Init();
      group->planFinished=true;
      solution[i]=group->basepaths[0];
    }
  }
}

bool detectIndependence(){
  bool independent(true);
  // Check all pairs for collision
  for(int i(0); i<solution.size(); ++i){
    for(int j(i+1); j<solution.size(); ++j){
      auto a(1);
      auto b(1);
      while(a < solution[i].size() && b < solution[j].size()) {
        if(collisionCheck3D(solution[i][a-1], solution[i][a], solution[j][b-1], solution[j][b], agentRadius)){
          independent = false;
          if(Params::verbose)std::cout << "NOT INDEPENDENT: " << i << ":" << solution[i][a-1] << "-->" << solution[i][a] << ", " << j << ":"
            << solution[j][b-1] << "-->" << solution[j][b] << std::endl;
          if(rgroups[i]==rgroups[j]){
            break; // This can happen if both collide with a common agent
          }
          // Combine groups i and j
          //groups[i]=new MACBSGroup(environs,groups[i]->agents);
          MACBSGroup* toDelete(groups[rgroups[j]]);
          //MACBSGroup* toDelete2(groups[i]);
          if(groups.find(rgroups[j])==groups.end()){
            std::cout << "ERROR: agent "<<j<<" belongs to group "<<rgroups[j]<<" but group not found\n";
            assert(false);
          }
          if(groups.find(rgroups[i])==groups.end()){
            std::cout << "ERROR: agent "<<i<<" belongs to group "<<rgroups[i]<<" but group not found\n";
            assert(false);
          }
          if(Params::verbose)std::cout << "Insert "<<groups[rgroups[j]]->GetNumMembers()<<"/"<<groups[rgroups[j]]->agents.size() << " agents\n";
          unsigned origgroup(rgroups[j]);
          unsigned k(0);
          for(auto ag:groups[rgroups[j]]->agents){
            if(Params::verbose)std::cout << "Inserting agent " << ag << " into group "<<rgroups[i]<<" for agent " << i << "("<<(uint64_t)units[ag]<<") from group "<<origgroup<<"\n";
            groups[rgroups[i]]->agents.push_back(ag);
            groups[rgroups[i]]->environments.push_back(groups[origgroup]->environments[k++]);
            groups[rgroups[i]]->AddUnit(units[ag],solution[ag]);
            rgroups[ag]=rgroups[i];
            //maxnagents=std::max(group[i]->agents.size(),maxnagents);
          }
          groups[rgroups[i]]->Init();
          if(Params::verbose)std::cout << "Group " << rgroups[i] << " now has: \n";
          k=0;
          if(Params::verbose)for(auto ag:groups[rgroups[i]]->agents){
            std::cout << "  "<<ag<<"-"<<(uint64_t)groups[rgroups[i]]->GetMember(k++)<<"\n";
          }
          if(Params::verbose)std::cout << groups[rgroups[i]]->donePlanning() << " done\n";
          /*k=0;
            for(auto ag:groups[i]->agents){
            groups[i]->AddUnit(groups[ag]->GetMember(k),*groups[ag]->tree[groups[ag]->bestNode].paths[k]);
            k++;
            groups[ag]=groups[i];
            }*/
          if(Params::verbose)std::cout << origgroup << "erased\n";
          groups.erase(origgroup);
          delete toDelete;
          break;
        }
        if (solution[i][a].t < solution[j][b].t) {
          ++a;
        } else if (solution[i][a].t > solution[j][b].t) {
          ++b;
        } else {
          ++a;
          ++b;
        }
      }
    }
  }
  return independent;
}


void InitSim(){
  InitHeadless();
}

void MyComputationHandler()
{
  while (true)
  {
    sim->StepTime2(stepsPerFrame);
  }
}

//std::vector<t3DDirection> acts;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
  for(auto const& group:groups){
    if (ace){
      for(auto& u : group.second->GetMembers()){
        glLineWidth(2.0);
        GLfloat r, g, b;
        u->GetColor(r, g, b);
        ace->SetColor(r,g,b);
        ace->GLDrawPath(((MACBSUnit const*)u)->GetPath(),((MACBSUnit const*)u)->GetWaypoints());
      }
    }

    //static double ptime[500];
    //memset(ptime,0,500*sizeof(double));
    if(sim){
      sim->OpenGLDraw();
      if (!paused) {
        if(group.second->donePlanning()){
          sim->StepTime2(stepsPerFrame);
        }else{
          sim->StepTime2(.00001);
        }

        /*std::cout << "Printing locations at time: " << sim->GetSimulationTime() << std::endl;
          for (int x = 0; x < group.second->GetNumMembers(); x ++) {
          MACBSUnit* c((MACBSUnit*)group.second->GetMember(x));
          xytLoc cur;
          c->GetLocation(cur);
        //if(!fequal(ptime[x],sim->GetSimulationTime())
        std::cout << "\t" << x << ":" << cur << std::endl;
        }*/
      }
    }


    if(recording){
      static int index = 0;
      if(group.second->donePlanning() || index%10==9){
        char fname[255];
        if(group.second->donePlanning())
          sprintf(fname, "movies/cbs-%05d", index);
        else
          sprintf(fname, "movies/cbs-%05d", index/10);
        SaveScreenshot(windowID, fname);
        printf("Saving '%s'\n", fname);
      }
      index++;
    }
  }
}

int MyCLHandler(char *argument[], int maxNumArgs){

  if(strcmp(argument[0], "-ECBSheuristic") == 0){
    ECBSheuristic = true;
    return 1;
  }
  if(strcmp(argument[0], "-greedyCT") == 0)
  {
    greedyCT = true;
    return 1;
  }
  if(strcmp(argument[0], "-toptwo") == 0)
  {
    Params::topTwo = true;
    return 1;
  }
  if(strcmp(argument[0], "-complete") == 0)
  {
    Params::complete=true;
    return 1;
  }
  if(strcmp(argument[0], "-overload") == 0)
  {
    Params::overload=true;
    return 1;
  }
  if(strcmp(argument[0], "-suboptimal") == 0)
  {
    Params::conditional=true;
    Grid3DConstrainedEnvironment::conditional=true;
    Params::extrinsicconstraints=true;
    return 1;
  }
  if(strcmp(argument[0], "-verify") == 0)
  {
    verify = true;
    return 1;
  }
  if(strcmp(argument[0], "-cat") == 0)
  {
    useCAT = true;
    return 1;
  }
  if(strcmp(argument[0], "-random") == 0)
  {
    randomalg = true;
    return 1;
  }
  if(strcmp(argument[0], "-mapdir") == 0){
    mapdir=argument[1];
    return 2;
  }
  if(strcmp(argument[0], "-scenfile") == 0)
  {
    if(num_agents==0){
      std::cout<<"-nagents must be specified before -scenfile\n";
      exit(1);
    }
    ScenarioLoader sl(argument[1]);
    if(sl.GetNumExperiments()==0)
    {
      std::cout<<"No experiments in this scenario file or invalid file.\n";
      exit(1);
    }
    mapfile=sl.GetNthExperiment(0).GetMapName();
    mapfile.insert(0,"/"); // Add prefix
    mapfile.insert(0,mapdir); // Add prefix
    Map3D* map=new Map3D(mapfile.c_str(),dtedfile.c_str());
    if(!envdata.size()){
      for(int i(0); i<num_agents; ++i){
        std::vector<EnvData> ev;
        for(int j(0); j<sizeof(cutoffs)/sizeof(cutoffs[0]); ++j){
          if(cutoffs[j]<9999)
            ev.emplace_back(j,envnames[j],'G',cutoffs[j],weights[j]);
        }
        envdata.push_back(ev);
      }
    }

    unsigned agent(0);
    for(auto a: envdata){
      // Add start/goal location
      std::vector<xyztLoc> wpts;
      Experiment e(sl.GetNthExperiment(agent));
      unsigned sx(e.GetStartX());
      unsigned sy(e.GetStartY());
      unsigned ex(e.GetGoalX());
      unsigned ey(e.GetGoalY());
      if(a[0].agentType=='S'){
        while(map->GetTerrain(sx,sy)!=Map3D::kWater){
          sx=rand()%map->GetMapWidth();
          sy=rand()%map->GetMapHeight();
        }
        while(map->GetTerrain(ex,ey)!=Map3D::kWater){
          ex=rand()%map->GetMapWidth();
          ey=rand()%map->GetMapHeight();
        }
      }
      unsigned sz(a[0].agentType=='A'?rand()%(map->GetMapDepth()-1)+1:0);
      unsigned ez(a[0].agentType=='A'?rand()%(map->GetMapDepth()-1)+1:0);
      while(true){
        bool bad(false);
        for(auto const& w:waypoints){
          if((sx==w[0].x && sy==w[0].y && sz==w[0].z) || (ex==w[1].x && ey==w[1].y && ez==w[0].z)){
            bad=true;
            break;
          }
        }
        if(!bad)break;
        e=sl.GetNthExperiment(agent);
        sx=e.GetStartX();
        sy=e.GetStartY();
        ex=e.GetGoalX();
        ey=e.GetGoalY();
        sz=a[0].agentType=='A'?rand()%(map->GetMapDepth()-1)+1:0;
        ez=a[0].agentType=='A'?rand()%(map->GetMapDepth()-1)+1:0;
        if(a[0].agentType=='S'){
          while(map->GetTerrain(sx,sy)!=Map3D::kWater){
            sx=rand()%map->GetMapWidth();
            sy=rand()%map->GetMapHeight();
          }
          while(map->GetTerrain(ex,ey)!=Map3D::kWater){
            ex=rand()%map->GetMapWidth();
            ey=rand()%map->GetMapHeight();
          }
        }
      }
      wpts.emplace_back(sx,sy,sz);
      wpts.emplace_back(ex,ey,ez);
      waypoints.push_back(wpts);

      // Add environments
      std::vector<EnvironmentContainer<xyztLoc,t3DDirection>> ev;
      for(auto e: a){
        Grid3DConstrainedEnvironment* newEnv(nullptr);
        Grid3DEnvironment* me(nullptr);
        if(e.name=="fourconnected"){
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="fiveconnected"){
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="eightconnected"){
          me = new Grid3DEnvironment(map); me->SetOneConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="nineconnected"){
          me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="twentyfourconnected"){
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=2;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="twentyfiveconnected"){
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=2;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="fortyeightconnected"){
          me = new Grid3DEnvironment(map); me->SetThreeConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=3;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="fortynineconnected"){
          me = new Grid3DEnvironment(map); me->SetThreeConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=3;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="3dcardinal"){
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="3done"){
          me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="3dtwo"){
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetAir();
          Params::conn=2;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else{
          std::cout << "Unknown environment " << e.name << "\n";
          assert(!"Unknown environment encountered");
        }
        if(maxcost){me->SetUniqueCosts(true);me->SetMaxCost(maxcost);}
        newEnv->GetMapEnv()->setGoal(waypoints[ev.size()].back());
        newEnv->WaitTime(wait);
        ev.emplace_back(e.name,newEnv,new Map3dPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),e.threshold,e.weight);
      }
      environs.push_back(ev);
      ++agent;
    }

    return 2;
  }
  if(strcmp(argument[0], "-dtedfile") == 0)
  {
    // If this flag is used, assume there is no scenfile flag
    dtedfile=argument[1];
    return 2;
  }
  if(strcmp(argument[0], "-xor") == 0){
    Params::xorconstraints=true;
    return 1;
  }
  if(strcmp(argument[0], "-ctype") == 0)
  {
    //1: identical constraints
    //2: Pyramid constraints
    //3: Collision constraints (sub-optimal)
    //4: Time range constraints (sub-optimal)
    //5: Box constraints (sub-optimal)
    unsigned scheme(atoi(argument[1]));
    Params::crossconstraints=false;
    Params::boxconstraints=false;
    Params::timerangeconstraints=false;
    Params::pyramidconstraints=false;
    Params::mutualconstraints=false;
    Params::mutualtimerange=false;
    Params::overlapconstraints=false;
    Params::identicalconstraints=false;
    Params::extrinsicconstraints=false;
    switch(scheme){
      case 1:
        Params::identicalconstraints=true;
        break;
      case 2:
        Params::pyramidconstraints=true;
        Params::extrinsicconstraints=true;
        break;
      case 3:
        Params::crossconstraints=true;
        Params::extrinsicconstraints=true;
        break;
      case 4:
        Params::timerangeconstraints=true;
        break;
      case 5:
        Params::mutualconstraints=true;
        break;
      case 6:
        Params::overlapconstraints=true;
        Params::extrinsicconstraints=true;
        break;
      case 7:
        Params::boxconstraints=true;
        Params::extrinsicconstraints=true;
        break;
      case 8:
        Params::crossconstraints=true;
        Params::mutualtimerange=true;
        Params::extrinsicconstraints=true;
        break;
      case 9:
        Params::mutualtimerange=true;
        break;
      default:
        Params::identicalconstraints=true;
        break;
    }
    return 2;
  }
  if(strcmp(argument[0], "-skip") == 0)
  {
    Params::skip = true;
    return 1;
  }
  if(strcmp(argument[0], "-apriori") == 0)
  {
    if(environs.empty()){
      std::cout<<"-scenfile must be specified before -apriori\n";
      exit(1);
    }
    Params::apriori = true;
    loadCollisionTable<Vector3D>(Params::array, Params::indices, Params::ivls,
                       environs[0][0].environment->branchingFactor(),
                       xyztLoc::TIME_RESOLUTION_U, agentRadius, agentRadius);
    return 1;
  }
  if(strcmp(argument[0], "-pc") == 0)
  {
    Params::prioritizeConf = true;
    return 1;
  }
  if(strcmp(argument[0], "-cct") == 0)
  {
    Params::cct = true;
    return 1;
  }
  if(strcmp(argument[0], "-precheck") == 0)
  {
    Params::precheck = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-mapfile") == 0)
  {
    // If this flag is used, assume there is no scenfile flag
    mapfile=argument[1];
    std::string pathprefix("../../"); // Because I always run from the build directory...
    mapfile.insert(0,pathprefix); // Add prefix
    Map3D* map=new Map3D(mapfile.c_str(),dtedfile.c_str());
    Grid3DConstrainedEnvironment* newEnv(nullptr);
    if(envdata.size()){
      int agent(0);
      for(auto a: envdata){
        // Add environments
        std::vector<EnvironmentContainer<xyztLoc,t3DDirection>> ev;
        for(auto e: a){
          Grid3DEnvironment* me(nullptr);
          Grid3DConstrainedEnvironment* newEnv(nullptr);
          if(e.name=="fourconnected"){
            me = new Grid3DEnvironment(map); me->SetZeroConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="fiveconnected"){
            me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="eightconnected"){
            me = new Grid3DEnvironment(map); me->SetOneConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="nineconnected"){
            me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="twentyfourconnected"){
            me = new Grid3DEnvironment(map); me->SetTwoConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
            Params::conn=2;
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="twentyfiveconnected"){
            me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
            Params::conn=2;
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="fortyeightconnected"){
            me = new Grid3DEnvironment(map); me->SetThreeConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
            Params::conn=3;
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="fortynineconnected"){
            me = new Grid3DEnvironment(map); me->SetThreeConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
            Params::conn=3;
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="3dcardinal"){
            me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetAir();
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="3done"){
            me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetAir();
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else if(e.name=="3dtwo"){
            me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetAir();
            Params::conn=2;
            newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          }else{
            std::cout << "Unknown environment " << e.name << "\n";
            assert(!"Unknown environment encountered");
          }
          if(maxcost){me->SetUniqueCosts(true);me->SetMaxCost(maxcost);}
          if(waypoints.size()>agent){
            if(!map->IsTraversable(waypoints[agent][0].x,waypoints[agent][0].y,waypoints[agent][0].z,newEnv->GetMapEnv()->agentType)){
              std::cout << "ERROR : start location for agent "<<agent<<"("<<e.agentType<<") is not traversable\n";assert(false);
            }
            if(!map->IsTraversable(waypoints[agent][1].x,waypoints[agent][1].y,waypoints[agent][1].z,newEnv->GetMapEnv()->agentType)){
              std::cout << "ERROR : goal location for agent "<<agent<<"("<<e.agentType<<") is not traversable\n";assert(false);
            }
          }
          ev.emplace_back(e.name,newEnv,new Map3dPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),e.threshold,e.weight);
        }
        environs.push_back(ev);
        agent++;
      }
    }else{
      // All agents have the same environments based on the cutoffs parameter
      for(int i(0); i<num_agents; ++i){
        std::vector<EnvironmentContainer<xyztLoc,t3DDirection>> ev;
        Grid3DEnvironment* me(nullptr);
        {
          me = new Grid3DEnvironment(map); me->SetZeroConnected();
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("fourconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[0],weights[0]);
        }{
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetWaitAllowed();
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("fiveconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[1],weights[1]);
        }{
          me = new Grid3DEnvironment(map); me->SetOneConnected();
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("eightconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[2],weights[2]);
        }{
          me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetWaitAllowed();
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("nineconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[3],weights[3]);
        }{
          me = new Grid3DEnvironment(map); me->SetTwoConnected();
          Params::conn=2;
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("twentyfourconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[4],weights[4]);
        }{
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetWaitAllowed();
          Params::conn=2;
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("twentyfiveconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[5],weights[5]);
        }{
          me = new Grid3DEnvironment(map); me->SetThreeConnected();
          Params::conn=3;
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("fortyeightconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[6],weights[6]);
        }{
          me = new Grid3DEnvironment(map); me->SetThreeConnected(); me->SetWaitAllowed();
          Params::conn=3;
          me->SetGround();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("fortynineconnected",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[7],weights[7]);
        }{
          me = new Grid3DEnvironment(map); me->SetZeroConnected();
          me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("3dcardinal",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[7],weights[7]);
        }{
          me = new Grid3DEnvironment(map); me->SetOneConnected();
          me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("3done",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[7],weights[7]);
        }{
          me = new Grid3DEnvironment(map); me->SetTwoConnected();
          Params::conn=2;
          me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
          ev.emplace_back("3dtwo",newEnv,new MapPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),cutoffs[7],weights[7]);
        }
        if(maxcost){me->SetUniqueCosts(true);me->SetMaxCost(maxcost);}
        environs.push_back(ev);
      }
    }
    return 2;
  }
  if(strcmp(argument[0], "-mergeThreshold") == 0)
  {
    mergeThreshold = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-killex") == 0)
  {
    killex = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-killmem") == 0)
  {
    killmem = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-killtime") == 0)
  {
    killtime = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-uniqcost") == 0)
  {
    maxcost=atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-animate") == 0)
  {
    animate=atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-nogui") == 0)
  {
    gui = false;
    return 1;
  }
  if(strcmp(argument[0], "-quiet") == 0)
  {
    quiet = true;
    return 1;
  }
  if(strcmp(argument[0], "-astarverbose") == 0)
  {
    Params::astarverbose = true;
    return 1;
  }
  if(strcmp(argument[0], "-vc") == 0)
  {
    Params::vc = true;
    return 1;
  }
  if(strcmp(argument[0], "-asym") == 0)
  {
    Params::asym = true;
    return 1;
  }
  if(strcmp(argument[0], "-verbose") == 0)
  {
    Params::verbose = true;
    return 1;
  }
  if(strcmp(argument[0], "-disappear") == 0)
  {
    disappearAtGoal = true;
    return 1;
  }
  if(strcmp(argument[0], "-resolution") == 0)
  {
    xyztLoc::TIME_RESOLUTION_U=xyztLoc::TIME_RESOLUTION=xyztLoc::TIME_RESOLUTION_D=atof(argument[1]);
    NonUnitTimeCAT<xyztLoc, t3DDirection>::bucketWidth=xyztLoc::TIME_RESOLUTION_D;
    return 2;
  }
  if(strcmp(argument[0], "-radius") == 0)
  {
    agentRadius=atof(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-noid") == 0)
  {
    noid = true;
    return 1;
  }
  if(strcmp(argument[0], "-nobypass") == 0)
  {
    nobypass = true;
    return 1;
  }
  if(strcmp(argument[0], "-record") == 0)
  {
    recording = true;
    return 1;
  }
  if(strcmp(argument[0], "-wait") == 0)
  {
    wait = atoi(argument[1]);
    return 1;
  }
  if(strcmp(argument[0], "-cfgfile") == 0){
    std::ifstream ss(argument[1]);
    int agentNumber(0);

    std::string line;
    while(std::getline(ss, line)){
      auto ln(Util::split(line,' '));
      unsigned group(atoi(ln[0].c_str()));
      int agent(atoi(ln[1].c_str()));
      char agentType(ln[2].c_str()[0]);
      while(agent>agentNumber){
        envdata.push_back(envdata.back()); // make copies
        agentNumber++;
      }
      agentNumber++;
      auto envs(Util::split(ln[3],','));
      std::vector<EnvData> envinfo;
      for(auto e:envs){
        auto info(Util::split(e,':'));
        envinfo.emplace_back(group,info[0],agentType,atoi(info[1].c_str()),atof(info[2].c_str()));
      }
      envdata.push_back(envinfo);
    }
    while(envdata.size()<num_agents){envdata.push_back(envdata.back());} // make copies
    return 2;
  }
  if(strcmp(argument[0], "-envfile") == 0){
    std::ifstream ss(argument[1]);
    int agentNumber(0);

    std::string line;
    while(std::getline(ss, line)){
      auto ln(Util::split(line,' '));
      int agent(atoi(ln[0].c_str()));
      char agentType(ln[1].c_str()[0]);
      while(agent>agentNumber){
        envdata.push_back(envdata.back()); // make copies
        agentNumber++;
      }
      agentNumber++;
      auto envs(Util::split(ln[2],','));
      std::vector<EnvData> envinfo;
      for(auto e:envs){
        auto info(Util::split(e,':'));
        envinfo.emplace_back(agentNumber,info[0],agentType,atoi(info[1].c_str()),atof(info[2].c_str()));
      }
      envdata.push_back(envinfo);
    }
    while(envdata.size()<num_agents){envdata.push_back(envdata.back());} // make copies
    return 2;
  }
  if(strcmp(argument[0], "-probfile") == 0){
    //std::cout << "Reading instance from file: \""<<argument[1]<<"\"\n";
    int agents=0;
    std::ifstream ss(argument[1]);
    int x,y,z;
    float t(0.0);
    std::string line;
    while(std::getline(ss, line)){
      std::vector<xyztLoc> wpts;
      std::istringstream is(line);
      std::string field;
      while(is >> field){
        size_t n(std::count(field.begin(), field.end(), ','));
        if(n==1){
          sscanf(field.c_str(),"%d,%d", &x,&y);
          z=0;
          t=0;
        }else if(n==2){
          sscanf(field.c_str(),"%d,%d,%f", &x,&y,&t);
          z=0;
        }else if(n==3){
          sscanf(field.c_str(),"%d,%d,%d,%f", &x,&y,&z,&t);
        }else{
          assert(!"Invalid value inside problem file");
        }
        wpts.emplace_back(x,y,z,t);
      }
      waypoints.push_back(wpts);
      agents++;
      if(num_agents>0 && agents==num_agents){num_agents=agents;break;}
    }
    if(waypoints.size()==0){
      assert(!"Invalid filename or empty file");
    }
    return 2;
  }
  if(strcmp(argument[0], "-constraints") == 0){
    std::cout << "Reading constraints from file: \""<<argument[1]<<"\"\n";
    std::ifstream ss(argument[1]);
    int x,y,r;
    std::string line;
    while(std::getline(ss, line)){
      std::vector<xytLoc> wpts;
      std::istringstream is(line);
      std::string field;
      while(is >> field){
        sscanf(field.c_str(),"%d,%d,%d", &x,&y,&r);
        //sconstraints.push_back(SoftConstraint<xytLoc>(xytLoc(x,y,0),r));
      }
    }
    return 2;
  }
  if(strcmp(argument[0], "-cutoffs") == 0)
  {
    std::string str = argument[1];

    std::stringstream ss(str);

    int i;
    int index(0);

    while (ss >> i)
    {
      cutoffs[index++] = i;

      if (ss.peek() == ',')
        ss.ignore();
    }
    return 2;
  }
  if(strcmp(argument[0], "-weights") == 0)
  {
    std::string str = argument[1];

    std::stringstream ss(str);

    double i;
    int index(0);

    while (ss >> i)
    {
      weights[index++] = i;

      if (ss.peek() == ',')
        ss.ignore();
    }
    return 2;
  }
  if(strcmp(argument[0], "-seed") == 0)
  {
    seed = atoi(argument[1]);	
    srand(seed);
    srandom(seed);
    return 2;
  }
  if(strcmp(argument[0], "-nsubgoals") == 0)
  {
    std::string str = argument[1];

    std::stringstream ss(str);

    int i;
    ss >> i;
    minsubgoals = i;
    if (ss.peek() == ',')
      ss.ignore();
    ss >> i;
    maxsubgoals = i;
    return 2;
  }
  if(strcmp(argument[0], "-dimensions") == 0)
  {
    std::string str = argument[1];
    std::stringstream ss(str);

    int i;
    ss >> i;
    width = i;
    if (ss.peek() == ',')
      ss.ignore();
    ss >> i;
    length = i;
    if (ss.peek() == ',')
      ss.ignore();
    ss >> i;
    height = i;

    Map3D* map(new Map3D(width,length,height));
    int agent(0);
    for(auto a: envdata){
      // Add environments
      std::vector<EnvironmentContainer<xyztLoc,t3DDirection>> ev;
      for(auto e: a){
        Grid3DEnvironment* me(nullptr);
        Grid3DConstrainedEnvironment* newEnv(nullptr);
        if(e.name=="fourconnected"){
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="fiveconnected"){
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="eightconnected"){
          me = new Grid3DEnvironment(map); me->SetOneConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="nineconnected"){
          me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="twentyfourconnected"){
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=2;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="twentyfiveconnected"){
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=2;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="fortyeightconnected"){
          me = new Grid3DEnvironment(map); me->SetThreeConnected(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=3;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="fortynineconnected"){
          me = new Grid3DEnvironment(map); me->SetThreeConnected(); me->SetWaitAllowed(); e.agentType=='G'?me->SetGround():me->SetSurface();
          Params::conn=3;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="3dcardinal"){
          me = new Grid3DEnvironment(map); me->SetZeroConnected(); me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="3done"){
          me = new Grid3DEnvironment(map); me->SetOneConnected(); me->SetAir();
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else if(e.name=="3dtwo"){
          me = new Grid3DEnvironment(map); me->SetTwoConnected(); me->SetAir();
          Params::conn=2;
          newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        }else{
          std::cout << "Unknown environment " << e.name << "\n";
          assert(!"Unknown environment encountered");
        }
        if(maxcost){
          me->SetUniqueCosts(true);
          me->SetMaxCost(maxcost);
          ev.emplace_back(e.name,newEnv,new Map3dPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),e.threshold,e.weight);
        }else{
          ev.emplace_back(e.name,newEnv,nullptr,e.threshold,e.weight);
        }
        // If -dimensions is specified, there is no map
        //ev.emplace_back(e.name,newEnv,nullptr,new Map3dPerfectHeuristic<xyztLoc,t3DDirection>(map,newEnv),e.threshold,e.weight);
        newEnv->setGoal(waypoints[agent][1]);
        newEnv->WaitTime(wait);
        newEnv->GetMapEnv()->setGoal(waypoints[agent][1]);
      }
      environs.push_back(ev);
      agent++;
    }
    return 2;
  }
  if(strcmp(argument[0], "-nagents") == 0)
  {
    num_agents = atoi(argument[1]);	
    return 2;
  }
  return 1; //ignore typos
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
  xyLoc b;
  switch (key)
  {
    case 'r': recording = !recording; break;
    case '[': recording = true; break;
    case ']': recording = false; break;
    case '\t':
              if (mod != kShiftDown)
                SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
              else
              {
                SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
              }
              break;
    case '-': 
              if(stepsPerFrame>0)stepsPerFrame-=frameIncrement;
              break;
    case '=': 
              stepsPerFrame+=frameIncrement;
              break;
    case 'p': 
              paused = !paused;
              break;//unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
    case 'o':
              //			if (unitSims[windowID]->GetPaused())
              //			{
              //				unitSims[windowID]->SetPaused(false);
              //				unitSims[windowID]->StepTime(1.0/30.0);
              //				unitSims[windowID]->SetPaused(true);
              //			}
              break;
    case 'd':

              break;
    default:
              break;
  }
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{

}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
  //	// attmpt to learn!
  //	Map3D m(100, 100);
  //	Directional2DEnvironment d(&m);
  //	//Directional2DEnvironment(Map3D *m, model envType = kVehicle, heuristicType heuristic = kExtendedPerimeterHeuristic);
  //	xySpeedHeading l1(50, 50), l2(50, 50);
  //	__gnu_cxx::hash_map<uint64_t, xySpeedHeading, Hash64 > stateTable;
  //	
  //	std::vector<xySpeedHeading> path;
  //	TemplateAStar2<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> alg;
  //	alg.SetStopAfterGoal(false);
  //	alg.InitializeSearch(&d, l1, l1, path);
  //	for (int x = 0; x < 2000; x++)
  //		alg.DoSingleSearchStep(path);
  //	int count = alg.GetNumItems();
  //	LinearRegression lr(37, 1, 1/37.0); // 10 x, 10 y, dist, heading offset [16]
  //	std::vector<double> inputs;
  //	std::vector<double> output(1);
  //	for (unsigned int x = 0; x < count; x++)
  //	{
  //		// note that the start state is always at rest;
  //		// we actually want the goal state at rest?
  //		// or generate everything by backtracking through the parents of each state
  //		const AStarOpenClosedData<xySpeedHeading> val = GetItem(x);
  //		inputs[0] = sqrt((val.data.x-l1.x)*(val.data.x-l1.x)+(val.data.y-l1.)*(val.data.y-l1.y));
  //		// fill in values
  //		if (fabs(val.data.x-l1.x) >= 10)
  //			inputs[10] = 1;
  //		else inputs[1+fabs(val.data.x-l1.x)] = 1;
  //		if (fabs(val.data.y-l1.y) >= 10)
  //			inputs[20] = 1;
  //		else inputs[11+fabs(val.data.y-l1.y)] = 1;
  //		// this is wrong -- I need the possibility of flipping 15/1 is only 2 apart
  //		intputs[30+((int)(fabs(l1.rotation-val.data.rotation)))%16] = 1;
  //		output[0] = val.g;
  //		lr.train(inputs, output);
  //		// get data and learn to predict the g-cost
  //		//val.data.
  //		//val.g;
  //	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
  return false;
  mouseTracking = false;
  if (button == kRightButton)
  {
    switch (mType)
    {
      case kMouseDown:
        //unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
        //printf("Mouse down at (%d, %d)\n", px1, py1);
        break;
      case kMouseDrag:
        mouseTracking = true;
        //unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
        //printf("Mouse tracking at (%d, %d)\n", px2, py2);
        break;
      case kMouseUp:
        {
          //				if ((px1 == -1) || (px2 == -1))
          //					break;
          //				xySpeedHeading l1, l2;
          //				l1.x = px1;
          //				l1.y = py1;
          //				l2.x = px2;
          //				l2.y = py2;
          //				DirPatrolUnit *ru1 = new DirPatrolUnit(l1, dp);
          //				ru1->SetNumPatrols(1);
          //				ru1->AddPatrolLocation(l2);
          //				ru1->AddPatrolLocation(l1);
          //				ru1->SetSpeed(2);
          //				unitSims[windowID]->AddUnit(ru1);
        }
        break;
    }
    return true;
  }
  return false;
}
