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
#include <memory>
#include <iostream>
#include <bitset>
#include <iomanip>
#include <unordered_set>
#include <set>
#include <stack>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>
#include "VelocityObstacle.h"
#include "PEAStar.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "MapPerfectHeuristic.h"
#include "Utilities.h"
#include "MultiAgentEnvironment.h"
#include "Map2DEnvironment.h"
#include "Map2DConstrainedEnvironment.h"
#include "Map.h"
#include "TemporalAStar.h"

// for agents to stay at goal
#define MAXTIME 1000000
// for inflation of floats to avoid rounding errors
#define INFLATION 1000
#define TOMSECS 0.001

typedef std::pair<std::vector<xytLoc>,std::vector<xytLoc>> Instance;
std::vector<MapEnvironment*> menv;
std::vector<Map2DConstrainedEnvironment*> env;
std::vector<Heuristic<xytLoc>*> heuristics;
std::string mapfile;
std::string dtedfile;

Timer certtimer;
size_t maxnagents(0);

extern double agentRadius;
bool epp(false);
bool verbose(false);
bool quiet(false);
bool verify(false);
bool ID(true);
bool precheck(true);
bool mouseTracking;
unsigned agentType(5);
unsigned killtime(300);
unsigned killmem(1024); // 1GB
int width = 64;
int length = 64;
int height = 0;
bool recording = false;
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double frameIncrement = 1.0/10000.0;
bool paused = false;
bool gui=true;
uint64_t jointnodes(0);
uint64_t branchingfactor(0);
uint64_t largestbranch(0);
float largestJoint(1);
uint32_t step(INFLATION);
int n(0);
std::unordered_map<std::string,bool> transTable;
std::unordered_map<uint64_t,bool> singleTransTable;
unsigned seed(clock());

std::string filepath;
std::vector<std::vector<xytLoc> > waypoints;

UnitSimulation<xytLoc, tDirection, Map2DConstrainedEnvironment> *sim = 0;

class Edge:public std::pair<xytLoc,xytLoc>{
  public:
    Edge(xytLoc const& a, xytLoc const& b):std::pair<xytLoc,xytLoc>(a,b),t(b.t){}
    uint32_t t;
};

void CreateSimulation(int id)
{
        SetNumPorts(id, 1);
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

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
  switch (key)
  {
    case 'r': recording = !recording; break;
    case 'p': paused = !paused; break;
    default: break;
  }
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
  return false;
  mouseTracking = false;
  if (button == kRightButton)
  {
    switch (mType)
    {
      case kMouseDown: break;
      case kMouseDrag: mouseTracking = true; break;
      case kMouseUp: break;
    }
    return true;
  }
  return false;
}

void InitSim(){
}

void MyComputationHandler()
{
  while (true)
  {
    sim->StepTime(stepsPerFrame);
  }
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
  if(sim){sim->OpenGLDraw();}
  if (!paused) {sim->StepTime(stepsPerFrame);}
  if (recording) {
    static int index = 0;
    char fname[255];
    sprintf(fname, "movies/cbs-%05d", index);
    SaveScreenshot(windowID, fname);
    printf("Saving '%s'\n", fname);
    index++;
  }
}

void InstallHandlers()
{
  InstallCommandLineHandler(MyCLHandler, "-dimensions", "-dimensions width,length,height", "Set the length,width and height of the environment (max 65K,65K,1024).");
  InstallCommandLineHandler(MyCLHandler, "-scenfile", "-scenfile", "Scenario file to use");
  InstallCommandLineHandler(MyCLHandler, "-mapfile", "-mapfile", "Map file to use");
  InstallCommandLineHandler(MyCLHandler, "-dtedfile", "-dtedfile", "Map file to use");
  InstallCommandLineHandler(MyCLHandler, "-agentType", "-agentType [5,9,25,49]","Set the agent movement model");
  InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
  InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime [value]", "Kill after this many seconds");
  InstallCommandLineHandler(MyCLHandler, "-killmem", "-killmem [value megabytes]", "Kill if a process exceeds this size in memory");
  InstallCommandLineHandler(MyCLHandler, "-radius", "-radius [value]", "Radius in units of agent");
  InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
  InstallCommandLineHandler(MyCLHandler, "-epp", "-epp", "Nogood pruning enhancement");
  InstallCommandLineHandler(MyCLHandler, "-quiet", "-quiet", "Turn off trace output");
  InstallCommandLineHandler(MyCLHandler, "-verbose", "-verbose", "Turn on verbose output");
  InstallCommandLineHandler(MyCLHandler, "-verify", "-verify", "Verify results");
  InstallCommandLineHandler(MyCLHandler, "-noID", "-noID", "No Independence Dection (ID) framework");
  InstallCommandLineHandler(MyCLHandler, "-noprecheck", "-noprecheck", "Perform simplified collision check before trying the expensive one");
  InstallCommandLineHandler(MyCLHandler, "-mode", "-mode s,b,p,a", "s=sub-optimal,p=pairwise,b=pairwise,sub-optimal,a=astar");
  InstallCommandLineHandler(MyCLHandler, "-increment", "-increment [value]", "High-level increment");
  InstallCommandLineHandler(MyCLHandler, "-seed", "-seed <number>", "Seed for random number generator (defaults to clock)");
  InstallCommandLineHandler(MyCLHandler, "-nagents", "-nagents <number>", "Select the number of agents.");

  InstallWindowHandler(MyWindowHandler);
  InstallMouseClickHandler(MyClickHandler);
}

//int renderScene(){return 1;}

typedef std::unordered_set<int> Group;

// Compute path cost, ignoring actions that wait at the goal
uint32_t computeSolutionCost(Solution<xytLoc> const& solution, bool ignoreWaitAtGoal=true){
  uint32_t cost(0);
  if(ignoreWaitAtGoal){
    for(auto const& path:solution){
      for(int j(path.size()-1); j>0; --j){
        if(path[j-1]!=path[j]){
          cost += path[j].t;
          break;
        }else if(j==1){
          cost += path[0].t;
        }
      }
    }
  }else{
    for(auto const& path:solution){
      cost+=path.back().t;
    }
  }
  return cost;
}


// Check that two paths have no collisions
bool checkPair(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2,bool loud=false){
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    Vector2D A((*ap).x,(*ap).y);
    Vector2D B((*bp).x,(*bp).y);
    Vector2D VA((*a).x-(*ap).x,(*a).y-(*ap).y);
    VA.Normalize();
    Vector2D VB((*b).x-(*bp).x,(*b).y-(*bp).y);
    VB.Normalize();
    if(collisionImminent(A,VA,agentRadius,(*ap).t,(*a).t,B,VB,agentRadius,(*bp).t,(*b).t)){
      if(loud)std::cout << "Collision: " << *ap << "-->" << *a << "," << *bp << "-->" << *b;
      return false;
    }
    if((*a).t<(*b).t){
      ++a;
      ++ap;
    }else if((*a).t>(*b).t){
      ++b;
      ++bp;
    }else{
      ++a;++b;
      ++ap;++bp;
    }
  }
  return true;
}

// Not part of the algorithm... just for validating the answers
bool checkAnswer(Solution<xytLoc> const& answer){
  for(int i(0);i<answer.size();++i){
    for(int j(i+1);j<answer.size();++j){
      if(!checkPair(answer[i],answer[j],verify)){
        if(verify)std::cout<< "for agents: " << i << " and " << j << "\n";
        return false;
      }
    }
  }
  return true;
}

bool detectIndependence(Solution<xytLoc>& solution, std::vector<Group*>& group, std::unordered_set<Group*>& groups){
  bool independent(true);
  // Check all pairs for collision
  uint32_t minTime(-1);
  for(int i(0); i<solution.size(); ++i){
    for(int j(i+1); j<solution.size(); ++j){
      // check collision between i and j
      int a(1);
      int b(1);
      if(solution[i].size() > a && solution[j].size() > b){
        //uint32_t t(min(solution[i][a].t,solution[j][b].t));
        while(1){
          if(a==solution[i].size() || b==solution[j].size()){break;}
          Vector2D A(solution[i][a-1].x,solution[i][a-1].y);
          Vector2D B(solution[j][b-1].x,solution[j][b-1].y);
          Vector2D VA(solution[i][a].x-solution[i][a-1].x,solution[i][a].y-solution[i][a-1].y);
          VA.Normalize();
          Vector2D VB(solution[j][b].x-solution[j][b-1].x,solution[j][b].y-solution[j][b-1].y);
          VB.Normalize();
          if(collisionImminent(A,VA,agentRadius,solution[i][a-1].t,solution[i][a].t,B,VB,agentRadius,solution[j][b-1].t,solution[j][b].t)){
            if(!quiet)std::cout << i << " and " << j << " collide at " << solution[i][a-1].t << "~" << solution[i][a].t << solution[i][a-1] << "-->" << solution[i][a] << " X " << solution[j][b-1] << "-->" << solution[j][b] << "\n";
            independent=false;
            if(group[i]==group[j]) break; // This can happen if both collide with a common agent
            // Combine groups i and j

            Group* toDelete(group[j]);
            groups.erase(group[j]);
            for(auto a:*group[j]){
              if(verbose)std::cout << "Inserting agent " << a << " into group for agent " << i << "\n";
              group[i]->insert(a);
              group[a]=group[i];
              maxnagents=std::max(group[i]->size(),maxnagents);
            }
            delete toDelete;

            break;
          }
          if(solution[i][a].t==solution[j][b].t){
            ++a;++b;
          }else if(solution[i][a].t<solution[j][b].t){
            ++a;
          }else{++b;}
        }
      }
    }
  }
  return independent;
}

Solution<xytLoc> solution;
double total(0.0);
uint32_t nacts(0.0);
int failed(0);
uint32_t cost(0);
Timer tmr;

void printResults(){
  if(verbose){
    std::cout << "Solution:\n";
    int ii=0;
    for(auto const& p:solution){
      std::cout << ii++ << "\n";
      for(auto const& t: p){
        // Print solution
        std::cout << t << "\n";
      }
    }
  }
  for(auto const& path:solution){
    for(int j(path.size()-1); j>0; --j){
      if(!path[j-1].sameLoc(path[j])){
        cost += path[j].t;
        nacts += j;
        if(verbose)std::cout << "Adding " << path[j]<<","<<path[j].t<<"\n";
        break;
      }else if(j==1){
        cost += path[0].t;
        nacts += 1;
        if(verbose)std::cout << "Adding_" << path[0]<<","<<path[0].t<<"\n";
      }
    }
  }
  if(!quiet)std::cout << std::endl;
  total=tmr.EndTimer();
  if(total<killtime&&verify){
    if(!checkAnswer(solution)) std::cout << "INVALID!\n";
    else if(!quiet) std::cout << "VALID\n";
  }
  
  //std::cout << elapsed << " elapsed";
  //std::cout << std::endl;
  //total += elapsed;
  if(!quiet)std::cout << "seed:filepath,jointnodes,largestJoint,maxnagents,total,nacts,cost\n";
  std::cout << seed << ":" << filepath << "," << jointnodes << "," <<largestJoint << "," << maxnagents << "," << total << "," << nacts << "," << cost;
  if(total >= killtime)std::cout << " failure";
  std::cout << std::endl;
  if(total>=killtime)exit(1);
}

// Scan the candidate answers and merge a non-conflicting answer set in with
// the main solution. Return the added cost amount. Returns zero if no solutions
// could be merged, or the cost of the valid solution is higher than the maximum
// allowable cost.
// returns the merged cost or zero if unable to merge and the best cost
// (the best solution is merged in either case).
std::pair<uint32_t,uint32_t> mergeSolution(std::vector<Solution<xytLoc>>& answers, Solution<xytLoc>& s, std::vector<int> const& insiders, uint32_t maxMergedCost){
  // Compute agents not in the answer group
  std::vector<int> outsiders;
  for(int k(0); k<s.size(); ++k){
    bool found(false);
    for(int i(0); i<answers[0].size(); ++i){
      if(k==insiders[i]){
        found=true;
        break;
      }
    }
    if(!found)
      outsiders.push_back(k);
  }
  std::vector<int> sorted(answers.size());
  std::iota(sorted.begin(),sorted.end(),0); // Fill with 0,1,2,3...
  std::vector<uint32_t> costs(answers.size());
  bool allSame(true);
  costs[0]=computeSolutionCost(answers[0]);
  for(int i(1); i<answers.size(); ++i){
    costs[i]=computeSolutionCost(answers[i]);
    if(allSame&&!costs[0]==costs[i]){allSame=false;}
  }
  if(!allSame){
    // Sort the cost indices
    std::sort(sorted.begin(),sorted.end(), [&](int a, int b){ return costs[a]<costs[b]; });
  }
  // Check all answers against current paths in solution outside of the group
  for(auto index:sorted){
    if(costs[index]>=maxMergedCost){
      break; // Nothing else will be good to merge (because of sorting)
    }
    auto const& ans(answers[index]);
    bool allValid(true);
    for(int i(0); i<ans.size(); ++i){
      for(int j:outsiders){
        if(!checkPair(ans[i],s[j])){
          allValid=false;
          break;
        }
      }
      if(!allValid) break;
    }
    // If a conflict free set is found, merge it and return the cost
    if(allValid){
      for(int i(0); i<ans.size(); ++i){
        s[insiders[i]].resize(0);
        for(auto& a:ans[i]){
          s[insiders[i]].push_back(a);
        }
      }
      // We have found a feasible global solution
      if(verbose)std::cout<<"Feasible solution merged\n";
      return {costs[index],costs[sorted[0]]};
    }
  }
  if(verbose)std::cout << "Solution merged but not conflict-free\n";
  if(maxMergedCost==INF){
    // No merge took place, just merge the first/best answer
    for(int i(0); i<answers[0].size(); ++i){
      s[insiders[i]].resize(0);
      for(auto& a:answers[0][i]){
        s[insiders[i]].push_back(a);
      }
    }
  }
  return {0.0f,costs[sorted[0]]};
}

int main(int argc, char ** argv){
  
  InstallHandlers();
  ProcessCommandLineArgs(argc, argv);
  Util::setmemlimit(killmem);
  Map* map(nullptr);
  if(mapfile.empty()){
    map = new Map(width,length);
  }else{
    map = new Map(mapfile.c_str());
  }

  for(int i(0); i<waypoints.size(); ++i){
    menv.push_back(new MapEnvironment(map));
    //menv.back()->SetGround();
    //menv.back()->SetWaitAllowed();
    env.push_back(new Map2DConstrainedEnvironment(menv.back()));
    env.back()->SetIgnoreHeading(true);

    switch(agentType){
      case 8:
        menv.back()->SetEightConnected();
      case 9:
        menv.back()->SetNineConnected();
        break;
      case 24:
        menv.back()->SetTwentyFourConnected();
      case 25:
        menv.back()->SetTwentyFiveConnected();
        break;
      case 48:
        menv.back()->SetFortyEightConnected();
      case 49:
        menv.back()->SetFortyNineConnected();
        break;
      default:
      case 4:
        menv.back()->SetFourConnected();
      case 5:
        menv.back()->SetFiveConnected();
        break;
    }
  }

  heuristics.resize(waypoints.size());
  if(mapfile.empty()){
    for(int i(0); i<heuristics.size(); ++i){
      heuristics[i]=env[i];
    }
  }else{
    for(int i(0); i<heuristics.size(); ++i){
      heuristics[i] = new MapPerfectHeuristic(map,env[i]);
    }
  }

  //TemporalAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  TemplateAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  //astar.SetVerbose(true);
  //std::cout << "Init groups\n";
  std::vector<Group*> group(n);
  std::unordered_set<Group*> groups;
    // Add a singleton group for all groups
    for(int i(0); i<n; ++i){
      //group[i]=new Group(i); // Initially in its own group
      group[i]=new Group();
      group[i]->insert(i);
      groups.insert(group[i]);
    }

  if(ID){
    // Initial individual paths.
    for(int i(0); i<n; ++i){
      std::vector<xytLoc> path;
      if(waypoints[i][0]==waypoints[i][1]){ // Already at goal
        path.push_back(waypoints[i][0]);
      }else{
        astar.SetVerbose(verbose);
        astar.SetHeuristic(heuristics[i]);
        env[i]->GetMapEnv()->setGoal(waypoints[i][1]); // For wait Actions
        astar.GetPath(env[i],waypoints[i][0],waypoints[i][1],path);
        if(!quiet)std::cout<<"Planned agent "<<i<<"\n";
      }
      std::vector<xytLoc> timePath;
      //std::cout << s[i] << "-->" << e[i] << std::endl;
      if(path.empty()){std::cout << "AStar failed on instance " << i << " - No solution\n"; return 0;}
      for(int i(0); i<path.size(); ++i){
        timePath.push_back(path[i]);
      }
      timePath.push_back(xytLoc(timePath.back(),MAXTIME)); // Add a final wait action that goes way out...
      solution.push_back(timePath);
    }
    if(verbose){
      std::cout << std::endl;
      std::cout << "Initial solution:\n";
      int ii(0);
      for(auto const& p:solution){
        std::cout << ii++ << "\n";
        for(auto const& t: p){
          // Print solution
          std::cout << t << "," << t.t << "\n";
        }
      }
    }
  }else{
    std::vector<xytLoc> path;
    path.emplace_back(xytLoc(0,0,0.0f));
    path.emplace_back(xytLoc(0,1,1.0f));
    for(int i(0); i<n; ++i){
      solution.push_back(path);
    }
  }

  // Start timing
  tmr.StartTimer();

  Timer::Timeout func(std::bind(&printResults));
  tmr.StartTimeout(std::chrono::seconds(killtime),func);
  while(!detectIndependence(solution,group,groups)){
    float maxTime(0);
    for(auto const& p:solution){
      if(p.back().t!=MAXTIME){
        maxTime=std::max(maxTime,p.back().t);
      }else{
        maxTime=std::max(maxTime,(p.rbegin()+1)->t);
      }
      
    }
    largestJoint = float(n)/float(groups.size());
    std::unordered_map<int,Instance> G;
    std::unordered_map<int,std::vector<int>> Gid;
    if(verbose)std::cout << "There are " << groups.size() << " groups" << std::endl;
    // Create groups
    int g(0);
    for(auto const& grp:groups){
      for(auto a:*grp){
        G[g].first.push_back(waypoints[a][0]);
        G[g].second.push_back(waypoints[a][1]);
        Gid[g].push_back(a);
      }
      ++g;
    }
    for(int j(0); j<G.size(); ++j){
      auto g(G[j]);
      if(!quiet)std::cout << "Group " << j <<":\n";
      if(!quiet)for(int i(0); i<g.first.size(); ++i){
        std::cout << Gid[j][i] << ":" << g.first[i] << "-->" << g.second[i] << "\n";
      }
      // Copy env pointers to vector
      std::vector<Map2DConstrainedEnvironment const*> envs(g.first.size());
      std::vector<Heuristic<xytLoc> const*> heu(g.first.size());
      for(int i(0); i<g.first.size(); ++i){
        envs[i]=env[Gid[j][i]];
        heu[i]=heuristics[Gid[j][i]];
      }

      if(g.first.size()>1){
        MultiAgentEnvironment<Edge,tDirection,Map2DConstrainedEnvironment> mae(envs);
        TemporalAStar<MAState<Edge>, MultiAgentEnvironment<Edge,tDirection,Map2DConstrainedEnvironment>::MultiAgentAction, MultiAgentEnvironment<Edge,tDirection,Map2DConstrainedEnvironment> > astar;

        MAState<Edge> start;
        MAState<Edge> goal;
        for(int i(0); i<g.first.size(); ++i){
          start.emplace_back(g.first[i],g.first[i]);
          goal.emplace_back(g.second[i],g.second[i]);
        }

        std::vector<MAState<Edge>> path;
        //astar.SetVerbose(verbose);
        astar.GetPath(&mae,start,goal,path,maxTime);

        std::vector<Solution<xytLoc>> answers(1);
        answers[0].resize(Gid[j].size());
        for(int i(0); i<Gid[j].size();++i){
          answers[0][i].reserve(path.size());
          for(auto const& a:path){
            if(!answers[0][i].size() || (answers[0][i].size() && !answers[0][i].back().sameLoc(a[i].second))){
              answers[0][i].push_back(a[i].second);
            }
          }
          //answers[0][i].push_back(xytLoc(answers[0][i].back(),MAXTIME)); // Add a final wait action that goes way out...
        }
          
        double bestMergedCost(999999999.0);
        mergeSolution(answers,solution,Gid[j],bestMergedCost);
      }
    }
  }
  printResults();
  return 0;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
  if(strcmp(argument[0], "-noprecheck") == 0)
  {
    precheck = false;
    return 1;
  }
  if(strcmp(argument[0], "-noID") == 0)
  {
    ID = false;
    return 1;
  }
  if(strcmp(argument[0], "-verify") == 0)
  {
    verify = true;
    return 1;
  }
  if(strcmp(argument[0], "-seed") == 0)
  {
    seed = atoi(argument[1]);
    srand(seed);
    srandom(seed);
    return 2;
  }
  if(strcmp(argument[0], "-nagents") == 0)
  {
    n = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-agentType") == 0)
  {
    agentType = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-killtime") == 0)
  {
    killtime = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-killmem") == 0)
  {
    killmem = atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-epp") == 0)
  {
    epp = true;
    return 1;
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
  if(strcmp(argument[0], "-verbose") == 0)
  {
    verbose = true;
    quiet=false;
    return 1;
  }
  if(strcmp(argument[0], "-radius") == 0)
  {
    agentRadius=atof(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-nagents") == 0)
  {
    n=atoi(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-increment") == 0)
  {
    step=atof(argument[1])*INFLATION;
    return 2;
  }
  if(strcmp(argument[0], "-probfile") == 0){
    if(!quiet)std::cout << "Reading instance from file: \""<<argument[1]<<"\"\n";
    filepath=argument[1];
    std::ifstream ss(argument[1]);
    int x,y;
    uint32_t t(0.0);
    std::string line;
    n=0;
    while(std::getline(ss, line)){
      std::vector<xytLoc> wpts;
      std::istringstream is(line);
      std::string field;
      while(is >> field){
        size_t a(std::count(field.begin(), field.end(), ','));
        if(a==1){
          sscanf(field.c_str(),"%d,%d", &x,&y);
        }else if(a==2){
          sscanf(field.c_str(),"%d,%d,%f", &x,&y,&t);
        }else{
          assert(!"Invalid value inside problem file");
        }
        wpts.emplace_back(x,y,t);
      }
      waypoints.push_back(wpts);
      n++;
    }
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
    return 2;
  }
  if(strcmp(argument[0], "-scenfile") == 0)
  {
    //if(n==0){
      //std::cout<<"-nagents must be specified before -scenfile\n";
      //exit(1);
    //}
    ScenarioLoader sl(argument[1]);
    if(sl.GetNumExperiments()==0)
    {
      std::cout<<"No experiments in this scenario file or invalid file.\n";
      exit(1);
    }
    std::string pathprefix("../../"); // Because I always run from the build directory...
    mapfile=sl.GetNthExperiment(0).GetMapName();
    mapfile.insert(0,pathprefix); // Add prefix

    n=sl.GetNumExperiments();
    for(int i(0); i<n; ++i){
      // Add start/goal location
      std::vector<xytLoc> wpts;
      ///Experiment e(sl.GetRandomExperiment());
      Experiment e(sl.GetNthExperiment(i));
      while(false){
        bool bad(false);
        for(auto const& w:waypoints){
          if((e.GetStartX()==w[0].x && e.GetStartY()==w[0].y) || (e.GetGoalX()==w[1].x && e.GetGoalY()==w[1].y)){
            bad=true;
            break;
          }
        }
        if(!bad)break;
        e=sl.GetRandomExperiment();
      }
      wpts.emplace_back(e.GetStartX(),e.GetStartY());
      wpts.emplace_back(e.GetGoalX(),e.GetGoalY());
      waypoints.push_back(wpts);
    }

    return 2;
  }
  if(strcmp(argument[0], "-dtedfile") == 0)
  {
    // If this flag is used, assume there is no scenfile flag
    dtedfile=argument[1];
    return 2;
  }
  if(strcmp(argument[0], "-mapfile") == 0)
  {
    // If this flag is used, assume there is no scenfile flag
    mapfile=argument[1];
    std::string pathprefix("../../"); // Because I always run from the build directory...
    mapfile.insert(0,pathprefix); // Add prefix
    if(n==0){
      std::cout<<"-nagents must be specified before -mapfile\n";
      exit(1);
    }
    return 2;
  }
  return 1;
}
