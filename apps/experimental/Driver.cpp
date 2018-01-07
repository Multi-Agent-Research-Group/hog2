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
#include "Grid3DConstrainedEnvironment.h"
#include "VelocityObstacle.h"
#include "PEAStar.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "Map3dPerfectHeuristic.h"
#include "Utilities.h"

// for agents to stay at goal
double MAXTIME=0xfffff; // 20 bits worth
// for inflation of floats to avoid rounding errors
double INFLATION=xyztLoc::TIME_RESOLUTION_D;
double TOMSECS=1.0/INFLATION;
double waitTime=1;


std::vector<Heuristic<xyztLoc>*> heuristics;
std::vector<Grid3DConstrainedEnvironment*> envs;
std::string mapfile;
std::string dtedfile;

struct MDDStats{
  unsigned depth;
  unsigned count;
  unsigned goals;
  double branchingfactor;
};

struct JointStats{
  std::vector<MDDStats> mdds;
  unsigned depth;
  unsigned count;
  unsigned goals;
  double branchingfactor;
};

std::string currentICT;
std::vector<JointStats> jointstats;

float jointTime(0);
float pairwiseTime(0);
float mddTime(0);
float nogoodTime(0);
float certifyTime(0);
Timer certtimer;
unsigned mdddepth(0);
unsigned mddbranchingfactor(0);
unsigned mddnonleaf(0);
unsigned mddgoals(0);
unsigned jointdepth(0);
unsigned jointbranchingfactor(0);
unsigned jointnonleaf(0);
unsigned jointgoals(0);
unsigned maxsingle(0);
unsigned minsingle(INF);
unsigned maxjoint(0);
unsigned minjoint(INF);
size_t maxnagents(0);
unsigned collChecks(0);

extern double agentRadius;
bool verbose(false);
bool quiet(false);
bool verify(false);
bool ID(true);
bool OD(true);
bool fulljoint(false);
bool crazystats(false);
bool precheck(true);
bool mouseTracking;
unsigned agentType(5);
unsigned killtime(300);
unsigned killmem(2048); // 1GB
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
uint64_t jointexpansions(0);
uint64_t largestbranch(0);
float largestJoint(0);
uint32_t step(INFLATION);
int n(0);
std::unordered_map<std::string,bool> transTable;
std::unordered_map<uint64_t,bool> singleTransTable;
unsigned seed(clock());

std::string filepath;
std::vector<std::vector<xyztLoc> > waypoints;

UnitSimulation<xyztLoc, t3DDirection, Grid3DEnvironment> *sim = 0;

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
  xyztLoc b;
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
  if (recording && !paused) {
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
  InstallCommandLineHandler(MyCLHandler, "-agentType", "-agentType [5,9,25,49]","Set the agent movement model");
  InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
  InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime [value]", "Kill after this many seconds");
  InstallCommandLineHandler(MyCLHandler, "-killmem", "-killmem [value megabytes]", "Kill if a process exceeds this size in memory");
  InstallCommandLineHandler(MyCLHandler, "-radius", "-radius [value]", "Radius in units of agent");
  InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
  InstallCommandLineHandler(MyCLHandler, "-quiet", "-quiet", "Turn off trace output");
  InstallCommandLineHandler(MyCLHandler, "-verbose", "-verbose", "Turn on verbose output");
  InstallCommandLineHandler(MyCLHandler, "-verify", "-verify", "Verify results");
  InstallCommandLineHandler(MyCLHandler, "-timeRes", "-timeRes", "Time resolution (size of one unit of time)");
  InstallCommandLineHandler(MyCLHandler, "-waitTime", "-waitTime", "Time duration for wait actions");
  InstallCommandLineHandler(MyCLHandler, "-noID", "-noID", "No Independence Dection (ID) framework");
  InstallCommandLineHandler(MyCLHandler, "-noOD", "-noOD", "No Operator Decomposition (OD)");
  InstallCommandLineHandler(MyCLHandler, "-fulljoint", "-fulljoint", "Turn off early exit from jointDFS");
  InstallCommandLineHandler(MyCLHandler, "-stats", "-stats", "Turn off early exit from jointDFS");
  InstallCommandLineHandler(MyCLHandler, "-noprecheck", "-noprecheck", "Perform simplified collision check before trying the expensive one");
  InstallCommandLineHandler(MyCLHandler, "-mode", "-mode s,b,p,a", "s=sub-optimal,p=pairwise,b=pairwise,sub-optimal,a=astar");
  InstallCommandLineHandler(MyCLHandler, "-increment", "-increment [value]", "High-level increment");
  InstallCommandLineHandler(MyCLHandler, "-seed", "-seed <number>", "Seed for random number generator (defaults to clock)");
  InstallCommandLineHandler(MyCLHandler, "-nagents", "-nagents <number>", "Select the number of agents.");

  InstallWindowHandler(MyWindowHandler);
  InstallMouseClickHandler(MyClickHandler);
}

typedef std::unordered_set<int> Group;

typedef std::vector<xyztLoc> Points;
typedef std::vector<xyztLoc> Path;
//typedef std::vector<std::vector<xyztLoc>> Solution;

typedef std::vector<xyztLoc> MultiState; // rank=agent num
typedef std::vector<std::pair<xyztLoc,xyztLoc>> MultiEdge; // rank=agent num
typedef std::vector<std::pair<xyztLoc,xyztLoc>> Instance; // rank=agent num

// OPEN list is just an ordered set of unique f-values representing plateaus in the search
std::set<uint64_t> open;
// Buckets of nodes per f-cost
std::unordered_map<uint64_t,std::vector<MultiEdge>> buckets;

static inline bool get(uint64_t const* bitarray, uint64_t idx) {
  return bitarray[idx / 64] & (1UL << (idx % 64));
}
static inline void set(uint64_t* bitarray, uint64_t idx) {
  bitarray[idx / 64] |= (1UL << (idx % 64));
}

std::ostream& operator << (std::ostream& ss, Group const* n){
  std::string sep("{");
  for(auto const& a: *n){
    ss << sep << a;
    sep=",";
  }
  ss << "}";
  return ss;
}

std::ostream& operator << (std::ostream& ss, MultiState const& n){
  int i(0);
  for(auto const& a: n)
    ss << " "<<++i<<"." << a;
  return ss;
}

std::ostream& operator << (std::ostream& ss, MultiEdge const& n){
  int i(0);
  for(auto const& a: n)
    ss << " "<<++i<<"." << a.second;
  ss << std::endl;
  /*
  for(auto const& m:n.successors)
    ss << "----"<<m;
  */
  //n.Print(ss,0);
  return ss;
}

// Compute path cost, ignoring actions that wait at the goal
uint32_t computeSolutionCost(Solution<xyztLoc> const& solution, bool ignoreWaitAtGoal=true){
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


void generatePermutations(std::vector<MultiEdge>& positions, std::vector<MultiEdge>& result, int agent, MultiEdge const& current, std::vector<Grid3DConstrainedEnvironment*> const& envs) {
  if(agent == positions.size()) {
    result.push_back(current);
    if(verbose)std::cout << "Generated joint move:\n";
    if(verbose)for(auto edge:current){
      std::cout << edge.first << "-->" << edge.second << "\n";
    }
    jointnodes++;
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    //std::cout << "AGENT "<< i<<":\n";
    MultiEdge copy(current);
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      collChecks++;
      if(collisionCheck3D(positions[agent][i].first,positions[agent][i].second,current[j].first,current[j].second,agentRadius)){
        if(verbose)std::cout << "Collision averted: " << positions[agent][i].first << "-->" << positions[agent][i].second << " " << current[j].first << "-->" << current[j].second << "\n";
        found=true;
        //checked.insert(hash);
        break;
      }
      if(verbose)std::cout << "generating: " << positions[agent][i].first << "-->" << positions[agent][i].second << " " << current[j].first << "-->" << current[j].second << "\n";
    }
    if(found) continue;
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy, envs);
  }
}

  // Compute hash for transposition table
void getHash(MultiEdge const&a,std::string& hash){
  int i(0);
  for(auto v:a){
    uint64_t h1(envs[0]->GetStateHash(v.second));
    uint8_t c[sizeof(uint64_t)];
    memcpy(c,&h1,sizeof(uint64_t));
    for(unsigned j(0); j<sizeof(uint64_t); ++j){
      hash[i*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
    }
    ++i;
  }
}
unsigned jointFCost(MultiEdge const& node, std::vector<Heuristic<xyztLoc>*> const& heuristic, std::vector<Grid3DConstrainedEnvironment*> const& envs){
  double total(0.0);
  int i(0);
  for(auto const& n:node){
    // G-cost + H-cost
    total+=n.second.t+heuristic[i]->HCost(n.second,envs[i++]->getGoal())*INFLATION;
  }
  return round(total);
}

// In order for this to work, we cannot generate sets of positions, we must generate sets of actions, since at time 1.0 an action from parent A at time 0.0 may have finished, while another action from the same parent A may still be in progress. 

// This is a joint f-limited DFS
bool jointDFS(MultiEdge const& s, Solution<xyztLoc> solution, std::vector<Solution<xyztLoc>>& solutions, std::vector<Grid3DConstrainedEnvironment*> const& env, std::vector<Heuristic<xyztLoc>*> const& heuristic, unsigned flimit, bool checkOnly=true){
  // Compute hash for transposition table
  std::string hash(s.size()*sizeof(uint64_t),1);
  getHash(s,hash);

  if(verbose)std::cout << "saw " << s << " hash ";
  if(verbose)for(unsigned int i(0); i<hash.size(); ++i){
    std::cout << (unsigned)hash[i]<<" ";
  }
  if(verbose)std::cout <<"\n";
  if(transTable.find(hash)!=transTable.end()){
    //std::cout << "AGAIN!\n";
    return transTable[hash];
    //if(!transTable[hash]){return false;}
  }

// TODO ... this is tricky... don't want to make it too expensive
/*
  if(!checkOnly&&s[0].first!=s[0].second){ // Don't do this for the root node
    // Copy solution so far, but only copy components if they are
    // a valid successor
    for(int i(0); i<solution.size(); ++i){
      auto const& p = solution[i];
      bool found(false);
      for(auto const& suc:p.back()->successors){
        // True successor or wait action...
        if(*s[i].second==*suc){
          found=true;
          break;
        }
      }
      if(found || (s[i].second.sameLoc(p.back()) && s[i].second.t>p.back().t)){
        solution[i].push_back(s[i].second);
      }
    }
  }
*/
  
  bool done(true);
  unsigned i(0);
  for(auto const& g:s){
    if(!env[i]->GoalTest(g.second,env[i]->getGoal())){
      done=false;
      break;
    }
    //maxCost=std::max(maxCost,g.second.t);
  }
  if(done){
    jointgoals++;
    // This is a leaf node
    // Copy the solution into the answer set
    if(!checkOnly){
      // Shore up with wait actions
      for(int i(0); i<solution.size(); ++i){
        if(solution[i].back().t<MAXTIME){
          solution[i].push_back(xyztLoc(solution[i].back(),unsigned(MAXTIME)));
        }
      }
      solutions.clear();
      solutions.push_back(solution);
    }
    return true;
  }
  //Get successors into a vector
  std::vector<MultiEdge> successors;
  successors.reserve(s.size());

  // Find minimum depth of current edges
  uint32_t sd(INF);
  unsigned minindex(0);
  int k(0);
  for(auto const& a: s){
    if(a.second.t<sd){
      sd=a.second.t;
      minindex=k;
    }
    k++;
    //sd=min(sd,a.second.t);
  }
  //std::cout << "min-depth: " << sd << "\n";

  uint32_t md(INF); // Min depth of successors
  //Add in successors for parents who are equal to the min
  k=0;
  for(auto const& a: s){
    MultiEdge output;
    if((OD && (k==minindex || a.second.t==0)) || (!OD && a.second.t<=sd)){
      //std::cout << "Keep Successors of " << *a.second << "\n";
      MultiState succ;
      envs[k]->GetSuccessors(a.second,succ);
      for(auto const& b: succ){
        output.emplace_back(a.second,b);
        md=min(md,b.t);
      }
    }else{
      //std::cout << "Keep Just " << *a.second << "\n";
      output.push_back(a);
      md=min(md,a.second.t);
    }
    if(output.empty()){
      // Stay at state...
      output.emplace_back(a.second,xyztLoc(a.second.x,a.second.y,a.second.z,unsigned(MAXTIME)));
      //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
      //md=min(md,a.second.t+1.0); // Amount of time to wait
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
    ++k;
  }
  if(verbose){
    std::cout << "Move set\n";
    for(int a(0);a<successors.size(); ++a){
      std::cout << "agent: " << a << "\n";
      for(auto const& m:successors[a]){
        std::cout << "  " << m.first << "-->" << m.second << "\n";
      }
    }
  }
  std::vector<MultiEdge> crossProduct;
  MultiEdge tmp;
  generatePermutations(successors,crossProduct,0,tmp,envs);
  if(crossProduct.size()){
    jointexpansions++;
    largestbranch=std::max(largestbranch,crossProduct.size());
    jointbranchingfactor+=crossProduct.size();
  }
  bool value(false);
  for(auto& a: crossProduct){
    unsigned fcost(jointFCost(a,heuristic,envs));
    if(fcost <= flimit){
      if(verbose)std::cout << "EVAL " << s << "-->" << a << "\n";
      if(jointDFS(a,solution,solutions,env,heuristic,flimit,checkOnly)){
        //maxjoint=std::max(recursions,maxjoint);
        //minjoint=std::min(recursions,minjoint);
        value=true;
        transTable[hash]=value;
        // Return first solution... (unless this is a pairwise check with pruning)
        if(!checkOnly&&!certifyTime)certtimer.StartTimer();
      }
    }else{
      // Put it on the open list
      if(verbose)std::cout << "OPEN " << s << "-->" << a << "\n";
      open.insert(fcost);
      buckets[fcost].push_back(a);
    }
  }
  transTable[hash]=value;
  return value;
}

// Check that two paths have no collisions
bool checkPair(Path const& p1, Path const& p2,bool loud=false){
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    collChecks++;
    if(collisionCheck3D(*ap,*a,*bp,*b,agentRadius)){
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
bool checkAnswer(Solution<xyztLoc> const& answer){
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

void join(std::stringstream& s, std::vector<uint32_t> const& x){
  copy(x.begin(),x.end(), std::ostream_iterator<uint32_t>(s,","));
}

bool detectIndependence(Solution<xyztLoc>& solution, std::vector<Group*>& group, std::unordered_set<Group*>& groups){
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
          collChecks++;
          if(collisionCheck3D(solution[i][a-1],solution[i][a],solution[j][b-1],solution[j][b],agentRadius)){
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

Solution<xyztLoc> solution;
double total(0.0);
uint32_t nacts(0.0);
int failed(0);
uint32_t cost(0);
Timer tmr;

void printResults(){
  certifyTime=certtimer.EndTimer();
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
  if(crazystats){
    for(auto const& s:jointstats){
      std::cout << s.depth << "," << s.count << "," << s.goals << "," << s.branchingfactor << "(";
      for(auto const& u:s.mdds)
        std::cout << u.depth << "," << u.count << "," << u.goals  << "," << u.branchingfactor << ";";
      std::cout << ")\n";
    }
  }
  for(auto const& path:solution){
    for(int j(path.size()-1); j>0; --j){
      if(path[j-1]!=path[j]){
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
  std::cout << "seed:filepath,Connectedness,jointnodes,largestJoint,largestbranch,branchingfactor,maxnagents,minsingle,maxsingle,minjoint,maxjoint,collChecks,total,mddTime,pairwiseTime,jointTime,nogoodTime,certifyTime,nacts,cost\n";
  std::cout << seed << ":" << filepath << "," << agentType << "," << jointnodes << "," <<largestJoint << "," << largestbranch << "," << (double(jointnodes)/double(jointexpansions)) << "," << maxnagents << "," << minsingle << "," << maxsingle << "," << minjoint << "," << maxjoint << "," << collChecks << "," << total << "," << mddTime << "," << pairwiseTime << "," << jointTime << "," << nogoodTime << "," << certifyTime << "," << nacts << "," << cost;
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
std::pair<uint32_t,uint32_t> mergeSolution(std::vector<Solution<xyztLoc>>& answers, Solution<xyztLoc>& s, std::vector<int> const& insiders, uint32_t maxMergedCost){
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
    if(outsiders.empty()){
      for(unsigned i(0); i<answers[index].size(); ++i){
        s[i].resize(answers[index][i].size());
        for(unsigned j(0); j<answers[index][i].size(); ++j){
          s[i][j]=answers[index][i][j];
        }
      }
    }
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

Solution<xyztLoc> getAnswer(Instance const& inst, std::vector<Grid3DConstrainedEnvironment*> const& envs, std::vector<Heuristic<xyztLoc>*> const& heuristic){
  MultiEdge root;
  for(auto const& i:inst){
    root.emplace_back(i.first,i.first);
  }
  unsigned flimit(jointFCost(root,heuristic,envs));
  open.insert(flimit);
  buckets[flimit].push_back(root);
  
  while(open.size()){
    Solution<xyztLoc> solution;
    std::vector<Solution<xyztLoc>> solutions;
    flimit=*open.begin();
    for(auto const& root: buckets[flimit]){
      if(jointDFS(root,solution,solutions,envs,heuristic,flimit)){
        return solutions[0];
      }
    }
    // Next plateau
    open.erase(open.begin());
    buckets.erase(flimit);
    transTable.clear();
  }
}

int main(int argc, char ** argv){
  
  InstallHandlers();
  ProcessCommandLineArgs(argc, argv);
  Util::setmemlimit(killmem);
  MapInterface* smap(nullptr);
  Map3D* map(nullptr);
  if(mapfile.empty()){
    smap = map = new Map3D(width,length,1);
  }else{
    smap = map = new Map3D(mapfile.c_str(),dtedfile.c_str());
  }

  TemplateAStar<xyztLoc,t3DDirection,SearchEnvironment<xyztLoc,t3DDirection>> astar;
  //PEAStar<xyztLoc,t3DDirection,MapEnvironment> astar;
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


  envs.resize(n);
  for(int i(0); i<n; ++i){
    Grid3DConstrainedEnvironment* newEnv(nullptr);
    Grid3DEnvironment* me(nullptr); 
    switch(agentType){
      case 4:
        me = new Grid3DEnvironment(map); me->SetZeroConnected();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 8:
        me = new Grid3DEnvironment(map); me->SetOneConnected();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 9:
        me = new Grid3DEnvironment(map); me->SetOneConnected();
        me->SetWaitAllowed();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 24:
        me = new Grid3DEnvironment(map); me->SetTwoConnected();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 25:
        me = new Grid3DEnvironment(map); me->SetTwoConnected();
        me->SetWaitAllowed();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 48:
        me = new Grid3DEnvironment(map); me->SetThreeConnected();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 49:
        me = new Grid3DEnvironment(map); me->SetThreeConnected();
        me->SetWaitAllowed();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
      case 5:
      default:
        me = new Grid3DEnvironment(map); me->SetZeroConnected();
        me->SetWaitAllowed();
        newEnv = new Grid3DConstrainedEnvironment(me); newEnv->SetIgnoreHeading(true);
        break;
    }
    me->setGoal(waypoints[i].back());
    me->SetGround();
    newEnv->WaitTime(waitTime*xyztLoc::TIME_RESOLUTION);
    envs[i]=newEnv;
  }

  heuristics.resize(waypoints.size());
  if(mapfile.empty()){
    for(int i(0); i<heuristics.size(); ++i){
      heuristics[i]=envs[i];
    }
  }else{
    for(int i(0); i<heuristics.size(); ++i){
      heuristics[i] = new Map3dPerfectHeuristic<xyztLoc,t3DDirection>(map,envs[i]);
    }
  }

  if(ID){
    // Initial individual paths.
    for(int i(0); i<n; ++i){
      Path path;
      if(waypoints[i][0]==waypoints[i][1]){ // Already at goal
        path.push_back(waypoints[i][0]);
      }else{
        astar.SetVerbose(verbose);
        astar.SetHeuristic(heuristics[i]);
        envs[i]->setGoal(waypoints[i][0]);
        astar.GetPath(envs[i],waypoints[i][0],waypoints[i][1],path);
        if(!quiet)std::cout<<"Planned agent "<<i<<"\n";
      }
      path.push_back(xyztLoc(path.back(),unsigned(MAXTIME))); // Add a final wait action that goes way out...
      solution.push_back(path);
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
    Path path;
    path.emplace_back(0,0,0,0);
    path.emplace_back(0,1,0,1);
    for(int i(0); i<n; ++i){
      solution.push_back(path);
    }
  }

  // Start timing
  tmr.StartTimer();

  Timer::Timeout func(std::bind(&printResults));
  tmr.StartTimeout(std::chrono::seconds(killtime),func);
  while(!detectIndependence(solution,group,groups)){
    std::unordered_map<int,Instance> G;
    std::unordered_map<int,std::vector<int>> Gid;
    if(verbose)std::cout << "There are " << groups.size() << " groups" << std::endl;
    // Create groups
    int g(0);
    for(auto const& grp:groups){
      for(auto a:*grp){
        G[g].emplace_back(waypoints[a][0],waypoints[a][1]);
        Gid[g].push_back(a);
      }
      ++g;
    }
    for(int j(0); j<G.size(); ++j){
      auto g(G[j]);
      if(!quiet)std::cout << "Group " << j <<":\n";
      if(!quiet)for(int i(0); i<g.size(); ++i){
        std::cout << Gid[j][i] << ":" << g[i].first << "-->" << g[i].second << "\n";
      }
      if(g.size()>1){
        // Get correct set of envs and heuristics
        std::vector<Grid3DConstrainedEnvironment*> genv(g.size());
        std::vector<Heuristic<xyztLoc>*> heuristic(g.size());
        for(int i(0); i<g.size(); ++i){
          genv[i]=envs[Gid[j][i]];
          heuristic[i]=heuristics[Gid[j][i]];
        }

        std::vector<Solution<xyztLoc>> answers(1);
        answers[0]=getAnswer(g,envs,heuristic);
        mergeSolution(answers,solution,Gid[j],INF);
      }
    }
    largestJoint = float(n)/float(groups.size());
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
  if(strcmp(argument[0], "-stats") == 0)
  {
    crazystats = true;
    return 1;
  }
  if(strcmp(argument[0], "-fulljoint") == 0)
  {
    fulljoint = false;
    return 1;
  }
  if(strcmp(argument[0], "-noOD") == 0)
  {
    OD = false;
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
  if(strcmp(argument[0], "-waitTime") == 0)
  {
    waitTime=atof(argument[1]);
    return 2;
  }
  if(strcmp(argument[0], "-timeRes") == 0)
  {
    INFLATION = atof(argument[1]);
    TOMSECS=1/INFLATION;
    return 2;
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
      std::vector<xyztLoc> wpts;
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
      std::vector<xyztLoc> wpts;
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
