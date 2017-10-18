/*
 * $Id: sample.h,v 1.6 2006/09/18 06:23:39 nathanst Exp $
 *
 *  Driver.h
 *  hog
 *
 *  Created by Thayne Walker on 3/17/17.
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
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
#include <iomanip>
#include <unordered_set>
#include <set>
#include <stack>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>
#include "Map2DEnvironment.h"
#include "VelocityObstacle.h"
#include "PEAStar.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "MapPerfectHeuristic.h"

// for agents to stay at goal
#define MAXTIME 1000000
// for inflation of floats to avoid rounding errors
#define INFLATION 1000
#define TOMSECS 0.001

MapEnvironment* env;
SearchEnvironment<xyLoc,tDirection>* senv;
std::vector<Heuristic<xyLoc>*> heuristics;
std::string mapfile;

float jointTime(0);
float pairwiseTime(0);
float mddTime(0);
float nogoodTime(0);
float certifyTime(0);
Timer certtimer;
unsigned maxsingle(0);
unsigned minsingle(INF);
unsigned maxjoint(0);
unsigned minjoint(INF);
size_t maxnagents(0);

extern double agentRadius;
bool bestonly(false);
bool epp(false);
bool verbose(false);
bool quiet(false);
bool verify(false);
bool ID(true);
bool precheck(true);
bool mouseTracking;
unsigned agentType(5);
unsigned killtime(300);
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
uint32_t step(INFLATION);
int n(0);
std::unordered_map<std::string,bool> transTable;
unsigned seed(clock());

std::string filepath;
std::vector<std::vector<xyLoc> > waypoints;

UnitSimulation<xyLoc, tDirection, MapEnvironment> *sim = 0;

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
  xyLoc b;
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
  InstallCommandLineHandler(MyCLHandler, "-agentType", "-agentType [5,9,25,49]","Set the agent movement model");
  InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
  InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime [value]", "Kill after this many seconds");
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
/*struct Group{
  Group(int i){agents.insert(i);}
  std::unordered_set<int> agents;
  ~Group(){std::cout << "Destroy " << this << "\n";}
};*/

/*
struct Hashable{
  virtual uint64_t Hash()const=0;
  virtual uint32_t Depth()const=0;
};
*/

// Used for std::set
struct NodePtrComp
{
  bool operator()(const Hashable* lhs, const Hashable* rhs) const  { return lhs->Depth()<rhs->Depth(); }
};

namespace std
{
    template <>
    struct hash<Hashable>
    {
        size_t operator()(Hashable* const & x) const noexcept
        {
            return x->Hash();
        }
    };
}

struct Node : public Hashable{
        static uint64_t count;

	Node(){count++;}
	Node(xyLoc a, uint32_t d):n(a),depth(d),optimal(false),unified(false),nogood(false){count++;}
	//Node(xyLoc a, float d):n(a),depth(d*INFLATION),optimal(false),unified(false),nogood(false){count++;}
	xyLoc n;
	uint32_t depth;
        bool optimal;
        bool unified;
        bool nogood;
        //bool connected()const{return parents.size()+successors.size();}
	//std::unordered_set<Node*> parents;
	std::unordered_set<Node*> successors;
	virtual uint64_t Hash()const{return (env->GetStateHash(n)<<32) | depth;}
	virtual uint32_t Depth()const{return depth; }
        virtual void Print(std::ostream& ss, int d=0) const {
          ss << std::string(d/INFLATION,' ')<<n << "_" << depth << std::endl;
          for(auto const& m: successors)
            m->Print(ss,d+1);
        }
        bool operator==(Node const& other)const{return n.sameLoc(other.n)&&depth==other.depth;}
};

//std::unordered_set<uint64_t> checked;
//uint64_t EdgeHash(std::pair<Node*,Node*> const& edge){
  //return (edge.first->Hash() * 16777619) ^ edge.second->Hash();
//}
//uint64_t EdgePairHash(std::pair<Node*,Node*> const& edge1, std::pair<Node*,Node*> const& edge2){
  //return (EdgeHash(edge1) * 16777619) ^ EdgeHash(edge2);
//}

typedef std::vector<xyLoc> Points;
typedef std::pair<Points,Points> Instance;
typedef std::vector<Node*> Path;
typedef std::vector<std::vector<Node*>> Solution;

typedef std::vector<Node*> MultiState; // rank=agent num
typedef std::vector<std::pair<Node*,Node*>> MultiEdge; // rank=agent num
typedef std::unordered_map<uint64_t,Node> DAG;
std::unordered_map<uint64_t,Node*> mddcache;
std::unordered_map<uint64_t,uint32_t> lbcache;

/*class MultiEdge: public Multiedge{
  public:
  MultiEdge():Multiedge(),parent(nullptr){}
  MultiEdge(Multiedge const& other):Multiedge(other),parent(nullptr){} // Do not copy successors!
  std::vector<MultiEdge> successors;
  MultiEdge* parent;
  void Print(std::ostream& ss, int d=0) const {
    ss << std::string(d,' ');
    int i(0);
    for(auto const& a: *this)
      ss << " "<<++i<<"." << a.second->n << "@" << a.second->depth;
    ss << std::endl;
    for(auto const& m: successors)
      m.Print(ss,d+1);
  }
};*/

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
    ss << " "<<++i<<"." << a->n << "@" << a->depth;
  return ss;
}

std::ostream& operator << (std::ostream& ss, MultiEdge const& n){
  int i(0);
  for(auto const& a: n)
    ss << " "<<++i<<"." << a.second->n << "@" << a.second->depth;
  ss << std::endl;
  /*
  for(auto const& m:n.successors)
    ss << "----"<<m;
  */
  //n.Print(ss,0);
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const& n){
  ss << n.n << "@" << n.depth;
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const* n){
  n->Print(ss);
  //ss << std::string(n->depth,' ')<<n->n << "_" << n->depth << std::endl;
  //for(auto const& m: n->successors)
    //ss << m;
  return ss;
}

// Compute path cost, ignoring actions that wait at the goal
uint32_t computeSolutionCost(Solution const& solution, bool ignoreWaitAtGoal=true){
  uint32_t cost(0);
  if(ignoreWaitAtGoal){
    for(auto const& path:solution){
      for(int j(path.size()-1); j>0; --j){
        if(path[j-1]->n!=path[j]->n){
          cost += path[j]->depth;
          break;
        }else if(j==1){
          cost += path[0]->depth;
        }
      }
    }
  }else{
    for(auto const& path:solution){
      cost+=path.back()->depth;
    }
  }
  return cost;
}


uint64_t Node::count(0);
//std::unordered_map<int,std::set<uint64_t>> costt;

bool LimitedDFS(xyLoc const& start, xyLoc const& end, DAG& dag, Node*& root, uint32_t depth, uint32_t maxDepth, uint32_t& best, unsigned agent, unsigned id, unsigned recursions=1){
  //std::cout << std::string((int)(maxDepth-depth),' ') << start << "g:" << (maxDepth-depth) << " h:" << Node::env->HCost(start,end) << " f:" << ((maxDepth-depth)+Node::env->HCost(start,end));
  if(depth<0 || maxDepth-depth+(int)(heuristics[id]->HCost(start,end)*INFLATION)>maxDepth){ // Note - this only works for a perfect heuristic.
    //std::cout << " pruned " << depth <<" "<< (maxDepth-depth+(int)(Node::env->HCost(start,end)*INFLATION))<<">"<<maxDepth<<"\n";
    return false;
  }
  //std::cout << "\n";

  if(env->GoalTest(start,end)){
    Node n(start,(maxDepth-depth));
    uint64_t hash(n.Hash());
      //std::cout << n<<"\n";
    dag[hash]=n;
    // This may happen if the agent starts at the goal
    if(maxDepth-depth<=0){
      root=&dag[hash];
      //std::cout << "root_ " << &dag[hash];
    }
    Node* parent(&dag[hash]);
    int d(maxDepth-depth);
    //if(d+INFLATION<=maxDepth){ // Insert one long wait action at goal
    while(d+INFLATION<=maxDepth){ // Increment depth by 1 for wait actions
      // Wait at goal
      Node current(start,(d+=INFLATION));
      uint64_t chash(current.Hash());
      //std::cout << current<<"\n";
      dag[chash]=current;
      if(verbose)std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      parent->successors.insert(&dag[chash]);
      //dag[chash].parents.insert(parent);
      parent=&dag[chash];
    }
    best=std::min(best,parent->depth);
    //std::cout << "found d\n";
    //costt[(int)maxDepth].insert(d);
    if(verbose)std::cout << "ABEST "<<best<<"\n";
    return true;
  }

  Points successors;
  env->GetSuccessors(start,successors);
  bool result(false);
  for(auto const& node: successors){
    int ddiff(std::max(Util::distance(node.x,node.y,start.x,start.y),1.0)*INFLATION);
    //std::cout << std::string(std::max(0,(maxDepth-(depth-ddiff))),' ') << "MDDEVAL " << start << "-->" << node << "\n";
    //if(abs(node.x-start.x)>=1 && abs(node.y-start.y)>=1){
      //ddiff = M_SQRT2;
    //}
    if(LimitedDFS(node,end,dag,root,depth-ddiff,maxDepth,best,agent,id,recursions+1)){
      maxsingle=std::max(recursions,maxsingle);
      minsingle=std::min(recursions,minsingle);
      Node n(start,(maxDepth-depth));
      uint64_t hash(n.Hash());
      if(dag.find(hash)==dag.end()){
        //std::cout << n<<"\n";
        dag[hash]=n;
        // This is the root if depth=0
        if(maxDepth-depth<=0){
          root=&dag[hash];
          if(verbose)std::cout << "Set root to: " << (uint64_t)root << "\n";
          //std::cout << "_root " << &dag[hash];
        }
        //if(maxDepth-depth==0.0)root.push_back(&dag[hash]);
      }else if(dag[hash].optimal){
        return true; // Already found a solution from search at this depth
      }

      Node* parent(&dag[hash]);

      //std::cout << "found " << start << "\n";
      uint64_t chash(Node(node,(maxDepth-depth+ddiff)).Hash());
      if(dag.find(chash)==dag.end()&&dag.find(chash+1)==dag.end()&&dag.find(chash-1)==dag.end()){
        std::cout << "Expected " << Node(node,maxDepth-depth+ddiff) << " " << chash << " to be in the dag\n";
        assert(!"Uh oh, node not already in the DAG!");
        //std::cout << "Add new.\n";
        Node c(node,(maxDepth-depth+ddiff));
        dag[chash]=c;
      }
      Node* current(&dag[chash]);
      current->optimal = result = true;
      //std::cout << *parent << " parent of " << *current << "\n";
      //dag[current->Hash()].parents.insert(parent);
      //std::cout << *current << " child of " << *parent << " " << parent->Hash() << "\n";
      //std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      dag[parent->Hash()].successors.insert(&dag[current->Hash()]);
      //std::cout << "at" << &dag[parent->Hash()] << "\n";
    }
  }
  return result;
}

// Perform conflict check by moving forward in time at increments of the smallest time step
// Test the efficiency of VO vs. time-vector approach
void GetMDD(unsigned agent,unsigned id,xyLoc const& start, xyLoc const& end, DAG& dag, MultiState& root, int depth, uint32_t& best){
  if(verbose)std::cout << "MDD up to depth: " << depth << start << "-->" << end << "\n";
  uint64_t hash(((uint32_t) depth)<<8|agent);
  bool found(mddcache.find(hash)!=mddcache.end());
  if(verbose)std::cout << "lookup "<< (found?"found":"missed") << "\n";
  if(!found){
    LimitedDFS(start,end,dag,root[agent],depth,depth,best,agent,id);
    mddcache[hash]=root[agent];
    lbcache[hash]=best;
  }else{
    root[agent]=mddcache[hash];
    best=lbcache[hash];
  }
  if(verbose)std::cout << "Finally set root to: " << (uint64_t)root[agent] << "\n";
}

void generatePermutations(std::vector<MultiEdge>& positions, std::vector<MultiEdge>& result, int agent, MultiEdge const& current, uint32_t lastTime) {
  if(agent == positions.size()) {
    result.push_back(current);
    if(verbose)std::cout << "Generated joint move:\n";
    if(verbose)for(auto edge:current){
      std::cout << *edge.first << "-->" << *edge.second << "\n";
    }
    jointnodes++;
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    //std::cout << "AGENT "<< i<<":\n";
    MultiEdge copy(current);
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      if(precheck && !env->collisionPreCheck(positions[agent][i].first->n,positions[agent][i].second->n,agentRadius,current[j].first->n,current[j].second->n,agentRadius)) continue;
      // Make sure we don't do any checks that were already done
      //if(positions[agent][i].first->depth==lastTime&&current[j].first->depth==lastTime)continue;
      //uint64_t hash(EdgePairHash(positions[agent][i],current[j]));
      //if(checked.find(hash)!=checked.end())
      //{std::cout << "SKIPPED " << *positions[agent][i].second << " " << *current[j].second << "\n"; continue; /*No collision check necessary; checked already*/}
      //std::cout << "COMPARE " << *positions[agent][i].second << " " << *current[j].second << "\n";
      Vector2D A(positions[agent][i].first->n.x,positions[agent][i].first->n.y);
      Vector2D B(current[j].first->n.x,current[j].first->n.y);
      Vector2D VA(positions[agent][i].second->n.x-positions[agent][i].first->n.x,positions[agent][i].second->n.y-positions[agent][i].first->n.y);
      VA.Normalize();
      Vector2D VB(current[j].second->n.x-current[j].first->n.x,current[j].second->n.y-current[j].first->n.y);
      VB.Normalize();
      //std::cout << "Test for collision: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
      if(collisionImminent(A,VA,agentRadius,positions[agent][i].first->depth*TOMSECS,positions[agent][i].second->depth*TOMSECS,B,VB,agentRadius,current[j].first->depth*TOMSECS,current[j].second->depth*TOMSECS)){
        if(verbose)std::cout << "Collision averted: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
        found=true;
        //checked.insert(hash);
        break;
      }
      if(verbose)std::cout << "generating: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
    }
    if(found) continue;
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy,lastTime);
  }
}

  // Compute hash for transposition table
void getHash(MultiEdge const&a,std::string& hash){
  int i(0);
  for(auto v:a){
    uint64_t h1(v.second->Hash());
    uint8_t c[sizeof(uint64_t)];
    memcpy(c,&h1,sizeof(uint64_t));
    for(unsigned j(0); j<sizeof(uint64_t); ++j){
      hash[i*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
    }
    ++i;
  }
}
// In order for this to work, we cannot generate sets of positions, we must generate sets of actions, since at time 1.0 an action from parent A at time 0.0 may have finished, while another action from the same parent A may still be in progress. 

struct stackObj{
  stackObj(MultiEdge const& a, std::string const& b, uint32_t c):value(a),hash(b),cost(c){}
  MultiEdge value;
  std::string hash;
  uint32_t cost;
};

// Return true if we get to the desired depth
// Check goal inside loop
bool jointDFS2(MultiEdge const& r, uint32_t d, Solution solution, std::vector<Solution>& solutions, std::vector<Node*>& toDelete, uint32_t& best, uint32_t bestSeen, unsigned recursions=1, bool suboptimal=false, bool checkOnly=false){

  std::unordered_map<std::string,std::pair<std::string,MultiEdge>> parentTable;
  std::unordered_map<std::string,bool> transpositionTable;
  std::vector<std::pair<std::string,MultiEdge>> finished;
  std::stack<stackObj> stack;

  std::string h(r.size()*sizeof(uint64_t),1);
  getHash(r,h);
  stack.push({r,h,0.0f});

  while(stack.size()){
    auto v(stack.top());
    stack.pop();
    MultiEdge const& s(v.value);
    //std::cout << "POP " << s << "\n";
    uint32_t cost(v.cost);

    //Get successors into a vector
    std::vector<MultiEdge> successors;

    // Find minimum depth of current edges
    uint32_t sd(INF);
    for(auto const& a: s){
      sd=min(sd,a.second->depth);
    }
    //std::cout << "min-depth: " << sd << "\n";

    uint32_t md(INF); // Min depth of successors
    //Add in successors for parents who are equal to the min
    for(auto const& a: s){
      if(epp){
        if(a.second->nogood){
          if(verbose)std::cout << *a.second << " is no good.\n";
          return false; // If any  of these are "no good" just exit now
        }else{
          // The fact that we are evaluating this node means that all components were unified with something
          if(checkOnly)a.second->unified=true;
        }
      }
      MultiEdge output;
      if(a.second->depth<=sd){
        //std::cout << "Keep Successors of " << *a.second << "\n";
        for(auto const& b: a.second->successors){
          output.emplace_back(a.second,b);
          md=min(md,b->depth);
        }
      }else{
        //std::cout << "Keep Just " << *a.second << "\n";
        output.push_back(a);
        md=min(md,a.second->depth);
      }
      if(output.empty()){
        // Stay at state...
        output.emplace_back(a.second,new Node(a.second->n,MAXTIME));
        //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
        toDelete.push_back(output.back().second);
        //md=min(md,a.second->depth+1.0); // Amount of time to wait
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
    }
    if(verbose){
      std::cout << "Move set\n";
      for(int a(0);a<successors.size(); ++a){
        std::cout << "agent: " << a << "\n";
        for(auto const& m:successors[a]){
          std::cout << "  " << *m.first << "-->" << *m.second << "\n";
        }
      }
    }
    std::vector<MultiEdge> crossProduct;
    MultiEdge tmp;
    generatePermutations(successors,crossProduct,0,tmp,sd);
    for(auto& a: crossProduct){

      // Compute hash for transposition table
      std::string hash(a.size()*sizeof(uint64_t),1);
      getHash(a,hash);

      if(verbose)std::cout << "saw " << a << " hash ";
      if(verbose)for(unsigned int i(0); i<hash.size(); ++i){
        std::cout << (unsigned)hash[i]<<" ";
      }
      if(verbose)std::cout <<"\n";
      if(transpositionTable.find(hash)!=transpositionTable.end()){
        if(transpositionTable[hash]){
          //Do we want multiple parents??
          //parentTable[hash].append(s);
        }
        continue;
      }

      // Compute the solution cost
      uint32_t currcost(cost);
      // Copy solution so far, but only copy components if they are
      // a valid successor
      for(int i(0); i<s.size(); ++i){
        auto const& p = s[i];
        bool found(false);
        for(auto const& suc:p.second->successors){
          // True successor or wait action...
          if(*a[i].second==*suc){
            found=true;
            currcost+=a[i].second->depth-p.second->depth;
            break;
          }
        }
        if(found || (a[i].second->n.sameLoc(p.second->n) && a[i].second->depth>p.second->depth)){
          if(parentTable.find(hash)==parentTable.end()){
            parentTable[hash]=std::make_pair(v.hash,s);
          }
        }
      }
      // Put it on the stack
      stack.emplace(a,hash,currcost);
      //std::cout << "PUSH " << a << "\n";

      // Check for solution
      bool done(true);
      for(auto const& g:a){
        if(!g.second->depth==MAXTIME){
          done=false;
          break;
        }
        //maxCost=std::max(maxCost,g.second->depth);
      }
      if(done){
        //maxjoint=std::max(stack.size(),maxjoint);
        //minjoint=std::min(stack.size(),minjoint);
        finished.emplace_back(hash,a);
        if(!checkOnly&&!certifyTime)certtimer.StartTimer();
        if(suboptimal&&!(epp&&checkOnly)) stack=std::stack<stackObj>(); // Exit search
        if(currcost<best){
          best=currcost;
          if(verbose)std::cout << "BEST="<<best<<std::endl;
          if(verbose)std::cout << "BS="<<bestSeen<<std::endl;
          // Return if solution is as good as any MDD
          if(!(epp&&checkOnly)&&best==bestSeen) stack=std::stack<stackObj>(); // Exit search
        }
        std::string h1=hash;
        transpositionTable[h1]=true;
        std::unordered_map<std::string,std::pair<std::string,MultiEdge>>::const_iterator val;
        while((val=parentTable.find(h1))!=parentTable.end()){
          h1=val->second.first;
          transpositionTable[h1]=true;
        }
      }
    }
  }
  // Accumulate results
  for(auto const& f:finished){
    Solution solution(f.second.size());
    std::string h1=f.first;
    std::unordered_map<std::string,std::pair<std::string,MultiEdge>>::const_iterator val;
    for(int i(0); i<solution.size(); ++i){
      solution[i].push_back(f.second[i].second);
    }
    while((val=parentTable.find(h1))!=parentTable.end()){
      h1=val->second.first;
      for(int i(0); i<solution.size(); ++i){
        solution[i].push_back(val->second.second[i].second);
      }
    }
    for(int i(0); i<solution.size(); ++i){
      std::reverse(solution[i].begin(),solution[i].end());
    }
    solutions.push_back(solution);
  }
}

// Return true if we get to the desired depth
bool jointDFS(MultiEdge const& s, uint32_t d, Solution solution, std::vector<Solution>& solutions, std::vector<Node*>& toDelete, uint32_t& best, uint32_t bestSeen, unsigned recursions=1, bool suboptimal=false, bool checkOnly=false){
  // Compute hash for transposition table
  std::string hash(s.size()*sizeof(uint64_t),1);
  int i(0);
  for(auto v:s){
    uint64_t h1(v.second->Hash());
    uint8_t c[sizeof(uint64_t)];
    memcpy(c,&h1,sizeof(uint64_t));
    for(unsigned j(0); j<sizeof(uint64_t); ++j){
      hash[i*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
    }
    ++i;
  }
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

  if(!checkOnly&&d>0){
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
      if(found || (s[i].second->n.sameLoc(p.back()->n) && s[i].second->depth>p.back()->depth)){
        solution[i].push_back(s[i].second);
      }
    }
  }
  
  bool done(true);
  for(auto const& g:s){
    if(g.second->depth!=MAXTIME){
      done=false;
      break;
    }
    //maxCost=std::max(maxCost,g.second->depth);
  }
  if(done){
    uint32_t cost(computeSolutionCost(solution));
    if(cost<best){
      best=cost;
      if(verbose)std::cout << "BEST="<<best<<std::endl;
      if(verbose)std::cout << "BS="<<bestSeen<<std::endl;

      // This is a leaf node
      // Copy the solution into the answer set
      if(!checkOnly){
        // Shore up with wait actions
        for(int i(0); i<solution.size(); ++i){
          if(solution[i].back()->depth<MAXTIME){
            solution[i].emplace_back(new Node(solution[i].back()->n,MAXTIME));
            toDelete.push_back(solution[i].back());
          }
        }
        solutions.clear();
        solutions.push_back(solution);
      }
    }
    return true;
  }
  //Get successors into a vector
  std::vector<MultiEdge> successors;

  // Find minimum depth of current edges
  uint32_t sd(INF);
  for(auto const& a: s){
    sd=min(sd,a.second->depth);
  }
  //std::cout << "min-depth: " << sd << "\n";

  uint32_t md(INF); // Min depth of successors
  //Add in successors for parents who are equal to the min
  for(auto const& a: s){
    if(epp){
      if(a.second->nogood){
        if(verbose)std::cout << *a.second << " is no good.\n";
        return false; // If any  of these are "no good" just exit now
      }else{
        // The fact that we are evaluating this node means that all components were unified with something
        if(checkOnly)a.second->unified=true;
      }
    }
    MultiEdge output;
    if(a.second->depth<=sd){
      //std::cout << "Keep Successors of " << *a.second << "\n";
      for(auto const& b: a.second->successors){
          output.emplace_back(a.second,b);
          md=min(md,b->depth);
      }
    }else{
      //std::cout << "Keep Just " << *a.second << "\n";
      output.push_back(a);
      md=min(md,a.second->depth);
    }
    if(output.empty()){
      // Stay at state...
      output.emplace_back(a.second,new Node(a.second->n,MAXTIME));
      //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
      toDelete.push_back(output.back().second);
      //md=min(md,a.second->depth+1.0); // Amount of time to wait
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
  }
  if(verbose){
    std::cout << "Move set\n";
    for(int a(0);a<successors.size(); ++a){
      std::cout << "agent: " << a << "\n";
      for(auto const& m:successors[a]){
        std::cout << "  " << *m.first << "-->" << *m.second << "\n";
      }
    }
  }
  std::vector<MultiEdge> crossProduct;
  MultiEdge tmp;
  generatePermutations(successors,crossProduct,0,tmp,sd);
  bool value(false);
  for(auto& a: crossProduct){
    if(verbose)std::cout << "EVAL " << s << "-->" << a << "\n";
    if(jointDFS(a,md,solution,solutions,toDelete,best,bestSeen,recursions+1,suboptimal,checkOnly)){
      maxjoint=std::max(recursions,maxjoint);
      minjoint=std::min(recursions,minjoint);
      value=true;
      transTable[hash]=value;
      // Return first solution... (unless this is a pairwise check with pruning)
      if(!checkOnly&&!certifyTime)certtimer.StartTimer();
      if(suboptimal&&!(epp&&checkOnly)) return true;
      // Return if solution is as good as any MDD
      if(!(epp&&checkOnly)&&best==bestSeen)return true;
    }
  }
  transTable[hash]=value;
  return value;
}

bool jointDFS(MultiState const& s, std::vector<Solution>& solutions, std::vector<Node*>& toDelete, uint32_t bestSeen, bool suboptimal=false, bool checkOnly=false){
  if(verbose)std::cout << "JointDFS\n";
  MultiEdge act;
  Solution solution;
  std::unordered_set<std::string> ttable;
  // Add null parents for the initial movements
  for(auto const& n:s){
    act.emplace_back(nullptr,n);
    if(!checkOnly){
      // Add initial state for solution
      solution.push_back({n});
    }
  }
  uint32_t best(INF);

  transTable.clear();
  return jointDFS(act,0.0,solution,solutions,toDelete,best,bestSeen,suboptimal,checkOnly);
}

// Check that two paths have no collisions
bool checkPair(Path const& p1, Path const& p2,bool loud=false){
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    if(!precheck || env->collisionPreCheck((*ap)->n,(*a)->n,agentRadius,(*bp)->n,(*b)->n,agentRadius)){
      Vector2D A((*ap)->n.x,(*ap)->n.y);
      Vector2D B((*bp)->n.x,(*bp)->n.y);
      Vector2D VA((*a)->n.x-(*ap)->n.x,(*a)->n.y-(*ap)->n.y);
      VA.Normalize();
      Vector2D VB((*b)->n.x-(*bp)->n.x,(*b)->n.y-(*bp)->n.y);
      VB.Normalize();
      if(collisionImminent(A,VA,agentRadius,(*ap)->depth*TOMSECS,(*a)->depth*TOMSECS,B,VB,agentRadius,(*bp)->depth*TOMSECS,(*b)->depth*TOMSECS)){
        if(loud)std::cout << "Collision: " << **ap << "-->" << **a << "," << **bp << "-->" << **b;
        return false;
      }
    }
    if((*a)->depth<(*b)->depth){
      ++a;
      ++ap;
    }else if((*a)->depth>(*b)->depth){
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
bool checkAnswer(Solution const& answer){
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

struct ICTSNode{
  ICTSNode(ICTSNode* parent,int agent, uint32_t size):instance(parent->instance),dag(parent->dag),best(parent->best),bestSeen(0),sizes(parent->sizes),root(parent->root),ids(parent->ids){
    count++;
    sizes[agent]=size;
    best[agent]=INF;
    if(verbose)std::cout << "replan agent " << agent << " GetMDD("<<(heuristics[ids[agent]]->HCost(instance.first[agent],instance.second[agent])+sizes[agent])<<")\n";
    dag[agent].clear();
    replanned.push_back(agent);
    Timer timer;
    timer.StartTimer();
    GetMDD(agent,ids[agent],instance.first[agent],instance.second[agent],dag[agent],root,(int)(heuristics[ids[agent]]->HCost(instance.first[agent],instance.second[agent])*INFLATION)+(int)(sizes[agent]),best[agent]);
    mddTime+=timer.EndTimer();
    bestSeen=std::accumulate(best.begin(),best.end(),0.0f);
    // Replace new root node on top of old.
    //std::swap(root[agent],root[root.size()-1]);
    //root.resize(root.size()-1);
    //if(verbose)std::cout << agent << ":\n" << root[agent] << "\n";
  }

  ICTSNode(Instance const& inst, std::vector<uint32_t> const& s, std::vector<int> const& id):instance(inst),dag(s.size()),best(s.size()),bestSeen(0),sizes(s),root(s.size()),ids(id){
    count++;
    root.reserve(s.size());
    replanned.resize(s.size());
    for(int i(0); i<instance.first.size(); ++i){
      best[i]=INF;
      replanned[i]=i;
      if(verbose)std::cout << "plan agent " << i << " GetMDD("<<(heuristics[ids[i]]->HCost(instance.first[i],instance.second[i])+sizes[i])<<")\n";
      //std::cout.precision(17);
      std::cout.precision(6);

      Timer timer;
      timer.StartTimer();
      GetMDD(i,ids[i],instance.first[i],instance.second[i],dag[i],root,(int)(heuristics[ids[i]]->HCost(instance.first[i],instance.second[i])*INFLATION)+(int)(sizes[i]),best[i]);
      mddTime+=timer.EndTimer();
      bestSeen=std::accumulate(best.begin(),best.end(),0.0f);
      //if(verbose)std::cout << i << ":\n" << root[i] << "\n";
    }
  }

  ~ICTSNode(){for(auto d:toDelete){delete d;}}

  // Get unique identifier for this node
  std::string key()const{
    std::stringstream sv;
    join(sv,sizes);
    return sv.str();
  }

  Instance instance;
  std::vector<DAG> dag;
  std::vector<uint32_t> sizes;
  std::vector<uint32_t> best;
  uint32_t bestSeen;
  MultiState root;
  std::vector<int> ids;
  Instance points;
  std::vector<Node*> toDelete;
  static uint64_t count;
  static bool pairwise;
  static bool suboptimal;
  static bool epsilon;
  std::vector<int> replanned; // Set of nodes that was just re-planned

  // Set all nodes that were never unified as "no good"
  void resetNoGoods(Node* root){
    if(!epp)return;
    root->nogood=false;
    root->unified=false;
    for(auto& a:root->successors)
      resetNoGoods(a);
  }
  void resetUnified(Node* root){
    root->unified=false;
    for(auto& a:root->successors)
      resetUnified(a);
  }
  void setNoGoods(Node* root){
    if(root->nogood)return;
    root->nogood=!root->unified;
    for(auto& a:root->successors)
      setNoGoods(a);
  }
  void updateNoGoods(Node* root){
    if(!epp)return;
    Timer timer;
    timer.StartTimer();
    setNoGoods(root); // Set them all first
    resetUnified(root); // Reset the unified flag
    nogoodTime+=timer.EndTimer();
  }
  bool isValid(std::vector<Solution>& answers){
    if(root.size()>2 && pairwise){
      Timer timer;
      timer.StartTimer();
      // Perform pairwise check
      if(!quiet)std::cout<<"Pairwise checks,max MDD depth: " << maxsingle << "\n";
      if(replanned.size()>1){
        for(int i(0); i<root.size(); ++i){
          for(int j(i+1); j<root.size(); ++j){
            MultiState tmproot(2);
            tmproot[0]=root[i];
            tmproot[1]=root[j];
            std::vector<Node*> toDeleteTmp;
            // This is a satisficing search, thus we only need do a sub-optimal check
            if(verbose)std::cout<<"pairwise for " << i << ","<<j<<"\n";
            if(!jointDFS(tmproot,answers,toDeleteTmp,INF,true,true)){
              if(verbose)std::cout << "Pairwise failed\n";
              //clearNoGoods();
              // Reset the MDDs
              if(epp){
                Timer timer;
                timer.StartTimer();
                for(int i(0); i<root.size(); ++i){
                  resetNoGoods(root[i]);
                }
                nogoodTime+=timer.EndTimer();
              }
              return false;
            }
            updateNoGoods(root[i]);
            updateNoGoods(root[j]);
          }
        }
      }else{
        for(int i(0); i<root.size(); ++i){
          if(i==replanned[0]){continue;}
          MultiState tmproot(2);
          tmproot[0]=root[i];
          tmproot[1]=root[replanned[0]];
          std::vector<Node*> toDeleteTmp;
          // This is a satisficing search, thus we only need do a sub-optimal check
          if(verbose)std::cout<<"pairwise for " << i << ","<<replanned[0]<<"\n";
          if(!jointDFS(tmproot,answers,toDeleteTmp,INF,true,true)){
            if(verbose)std::cout << "Pairwise failed\n";
            //clearNoGoods();
              // Reset the MDDs
            if(epp){
              Timer timer;
              timer.StartTimer();
              for(int i(0); i<root.size(); ++i){
                resetNoGoods(root[i]);
              }
              nogoodTime+=timer.EndTimer();
            }
            return false;
          }
          updateNoGoods(root[i]);
          updateNoGoods(root[replanned[0]]);
        }
      }
      pairwiseTime+=timer.EndTimer();
    }
    // Do a depth-first search; if the search terminates at a goal, its valid.
    if(!quiet)std::cout<<"Pairwise passed\nFull check, max joint MDD depth: " << maxjoint << "\n";
    Timer timer;
    timer.StartTimer();
    if(jointDFS(root,answers,toDelete,lb(),suboptimal)){
      jointTime+=timer.EndTimer();
      if(verbose){
        std::cout << "Answer:\n";
        for(int num(0); num<answers.size(); ++num){
          std::cout << "number " << num << ":\n";
          for(int agent(0); agent<answers[num].size(); ++agent){
            std::cout << "  " << agent << ":\n";
            for(auto a(answers[num][agent].begin()); a!=answers[num][agent].end(); ++a){
              std::cout  << "  " << std::string((*a)->depth/INFLATION,' ') << **a << "\n";
            }
            std::cout << "\n";
          }
          std::cout << "Cost: " << computeSolutionCost(answers[num]) << std::endl;
          if(verify&&!checkAnswer(answers[num]))std::cout<< "Failed in ICT node\n";
        }
      }
      //clearNoGoods();
      if(verbose)std::cout << "Full check passed\n";
      return true;
    }
    // Reset the MDDs
    if(epp){
      Timer timer;
      timer.StartTimer();
      for(int i(0); i<root.size(); ++i){
        resetNoGoods(root[i]);
      }
      nogoodTime=timer.EndTimer();
    }
    
    if(verbose)std::cout << "Full check failed\n";
    //clearNoGoods();
    return false;
  }

  bool operator<(ICTSNode const& other)const{
    uint32_t sic1(lb());
    uint32_t sic2(other.lb());
    if(sic1==sic2){
      uint32_t t1(0);
      for(auto const& s:sizes){
        t1 += s;
      }
      uint32_t t2(0);
      for(auto const& s:other.sizes){
        t2 += s;
      }
      return t1>t2;
    }else{
      return sic1>sic2;
    }
  }
  uint32_t ub()const{
    uint32_t total(0);
    for(auto const& s:sizes){
      total += s;
    }
    return total;
  }
  uint32_t lb()const{ return bestSeen; }
};

bool ICTSNode::epsilon(false);
bool ICTSNode::suboptimal(false); // Sub-optimal variant
uint64_t ICTSNode::count(0);
bool ICTSNode::pairwise(true);

struct ICTSNodePtrComp
{
  bool operator()(const ICTSNode* lhs, const ICTSNode* rhs) const  { return *lhs<*rhs; }
};

bool detectIndependence(Solution& solution, std::vector<Group*>& group, std::unordered_set<Group*>& groups){
  bool independent(true);
  // Check all pairs for collision
  uint32_t minTime(-1);
  for(int i(0); i<solution.size(); ++i){
    for(int j(i+1); j<solution.size(); ++j){
      // check collision between i and j
      int a(1);
      int b(1);
      if(solution[i].size() > a && solution[j].size() > b){
        //uint32_t t(min(solution[i][a]->depth,solution[j][b]->depth));
        while(1){
          if(a==solution[i].size() || b==solution[j].size()){break;}
          if(!precheck || env->collisionPreCheck(solution[i][a-1]->n,solution[i][a]->n,agentRadius,solution[j][b-1]->n,solution[j][b]->n,agentRadius)){
            Vector2D A(solution[i][a-1]->n.x,solution[i][a-1]->n.y);
            Vector2D B(solution[j][b-1]->n.x,solution[j][b-1]->n.y);
            Vector2D VA(solution[i][a]->n.x-solution[i][a-1]->n.x,solution[i][a]->n.y-solution[i][a-1]->n.y);
            VA.Normalize();
            Vector2D VB(solution[j][b]->n.x-solution[j][b-1]->n.x,solution[j][b]->n.y-solution[j][b-1]->n.y);
            VB.Normalize();
            if(collisionImminent(A,VA,agentRadius,solution[i][a-1]->depth,solution[i][a]->depth,B,VB,agentRadius,solution[j][b-1]->depth,solution[j][b]->depth)){
              if(verbose)std::cout << i << " and " << j << " collide at " << solution[i][a-1]->depth << "~" << solution[i][a]->depth << solution[i][a-1]->n << "-->" << solution[i][a]->n << " X " << solution[j][b-1]->n << "-->" << solution[j][b]->n << "\n";
              independent=false;
              if(group[i]==group[j]) continue; // This can happen if both collide with a common agent
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
          }
          if(solution[i][a]->depth==solution[j][b]->depth){
            ++a;++b;
          }else if(solution[i][a]->depth<solution[j][b]->depth){
            ++a;
          }else{++b;}
        }
      }
    }
  }
  return independent;
}

Solution solution;
uint32_t total(0.0);
uint32_t nacts(0.0);
int failed(0);
uint32_t cost(0);
Timer tmr;

void printResults(){
  certifyTime=certtimer.EndTimer();
  if(!quiet){
    std::cout << "Solution:\n";
    int ii=0;
    for(auto const& p:solution){
      std::cout << ii++ << "\n";
      for(auto const& t: p){
        // Print solution
        std::cout << t->n << "," << t->depth/1000. << "\n";
      }
    }
  }
  for(auto const& path:solution){
    for(int j(path.size()-1); j>0; --j){
      if(path[j-1]->n!=path[j]->n){
        cost += path[j]->depth;
        nacts += j;
        if(verbose)std::cout << "Adding " << path[j]->n<<","<<path[j]->depth<<"\n";
        break;
      }else if(j==1){
        cost += path[0]->depth;
        nacts += 1;
        if(verbose)std::cout << "Adding_" << path[0]->n<<","<<path[0]->depth<<"\n";
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
  if(!quiet)std::cout << "seed:filepath,Connectedness,ICTSNode::count,jointnodes,Node::count,maxnagents,minsingle,maxsingle,minjoint,maxjoint,total,mddTime,pairwiseTime,jointTime,nogoodTime,certifyTime,nacts,cost\n";
  std::cout << seed << ":" << filepath << "," << int(env->GetConnectedness()) << "," << ICTSNode::count << "," << jointnodes << "," << Node::count << "," << maxnagents << "," << minsingle << "," << maxsingle << "," << minjoint << "," << maxjoint << "," << total << "," << mddTime << "," << pairwiseTime << "," << jointTime << "," << nogoodTime << "," << certifyTime << "," << nacts << "," << cost;
  if(total >= killtime)std::cout << " failure";
  std::cout << std::endl;
  if(total>=killtime)exit(1);
}

// Scan the candidate answers and merge a non-conflicting answer set in with
// the main solution. Return the added cost amount. Returns zero if no solutions
// could be merged, or the cost of the valid solution is higher than the maximum
// allowable cost.
// returns the merged cost or zero if unable to merge and teh best cost
// (the best solution is merged in either case).
std::pair<uint32_t,uint32_t> mergeSolution(std::vector<Solution>& answers, Solution& s, std::vector<int> const& insiders, uint32_t maxMergedCost){
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
        for(auto& a:s[insiders[i]]){
          delete a;
        }
        s[insiders[i]].resize(0);
        for(auto& a:ans[i]){
          s[insiders[i]].push_back(new Node(*a));
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
      for(auto& a:s[insiders[i]]){
        delete a;
      }
      s[insiders[i]].resize(0);
      for(auto& a:answers[0][i]){
        s[insiders[i]].push_back(new Node(*a));
      }
    }
  }
  return {0.0f,costs[sorted[0]]};
}

int main(int argc, char ** argv){
  
  InstallHandlers();
  ProcessCommandLineArgs(argc, argv);
  MapInterface* smap(nullptr);
  Map* map(nullptr);
  if(mapfile.empty()){
    smap = map = new Map(width,length);
  }else{
    smap = map = new Map(mapfile.c_str());
  }
  senv = env = new MapEnvironment(map);
  
  switch(agentType){
    case 4:
      env->SetFourConnected();
      break;
    case 8:
      env->SetEightConnected();
      break;
    case 9:
      env->SetNineConnected();
      break;
    case 24:
      env->SetTwentyFourConnected();
      break;
    case 25:
      env->SetTwentyFiveConnected();
      break;
    case 48:
      env->SetFortyEightConnected();
      break;
    case 49:
      env->SetFortyNineConnected();
      break;
    case 5:
    default:
      env->SetFiveConnected();
      break;
  }

  if(false){
    waypoints.push_back({{10,10},{14,14}});
  }

  heuristics.resize(waypoints.size());
  if(mapfile.empty()){
    for(int i(0); i<heuristics.size(); ++i){
      heuristics[i]=env;
    }
  }else{
    for(int i(0); i<heuristics.size(); ++i){
      heuristics[i] = new MapPerfectHeuristic<xyLoc,tDirection>(smap,env);
    }
  }

  if(false){
    xyLoc f(10,10);
    xyLoc w(14,14);
    MultiState rt(1);
    for(uint32_t i(0);i<11;++i){
    DAG dg;
      //std::cout << f << " " << w << std::endl;
      uint32_t bst(9999999);
      int hc(env->HCost(f,w)*INFLATION);
      GetMDD(0,0,f,w,dg,rt,(hc+INFLATION*i/5.),bst);
      std::cout<<"\n";
      for(auto const& d:dg){std::cout << d.second<<"\n";}
      std::cout << i/5. << " " << dg.size() << std::endl;
    }
    exit(0);
  }

  TemplateAStar<xyLoc,tDirection,SearchEnvironment<xyLoc,tDirection>> astar;
  //PEAStar<xyLoc,tDirection,MapEnvironment> astar;
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
      Points path;
      if(waypoints[i][0]==waypoints[i][1]){ // Already at goal
        path.push_back(waypoints[i][0]);
      }else{
        astar.SetHeuristic(heuristics[i]);
        astar.GetPath(env,waypoints[i][0],waypoints[i][1],path);
      }
      Path timePath;
      //std::cout << s[i] << "-->" << e[i] << std::endl;
      if(path.empty()){std::cout << "AStar failed on instance " << i << " - No solution\n"; return 0;}
      timePath.push_back(new Node(path[0],0.0));
      for(int i(1); i<path.size(); ++i){
        timePath.push_back(new Node(path[i],timePath.back()->depth+Util::distance(path[i-1].x,path[i-1].y,path[i].x,path[i].y)*INFLATION));
      }
      timePath.push_back(new Node(timePath.back()->n,MAXTIME)); // Add a final wait action that goes way out...
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
          std::cout << t->n << "," << t->depth << "\n";
        }
      }
    }
  }else{
    Path path;
    path.emplace_back(new Node(xyLoc(0,0),0));
    path.emplace_back(new Node(xyLoc(0,1),1));
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
        G[g].first.push_back(waypoints[a][0]);
        G[g].second.push_back(waypoints[a][1]);
        Gid[g].push_back(a);
      }
      ++g;
    }
    for(int j(0); j<G.size(); ++j){
      auto g(G[j]);
      if(verbose)std::cout << "Group " << j <<":\n";
      if(verbose)for(int i(0); i<g.first.size(); ++i){
        std::cout << Gid[j][i] << ":" << g.first[i] << "-->" << g.second[i] << "\n";
      }
      if(g.first.size()>1){
        std::vector<uint32_t> sizes(g.first.size());
        custom_priority_queue<ICTSNode*,ICTSNodePtrComp> q;
        std::unordered_set<std::string> deconf;

        q.push(new ICTSNode(g,sizes,Gid[j]));

        std::vector<std::set<Node*,NodePtrComp>> answer;
        std::vector<ICTSNode*> toDelete;
        uint32_t lastPlateau(q.top()->lb());
        uint32_t bestCost(INF);
        uint32_t bestMergedCost(INF);
        uint32_t delta(0.0);
        bool findOptimal(false);
        //clearNoGoods();
        while(q.size()){
          // Keep searching until we've found a candidate with greater cost than 'best'
          // To keep going on all plateaus <= the best is how we ensure optimality
          // It is impossible for a better solution to be further down in the tree - 
          // no node of cost>=best can possibly produce a child with better cost
          if(verbose)std::cout << "TOP: " << q.top()->lb() << " BEST: " << bestCost << "\n";
          if(q.top()->lb()>=bestCost){
            break;
          }
          // This node could contain a solution since its lb is <=
          ICTSNode* parent(q.popTop());
          if(!quiet){
            std::cout << "pop ";
            for(auto const& a: parent->sizes){
              std::cout << a << " ";
            }
            std::cout << "\n";
            //best: 5.82843 6.65685 5.41421
            //best: 5.82843 6.65685 5.24264
            std::cout << "best: ";
            for(auto b:parent->best)std::cout << b << " ";
            std::cout << "\n";
            std::cout << "SIC: " << parent->lb() << std::endl;
          }
          std::vector<Solution> answers;
          // If we found an answer set and any of the answers are not in conflict
          // with other paths in the solution, we can quit if the answer has a cost
          // equal to that of the best SIC in the current plateau. Otherwise, we will
          // continue the ICT search until the next plateau
          if(parent->isValid(answers)){
            auto cost(mergeSolution(answers,solution,Gid[j],bestMergedCost)); // Returns the cost of the merged solution if < best cost; 0 otherwise
            bestCost=std::min(bestCost,cost.second);
            if(bestCost==parent->lb()&&(q.empty()||q.top()->lb()>bestCost)){break;}
            if(cost.first){
              bestMergedCost=cost.first;
              // Just exit since we found a feasible solution
              if(ICTSNode::suboptimal||ICTSNode::epsilon||bestMergedCost==parent->lb()){
                break;
              }
            }
          }

          for(int i(0); i<parent->sizes.size(); ++i){
            std::vector<uint32_t> sz(parent->sizes);
            sz[i]+=step;
            std::stringstream sv;
            join(sv,sz);
            if(deconf.find(sv.str())==deconf.end()){
              ICTSNode* tmp(new ICTSNode(parent,i,sz[i]));
              if(verbose){
                std::cout << "push ";
                for(auto const& a: tmp->sizes){
                  std::cout << a << " ";
                }
                std::cout << "\n";
                //best: 5.82843 6.65685 5.41421
                //best: 5.82843 6.65685 5.24264
                std::cout << "  best: ";
                for(auto b:tmp->best)std::cout << b << " ";
                std::cout << "\n";
                std::cout << "  SIC: " << tmp->lb() << std::endl;
              }
              q.push(tmp);
              deconf.insert(sv.str());
            }
          }
          toDelete.push_back(parent);
        }
        for(auto z:toDelete){
          delete z;
        }
        mddcache.clear();
        lbcache.clear();
        while(!q.empty()){
          delete q.top();
          q.pop();
        }
      }
    }

    for(auto y:groups){
      //delete y;
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
  if(strcmp(argument[0], "-mode") == 0)
  {
    if(argument[1][0]=='s'){
      ICTSNode::suboptimal=true;
      std::cout << "suboptimal\n";
    }
    else if(argument[1][0]=='p'){
      ICTSNode::pairwise=true;
      if(!quiet)std::cout << "pairwise\n";
    }
    else if(argument[1][0]=='b'){
      ICTSNode::pairwise=true;
      ICTSNode::suboptimal=true;
      if(!quiet)std::cout << "pairwise,suboptimal\n";
    }
    else if(argument[1][0]=='f'){
      ICTSNode::pairwise=true;
      bestonly=true;
      if(!quiet)std::cout << "pairwise,bestonly\n";
    }
    else if(argument[1][0]=='e'){
      ICTSNode::pairwise=true;
      ICTSNode::epsilon=true;
      if(!quiet)std::cout << "pairwise,epsilon\n";
    }
    else if(argument[1][0]=='a'){
      ICTSNode::pairwise=true;
      if(!quiet)std::cout << "pairwise,astar\n";
    }
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
      std::vector<xyLoc> wpts;
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
    if(n==0){
      std::cout<<"-nagents must be specified before -scenfile\n";
      exit(1);
    }
    ScenarioLoader sl(argument[1]);
    if(sl.GetNumExperiments()==0)
    {
      std::cout<<"No experiments in this scenario file or invalid file.\n";
      exit(1);
    }
    std::string pathprefix("../../"); // Because I always run from the build directory...
    mapfile=sl.GetNthExperiment(0).GetMapName();
    mapfile.insert(0,pathprefix); // Add prefix

    for(int i(0); i<n; ++i){
      // Add start/goal location
      std::vector<xyLoc> wpts;
      Experiment e(sl.GetRandomExperiment());
      while(true){
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

