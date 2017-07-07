#ifndef UnitTests_h_
#define UnitTests_h_

#include "AirplaneGridCardinal.h"
#include "AirplanePerimeterBuilder.h"
#include "AdmissibilityChecker.h"
#include "AirplaneHighway4Cardinal.h"
#include "AirplaneNaiveHiFiGridless.h"
#include <gtest/gtest.h>
#include "BucketHash.h"
#include "TemplateAStar.h"
#include "AirStates.h"

template<typename environ, typename state>
void TestReverseSuccessors(environ& env, state const& s, state const& g, state o) {
  //std::cout << "original " << o << "\n";
  std::vector<state> ancestors;
  env.GetReverseSuccessors(o,ancestors);
  for (auto &a : ancestors)
  {
    //std::cout << " Ancestor " << a << "\n";
    // All ancestors should have a successor that is equal to the original state
    std::vector<state> successors;
    env.GetSuccessors(a,successors);
    bool found(false);
    for(auto &c: successors){
      //std::cout << "        " << c << "\n";
      if(c==o){found=true; break;}
    }
    ASSERT_TRUE(found);
  }
}
TEST(Heuristic, CardinalHighway){
  airplaneState s(67, 13, 16, 1, 7);
  airplaneState g(12, 39, 4, 1, 7);

  AirplaneHighway4CardinalEnvironment env;
  env.setGoal(airtimeState());
  env.loadPerimeterDB();
  AdmissibilityChecker<airplaneState,airplaneAction,AirplaneEnvironment> checker;
  std::vector<airplaneState> states;
  states.emplace_back(40,40,10,3,0);
  //states.emplace_back(40,40,10,2,0);
  //states.emplace_back(40,40,10,3,1);
  ASSERT_TRUE(checker.check(env,states,100.,9));
  ASSERT_TRUE(checker.checkReverse(env,states,100.,9));
}
/*
TEST(Heuristic, Admissibile) { 
  AirplaneEnvironment env;
  env.setGoal(airtimeState());
  env.loadPerimeterDB();
  AdmissibilityChecker<airplaneState,airplaneAction,AirplaneEnvironment> checker;
  std::vector<airplaneState> states;
  states.emplace_back(40,40,10,3,0);
  //states.emplace_back(40,40,10,2,0);
  //states.emplace_back(40,40,10,3,1);
  ASSERT_TRUE(checker.check(env,states,100.,9));
  ASSERT_TRUE(checker.checkReverse(env,states,100.,9));
}
TEST(AirplaneEnvironmentTest, GetActions) { 
  AirplaneEnvironment env;
  env.loadPerimeterDB();
  airplaneAction a(0,0,0);
  airplaneState s(50,50,16,3,0);
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  ASSERT_EQ(63, actions.size());
  ASSERT_EQ(63, ractions.size());

  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    ASSERT_EQ(s,s1);
    ASSERT_EQ(a,a2);
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  TestReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
}

TEST(GridlessEnvironmentTest, GetActions) { 
  AirplaneGridlessEnvironment env;
  PlatformAction a(0,0,0);
  PlatformState s(50,50,16,0,0,3);
  PlatformState g(50,40,18,0,0,3);
  env.setGoal(g);
  std::vector<PlatformAction> actions;
  env.GetActions(s,actions);
  std::vector<PlatformAction> ractions;
  env.GetReverseActions(s,ractions);
  ASSERT_EQ(45, actions.size());
  ASSERT_EQ(45, ractions.size());

  for (auto &a : actions)
  {
    PlatformState s1=s;
    env.ApplyAction(s,a);
    ASSERT_FALSE(s==s1);
    PlatformAction a2(env.GetAction(s1,s));
    std::cout <<a;
    std::cout <<a2;
    env.UndoAction(s,a);
    ASSERT_EQ(s,s1);
    ASSERT_EQ(a,a2);
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  PlatformState o(56,44,11,0,0,3);
  TestReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  TestReverseSuccessors(env,s,g,o);
}

TEST(HeuristicTest, LearnSpace) { 
  AirplaneEnvironment env;
  env.setGoal(airtimeState());
  AirplanePerimeterBuilder<airplaneState,airplaneAction,AirplaneEnvironment,5> builder;
  std::vector<airplaneState> states;
  states.emplace_back(40,40,10,3,0);
  builder.learnSpace(env,states,5);
  airplaneState s(40,39,10,3,0);
  airplaneState s2(40,41,10,3,0);
  std::cout << env.GCost(s,states[0]) << "\n";
  std::cout << builder.GCost(s,states[0],env) << "\n";
  std::cout << builder.GCost(s2,states[0],env) << "\n";
}

TEST(HeuristicTest, HashTest) { 
  AirplaneEnvironment env;
  env.setGoal(airtimeState());
  AirplanePerimeterBuilder<airplaneState,airplaneAction,AirplaneEnvironment,5> builder;
  for(int i(0); i<1000; ++i){
    std::set<uint64_t> hashes;
    airplaneState start(rand() % 80, rand() % 80, rand() % 20, rand() % 5 + 1, rand() % 8, false);
    airplaneState goal(rand() % 80, rand() % 80, rand() % 20, rand() % 5 + 1, rand() % 8, false);
    std::vector<double> data(5);
    env.GetDifference(start,goal,data);
    uint64_t hash(builder.getHash(data,env.GetRanges()));
    std::vector<double> reversed(5);
    builder.fromHash(reversed,hash,env.GetRanges());
    ASSERT_EQ(data,reversed);
  }
  //std::cout << builder.GCost(s,states[0],env) << "\n";
  //std::cout << builder.GCost(s2,states[0],env) << "\n";
}

TEST(HeuristicTest, LoadTest) { 
  AirplaneEnvironment env;
  env.setGoal(airtimeState());
  AirplanePerimeterBuilder<airplaneState,airplaneAction,AirplaneEnvironment,5> builder;
  std::vector<airplaneState> states;
  for(int s=3; s<4; ++s){
    for(int h=0; h<1; ++h){
      states.emplace_back(40,40,10,s,h);
    }
  }
  builder.loadDB(env,states,5,99,3,99,"airplane",true); // Force build
  {
    airplaneState g(40,40,10,3,0);
    airplaneState s(40,41,10,3,0);
    airplaneState s2(40,39,10,3,0);
    builder.loadDB(env,states,5,99,3,99,"airplane");
    ASSERT_EQ(.006,builder.GCost(s,g,env));
    ASSERT_EQ(.03,builder.GCost(s2,g,env));
    ASSERT_EQ(.012,builder.GCost(s,s2,env));
  }
}

TEST(HeuristicTest, GetTestTest) { 
  std::vector<std::pair<airplaneState,airplaneState> >tests;

  /////////////////////////////////////////////////////////////////////////////
  // Inside radius
  /////////////////////////////////////////////////////////////////////////////

  // Similar X,Y,Z
  tests.push_back(std::make_pair(airplaneState(32,28,12,3,2),airplaneState(30,30,10,3,2)));

  /////////////////////////////////////////////////////////////////////////////
  // 3 Pillars
  /////////////////////////////////////////////////////////////////////////////

  // Same X,Y; +Z
  tests.push_back(std::make_pair(airplaneState(30,30,10,3,2),airplaneState(30,30,16,3,2)));
  // Same X,Y; -Z
  tests.push_back(std::make_pair(airplaneState(30,30,10,3,2),airplaneState(30,30,6,3,2)));
  // Similar X, Same Y; +Z
  tests.push_back(std::make_pair(airplaneState(32,30,10,3,2),airplaneState(30,30,16,3,2)));
  // Same X, Similar Y; -Z
  tests.push_back(std::make_pair(airplaneState(30,29,10,3,2),airplaneState(30,30,6,3,2)));
  // Similar X, Similar Y; -Z
  tests.push_back(std::make_pair(airplaneState(32,29,10,3,2),airplaneState(30,30,6,3,2)));

  // Same X,Z; +Y
  tests.push_back(std::make_pair(airplaneState(30,36,10,3,2),airplaneState(30,30,10,3,2)));
  // Same X,Z; -Y
  tests.push_back(std::make_pair(airplaneState(30,24,10,3,2),airplaneState(30,30,10,3,2)));
  // Similar X, Same Z; +Y
  tests.push_back(std::make_pair(airplaneState(32,36,10,3,2),airplaneState(30,30,10,3,2)));
  // Same X, Similar Z; -Y
  tests.push_back(std::make_pair(airplaneState(30,24,8,3,2),airplaneState(30,30,10,3,2)));
  // Similar X, Similar Z; -Y
  tests.push_back(std::make_pair(airplaneState(32,24,12,3,2),airplaneState(30,30,10,3,2)));

  // Same Y,Z; +X
  tests.push_back(std::make_pair(airplaneState(36,30,10,3,2),airplaneState(30,30,10,3,2)));
  // Same Y,Z; -X
  tests.push_back(std::make_pair(airplaneState(24,30,10,3,6),airplaneState(30,30,10,3,6)));
  // Similar Y, Same Z; +X
  tests.push_back(std::make_pair(airplaneState(36,32,10,3,2),airplaneState(30,30,10,3,2)));
  // Same Y, Similar Z; -X
  tests.push_back(std::make_pair(airplaneState(24,30,8,3,6),airplaneState(30,30,10,3,6)));
  // Similar Y, Similar Z; -X
  tests.push_back(std::make_pair(airplaneState(24,28,12,3,6),airplaneState(30,30,10,3,6)));

  /////////////////////////////////////////////////////////////////////////////
  // 3 Planes
  /////////////////////////////////////////////////////////////////////////////

  // Same X
  tests.push_back(std::make_pair(airplaneState(30,35,15,3,2),airplaneState(30,30,10,3,2)));
  // Similar X
  tests.push_back(std::make_pair(airplaneState(32,35,5,3,2),airplaneState(30,30,10,3,2)));

  // Same Y
  tests.push_back(std::make_pair(airplaneState(35,30,15,3,2),airplaneState(30,30,10,3,2)));
  // Similar Y
  tests.push_back(std::make_pair(airplaneState(35,32,5,3,2),airplaneState(30,30,10,3,2)));

  // Same Z
  tests.push_back(std::make_pair(airplaneState(35,25,10,3,2),airplaneState(30,30,10,3,2)));
  // Similar Z
  tests.push_back(std::make_pair(airplaneState(35,25,8,3,2),airplaneState(30,30,10,3,2)));

  /////////////////////////////////////////////////////////////////////////////
  // 8 Corners
  /////////////////////////////////////////////////////////////////////////////

  // +X+Y+Z
  tests.push_back(std::make_pair(airplaneState(35,35,15,3,2),airplaneState(30,30,10,3,2)));
  // +X+Y-Z
  tests.push_back(std::make_pair(airplaneState(35,35,5,3,2),airplaneState(30,30,10,3,2)));
  // +X-Y+Z
  tests.push_back(std::make_pair(airplaneState(35,25,15,3,2),airplaneState(30,30,10,3,2)));
  // +X-Y-Z
  tests.push_back(std::make_pair(airplaneState(35,25,5,3,2),airplaneState(30,30,10,3,2)));
  // -X+Y+Z
  tests.push_back(std::make_pair(airplaneState(25,35,15,3,6),airplaneState(30,30,10,3,6)));
  // -X+Y-Z
  tests.push_back(std::make_pair(airplaneState(25,35,5,3,6),airplaneState(30,30,10,3,6)));
  // -X-Y+Z
  tests.push_back(std::make_pair(airplaneState(25,25,15,3,6),airplaneState(30,30,10,3,6)));
  // -X-Y-Z
  tests.push_back(std::make_pair(airplaneState(25,25,5,3,6),airplaneState(30,30,10,3,6)));


  AirplaneEnvironment env;
  env.loadPerimeterDB();
  std::vector<airplaneState> path;
  TemplateAStar<airplaneState,airplaneAction,AirplaneEnvironment> astar;
  for(auto const& inst: tests){
    env.setStart(inst.first);
    env.setGoal(inst.second);
    astar.GetPath(&env,env.getStart(),env.getGoal(),path);
    double cost=env.GetPathLength(path);
    double hcost=env.HCost(env.getStart(),env.getGoal()); 
    ASSERT_LE(hcost,cost);
  }
}
TEST(AirplaneEnvironmentTest, ProblemChild) { 
  AirplaneEnvironment env;
  env.loadPerimeterDB();
  std::vector<airplaneState> path;
  TemplateAStar<airplaneState,airplaneAction,AirplaneEnvironment> astar;
  env.setStart(airplaneState(42,44,15,1,3));
  env.setGoal(airplaneState(40,40,10,1,1));
  //env.setStart(airplaneState(40,44,15,1,5));
  //env.setGoal(airplaneState(40,40,10,1,0));
  //env.setStart(airplaneState(40,45,12,1,0));
  //env.setGoal(airplaneState(40,40,10,1,0));
  //env.setStart(airplaneState(41,40,15,1,1));
  //env.setGoal(airplaneState(40,40,10,1,0));
  //env.setStart(airplaneState(37,45,10,1,0));
  //env.setGoal(airplaneState(40,40,10,1,0));
  astar.GetPath(&env,env.getStart(),env.getGoal(),path);
  for(int i(0); i<path.size(); ++i){
    std::cout << path[i]<<" ";
    if(i)
      std::cout << env.GCost(path[i-1],path[i]) << " " << env.myHCost(path[i-1],path[i]) << "\n";
    else
      std::cout << "\n";
  }
  std::cout << env.myHCost(env.getStart(),env.getGoal());
  double cost=env.GetPathLength(path);
  double hcost=env.HCost(env.getStart(),env.getGoal()); 
  ASSERT_LE(hcost,cost);
}

TEST(BucketHash, Happy) { 
  struct HashThing{
    HashThing(int i):id(i){}
    int id;
    bool operator==(HashThing const& other)const{return id==other.id;}
    bool operator<(HashThing const& other)const{return id<other.id;}
  };

  BucketHash<HashThing> bh(.1);
  bh.insert(0,.1,HashThing(0));
  bh.insert(.1,.2,HashThing(1));
  ASSERT_EQ(2,bh.size());
  std::set<HashThing> s;
  bh.get(0,.099,s);
  ASSERT_EQ(1,s.size());
  ASSERT_TRUE(s.find(0)!=s.end());
  ASSERT_TRUE(s.find(1)==s.end());

  bh.get(.1,.101,s);
  ASSERT_EQ(2,s.size());
  ASSERT_TRUE(s.find(0)!=s.end());
  ASSERT_TRUE(s.find(1)!=s.end());

  bh.get(.2,.3,s);
  ASSERT_EQ(1,s.size());
  ASSERT_TRUE(s.find(0)==s.end());
  ASSERT_TRUE(s.find(1)!=s.end());

  bh.remove(0,.099,HashThing(0));
  bh.get(0,.099,s);
  ASSERT_EQ(0,s.size());
  ASSERT_TRUE(s.find(0)==s.end());
  ASSERT_TRUE(s.find(1)==s.end());

}

TEST(PlatformState, apply) {
  AirplaneHiFiGridlessEnvironment env;
  PlatformState s(20,20,10,180.0,0,3);
  PlatformAction a(-2.5,0,1);
  env.ApplyAction(s,a);
  ASSERT_EQ(177.5,s.hdg());
}

TEST(PlatformState, heading) {
  PlatformState s(20,20,10,0,0,3);
  PlatformState g(30,30,10,0,0,3);
  ASSERT_EQ(45.0,s.headingTo(g));
}

TEST(PlatformState, elevation1) {
  PlatformState s(20,20,10,0,0,3);
  PlatformState g(30,20,10,0,0,3);
  ASSERT_EQ(0,s.elevationTo(g));
}

TEST(PlatformState, elevation2) {
  PlatformState s(20,20,20,0,0,3);
  PlatformState g(30,20,10,0,0,3);
  ASSERT_EQ(-45.0,s.elevationTo(g));
}

TEST(PlatformState, elevation) {
  PlatformState s(20,20,10,0,0,3);
  PlatformState g(30,20,20,0,0,3);
  ASSERT_EQ(45.0,s.elevationTo(g));
}

TEST(PlatformState, distance) {
  PlatformState s(20,20,10,0,0,3);
  PlatformState g(30,20,10,0,0,3);
  ASSERT_EQ(10.0,s.distanceTo(g));
}

TEST(PlatformState, GetActions) {
  PlatformState s(20,20,10,0,0,3);
  PlatformState g(30,20,10,0,0,3);
  AirplaneHiFiGridlessEnvironment env;
  env.setStart(s);
  env.setGoal(g);
  std::vector<PlatformAction> actions;
  env.GetActions(s,actions);
  // there should be a positive 90 deg turn
  ASSERT_EQ(7.5, actions[0].turn());
  s.x=40;
  env.GetActions(s,actions);
  // neg. 90 degree turn
   ASSERT_EQ(-7.5, actions[0].turn());
}

TEST(PlatformState, HCost) {
  PlatformState s(42,48,10,0,0,3);
  PlatformState g(38,35,10,0,0,3);
  AirplaneHiFiGridlessEnvironment env;
  env.setStart(s);
  env.setGoal(g);
  TemplateAStar<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment> astar;
  std::vector<PlatformState> path;
  astar.GetPath(&env,s,g,path);
  double cost(env.GetPathLength(path));
  ASSERT_LE(env.HCost(s,g),cost);
  std::cout << "len: " << cost << "\n";
  std::cout << "h: " << env.HCost(s,g) << "\n";
}

*/

/* Tests for running on state space without a latticec
TEST(AStarNonHolonomicComparator, experiment){
  PlatformState s(42,48,10,0,0,3);
  PlatformState g(38,35,10,0,0,3);

  AirplaneHiFiGridlessEnvironment env;
  env.setStart(s);
  env.setGoal(g);
  TemplateAStar<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment,AStarOpenClosed<PlatformState, NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment> > > astar;
  NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::currentEnv=&env;
  StraightLineHeuristic1<PlatformState> z;
  astar.SetHeuristic(&z);
  env.SetNilGCosts();
  astar.SetVerbose(true);
  std::vector<PlatformState> sol;
  astar.GetPath(&env,s,g,sol);
  std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << " cost: " << env.GetPathLength(sol) << " Hval: " << env.HCost(s,g) << "\n";
}
*/

TEST(AStar, PEA)
{
  /*
  {
    PlatformState s(42,48,10,0,0,3);
    //PlatformState g(38,35,10,0,0,3);
    PlatformState g(34,48,10,0,0,3);
    //PlatformState s(20, 20, 12, 90, 0, 1);
    //PlatformState g(30, 20, 11, 0, 0, 1);

    AirplaneNaiveHiFiGridlessEnvironment env;
    env.setStart(s);
    env.setGoal(g);
    std::cout << env.name() << "\n";

    TemplateAStar<PlatformState,PlatformAction,AirplaneNaiveHiFiGridlessEnvironment> astar;
    astar.SetVerbose(true);
    std::vector<PlatformState> sol;
    astar.GetPath(&env,s,g,sol);
    std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << " cost: " << env.GetPathLength(sol) << " Hval: " << env.HCost(s,g) << "\n";

    TemplateAStar<PlatformState,PlatformAction,AirplaneNaiveHiFiGridlessEnvironment> astar2;
    astar2.SetVerbose(true);
    std::vector<PlatformState> sol2;
    astar2.SetDoPartialExpansion(true);
    astar2.GetPath(&env,s,g,sol2);
    std::cout << "PEA* expansions: " << astar2.GetNodesExpanded() << " unique:" << astar2.GetUniqueNodesExpanded() << " generations: " << astar2.GetNodesTouched() << " mem: " << astar2.GetMemoryUsage() << " path len: " << sol2.size() << " cost: " << env.GetPathLength(sol2) << " Hval: " << env.HCost(s,g) << "\n";

    ASSERT_LE(astar2.GetMemoryUsage(),astar.GetMemoryUsage());
    ASSERT_EQ(sol2.size(),sol.size());
    ASSERT_TRUE(fequal(env.GetPathLength(sol2),env.GetPathLength(sol)));
  }
*/
  {
    airplaneState s(10, 10, 16, 3, 0, false);
    airplaneState g(40, 20, 11, 3, 0, false);
    AirplaneGridCardinalEnvironment env;
    env.setGoal(g);
    env.loadPerimeterDB();
    TemplateAStar<airplaneState, airplaneAction, AirplaneGridCardinalEnvironment> astar;
    std::vector<airplaneState> sol;
    astar.GetPath(&env,s,g,sol);
    std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << "\n";

    TemplateAStar<airplaneState, airplaneAction, AirplaneGridCardinalEnvironment> astar2;
    astar2.SetDoPartialExpansion(true);
    std::vector<airplaneState> sol2;
    astar2.GetPath(&env,s,g,sol2);
    //std::cout << "PEA* expansions: " << astar2.GetNodesExpanded() << " unique:" << astar2.GetUniqueNodesExpanded() << " generations: " << astar2.GetNodesTouched() << " mem: " << astar2.GetMemoryUsage() << " path len: " << sol2.size() << "\n";
    ASSERT_LE(astar2.GetMemoryUsage(),astar.GetMemoryUsage());
    ASSERT_EQ(sol2.size(),sol.size());
    ASSERT_TRUE(fequal(env.GetPathLength(sol2),env.GetPathLength(sol)));
  }
  {
    airplaneState s(10, 10, 16, 3, 0, false);
    airplaneState g(40, 20, 11, 3, 0, false);
    AirplaneEnvironment env;
    env.setGoal(g);
    env.loadPerimeterDB();
    TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
    std::vector<airplaneState> sol;
    astar.GetPath(&env,s,g,sol);
    std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << "\n";

    TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar2;
    std::vector<airplaneState> sol2;
    astar2.SetDoPartialExpansion(true);
    astar2.GetPath(&env,s,g,sol2);
    std::cout << "PEA* expansions: " << astar2.GetNodesExpanded() << " unique:" << astar2.GetUniqueNodesExpanded() << " generations: " << astar2.GetNodesTouched() << " mem: " << astar2.GetMemoryUsage() << " path len: " << sol2.size() << "\n";
    ASSERT_LE(astar2.GetMemoryUsage(),astar.GetMemoryUsage());
    ASSERT_EQ(sol2.size(),sol.size());
    ASSERT_TRUE(fequal(env.GetPathLength(sol2),env.GetPathLength(sol)));
  }
}

#endif
