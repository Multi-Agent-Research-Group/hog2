#ifndef mapf_UnitTests_h_
#define mapf_UnitTests_h_

#include "Timer.h"
#include <gtest/gtest.h>
#include "Grid3DEnvironment.h"
#include "Grid3DConstrainedEnvironment.h"
//#include "Grid3DEnvironment.h"
//#include "Grid3DConstrainedEnvironment.h"
//#include "Map3d.h"
#include "TemplateAStar.h"
#include "TemporalAStar.h"
#include "dtedreader.h"
#include "ICTSAlgorithm.h"

extern double agentRadius;

TEST(ICTSAlgorithm, SimpleTest){
  agentRadius=.25;
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);

  std::vector<EnvironmentContainer<xyztLoc,t3DDirection>*> envs(4);
  for(auto& e:envs){
    e=new EnvironmentContainer<xyztLoc,t3DDirection>();
    e->environment=&env;
  }
  Solution<xyztLoc> solution;
  MultiAgentState<xyztLoc> start = {{0,0},{7,7},{0,7},{7,0}};
  MultiAgentState<xyztLoc> goal = {{7,7},{0,0},{7,0},{0,7}};
  ICTSAlgorithm<xyztLoc,t3DDirection> ia;
  ia.verify=true;
  ia.verbose=false;
  std::string hint;
  ia.GetSolution(envs,start,goal,solution,hint);
  for(auto const& ss: solution)
    for(auto const& s:ss)
      std::cout << s << "\n";
  ASSERT_TRUE(validateSolution(solution));
}

TEST(ICTSAlgorithm, Funky){
  agentRadius=.25;
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);

  std::vector<EnvironmentContainer<xyztLoc,t3DDirection>*> envs(2);
  for(auto& e:envs){
    e=new EnvironmentContainer<xyztLoc,t3DDirection>();
    e->environment=&env;
  }
  Solution<xyztLoc> solution;
  MultiAgentState<xyztLoc> start = {{3,6},{4,5}};
  MultiAgentState<xyztLoc> goal = {{5,6},{5,7}};
  ICTSAlgorithm<xyztLoc,t3DDirection> ia;
  ia.verify=true;
  ia.verbose=true;
  std::string hint;
  ia.GetSolution(envs,start,goal,solution,hint);
  for(auto const& ss: solution)
    for(auto const& s:ss)
      std::cout << s << "\n";
  ASSERT_TRUE(validateSolution(solution));
}

TEST(LOSConstraint, Tether){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  AllLOS<xyztLoc> tether({0,1},2);
  // The distance between A1 and B1 is sqrt(5)>2
  ASSERT_TRUE(tether.HasConflict(0,a1,a2,1,b1,b2));
  a1.t+=900;
  a2.t+=900;
  ASSERT_FALSE(tether.HasConflict(0,a1,a2,1,b1,b2));
}

TEST(LOSConstraint, Cluster){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  xyztLoc c1(2,2,0,0.0f);
  xyztLoc c2(1,1,0,1.414f);
  AllLOS<xyztLoc> tether({0,1},2);
  // The distance between A1 and B1 is sqrt(5)>2
  ASSERT_TRUE(tether.HasConflict(0,a1,a2,1,b1,b2));
  a1.t+=900;
  a2.t+=900;
  ASSERT_FALSE(tether.HasConflict(0,a1,a2,1,b1,b2));
}

TEST(LOSConstraint, Chain){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  xyztLoc c1(2,2,0,0.0f);
  xyztLoc c2(1,1,0,1.414f);
  AnyLOS<xyztLoc> tether(0,{1,2},2); // 0 must maintain line of sight (max dist 2) with one of 1,2
  // The distance between A1 and B1 is sqrt(5)(>2), but A1 to C1 is 1(<2)
  ASSERT_EQ(-1,tether.agent);
  ASSERT_EQ(0,tether.mainAgent);
  ASSERT_TRUE(tether.finalResult);
  ASSERT_TRUE(tether.HasConflict(0,a1,a2,1,b1,b2)); // No LOS here
  ASSERT_EQ(-1,tether.agent);
  ASSERT_FALSE(tether.HasConflict(0,a1,a2,2,c1,c2)); // LOS here
  ASSERT_FALSE(tether.finalResult); // At least one LOS found... :)
  ASSERT_EQ(2,tether.agent); // LOS with agent 2
  ASSERT_EQ(a1,tether.agent1.first);
  ASSERT_EQ(a2,tether.agent1.second);
  ASSERT_EQ(c1,tether.agent2.first);
  ASSERT_EQ(c2,tether.agent2.second);
}

#endif
