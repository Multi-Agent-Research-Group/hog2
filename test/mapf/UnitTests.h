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
#ifndef mapf_UnitTests_h_
#define mapf_UnitTests_h_

#include "Timer.h"
#include <gtest/gtest.h>
#include "Grid3DEnvironment.h"
#include "Grid3DConstrainedEnvironment.h"
#include "MultiAgentEnvironment.h"
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
  env.WaitTime(10);

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

/*TEST(ICTSAlgorithm, Funky){
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
}*/

TEST(MinDistConstraint, Tether){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  AllMinDist<xyztLoc> tether({0,1},2);
  // The distance between A1 and B1 is sqrt(5)>2
  ASSERT_TRUE(tether.HasConflict(0,a1,a2,1,b1,b2));
  a1.t+=.9*xyztLoc::TIME_RESOLUTION;
  a2.t+=.9*xyztLoc::TIME_RESOLUTION;
  ASSERT_FALSE(tether.HasConflict(0,a1,a2,1,b1,b2));
}

TEST(MinDistConstraint, Cluster){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  xyztLoc c1(2,2,0,0.0f);
  xyztLoc c2(1,1,0,1.414f);
  AllMinDist<xyztLoc> tether({0,1},2);
  // The distance between A1 and B1 is sqrt(5)>2
  ASSERT_TRUE(tether.HasConflict(0,a1,a2,1,b1,b2));
  a1.t+=.9*xyztLoc::TIME_RESOLUTION;
  a2.t+=.9*xyztLoc::TIME_RESOLUTION;
  ASSERT_FALSE(tether.HasConflict(0,a1,a2,1,b1,b2));
}

TEST(MinDistConstraint, Chain){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  xyztLoc c1(2,2,0,0.0f);
  xyztLoc c2(1,1,0,1.414f);
  AnyMinDist<xyztLoc> tether(0,{1,2},2); // 0 must maintain line of sight (max dist 2) with one of 1,2
  // The distance between A1 and B1 is sqrt(5)(>2), but A1 to C1 is 1(<2)
  ASSERT_EQ(-1,tether.agent2);
  ASSERT_EQ(0,tether.agent1);
  std::vector<std::vector<xyztLoc>> sln = {{a1,a2},{b1,b2},{c1,c2}};
  Constraint<xyztLoc>* a(nullptr);
  Constraint<xyztLoc>* b(nullptr);
  std::pair<unsigned,unsigned> c;
  ASSERT_FALSE(tether.HasConflict(sln,a,b,c));
  //ASSERT_EQ(-1,tether.agent2);
}

TEST(MinDistConstraint, ChainWithViolation){
  xyztLoc a1(1,2,0,0.0f);
  xyztLoc a2(2,3,0,1.414f);
  xyztLoc b1(0,0,0,0.0f);
  xyztLoc b2(0,1,0,1.0f);
  xyztLoc c1(1,1,0,0.0f);
  xyztLoc c2(2,2,0,1.414f);
  AnyMinDist<xyztLoc> tether(0,{1,2},2); // 0 must maintain line of sight (max dist 2) with one of 1,2
  // The distance between A1 and B1 is sqrt(5)(>2), but A1 to C1 is 1(<2)
  ASSERT_EQ(-1,tether.agent2);
  ASSERT_EQ(0,tether.agent1);
  std::vector<std::vector<xyztLoc>> sln = {{a1,a2},{b1,b2},{c1,c2}};
  Constraint<xyztLoc>* a(nullptr);
  Constraint<xyztLoc>* b(nullptr);
  std::pair<unsigned,unsigned> c;
  ASSERT_FALSE(tether.HasConflict(sln,a,b,c));
  ASSERT_EQ(1,tether.agent2);
  ASSERT_EQ(0,c.first);
}

TEST(MAAStar, search){
  Map3D map(64,64,1);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);
  MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment> mae(&env);
  TemplateAStar<MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState, MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentAction, MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment> > astar;
  MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState start;
  start.emplace_back(xyztLoc(50,45,0),xyztLoc(50,45,0));
  start.emplace_back(xyztLoc(55,50,0),xyztLoc(55,50,0));
  MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState goal;
  goal.emplace_back(xyztLoc(50,55,0),xyztLoc(50,55,0));
  goal.emplace_back(xyztLoc(45,50,0),xyztLoc(54,50,0));
  std::vector<MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState> path;
  astar.GetPath(&mae,start,goal,path);
  for(auto const& s:path){
    std::cout << s << "\n";
  }
}

TEST(AnyLOS, HasConflict){
  Map3D map(8,8,8);
  map.SetGrid(4,4,8,Map3D::kGround); // Set height to 8
  map.SetGrid(3,3,8,Map3D::kGround); // Set height to 8
  map.SetGrid(4,3,8,Map3D::kGround); // Set height to 8
  map.SetGrid(3,4,8,Map3D::kGround); // Set height to 8
  Grid3DEnvironment env(&map);
  AnyLOS<xyztLoc> loscon(0,{1,2},&env);
//##########
//# a    b #
//# |    | #
//# |^   | #
//# ||## | #
//# ||## | #
//# |+--c| #
//# |    | #
//# v    v #
//##########

// LOS Timetable
// t| b c
// ======
// 0| Y N --> OK
// 1| Y N --> OK
// 2| Y N --> OK
// 3| N Y --> OK
// 4| N Y --> OK
// 5| Y Y --> OK
// 6| Y Y --> OK
  std::vector<std::vector<xyztLoc>> solution(3);
  solution[0]={{1,1,1,0.0f},{1,2,1,1.0f},{1,3,1,2.0f},{1,4,1,3.0f},{1,5,1,4.0f},{1,6,1,5.0f},{1,7,1,6.0f}};
  solution[1]={{6,1,1,0.0f},{6,2,1,1.0f},{6,3,1,2.0f},{6,4,1,3.0f},{6,5,1,4.0f},{6,6,1,5.0f},{6,7,1,6.0f}};
  solution[2]={{5,5,1,0.0f},{4,5,1,1.0f},{3,5,1,2.0f},{2,5,1,3.0f},{2,4,1,4.0f},{2,3,1,5.0f},{2,2,1,6.0f}};
  ASSERT_FALSE(loscon.HasConflict(solution));
}

#endif
