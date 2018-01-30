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

TEST(MAAStar, search){
  Map3D map(64,64,1);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);
  std::vector<Grid3DConstrainedEnvironment const*> envs ={&env,&env};
  /*MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment> mae(envs);
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
  }*/
}

#endif
