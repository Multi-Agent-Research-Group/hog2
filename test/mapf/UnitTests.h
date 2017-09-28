#ifndef mapf_UnitTests_h_
#define mapf_UnitTests_h_

#include "Timer.h"
#include <gtest/gtest.h>
#include "Map2DEnvironment.h"
#include "Map2DConstrainedEnvironment.h"
//#include "Grid3DEnvironment.h"
//#include "Grid3DConstrainedEnvironment.h"
//#include "Map3d.h"
#include "TemplateAStar.h"
#include "TemporalAStar.h"
#include "dtedreader.h"
#include "ICTSAlgorithm.h"

TEST(ICTSAlgorithm, SimpleTest){
  Map map(8,8);
  MapEnvironment menv(&map);
  menv.SetNineConnected();
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);

  std::vector<EnvironmentContainer<xytLoc,tDirection>*> envs(4);
  for(auto& e:envs){
    e=new EnvironmentContainer<xytLoc,tDirection>();
    e->environment=&env;
  }
  Solution<xytLoc> solution;
  MultiAgentState<xytLoc> start = {{0,0},{7,7},{0,7},{7,0}};
  MultiAgentState<xytLoc> goal = {{7,7},{0,0},{7,0},{0,7}};
  ICTSAlgorithm<xytLoc,tDirection> ia;
  ia.verify=true;
  ia.verbose=false;
  ia.GetSolution(envs,start,goal,solution);
  for(auto const& ss: solution)
    for(auto const& s:ss)
      std::cout << s << "\n";
  ASSERT_TRUE(validateSolution(solution));
}

TEST(ICTSAlgorithm, Funky){
  Map map(8,8);
  MapEnvironment menv(&map);
  menv.SetNineConnected();
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);

  std::vector<EnvironmentContainer<xytLoc,tDirection>*> envs(4);
  for(auto& e:envs){
    e=new EnvironmentContainer<xytLoc,tDirection>();
    e->environment=&env;
  }
  Solution<xytLoc> solution;
  MultiAgentState<xytLoc> start = {{3,6},{4,5}};
  MultiAgentState<xytLoc> goal = {{5,6},{5,7}};
  ICTSAlgorithm<xytLoc,tDirection> ia;
  ia.verify=true;
  ia.verbose=true;
  ia.GetSolution(envs,start,goal,solution);
  for(auto const& ss: solution)
    for(auto const& s:ss)
      std::cout << s << "\n";
  ASSERT_TRUE(validateSolution(solution));
}

#endif
