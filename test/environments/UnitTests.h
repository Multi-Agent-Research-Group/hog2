#ifndef env_UnitTests_h_
#define env_UnitTests_h_

#include "Timer.h"
#include <gtest/gtest.h>
#include "Map2DEnvironment.h"
#include "Map2DConstrainedEnvironment.h"
#include "Grid3DEnvironment.h"
#include "Grid3DConstrainedEnvironment.h"
#include "Map3d.h"
#include "TemplateAStar.h"

TEST(Map2D, TwentyFourConnected_Successors){
  Map map(8,8);
  MapEnvironment env(&map);
  env.SetTwentyFourConnected();
  std::vector<xyLoc> successors;
  env.GetSuccessors({3,3},successors);
  ASSERT_EQ(24,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map2D, TwentyFourConnectedWObstacle_Successors){
  Map map(8,8);
  MapEnvironment env(&map);
  map.SetTerrainType(4,3,kOutOfBounds); // Should knock out 10: r,ur,u2r,dr,d2r,r2,u2r2,ur2,dr2,d2r2
  env.SetTwentyFourConnected();
  std::vector<xyLoc> successors;
  env.GetSuccessors({3,3},successors);
  ASSERT_EQ(14,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map2D, FortyEightConnected_Successors){
  Map map(8,8);
  MapEnvironment env(&map);
  env.SetFortyEightConnected();
  std::vector<xyLoc> successors;
  env.GetSuccessors({3,3},successors);
  ASSERT_EQ(48,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map2D, FortyEightConnectedWObstacle_Successors){
  Map map(8,8);
  MapEnvironment env(&map);
  map.SetTerrainType(4,3,kOutOfBounds);
  env.SetFortyEightConnected();
  std::vector<xyLoc> successors;
  env.GetSuccessors({3,3},successors);
  ASSERT_EQ(27,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map3D, LineOfSight){
  Map3D map(8,8,8);
  map.AddObstacle(4,4,4);
  ASSERT_TRUE(map.LineOfSight(3,3,3,0,0,0));
  ASSERT_FALSE(map.LineOfSight(5,5,5,0,0,0));
  ASSERT_FALSE(map.LineOfSight(7,1,7,1,7,1));
  ASSERT_FALSE(map.LineOfSight(4,0,4,4,7,4));
  ASSERT_FALSE(map.LineOfSight(0,3,7,7,4,1));
}

TEST(Map3D, ZeroConnected){
  Map3D map(8,8,8);
  Grid3DEnvironment env(&map);
  std::vector<xyzLoc> successors;
  env.GetSuccessors({3,3,3},successors);
  ASSERT_EQ(6,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map3D, OneConnected){
  Map3D map(8,8,8);
  Grid3DEnvironment env(&map);
  env.SetOneConnected();
  std::vector<xyzLoc> successors;
  env.GetSuccessors({3,3,3},successors);
  ASSERT_EQ(26,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map3D, TwoConnected){
  Map3D map(8,8,8);
  Grid3DEnvironment env(&map);
  env.SetTwoConnected();
  std::vector<xyzLoc> successors;
  env.GetSuccessors({3,3,3},successors);
  ASSERT_EQ(124,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}
TEST(Map3D, ThreeConnected){
  Map3D map(8,8,8);
  Grid3DEnvironment env(&map);
  env.SetThreeConnected();
  std::vector<xyzLoc> successors;
  env.GetSuccessors({3,3,3},successors);
  ASSERT_EQ(342,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}
TEST(Map3D, SingleConnected){
  Map3D map(8,8,8);
  Grid3DEnvironment env(&map);
  std::vector<xyzLoc> successors;
  env.GetSuccessors({3,3,3},successors);
  ASSERT_EQ(6,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map2D, SingleConnectedWObstacle){
  Map map(8,8);
  MapEnvironment env(&map);
  map.SetTerrainType(4,3,kOutOfBounds);
  env.SetFortyEightConnected();
  std::vector<xyLoc> successors;
  env.GetSuccessors({3,3},successors);
  ASSERT_EQ(27,successors.size());
  for(int i(0);i<successors.size(); ++i){
    ASSERT_TRUE(env.LineOfSight({3,3},successors[i]));
    //std::cout << successors[i] << "\n";
    for(int j(i+1);j<successors.size(); ++j){
      ASSERT_NE(successors[i],successors[j]);
    }
  }
}

TEST(Map2D, TwentyFourConnected_AdmissTest){
  return;
  TemplateAStar<xyLoc,tDirection,MapEnvironment> astar;
  Map map(100,100);
  MapEnvironment env(&map);
  env.SetTwentyFourConnected();
  for(int i(0); i<1000; ++i){
    std::vector<xyLoc> path;
    xyLoc s({rand()%100,rand()%100});
    xyLoc g({rand()%100,rand()%100});
    astar.GetPath(&env,s,g,path);
    double t(0);
    std::cout << path.size() << "----------------\n" << path[0] << "\n";
    for(int j(1); j<path.size(); ++j){
      t += env.GCost(path[j-1],path[j]);
      //std::cout << path[j] << "\n";
    }
    //std::cout << s << " " << g << " " << t << " " << env.HCost(s,g) << "\n";
    ASSERT_TRUE(fequal(t,env.HCost(s,g)));
  }
}

TEST(Map2D, FortyEightConnected_AdmissTest){
  TemplateAStar<xyLoc,tDirection,MapEnvironment> astar;
  Map map(100,100);
  MapEnvironment env(&map);
  env.SetFortyEightConnected();
  for(int i(0); i<1000; ++i){
    std::vector<xyLoc> path;
    xyLoc s({rand()%100,rand()%100});
    xyLoc g({rand()%100,rand()%100});
    //std::cout << s << " " << g << "\n";
    astar.GetPath(&env,s,g,path);
    double t(0);
    //std::cout << path.size() << "----------------\n" << path[0] << "\n";
    for(int j(1); j<path.size(); ++j){
      t += env.GCost(path[j-1],path[j]);
      //std::cout << path[j] << "\n";
    }
    //std::cout << s << " " << g << " " << t << " " << env.HCost(s,g) << "\n";
    ASSERT_TRUE(fequal(t,env.HCost(s,g)));
  }
}

TEST(Map2D, FortyEightConnected_GCost){
  Map map(8,8);
  MapEnvironment env(&map);
  env.SetFortyNineConnected();
  ASSERT_EQ(1.0,env.GCost({0,0},{0,0}));
  ASSERT_EQ(0.0,env.HCost({0,0},{0,0}));
  ASSERT_EQ(1.0,env.GCost({0,0},{1,0}));
  ASSERT_EQ(1.0,env.HCost({0,0},{1,0}));
  ASSERT_DOUBLE_EQ(sqrt(2),env.GCost({0,0},{1,1}));
  ASSERT_DOUBLE_EQ(sqrt(2),env.HCost({0,0},{1,1}));
  ASSERT_DOUBLE_EQ(sqrt(5),env.GCost({0,0},{1,2}));
  ASSERT_DOUBLE_EQ(sqrt(5),env.HCost({0,0},{1,2}));
  ASSERT_DOUBLE_EQ(sqrt(10),env.GCost({0,0},{1,3}));
  ASSERT_DOUBLE_EQ(sqrt(10),env.HCost({0,0},{1,3}));
  ASSERT_DOUBLE_EQ(sqrt(13),env.GCost({0,0},{2,3}));
  ASSERT_DOUBLE_EQ(sqrt(13),env.HCost({0,0},{2,3}));
  ASSERT_DOUBLE_EQ(sqrt(13)*2.,env.HCost({0,0},{4,6}));
  ASSERT_DOUBLE_EQ(sqrt(13)+2*sqrt(5),env.HCost({0,0},{4,7}));
  ASSERT_DOUBLE_EQ(sqrt(5)*4.,env.HCost({0,0},{4,8}));
  ASSERT_DOUBLE_EQ(sqrt(10)+3.*sqrt(5),env.HCost({0,0},{4,9}));
  ASSERT_DOUBLE_EQ(sqrt(13)+3.*sqrt(5),env.HCost({0,0},{5,9}));
  ASSERT_DOUBLE_EQ(3.*sqrt(13),env.HCost({0,0},{6,9}));
  ASSERT_DOUBLE_EQ(2.*sqrt(13)+3*sqrt(2),env.HCost({0,0},{7,9}));
  ASSERT_DOUBLE_EQ(sqrt(13)+6*sqrt(2),env.HCost({0,0},{8,9}));
  ASSERT_DOUBLE_EQ(9*sqrt(2),env.HCost({0,0},{9,9}));
  ASSERT_DOUBLE_EQ(7*sqrt(2)+sqrt(13),env.HCost({0,0},{9,10}));
  ASSERT_DOUBLE_EQ(5*sqrt(2)+2*sqrt(13),env.HCost({0,0},{9,11}));
  ASSERT_DOUBLE_EQ(6*sqrt(5)+sqrt(13),env.HCost({0,0},{8,15}));
  ASSERT_DOUBLE_EQ(5*sqrt(5)+sqrt(10),env.HCost({0,0},{13,6}));
  ASSERT_DOUBLE_EQ(5*sqrt(2)+17.*sqrt(13),env.HCost({0,0},{56,39}));
  ASSERT_DOUBLE_EQ(sqrt(5)+6.*sqrt(10),env.HCost({0,0},{20,7}));
  //ASSERT_DOUBLE_EQ(sqrt(5)+6.*sqrt(10),env.HCost({12,44},{51,41}));
  //ASSERT_DOUBLE_EQ(sqrt(5)+6.*sqrt(10),env.HCost({12,38},{51,41}));
}

TEST(Map2D, HashUnhash){
  Map map(100,100);
  MapEnvironment env(&map);
  Map2DConstrainedEnvironment e2(&env);
  for(int i(0); i<1000; ++i){
    xytLoc s({rand()%100,rand()%100},(rand()%100000)/1000.0);
    uint64_t hash=e2.GetStateHash(s);
    xytLoc x;
    e2.GetStateFromHash(hash,x);
    ASSERT_EQ(s,x);
  }
}

#endif
