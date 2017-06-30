#ifndef UnitTests_h_
#define UnitTests_h_

#include "Timer.h"
#include <gtest/gtest.h>
#include "Map2DEnvironment.h"
#include "Grid3DEnvironment.h"

TEST(Map2D, TwentyFourConnected){
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

TEST(Map2D, TwentyFourConnectedWObstacle){
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

TEST(Map2D, FortyEightConnected){
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

TEST(Map2D, FortyEightConnectedWObstacle){
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
#endif
