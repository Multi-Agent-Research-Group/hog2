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
#include "TemporalAStar.h"
#include "RadialSafety2DObjectiveEnvironment.h"
#include "dtedreader.h"

TEST(util, dtedreader){
  float** array;
  array = new float*[10];
  for(int i = 0; i <10; i++)
    array[i] = new float[10];
  ASSERT_TRUE(readdted1("w110n38.dt1",array,10,10,0,0,5.0));
  for(int x(0); x<10; ++x){
    for(int y(0); y<10; ++y){
      std::cout << array[x][y] << " ";
    }
    std::cout << std::endl;
  }
}

TEST(Map2D, TwentyFourConnected_Successors){
  Map map(8,8);
  MapEnvironment env(&map);
  env.SetTwentyFourConnected();
  std::vector<xyLoc> successors;
  env.GetSuccessors({3,3},successors);
  ASSERT_EQ(16,successors.size());
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
  ASSERT_EQ(9,successors.size());
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
  ASSERT_EQ(32,successors.size());
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
  ASSERT_EQ(17,successors.size());
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
  map.SetGrid(4,4,4,Map3D::kOutOfBounds);
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

TEST(TemporalAStar, TestGetNextPath){
  TemporalAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  Map map(100,100);
  MapEnvironment env1(&map);
  env1.SetNineConnected();
  Map2DConstrainedEnvironment env(&env1);
  std::vector<xytLoc> path;
  xytLoc s(0,1);
  xytLoc g(99,99);
  //std::cout << s << " " << g << "\n";

  int n(0);
  astar.GetNextPath(&env,s,g,path,142);
  double t(0);
  //std::cout << path.size() << "----------------\n" << path[0] << "\n";
  std::cout << "path number: " << ++n << "\n";
  std::cout << path[0] << "\n";
  for(int j(1); j<path.size(); ++j){
    t += env.GCost(path[j-1],path[j]);
    std::cout << path[j] << "\n";
  }

  while(fleq(astar.GetNextPath(&env,s,g,path,142),t)){ // Fetch all paths of equal cost
    std::cout << "path number: " << ++n << "\n";
    for(int j(0); j<path.size(); ++j){
      std::cout << path[j] << "\n";
    }
  }
}

TEST(TemporalAStar, TestGetPaths){
  TemporalAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  Map map(100,100);
  MapEnvironment env1(&map);
  env1.SetNineConnected();
  Map2DConstrainedEnvironment env(&env1);
  std::vector<std::vector<xytLoc>> paths;
  xytLoc s(0,1);
  xytLoc g(99,99);
  //std::cout << s << " " << g << "\n";

  astar.GetPaths(&env,s,g,paths,0);
  for(auto const& path:paths){
    ASSERT_DOUBLE_EQ(paths[0].back().t,path.back().t);
  }
  double opt(paths[0].back().t);

  paths.clear();
  astar.GetPaths(&env,s,g,paths,0);
  ASSERT_TRUE(paths.empty());

  paths.clear();
  astar.GetPaths(&env,s,g,paths,1);
  for(auto const& path:paths){
    std::cout << opt << " " << path.back().t << "\n";
    ASSERT_TRUE(fgeq(opt+1.0,path.back().t));
  }

  paths.clear();
  astar.GetPaths(&env,s,g,paths,1);
  for(auto const& path:paths){
    ASSERT_TRUE(fgeq(opt+2.0,path.back().t));
  }
}
/*
// This is no longer reversible
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
*/

TEST(RadiusEnv, EdgeDistance){
  Vector2D A(0,0);
  Vector2D B(4,0);
  Vector2D C(4,3);
  ASSERT_DOUBLE_EQ(3.7359388247516234,Util::meanDistanceOfPointToLine(A,B,C));
}

TEST(RadiusEnv, GCostTest){
  Vector2D A(5,1);
  Vector2D B(4,1);
  Vector2D C(1,1);
  ASSERT_DOUBLE_EQ(3.5,Util::meanDistanceOfPointToLine(A,B,C));

  xytLoc a(5,1,0);
  xytLoc b(4,1,0);
  RadialSafety2DObjectiveEnvironment renv({{1,1,0},{3,3,0}},{2.,2.},8,8);
  ASSERT_DOUBLE_EQ(0.0,renv.GCost(a,b));

  renv=RadialSafety2DObjectiveEnvironment({{4,3,0}},{4.},8,8);
  a.x=7;a.y=0;
  b.x=6;b.y=1;
  std::cout << "GCOST " << renv.GCost(a,b) << "\n";
  ASSERT_TRUE(0.0<renv.GCost(a,b));

  a.x=6;a.y=7;
  b.x=6;b.y=6;
  std::cout << "GCOST1 " << renv.GCost(a,b) << "\n";
  ASSERT_TRUE(0.0<renv.GCost(a,b));
}

TEST(RadiusEnv, HCostTest){
  xytLoc a(7,1,0);
  xytLoc b(6,1,0);
  RadialSafety2DObjectiveEnvironment renv({{1,1,0}},{4.},8,8);
  std::cout << "GCOST " << renv.GCost(a,b) << "\n";
  std::cout << "HCOST " << renv.HCost(a,b) << "\n";
  ASSERT_DOUBLE_EQ(renv.HCost(a,b),renv.GCost(a,b));

  a.x=5;
  b.x=3;
  std::cout << "GCOST " << renv.GCost(a,b) << "\n";
  std::cout << "HCOST " << renv.HCost(a,b) << "\n";
  ASSERT_DOUBLE_EQ(renv.HCost(a,b),renv.GCost(a,b));
}

TEST(SecantLine, intersection){
  Vector2D c(4,3);
  double r(4.0);
  Vector2D a(0,6);
  Vector2D b(7,6);
  auto d(Util::secantLine(a,b,c,r));
  std::cout << "Secant="<<d.first<<","<<d.second<<"\n";
  ASSERT_DOUBLE_EQ(6.0,d.first.y);
  ASSERT_DOUBLE_EQ(6.0,d.second.y);
  ASSERT_TRUE(d.first.x>1 && d.first.x<2);
  ASSERT_TRUE(d.second.x>6 && d.second.x<7);
}

TEST(SecantLine, OneIntersection){
  Vector2D c(4,3);
  double r(4.0);
  Vector2D a(2,5);
  Vector2D b(7,6);
  auto d(Util::secantLine(a,b,c,r));
  std::cout << "Secant="<<d.first<<","<<d.second<<"\n";
  ASSERT_DOUBLE_EQ(2.0,d.first.x);
  ASSERT_DOUBLE_EQ(5.0,d.first.y);
  ASSERT_TRUE(d.second.y>5 && d.second.y<6);
}

TEST(SecantLine, NoIntersectionOutside){
  Vector2D c(4,3);
  double r(4.0);
  Vector2D a(0,7);
  Vector2D b(7,7);
  // This has a tangent line, but no secant line
  auto d(Util::secantLine(a,b,c,r));
  std::cout << "Secant="<<d.first<<","<<d.second<<"\n";
  ASSERT_DOUBLE_EQ(0.0,d.first.x);
  ASSERT_DOUBLE_EQ(0.0,d.first.y);
  ASSERT_DOUBLE_EQ(0.0,d.second.x);
  ASSERT_DOUBLE_EQ(0.0,d.second.y);
}

TEST(SecantLine, NoIntersectionInside){
  Vector2D c(4,3);
  double r(4.0);
  Vector2D a(2,5);
  Vector2D b(5,6);
  // This has a tangent line, but no secant line
  auto d(Util::secantLine(a,b,c,r));
  std::cout << "Secant="<<d.first<<","<<d.second<<"\n";
  ASSERT_DOUBLE_EQ(2.0,d.first.x);
  ASSERT_DOUBLE_EQ(5.0,d.first.y);
  ASSERT_DOUBLE_EQ(5.0,d.second.x);
  ASSERT_DOUBLE_EQ(6.0,d.second.y);
}

TEST(Grid3DConstrained, GetStateHash){
  xyztLoc a={103,41,0,6.24264f};
  xyztLoc b={103,43,0,6.24264f};
  Map3D map(99,99,99);
  Grid3DConstrainedEnvironment e(&map);
  e.SetIgnoreHeading(true);
  uint64_t ha(e.GetStateHash(a));
  uint64_t hb(e.GetStateHash(b));
  ASSERT_NE(ha,hb);
}

#endif
