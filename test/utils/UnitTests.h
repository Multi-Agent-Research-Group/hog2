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
#ifndef env_UnitTests_h_
#define env_UnitTests_h_

#include <gtest/gtest.h>
#include "dtedreader.h"
#include "BucketHash.h"
//#include "TemporalAStar.h"
//#include "Map2DEnvironment.h"
//#include "Map2DConstrainedEnvironment.h"
#include "CollisionDetection.h"
#include "Hungarian.h"
#include "Map3d.h"

/*TEST(util, dtedreader){
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
}*/

TEST(util, BucketHash){
  BucketHash<std::vector<double>> hash(1.0);
  hash.insert(0,1,{1,2,1});
  hash.insert(0,2,{4,6,2});
  hash.insert(1,3,{3,5,3});
  std::set<std::vector<double>> result;
  hash.get(.5,1.5,result);
  ASSERT_EQ(3,result.size());
  hash.get(.5,0.99,result);
  ASSERT_EQ(2,result.size());
  hash.get(.5,1,result);
  ASSERT_EQ(3,result.size());
  hash.get(2.5,3,result);
  ASSERT_EQ(2,result.size());
}
TEST(util, 3DBucketHash){
  KDBucketHash<std::vector<double>,3> hash(1.0);
  std::cout << "insert: {{0,0,0},{1,2,1}}\n";
  hash.insert({{0,0,0},{1,2,1}});
  std::cout << "insert: {{1,3,0},{4,6,2}}\n";
  hash.insert({{1,3,0},{4,6,2}});
  std::cout << "insert: {{2,4,1},{3,5,3}}\n";
  hash.insert({{2,4,1},{3,5,3}});
  std::set<std::pair<std::vector<double>,std::vector<double>>> result;
  std::cout << "get: {{.5,.5,.5},{1.5,1.5,1.5}}\n";
  hash.get({{.5,.5,.5},{1.5,1.5,1.5}},result);
  ASSERT_EQ(1,result.size());
  ASSERT_EQ(0,result.begin()->first[0]);
  ASSERT_EQ(0,result.begin()->first[1]);
  ASSERT_EQ(0,result.begin()->first[2]);
  ASSERT_EQ(1,result.begin()->second[0]);
  ASSERT_EQ(2,result.begin()->second[1]);
  ASSERT_EQ(1,result.begin()->second[2]);
  result.clear();
  std::cout << "get: {{.5,.5,.5},{1.5,3.0,1.5}}\n";
  hash.get({{.5,.5,.5},{1.5,3.0,1.5}},result);
  ASSERT_EQ(2,result.size());
  result.clear();
  std::cout << "get: {{2.0,2.5,.5},{2.6,3.0,1.5}}\n";
  hash.get({{2.0,2.5,.5},{2.6,3.0,1.5}},result);
  ASSERT_EQ(1,result.size());
  std::cout << "remove: {{1,3,0},{4,6,2}}\n";
  hash.remove({{1,3,0},{4,6,2}});
  std::cout << "get: {{2.0,2.5,.5},{2.6,3.0,1.5}}\n";
  result.clear();
  hash.get({{2.0,2.5,.5},{2.6,3.0,1.5}},result);
  ASSERT_EQ(0,result.size());

  // Insert it twice and make sure that I have to remove it twice
  std::cout << "insert once: {{1,3,0},{4,6,2}}\n";
  hash.insert({{1,3,0},{4,6,2}});
  std::cout << "get: {{2.0,2.5,.5},{2.6,3.0,1.5}}\n";
  result.clear();
  hash.get({{2.0,2.5,.5},{2.6,3.0,1.5}},result);
  ASSERT_EQ(1,result.size());
  std::cout << "insert twice: {{1,3,0},{4,6,2}}\n";
  hash.insert({{1,3,0},{4,6,2}});
  std::cout << "get: {{2.0,2.5,.5},{2.6,3.0,1.5}}\n";
  result.clear();
  hash.get({{2.0,2.5,.5},{2.6,3.0,1.5}},result);
  ASSERT_EQ(1,result.size());
  std::cout << "remove once: {{1,3,0},{4,6,2}}\n";
  hash.remove({{1,3,0},{4,6,2}});
  std::cout << "get: {{2.0,2.5,.5},{2.6,3.0,1.5}}\n";
  result.clear();
  hash.get({{2.0,2.5,.5},{2.6,3.0,1.5}},result);
  ASSERT_EQ(1,result.size());
  std::cout << "remove twice: {{1,3,0},{4,6,2}}\n";
  hash.remove({{1,3,0},{4,6,2}});
  std::cout << "get: {{2.0,2.5,.5},{2.6,3.0,1.5}}\n";
  result.clear();
  hash.get({{2.0,2.5,.5},{2.6,3.0,1.5}},result);
  ASSERT_EQ(0,result.size());
}


/*TEST(generic, GetFurthestPoint){
  TemporalAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  astar.SetHeuristic(new ZeroHeuristic<xytLoc>);
  Map map("../../maps/dao/den520d.map"); // Must be run from builds/gmake dir.
  MapEnvironment env1(&map);
  env1.SetNineConnected();
  Map2DConstrainedEnvironment env(&env1);
  env.SetIgnoreTime(true); // Otherwise the search would never terminate
  env.SetIgnoreHeading(true);  // Don't care about alternate paths to this state
  std::vector<xytLoc> path;
  xytLoc s(0,1);
  xytLoc g(99,99);
  //std::cout << s << " " << g << "\n";

  astar.SetStopAfterGoal(false); // Search the entire space
  astar.GetPath(&env,s,g,path,0);
  std::cout << "furthest point " << path.back() << "\n";
  std::cout << "depth " << env.GetPathLength(path) << "\n";
  
}

TEST(algorithms, hungarian){
  std::vector<xyztLoc> goals = {{1,1,1,0.0f},{60,3,7,0.0f},{12,36,9,0.0f},{18,22,32,0.0f}};
  std::vector<xyztLoc> starts = {{3,1,9,0.0f},{41,16,2,0.0f}};
  std::vector<std::vector<int>> costs(starts.size(),std::vector<int>(goals.size()));
  // Represent costs using distances
  int x(0);
  for(auto const& s:starts){
    int y(0);
    for(auto const& g:goals){
      costs[x][y++] = Util::distance(s,g)*xyztLoc::TIME_RESOLUTION;
    }
    ++x;
  }

  Hungarian::hungarian_print_matrix(costs);
  Hungarian h(costs);
  std::vector<std::vector<unsigned>> assignments;
  h.solve(assignments,2);
  h.print_status();
  
  ASSERT_EQ(assignments[0][0],0U);
  ASSERT_EQ(assignments[0][1],3U);
  ASSERT_EQ(assignments[1][0],1U);
  ASSERT_EQ(assignments[1][1],2U);
}

TEST(algorithms, hungarian2){
  std::vector<xyztLoc> goals = {{1,1,1,0.0f},{60,3,7,0.0f},{12,36,9,0.0f}};
  std::vector<xyztLoc> starts = {{3,1,9,0.0f},{41,16,2,0.0f}};
  std::vector<std::vector<int>> costs(starts.size(),std::vector<int>(goals.size()));
  // Represent costs using distances
  int x(0);
  for(auto const& s:starts){
    int y(0);
    for(auto const& g:goals){
      costs[x][y++] = Util::distance(s,g)*xyztLoc::TIME_RESOLUTION;
    }
    ++x;
  }

  Hungarian::hungarian_print_matrix(costs);
  Hungarian h(costs);
  std::vector<std::vector<unsigned>> assignments;
  h.solve(assignments,2);
  h.print_status();
  
  ASSERT_EQ(assignments[0][0],0U);
  ASSERT_EQ(assignments[1][0],1U);
  ASSERT_EQ(assignments[1][1],2U);
}

TEST(algorithms, hungarian3){
  std::vector<xyztLoc> starts = {{1,1,1,0.0f},{60,3,7,0.0f},{12,36,9,0.0f},{18,22,32,0.0f}};
  std::vector<xyztLoc> goals = {{3,1,9,0.0f},{41,16,2,0.0f}};
  std::vector<std::vector<int>> costs(starts.size(),std::vector<int>(goals.size()));
  // Represent costs using distances
  int x(0);
  for(auto const& s:starts){
    int y(0);
    for(auto const& g:goals){
      costs[x][y++] = Util::distance(s,g)*xyztLoc::TIME_RESOLUTION;
    }
    ++x;
  }

  Hungarian::hungarian_print_matrix(costs);
  Hungarian h(costs);
  std::vector<std::vector<unsigned>> assignments;
  h.solve(assignments,2);
  h.print_status();
  
  ASSERT_EQ(assignments[0][0],0U);
  ASSERT_EQ(assignments[1][0],1U);
}*/

/*TEST(CollisionDetection, loadTable){
  std::vector<unsigned> array1;
  std::vector<unsigned> indices1;
  std::vector<float> ivls1;
  loadCollisionTable<Vector3D>(array1, indices1, ivls1, 9,10,0.35,0.35,true); // Force compute
  std::vector<unsigned> array2;
  std::vector<unsigned> indices2;
  std::vector<float> ivls2;
  loadCollisionTable<Vector3D>(array2, indices2, ivls2, 9,10,0.35,0.35); // Load from file
  ASSERT_EQ(array1,array2);
  ASSERT_EQ(indices1,indices2);
  ASSERT_EQ(ivls1,ivls2);
}*/

TEST(RankingTest, MoveNumber){
  EXPECT_EQ(moveNum(Vector3D(8, 7, 0),Vector3D(9, 6, 0),0,17),12);
  EXPECT_EQ(moveNum(Vector3D(9, 6, 0),Vector3D(7, 5, 0),0,17),1);
}

TEST(RankingTest, mirrorUnmirror){
  {
    Vector3D a1;
    Vector3D b1(2,2,0);
    for(unsigned ax(0); ax<5; ++ax){
      a1.x=ax;
      for(unsigned ay(0); ay<5; ++ay){
        a1.y=ay;
        for(unsigned i(0); i<9; ++i){
          Vector3D a2;
          fetch(a1,i,a2,9);
          for(unsigned j(0); j<9; ++j){
            Vector3D b2;
            fetch(b1,j,b2,9);
            bool swap=false, ortho=false, y=false;
            locationIndex(a1,b1,swap,ortho,y,9);
            auto moveA(moveNum(a1,a2,0,9));
            ASSERT_EQ(i,moveA);
            auto mirroredA(getMirroredMove(i,swap,ortho,y,9));
            auto unmirroredA(invertMirroredMove(mirroredA,swap,ortho,y,9));
            if(unmirroredA!=i){
              (invertMirroredMove(mirroredA,swap,ortho,y,9));
            }
            EXPECT_EQ(unmirroredA,i);
          }
        }
      }
    }
  }
  {
    Vector3D a1(4, 4, 0);Vector3D a2(3, 4, 0);Vector3D b1(3, 3, 0);Vector3D b2(3, 4, 0);
    bool swap=false, ortho=false, y=false;
    locationIndex(a1,b1,swap,ortho,y,17);
    auto moveA(moveNum(a1,a2,0,17));
    ASSERT_EQ(2,moveA);
    auto moveB(moveNum(b1,b2,0,17));
    ASSERT_EQ(6,moveB);
    auto mirroredA(getMirroredMove(moveA,swap,ortho,y,17));
    ASSERT_EQ(6,mirroredA);
    auto mirroredB(getMirroredMove(moveB,swap,ortho,y,17));
    ASSERT_EQ(2,mirroredB);

  }
}

void assertInterval(Vector3D const& A1, Vector3D const& A2, float startTimeA, Vector3D const& B1, Vector3D const& B2, float startTimeB, float rA=0.25, float rB=0.25, bool print=false){
  float la((A2-A1).len());
  //float lb((B2-B1).len());
  if(la==0){la=0.9;}
  float endTimeA=startTimeA+la;
  float endTimeB=startTimeB+(B2-B1).len();
  if(endTimeB==startTimeB){endTimeB += 1.1;} // Wait action
  auto VA=(A2-A1);
  VA.Normalize();
  auto VB=(B2-B1);
  VB.Normalize();
  auto intvl(getForbiddenInterval(A1,A2,startTimeA,endTimeA,rA,B1,B2,startTimeB,endTimeB,rB));
  float res(std::min(0.01f,(intvl.second-intvl.first)/10));
  float first=intvl.first-10;
  while(!collisionImminent(A1,VA,rA,first,first+la,B1,VB,rB,startTimeB,endTimeB)){first+=res;}
  float second(first);
  first-=res/2.0;
  while(collisionImminent(A1,VA,rA,second,second+la,B1,VB,rB,startTimeB,endTimeB)){
    //std::cout << second << "\n";
    second+=res;
  }
  second-=res/2.0;
  /*if(collisionImminent(A1,VA,rA,intvl.first-.001,intvl.first+la-.001,B1,VB,rB,startTimeB,endTimeB)){
    collisionImminent(A1,VA,rA,intvl.first-.001,intvl.first+la-.001,B1,VB,rB,startTimeB,endTimeB);
  }
  if(!collisionImminent(A1,VA,rA,intvl.first+.001,intvl.first+la+.001,B1,VB,rB,startTimeB,endTimeB)){
    collisionImminent(B1,VB,rB,-intvl.second+.001,-intvl.second+lb+.001,A1,VA,rA,startTimeA,endTimeA);
    collisionImminent(A1,VA,rA,intvl.first+.001,intvl.first+la+.001,B1,VB,rB,startTimeB,endTimeB);
  }
  if(!collisionImminent(A1,VA,rA,intvl.second-.001,intvl.second+la-.001,B1,VB,rB,startTimeB,endTimeB)){
    collisionImminent(A1,VA,rA,intvl.first-.001,intvl.first+la-.001,B1,VB,rB,startTimeB,endTimeB);
  }
  if(collisionImminent(A1,VA,rA,intvl.second+.001,intvl.second+la+.001,B1,VB,rB,startTimeB,endTimeB)){
    collisionImminent(A1,VA,rA,intvl.first+.001,intvl.first+la+.001,B1,VB,rB,startTimeB,endTimeB);
  }*/
  intvl=(getForbiddenInterval(A1,A2,startTimeA,endTimeA,rA,B1,B2,startTimeB,endTimeB,rB));
  if(fabs(first-intvl.first)>0.01 || fabs(second-intvl.second)>.01){
  std::cout << "\n" << A1 << "-->" << A2 << "," << B1 << "-->" << B2 << "\n";
  std::cout << "Expected interval is near {" << first << "," << second << "}\n";
  std::cout << "Projected interval is {" << intvl.first << "," << intvl.second << "}\n";
  }
  ASSERT_NEAR(first,intvl.first,.01);
  ASSERT_NEAR(second,intvl.second,.01);
}

void assertNoInterval(Vector3D const& A1, Vector3D const& A2, float startTimeA, Vector3D const& B1, Vector3D const& B2, float startTimeB, float rA=0.25, float rB=0.25, bool print=false){
  float la((A2-A1).len());
  float endTimeA=startTimeA+la;
  float endTimeB=startTimeB+(B2-B1).len();
  auto VA=(A2-A1);
  VA.Normalize();
  auto VB=(B2-B1);
  VB.Normalize();
  auto intvl(getForbiddenInterval(A1,A2,startTimeA,endTimeA,rA,B1,B2,startTimeB,endTimeB,rB));
  std::cout << "Expected no interval but got: {"<<intvl.first<<","<<intvl.second<<"}\n";
  ASSERT_DOUBLE_EQ(std::numeric_limits<float>::infinity(),intvl.first);
  ASSERT_DOUBLE_EQ(-std::numeric_limits<float>::infinity(),intvl.second);
}

void assertCorrectExtraction(Vector3D const& a1, Vector3D const& a2, Vector3D const& b1, Vector3D const& b2,
  std::vector<unsigned> const& array,
  std::vector<unsigned> const& indices,
  std::vector<float> const& ivls,
  unsigned bfactor=9){
  //std::cout << a1 << "-->" << a2 << ", " << b1 << "-->" << b2 << "\n";
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  double startTimeA(0);
  double startTimeB(0.1);
  double endTimeA(startTimeA+std::max((a1-a2).len(),1.0));
  double endTimeB(startTimeB+std::max((b1-b2).len(),1.0));
  getVertexAnnotatedBiclique(a1,a2,b1,b2,startTimeA,endTimeA,startTimeB,endTimeB,array,indices,ivls,left,right,livls,rivls,bfactor,.25,.25);

  std::vector<unsigned> left1;
  std::vector<unsigned> right1;
  std::vector<std::pair<float,float>> livls1;
  std::vector<std::pair<float,float>> rivls1;
  getVertexAnnotatedBiclique(a1,a2,b1,b2,startTimeA,endTimeA,startTimeB,endTimeB,left1,right1,livls1,rivls1,bfactor);
  if(left.size() != left1.size() || right.size() != right1.size()){
    std::cout << "Vector3D a1"<<a1 <<";Vector3D a2"<<a2 <<";Vector3D b1"<<b1<<";Vector3D b2"<<b2<<";\n";
    std::cout << "discrepancy\n";
    std::cout << left << "\n";
    std::cout << left1 << "\n";
    std::cout << right << "\n";
    std::cout << right1 << "\n";
    std::cout << "discrepancy\n";
  }
  ASSERT_EQ(left.size()+right.size(),left1.size()+right1.size());
  //ASSERT_EQ(right.size(),right1.size());
  /*std::cout << "LEFT:\n";
  for(unsigned i(0); i<left.size(); ++i){
    std::cout << left[i] << ": " << livls[i].first << "," <<livls[i].second << " vs " << left1[i] << ": " << livls1[i].first << "," << livls1[i].second << "\n";
  }*/
  //if(left.size()==left1.size())
  //for(unsigned i(0); i<left.size(); ++i){
    //auto ptr(std::find_if(livls.begin(),livls.end(),[&](std::pair<float,float> const& b){return fabs(livls1[i].first-b.first)<0.01 && fabs(livls1[i].second-b.second)<0.01;}));
    //EXPECT_TRUE(ptr!=livls.end());
    //if(ptr==livls.end()){
      //std::cout << "Vector3D a1"<<a1 <<";Vector3D a2"<<a2 <<";Vector3D b1"<<b1<<";Vector3D b2"<<b2<<";\n";
      //std::cout << livls1[i] << "not in " << livls << "\n";
    //}
    //livls.erase(ptr);
    //EXPECT_NEAR(livls[i].first,livls1[i].first,.01);
    //EXPECT_NEAR(livls[i].second,livls1[i].second,.01);
  //}
  /*std::cout << "RIGHT:\n";
  for(unsigned i(0); i<right.size(); ++i){
    std::cout << right[i] << ": " << rivls[i].first << "," <<rivls[i].second << " vs " << right1[i] << ": " << rivls1[i].first << "," << rivls1[i].second << "\n";
  }*/
  //if(right.size()==right1.size())
  //for(unsigned i(0); i<right.size(); ++i){
    //auto ptr(std::find_if(rivls.begin(),rivls.end(),[&](std::pair<float,float> const& b){return fabs(rivls1[i].first-b.first)<0.01 && fabs(rivls1[i].second-b.second)<0.01;}));
    //EXPECT_TRUE(ptr!=rivls.end());
    //if(ptr==rivls.end()){
      //std::cout << "Vector3D a1"<<a1 <<";Vector3D a2"<<a2 <<";Vector3D b1"<<b1<<";Vector3D b2"<<b2<<";\n";
      //std::cout << rivls1[i] << "not in " << rivls << "\n";
    //}
    //rivls.erase(ptr);
    //EXPECT_NEAR(rivls[i].first,rivls1[i].first,.01);
    //EXPECT_NEAR(rivls[i].second,rivls1[i].second,.01);
  //}
}

TEST(ForbiddenInterval, Nominal){
  Vector3D A1(0,0,0);
  Vector3D A2(1,2,0);
  Vector3D B1(1,0,0);
  Vector3D B2(0,1,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenIntervalParallel, ExactOverlap){
  // Exactly overlapping
  Vector3D A1(0,0,0);
  Vector3D A2(1,2,0);
  Vector3D B1(0,0,0);
  Vector3D B2(1,2,0);
  assertInterval(A1,A2,10.2,B1,B2,10.0);
}

TEST(ForbiddenIntervalParallel, DisjointTime){
  // Parallel but not overlapping because of time
  Vector3D A1(0,0,0);
  Vector3D A2(1,2,0);
  Vector3D B1(0,0,0);
  Vector3D B2(1,2,0);
  assertInterval(A1,A2,10.6,B1,B2,10.0);
}

TEST(ForbiddenIntervalParallel, DisjointTime2){
  // Parallel but not overlapping because of time
  Vector3D B1(0,0,0);
  Vector3D B2(1,2,0);
  Vector3D A1(0,0,0);
  Vector3D A2(1,2,0);
  assertInterval(A1,A2,10.6,B1,B2,10.0);
}

TEST(ForbiddenIntervalParallel, NoOverlap){
  // Parallel but not overlapping because of distance
  Vector3D B1(0,0,0);
  Vector3D B2(1,2,0);
  Vector3D A1(1,0,0);
  Vector3D A2(2,2,0);
  assertNoInterval(A1,A2,10.0,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ExactOverlap){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1,1,0);
  Vector3D B2(0,0,0);
  assertInterval(A1,A2,10.0,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ExactOverlapOffsetTimes){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1,1,0);
  Vector3D B2(0,0,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ExactOverlapOffsetTimes2){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1,1,0);
  Vector3D B2(0,0,0);
  assertInterval(A1,A2,10.0,B1,B2,10.3);
}

TEST(ForbiddenIntervalOpposing, OffsetOverlapOffsetTimes){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1.2,1,0);
  Vector3D B2(0.2,0,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, OffsetOverlapOffsetTimes2){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1.2,1,0);
  Vector3D B2(0.2,0,0);
  assertInterval(A1,A2,10,B1,B2,10.3);
}

TEST(ForbiddenIntervalOpposing, OffsetOverlapOffsetTimes3){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(.8,1,0);
  Vector3D B2(-.2,0,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ShiftedOffset){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1,.8,0);
  Vector3D B2(0,-.2,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ShiftedOffset2){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1,1.2,0);
  Vector3D B2(0,0.2,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ShiftedOffset3){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(.9,.7,0);
  Vector3D B2(-.1,-.3,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ShiftedOffset4){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1.1,1.2,0);
  Vector3D B2(0.1,0.2,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ShiftedOffset5){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1.4,1.5,0);
  Vector3D B2(0.4,0.5,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, ShiftedOffset6){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(.8,.8,0);
  Vector3D B2(.3,.3,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, FullOverlapOffsetDiagonal){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(.6,.6,0);
  Vector3D B2(.4,.4,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, FullOffsetOverlap){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1.4,1.4,0);
  Vector3D B2(0.3,0.3,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, FrontOffsetOverlap){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(.6,.5,0);
  Vector3D B2(-.1,-.2,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, FullOffsetOverlap2){
  Vector3D A1(0,0,0);
  Vector3D A2(1,0,0);
  Vector3D B1(.6,.3,0);
  Vector3D B2(.4,.3,0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
}

TEST(ForbiddenIntervalOpposing, RearOffsetOverlap){
  // B starts before A
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1.5,1,0);
  Vector3D B2(0.5,0,0);
  assertInterval(A1,A2,10.0,B1,B2,10.0);
}

TEST(ForbiddenIntervalWaiting, ExactPassthrough){
  Vector3D A1(1,1,0);
  Vector3D A2(1,1,0);
  Vector3D B1(2,2,0);
  Vector3D B2(0,0,0);
  assertInterval(A1,A2,10.0,B1,B2,10.0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
  assertInterval(A1,A2,10.0,B1,B2,10.3);
}

TEST(ForbiddenIntervalWaiting, OffsetPassthrough){
  Vector3D A1(1,1,0);
  Vector3D A2(1,1,0);
  Vector3D B1(2,2.1,0);
  Vector3D B2(0,0.1,0);
  assertInterval(A1,A2,10.0,B1,B2,10.0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
  assertInterval(A1,A2,10.0,B1,B2,10.3);
}

TEST(ForbiddenIntervalWaiting, BothWaiting){
  Vector3D A1(1,1,0);
  Vector3D A2(1,1,0);
  Vector3D B1(1,1.1,0);
  Vector3D B2(1,1.1,0);
  assertInterval(A1,A2,10.0,B1,B2,10.0);
  assertInterval(A1,A2,10.3,B1,B2,10.0);
  assertInterval(A1,A2,10.0,B1,B2,10.3);
}

TEST(ForbiddenInterval, ShortAndLong){
  Vector3D A1(2,1.5,0);
  Vector3D A2(2.6,1.3,0);
  Vector3D B1(3,4.1,0);
  Vector3D B2(2.3,0.6,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, TrickyOne){
  Vector3D A1(2.1,4.9,0);
  Vector3D A2(2.7,1.2,0);
  Vector3D B1(0.9,4,0);
  Vector3D B2(2.6,1.3,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, TinyOverlapAtStartNoCrossing){
  Vector3D A1(4.0,4.4,0);
  Vector3D A2(3.9,0.5,0);
  Vector3D B1(3.6,0.4,0);
  Vector3D B2(3.2,1.9,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, TinyOverlapInMiddleNoCrossing){
  Vector3D A1(1.4,3.4,0);
  Vector3D A2(1.4,2.4,0);
  Vector3D B1(0.6,3.7,0);
  Vector3D B2(4.1,4.3,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, LargeOverlapInMiddleNoCrossing){
  Vector3D A1(4.9,0.4,0);
  Vector3D A2(1.0,3.2,0);
  Vector3D B1(1.9,2.7,0);
  Vector3D B2(1.2,3.9,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, LargeOverlapInMiddleWithCrossing){
  Vector3D A1(3.9,1.5,0);
  Vector3D A2(1.9,4.4,0);
  Vector3D B1(3.0,0.1,0);
  Vector3D B2(4.1,3.6,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, CrossingNearStart){
  Vector3D A1(1.9,0.3,0);
  Vector3D A2(1.8,2.0,0);
  Vector3D B1(1.5,0.8,0);
  Vector3D B2(4.9,4.0,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, CrossingNearEnd){
  Vector3D A1(4.9,4.0,0);
  Vector3D A2(1.0,3.2,0);
  Vector3D B1(1.8,2.6,0);
  Vector3D B2(1.2,3.9,0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, EndingInTheMiddle){
  Vector3D A1(2.5, 0.5, 0);
  Vector3D A2(2.7, 3.4, 0);
  Vector3D B1(2, 1.2, 0);
  Vector3D B2(3.1, 4.6, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, ExactlyTouching){
  Vector3D A1(2.6, 1.8, 0);
  Vector3D A2(2.3, 0.9, 0);
  Vector3D B1(3.7, 0.3, 0);
  Vector3D B2(2.3, 0.4, 0);
  assertNoInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, EndNearBStart){
  Vector3D A1(1, 2.8, 0);
  Vector3D A2(1.8, 3.5, 0);
  Vector3D B1(1.2, 0.8, 0);
  Vector3D B2(0.6, 2.8, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, SpecialCase){
  Vector3D B1(2.1, 0.5, 0);
  Vector3D B2(1.5, 4.5, 0);
  Vector3D A1(2.3, 0.3, 0);
  Vector3D A2(2.3, 0.3, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, test){
  Vector3D A1(4.4, 4.1, 0);
  Vector3D A2(3.7, 4.1, 0);
  Vector3D B1(3.6, 4.5, 0);
  Vector3D B2(3.4, 4.5, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, testSameStart){
  Vector3D A1(5, 5, 0);
  Vector3D A2(4, 4, 0);
  Vector3D B1(5, 5, 0);
  Vector3D B2(6, 6, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, waitAtEnd){
  Vector3D A1(4, 5, 0);
  Vector3D A2(4, 4, 0);
  Vector3D B1(4, 4, 0);
  Vector3D B2(4, 4, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, waitAtEnd2){
  Vector3D A1(5, 5, 0);
  Vector3D A2(4, 4, 0);
  Vector3D B1(4, 4, 0);
  Vector3D B2(4, 4, 0);
  assertInterval(A1,A2,0.0,B1,B2,0.0);
}

TEST(ForbiddenInterval, RandomInstances){
  unsigned ntests(000000);
  for(int i(0); i<ntests; ++i){
    if(i%(ntests/100)==0){std::cout << i << "\n";}
    float rA(.25);
    float rB(.25);
    Vector3D A1(float(rand()%50)/10.0,float(rand()%50)/10.0,0);
    Vector3D A2(float(rand()%50)/10.0,float(rand()%50)/10.0,0);
    Vector3D B1(float(rand()%50)/10.0,float(rand()%50)/10.0,0);
    Vector3D B2(float(rand()%50)/10.0,float(rand()%50)/10.0,0);
    if(Util::fatLinesIntersect(A1,A2,rA,B1,B2,rB)){
      assertInterval(A1,A2,0,B1,B2,0,rA,rB,true);
    }else{
      assertNoInterval(A1,A2,0,B1,B2,0,rA,rB);
    }
  }
}

TEST(PositionalUtils, distanceToLine){
  Vector3D A1(0,0,0);
  Vector3D A2(1,1,0);
  Vector3D B(1,0,0);
  float d(Util::distanceOfPointToLine(A1,A2,B));
  ASSERT_DOUBLE_EQ(float(sqrt(2.0f)/2.0f),d);
}
/*
TEST(CompactBiclique, testUnrank){
  Vector2D a,b;
  fromLocationIndex(10,a,b,24);
  EXPECT_EQ(b.x,4);
  EXPECT_EQ(b.y,4);
  EXPECT_EQ(a.x,3);
  EXPECT_EQ(a.y,2);

  fromLocationIndex(5,a,b,24);
  EXPECT_EQ(b.x,4);
  EXPECT_EQ(b.y,4);
  EXPECT_EQ(a.x,1);
  EXPECT_EQ(a.y,1);

  fromLocationIndex(7,a,b,24);
  EXPECT_EQ(b.x,4);
  EXPECT_EQ(b.y,4);
  EXPECT_EQ(a.x,3);
  EXPECT_EQ(a.y,1);

  fromLocationIndex(0,a,b,8);
  EXPECT_EQ(b.x,2.0);
  EXPECT_EQ(b.y,2.0);
  EXPECT_EQ(a.x,0.0);
  EXPECT_EQ(a.y,0.0);

  fromLocationIndex(1,a,b,8);
  EXPECT_EQ(b.x,2.0);
  EXPECT_EQ(b.y,2.0);
  EXPECT_EQ(a.x,1.0);
  EXPECT_EQ(a.y,0.0);

  fromLocationIndex(2,a,b,8);
  EXPECT_EQ(b.x,2.0);
  EXPECT_EQ(b.y,2.0);
  EXPECT_EQ(a.x,2.0);
  EXPECT_EQ(a.y,0.0);

  fromLocationIndex(3,a,b,8);
  EXPECT_EQ(b.x,2.0);
  EXPECT_EQ(b.y,2.0);
  EXPECT_EQ(a.x,1.0);
  EXPECT_EQ(a.y,1.0);

  fromLocationIndex(4,a,b,8);
  EXPECT_EQ(b.x,2.0);
  EXPECT_EQ(b.y,2.0);
  EXPECT_EQ(a.x,2.0);
  EXPECT_EQ(a.y,1.0);

  fromLocationIndex(5,a,b,8);
  EXPECT_EQ(b.x,2.0);
  EXPECT_EQ(b.y,2.0);
  EXPECT_EQ(a.x,2.0);
  EXPECT_EQ(a.y,2.0);

}

TEST(CompactBiclique, testRank){
  bool swap,ortho,y;
  {
    auto ix(locationIndex(Vector2D(4,4),Vector2D(3,3),swap,ortho,y,17));
    EXPECT_EQ(ix,12);
  }
  {
    auto ix(locationIndex(Vector3D(11, 6, 0),Vector3D(14, 6, 0),swap,ortho,y,17));
    EXPECT_EQ(ix,8);
  }
  {
    auto ix(locationIndex(Vector2D(4,4),Vector2D(4,4),swap,ortho,y,17));
    EXPECT_EQ(ix,14);
    ix=locationIndex(Vector2D(4,3),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,13);
    ix=locationIndex(Vector2D(3,3),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,12);
    ix=locationIndex(Vector2D(4,2),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,11);
    ix=locationIndex(Vector2D(3,2),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,10);
    ix=locationIndex(Vector2D(2,2),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,9);
    ix=locationIndex(Vector2D(1,4),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,8);
    ix=locationIndex(Vector2D(1,3),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,7);
    ix=locationIndex(Vector2D(2,1),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,6);
    ix=locationIndex(Vector2D(1,1),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,5);
    ix=locationIndex(Vector2D(0,4),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,4);
    ix=locationIndex(Vector2D(3,0),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,3);
    ix=locationIndex(Vector2D(0,2),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,2);
    ix=locationIndex(Vector2D(1,0),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,1);
    ix=locationIndex(Vector2D(0,0),Vector2D(4,4),swap,ortho,y,17);
    EXPECT_EQ(ix,0);
  }
  {
    auto ix(locationIndex(Vector2D(43,73),Vector2D(42,75),swap,ortho,y,9));
    EXPECT_EQ(ix,1);
  }
  {
    auto ix(locationIndex(Vector2D(3,3),Vector2D(3,3),swap,ortho,y,9));
    EXPECT_EQ(ix,5);
  }
  {
    auto ix(locationIndex(Vector2D(1,3),Vector2D(3,1),swap,ortho,y,9));
    EXPECT_EQ(ix,0);
  }
  {
    auto ix(locationIndex(Vector2D(1,3),Vector2D(2,2),swap,ortho,y,9));
    EXPECT_EQ(ix,3);
  }
  {
    auto ix(locationIndex(Vector2D(4,3),Vector2D(2,2),swap,ortho,y,9));
    EXPECT_EQ(ix,1);
  }
  {
    auto ix(locationIndex(Vector2D(3,2),Vector2D(2,2),swap,ortho,y,9));
    EXPECT_EQ(ix,4);
  }
  {
    auto ix(locationIndex(Vector2D(0,0),Vector2D(2,2),swap,ortho,y,9));
    EXPECT_EQ(ix,0);
  }
  {
    auto ix(locationIndex(Vector2D(1,2),Vector2D(0,0),swap,ortho,y,9));
    EXPECT_EQ(ix,1);
  }
  {
    auto ix(locationIndex(Vector2D(2,2),Vector2D(0,0),swap,ortho,y,9));
    EXPECT_EQ(ix,0);
  }
  auto ix(locationIndex(Vector2D(0,0),Vector2D(1,1),swap,ortho,y,9));
  EXPECT_EQ(ix,3);
  ix=locationIndex(Vector2D(1,1),Vector2D(0,0),swap,ortho,y,9);
  EXPECT_EQ(ix,3);
  ix=locationIndex(Vector2D(1,3),Vector2D(0,0),swap,ortho,y,9);
  EXPECT_EQ(ix,-1);
  ix=locationIndex(Vector2D(3,1),Vector2D(0,0),swap,ortho,y,9);
  EXPECT_EQ(ix,-1);
  ix=locationIndex(Vector2D(0,0),Vector2D(3,1),swap,ortho,y,9);
  EXPECT_EQ(ix,-1);
}

TEST(EncodeFloats, encodeTestsLonger){
  std::vector<std::vector<std::pair<float,float>>> i1={{{8.9,6.7},{4.5,2.3},{0.1,9.9},{8.9,6.7},{4.5,2.3},{0.1,9.9},{8.9,6.7},{4.5,2.3},{0.1,9.9}},
    {{8.9,6.7},{4.5,2.3},{0.1,9.9},{8.9,6.7},{4.5,2.3},{0.1,9.9},{8.9,6.7},{4.5,2.3},{0.1,9.9}}};
  std::vector<float> output;
  encodeIvls(i1,output);

  unsigned ix(0);
  for(auto const&j:i1){
    for(auto const&i:j){
      EXPECT_EQ(output[ix++],float(i.first));
      EXPECT_EQ(output[ix++],float(i.second));
    }
  }

  std::vector<std::vector<std::pair<float,float>>> x1;
  decodeIvls(output,2,x1);
  EXPECT_EQ(i1,x1);
}

TEST(Biclique, annotatedBicliqueSingleton){
  Vector3D a1(0,1,0);
  Vector3D a2(1,1,0);
  Vector3D b1(2,1,0);
  Vector3D b2(1,1,0);
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  double startTimeA(0);
  double startTimeB(0);
  double endTimeA(startTimeA+std::max((a1-a2).len(),1.0));
  double endTimeB(startTimeB+std::max((b1-b2).len(),1.0));
  getVertexAnnotatedBiclique(a1,a2,b1,b2,startTimeA,endTimeA,startTimeB,endTimeB,left,right,livls,rivls,9);
  ASSERT_EQ(left.size(),1);
  ASSERT_EQ(right.size(),1);
  ASSERT_EQ(left.size(),livls.size());
  ASSERT_EQ(right.size(),rivls.size());
}

TEST(Biclique, SameEnd){
  Vector3D a1(174,117,0);
  Vector3D a2(175,116,0);
  Vector3D b1(174,117,0);
  Vector3D b2(174,117,0);
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  double startTimeA(84.6);
  double startTimeB(83.8);
  double endTimeA(startTimeA+std::max((a1-a2).len(),1.0));
  double endTimeB(startTimeB+std::max((b1-b2).len(),1.0));
  getVertexAnnotatedBiclique(a1,a2,b1,b2,startTimeA,endTimeA,startTimeB,endTimeB,left,right,livls,rivls,9);
  std::cout << "############" << livls[0].first << "," << livls[0].second << "\n";
  std::cout << "############" << rivls[0].first << "," << rivls[0].second << "\n";
  ASSERT_EQ(left.size(),9);
  ASSERT_EQ(right.size(),1);
  ASSERT_EQ(left.size(),livls.size());
  ASSERT_EQ(right.size(),rivls.size());
}

TEST(Biclique, annotatedBiclique1xn){
  Vector3D a1(0,1,0);
  Vector3D a2(1,1,0);
  Vector3D b1(1,1,0);
  Vector3D b2(0,1,0);
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  double startTimeA(0);
  double startTimeB(0);
  double endTimeA(startTimeA+std::max((a1-a2).len(),1.0));
  double endTimeB(startTimeB+std::max((b1-b2).len(),1.0));
  getVertexAnnotatedBiclique(a1,a2,b1,b2,startTimeA,endTimeA,startTimeB,endTimeB,left,right,livls,rivls,9);
  ASSERT_EQ(left.size(),4);
  ASSERT_EQ(right.size(),1);
  ASSERT_EQ(left.size(),livls.size());
  ASSERT_EQ(right.size(),rivls.size());
}

TEST(Biclique, annotatedBiclique2x2){
  Vector3D a1(0,1,0);
  Vector3D a2(1,2,0);
  Vector3D b1(1,1,0);
  Vector3D b2(0,2,0);
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  double startTimeA(0);
  double startTimeB(0);
  double endTimeA(startTimeA+std::max((a1-a2).len(),1.0));
  double endTimeB(startTimeB+std::max((b1-b2).len(),1.0));
  getVertexAnnotatedBiclique(a1,a2,b1,b2,startTimeA,endTimeA,startTimeB,endTimeB,left,right,livls,rivls,9);
  ASSERT_EQ(left.size(),2);
  ASSERT_EQ(right.size(),2);
  ASSERT_EQ(left.size(),livls.size());
  ASSERT_EQ(right.size(),rivls.size());
}
*/

TEST(Fetch, testFetch){
  Vector3D a(5,5,0);
  for(unsigned b:{9,17,33}){
    //std::cout << b << "\n";
    for(unsigned i(0); i<b; ++i){
      Vector3D d;
      fetch(a,i,d,b);
      //std::cout << "  " << i << ": " << a << "-->" << d << "\n";
      ASSERT_EQ(i,moveNum(a,d,0,b));
    }
  }
}

/*
TEST(Biclique, loadFromFile9){
  std::vector<unsigned> array;
  std::vector<unsigned> indices;
  std::vector<float> ivls;
  loadCollisionTable<Vector3D>(array, indices, ivls, 9, 10, .25, .25, true);

  {
    Vector3D a1(10, 11, 0);Vector3D a2(11, 10, 0);Vector3D b1(11, 11, 0);Vector3D b2(10, 10, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,9);
  }
  {
    Vector3D a1(6, 13, 0);Vector3D a2(7, 12, 0);Vector3D b1(7, 13, 0);Vector3D b2(7, 12, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,9);
  }
  {
  Vector3D a1(7, 7, 0);
  Vector3D a2(7, 8, 0);
  Vector3D b1(6, 8, 0);
  Vector3D b2(7, 8, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(7, 8, 0);
  Vector3D a2(7, 7, 0);
  Vector3D b1(6, 8, 0);
  Vector3D b2(7, 7, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(7, 8, 0);
  Vector3D a2(8, 8, 0);
  Vector3D b1(8, 7, 0);
  Vector3D b2(8, 8, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(11, 6, 0);
  Vector3D a2(11, 5, 0);
  Vector3D b1(11, 5, 0);
  Vector3D b2(12, 6, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(5, 6, 0);
  Vector3D a2(5, 5, 0);
  Vector3D b1(5, 6, 0);
  Vector3D b2(6, 6, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(14, 13, 0);
  Vector3D a2(13, 13, 0);
  Vector3D b1(13, 13, 0);
  Vector3D b2(12, 12, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(10,10,0);
  Vector3D a2(11,10,0);
  Vector3D b1(12,10,0);
  Vector3D b2(11,10,0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  {
  Vector3D a1(10,10,0);
  Vector3D a2(11,10,0);
  Vector3D b1(11,10,0);
  Vector3D b2(10,10,0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }{
  Vector3D a1(11, 8, 0);
  Vector3D a2(11, 7, 0);
  Vector3D b1(10, 8, 0);
  Vector3D b2(9, 8, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }{
  Vector3D a1(12, 14, 0);
  Vector3D a2(12, 13, 0);
  Vector3D b1(13, 12, 0);
  Vector3D b2(12, 13, 0);
  assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
  for(unsigned i(0); i<10000; ++i){
    Vector3D a1(rand()%10+5,rand()%10+5,0);
    Vector3D a2(a1.x+(rand()%3)-1,a1.y+(rand()%3)-1,0);
    Vector3D b1(rand()%10+5,rand()%10+5,0);
    Vector3D b2(b1.x+(rand()%3)-1,b1.y+(rand()%3)-1,0);
    //std::cout<<a1<<a2<<b1<<b2<<"\n";
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls);
  }
}

TEST(interval, getIntervals){
  Vector3D a1(8, 7, 0);Vector3D a2(9, 6, 0);Vector3D b1(9, 6, 0);Vector3D b2(7, 5, 0);
  auto intvl(getForbiddenInterval(a1,a2,0,(a1-a2).len(),.25,b1,b2,0,(b1-b2).len(),.25));
  Vector3D A1(3, 3, 0);Vector3D A2(4, 4, 0);Vector3D B1(4, 4, 0);Vector3D B2(5, 2, 0);
  auto intvl2(getForbiddenInterval(A1,A2,0,(A1-A2).len(),.25,B1,B2,0,(B1-B2).len(),.25));
  EXPECT_NEAR(intvl.first,intvl2.first,.001);
  EXPECT_NEAR(intvl.second,intvl2.second,.001);

  bool swap=false, ortho=false, y=false;
  locationIndex(a1,b1,swap,ortho,y,17);
  auto moveA(moveNum(a1,a2,0,17));
  auto moveB(moveNum(b1,b2,0,17));
  EXPECT_EQ(moveA,12);
  EXPECT_EQ(moveB,1);
  auto mirroredA(getMirroredMove(moveA,swap,ortho,y,17));
  auto mirroredB(getMirroredMove(moveB,swap,ortho,y,17));
  EXPECT_EQ(mirroredA,8);
  EXPECT_EQ(mirroredB,13);
}

TEST(interval, getIntervals2){
  Vector3D a1(12, 13, 0);Vector3D a2(14, 12, 0);Vector3D b1(14, 12, 0);Vector3D b2(12, 13, 0);
  auto intvl(getForbiddenInterval(a1,a2,0,(a1-a2).len(),.25,b1,b2,0,(b1-b2).len(),.25));
  Vector3D A1(3, 2, 0);Vector3D A2(4, 4, 0);Vector3D B1(4, 4, 0);Vector3D B2(3, 2, 0);
  auto intvl2(getForbiddenInterval(A1,A2,0,(A1-A2).len(),.25,B1,B2,0,(B1-B2).len(),.25));
  EXPECT_NEAR(intvl.first,intvl2.first,.001);
  EXPECT_NEAR(intvl.second,intvl2.second,.001);

  bool swap=false, ortho=false, y=false;
  locationIndex(a1,b1,swap,ortho,y,17);
  auto moveA(moveNum(a1,a2,0,17));
  auto moveB(moveNum(b1,b2,0,17));
  EXPECT_EQ(moveA,11);
  EXPECT_EQ(moveB,3);
  auto mirroredA(getMirroredMove(moveA,swap,ortho,y,17));
  auto mirroredB(getMirroredMove(moveB,swap,ortho,y,17));
  EXPECT_EQ(mirroredA,7);
  EXPECT_EQ(mirroredB,15);
}

TEST(interval, getIntervals3){
  Vector3D a1(12, 8, 0);Vector3D a2(13, 7, 0);Vector3D b1(13, 8, 0);Vector3D b2(13, 7, 0);
  auto intvl(getForbiddenInterval(a1,a2,0,(a1-a2).len(),.25,b1,b2,0,(b1-b2).len(),.25));
  Vector3D A1(4, 3, 0);Vector3D A2(3, 4, 0);Vector3D B1(4, 4, 0);Vector3D B2(3, 4, 0);
  auto intvl2(getForbiddenInterval(A1,A2,0,(A1-A2).len(),.25,B1,B2,0,(B1-B2).len(),.25));
  EXPECT_NEAR(intvl.first,intvl2.first,.001);
  EXPECT_NEAR(intvl.second,intvl2.second,.001);

  bool swap=false, ortho=false, y=false;
  locationIndex(a1,b1,swap,ortho,y,17);
  auto moveA(moveNum(a1,a2,0,17));
  auto moveB(moveNum(b1,b2,0,17));
  EXPECT_EQ(moveA,12);
  EXPECT_EQ(moveB,14);
  auto mirroredA(getMirroredMove(moveA,swap,ortho,y,17));
  auto mirroredB(getMirroredMove(moveB,swap,ortho,y,17));
  EXPECT_EQ(mirroredA,4);
  EXPECT_EQ(mirroredB,2);

  std::vector<unsigned> left;
  std::vector<unsigned> right;
  getBiclique(a1,a2,b1,b2,0,(a1-a2).len(),0,(b1-b2).len(),left,right,17,.25,.25);
  std::vector<unsigned> left1;
  std::vector<unsigned> right1;
  getBiclique(A1,A2,B1,B2,0,(A1-A2).len(),0,(B1-B2).len(),left1,right1,17,.25,.25);
  if(left.size() != left1.size() || right.size() != right1.size()){
    std::cout << "Vector3D a1"<<a1 <<";Vector3D a2"<<a2 <<";Vector3D b1"<<b1<<";Vector3D b2"<<b2<<";\n";
    std::cout << "discrepancy\n";
    std::cout << left << "\n";
    std::cout << left1 << "\n";
    std::cout << right << "\n";
    std::cout << right1 << "\n";
    std::cout << "discrepancy\n";
  }
  ASSERT_EQ(left.size()+right.size(),left1.size()+right1.size());

  left.clear();
  right.clear();
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  getVertexAnnotatedBiclique(a1,a2,b1,b2,0,(a1-a2).len(),0,(b1-b2).len(),left,right,livls,rivls,17,.25,.25);
  left1.clear();
  right1.clear();
  std::vector<std::pair<float,float>> livls1;
  std::vector<std::pair<float,float>> rivls1;
  getVertexAnnotatedBiclique(A1,A2,B1,B2,0,(A1-A2).len(),0,(B1-B2).len(),left1,right1,livls1,rivls1,17,.25,.25);
  ASSERT_EQ(left.size()+right.size(),left1.size()+right1.size());
  ASSERT_EQ(livls.size()+rivls.size(),livls1.size()+rivls1.size());
}

void doBicliques(Vector3D & a1,
                 Vector3D & a2,
                 Vector3D & b1,
                 Vector3D & b2,
                 unsigned bf){
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  getBiclique(a1,a2,b1,b2,0,std::max(1.0,(a1-a2).len()),0,std::max(1.0,(b1-b2).len()),left,right,bf,.25,.25);
  bool swap=false, ortho=false, y=false;
  auto li(locationIndex(a1,b1,swap,ortho,y,bf));
  auto moveA(moveNum(a1,a2,0,bf));
  auto moveB(moveNum(b1,b2,0,bf));
  fromLocationIndex(li,a1,b1,bf-1);
  auto mirroredA(getMirroredMove(moveA,swap,ortho,y,bf));
  auto mirroredB(getMirroredMove(moveB,swap,ortho,y,bf));
  fetch(a1,mirroredA,a2,bf);
  fetch(b1,mirroredB,b2,bf);
  std::vector<unsigned> left1;
  std::vector<unsigned> right1;
  getBiclique(a1,a2,b1,b2,0,std::max(1.0,(a1-a2).len()),0,std::max(1.0,(b1-b2).len()),left1,right1,bf,.25,.25);
  EXPECT_EQ(left.size()+right.size(),left1.size()+right1.size());
}

TEST(Biclique, loadFromFile25){
  std::vector<unsigned> array;
  std::vector<unsigned> indices;
  std::vector<float> ivls;
  loadCollisionTable<Vector3D>(array, indices, ivls, 17, 10, .25, .25);//, true);
  {
    Vector3D a1(14, 5, 0);Vector3D a2(13, 7, 0);Vector3D b1(14, 6, 0);Vector3D b2(15, 4, 0);
    doBicliques(a1,a2,b1,b2,17);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D b1(4, 4, 0);Vector3D b2(3, 4, 0);Vector3D a1(2, 4, 0);Vector3D a2(3, 4, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D b1(4, 4, 0);Vector3D b2(3, 4, 0);Vector3D a1(3, 3, 0);Vector3D a2(3, 4, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(4, 4, 0);Vector3D a2(3, 4, 0);Vector3D b1(3, 3, 0);Vector3D b2(3, 4, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(7, 12, 0);Vector3D a2(5, 11, 0);Vector3D b1(7, 11, 0);Vector3D b2(6, 12, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(7, 8, 0);Vector3D a2(5, 7, 0);Vector3D b1(7, 6, 0);Vector3D b2(5, 7, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(6, 13, 0);Vector3D a2(7, 12, 0);Vector3D b1(7, 13, 0);Vector3D b2(7, 12, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(8, 7, 0);Vector3D a2(9, 6, 0);Vector3D b1(9, 6, 0);Vector3D b2(7, 5, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(6, 12, 0);Vector3D a2(8, 11, 0);Vector3D b1(6, 10, 0);Vector3D b2(4, 11, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(12, 6, 0);Vector3D a2(13, 8, 0);Vector3D b1(14, 6, 0);Vector3D b2(12, 7, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  {
    Vector3D a1(4, 4, 0);Vector3D a2(3, 4, 0);Vector3D b1(3, 3, 0);Vector3D b2(3, 4, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
  for(unsigned i(0); i<10000; ++i){
    Vector3D a1(rand()%10+5,rand()%10+5,0);
    Vector3D a2;
    fetch(a1,rand()%17,a2,17);
    Vector3D b1(rand()%10+5,rand()%10+5,0);
    Vector3D b2;
    fetch(b1,rand()%17,b2,17);
    //std::cout<<a1<<a2<<b1<<b2<<"\n";
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,17);
  }
}
TEST(Biclique, loadFromFile49){
  std::vector<unsigned> array;
  std::vector<unsigned> indices;
  std::vector<float> ivls;
  loadCollisionTable<Vector3D>(array, indices, ivls, 33, 10, .25, .25);//, true);
  {
    Vector3D a1(16, 10, 0);Vector3D a2(14, 9, 0);Vector3D b1(15, 11, 0);Vector3D b2(14, 9, 0);
    doBicliques(a1,a2,b1,b2,17);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,33);
  }
  {
    Vector3D a1(17, 18, 0);Vector3D a2(19, 21, 0);Vector3D b1(20, 16, 0);Vector3D b2(17, 17, 0);
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,33);
  }
  for(unsigned i(0); i<10000; ++i){
    Vector3D a1(rand()%14+7,rand()%14+7,0);
    Vector3D a2;
    fetch(a1,rand()%33,a2,33);
    Vector3D b1(rand()%14+7,rand()%14+7,0);
    Vector3D b2;
    fetch(b1,rand()%33,b2,33);
    //std::cout<<a1<<a2<<b1<<b2<<"\n";
    assertCorrectExtraction(a1,a2,b1,b2,array,indices,ivls,33);
  }
}*/

TEST(biclique, assertCore){
  Vector3D a1(91,128,0);
  Vector3D a2(92,127,0);
  Vector3D b1(92,128,0);
  Vector3D b2(91,128,0);
  float startA(36.4);
  float stopA(37.8);
  float startB(36.6);
  float stopB(37.6);
  std::vector<unsigned> left;
  std::vector<unsigned> right;
  std::vector<std::pair<float,float>> livls;
  std::vector<std::pair<float,float>> rivls;
  getExtendedAreaVertexAnnotatedBiclique(a1,a2,b1,b2,
                                         startA,stopA,startB,stopB,
                                         left, right, livls, rivls,
                                         17,.35,.35);
  int d=4;
  int span=9;
  int bf=17;
  bool swap=false, ortho=false, y=false;
  locationIndex(a1,b1,swap,ortho,y,bf); // Get rotation params from reverse action
  Vector3D dest;
  bool found(false);
  unsigned p,q;
  signed xx,yy;
  for(unsigned i(0); i<left.size(); ++i){
    p=left[i]/bf;
    q=left[i]%bf;
    xx=p%span;
    yy=p/span;
    Vector3D src(a1);
    src.x+=xx-d; // relative to a1
    if(signed(src.x)<0)continue;
    src.y+=yy-d; // relative to a1
    if(signed(src.y)<0)continue;

    //auto move(invertMirroredMove(q,swap,ortho,y,bf));
    fetch(src,q,dest,bf);
    if(src.sameLoc(a1) && dest.sameLoc(a2)) found=true;
  }
  ASSERT_TRUE(found);
}
#endif
