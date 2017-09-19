#ifndef env_UnitTests_h_
#define env_UnitTests_h_

#include <gtest/gtest.h>
#include "dtedreader.h"
#include "BucketHash.h"

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

TEST(util, BucketHash){
  BucketHash<std::vector<double>,100> hash;
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
  KDBucketHash<std::vector<double>,100,3> hash;
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
#endif
