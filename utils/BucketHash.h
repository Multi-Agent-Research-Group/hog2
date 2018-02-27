//  BucketHash.cpp
//  hog2 glut
//
//  Created by Thayne Walker 2/17/2017
//  Copyright (c) 2017 University of Denver. All rights reserved.
//

#ifndef BucketHash_h
#define BucketHash_h

#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <assert.h>


template <typename T>
class BucketHash {
  public:
    BucketHash(double bw):count(0),bucketWidth(bw){}

    void insert(float t, float te, T const& it){
      ++count;
      size_t index1(t/bucketWidth);
      size_t index2(te/bucketWidth);
      assert(index1<=index2);
      if(buckets.size()<index2+1){
        buckets.resize(index2+1);
      }
      for(;index1<index2+1; ++index1){
        buckets[index1].insert(it);
      }
    }

    inline unsigned size()const{return count;}

    void get(float t, float te, std::set<T>& result)const{
      result.clear();
      size_t index1(t/bucketWidth);
      size_t index2(te/bucketWidth);
      assert(index1<=index2);
      for(;index1<index2+1&&index1<buckets.size();++index1){
        result.insert(buckets[index1].begin(), buckets[index1].end());
      }
    }

    void remove(float t, float te, T const& val){
      size_t index1(t/bucketWidth);
      size_t index2(te/bucketWidth);
      assert(index1<=index2);
      for(;index1<index2+1; ++index1){
        buckets[index1].erase(val);
      }
      // Naively assume the user has specified the right interval...
      --count;
    }

  private:
    unsigned count;
    double bucketWidth;
    std::vector<std::set<T> > buckets;
};

// K-Dimensional bucketized hash table
// T should be a class or struct with fields accessible using operator[]
// dim is the number of dimensions and should correspond to the dimensions in T
//
// Data that falls into multiple buckets is unfortunately duplicated. This could
// be minimized by using pointers, however, we assume that structures in this hash table
// are not very large to begin with.
//
// This could be done differently as a K-D Interval tree, however, the algorithm is more complicated
// for example, tree balancing would be needed, and insert, query and remove would be O([log n]^d) with O(nlogn) space.
// This structure is insert, query, remove: O(ceil(i/b)^d) where i=average interval length, b=bucket size with O(n*ceil(i/b)^d) space.
// We're OK as long as n is relatively small.
template <typename T, unsigned dim>
class KDBucketHash {
  public:
    KDBucketHash(double bw){bucketWidth=bw;}
    KDBucketHash(){}

    inline void indices(std::pair<T,T> const& it, size_t& index1, size_t& index2)const{
      if(it.first[dim-1]<it.second[dim-1]){
        index1=floor(it.first[dim-1])/bucketWidth;
        index2=ceil(it.second[dim-1])/bucketWidth;
      }else{
        index1=floor(it.second[dim-1])/bucketWidth;
        index2=ceil(it.first[dim-1])/bucketWidth;
      }
    }

    void insert(std::pair<T,T> const& it){
      size_t index1;
      size_t index2;
      indices(it,index1,index2);
      //std::cout << std::string(dim,' ') << index1<<"<"<<index2<<" insert("<<it.first[dim-1]<<","<<it.second[dim-1]<<")\n";
      if(buckets.size()<index2+1){
        buckets.resize(index2+1);
      }
      for(;index1<=index2; ++index1){
        buckets[index1].insert(it);
      }
    }

    void get(std::pair<T,T> const& it, std::set<std::pair<T,T>>& result)const{
      size_t index1;
      size_t index2;
      indices(it,index1,index2);
      //std::cout << std::string(dim,' ') << index1<<"<"<<index2<<" get("<<it.first[dim-1]<<","<<it.second[dim-1]<<")\n";
      for(;index1<=index2&&index1<buckets.size();++index1){
        buckets[index1].get(it,result);
      }
    }

    void remove(std::pair<T,T> const& it){
      //std::cout << std::string(dim,' ') << index1<<"<"<<index2<<" remove("<<it.first[dim-1]<<","<<it.second[dim-1]<<")\n";
      size_t index1;
      size_t index2;
      indices(it,index1,index2);
      for(;index1<=index2; ++index1){
        buckets[index1].remove(it);
      }
    }

  private:
    static double bucketWidth;
    std::vector<KDBucketHash<T,dim-1>> buckets;
};

// Specialize the first dimension
template <typename T>
class KDBucketHash<T,1> {
  struct container{
    container(std::pair<T,T> const&v):count(1),value(v){}
    container(std::pair<T,T> const&v, unsigned c):count(c),value(v){}
    mutable unsigned count; // Changeable for even const copies; use with care
    std::pair<T,T> value;
    bool operator<(container const& other)const{return value<other.value;}
  };
  public:
  KDBucketHash(double bw){bucketWidth=bw;}
  KDBucketHash(){}

  inline void indices(std::pair<T,T> const& it, size_t& index1, size_t& index2)const{
    if(it.first[0]<it.second[0]){
      index1=floor(it.first[0])/bucketWidth;
      index2=ceil(it.second[0])/bucketWidth;
    }else{
      index1=floor(it.second[0])/bucketWidth;
      index2=ceil(it.first[0])/bucketWidth;
    }
  }

  void insert(std::pair<T,T> const& it){
    size_t index1;
    size_t index2;
    indices(it,index1,index2);
    //std::cout <<" "<< index1<<"<"<<index2<<" insert("<<it.first[0]<<","<<it.second[0]<<")\n";
    if(buckets.size()<index2+1){
      buckets.resize(index2+1);
    }
    for(;index1<=index2; ++index1){
      container c(it);
      auto v(buckets[index1].find(c));
      if(v==buckets[index1].end()){
        buckets[index1].insert(c);
      }else{
        v->count++;
      }
    }
  }

  void get(std::pair<T,T> const& it, std::set<std::pair<T,T>>& result)const{
    size_t index1;
    size_t index2;
    indices(it,index1,index2);
    //std::cout <<" "<< index1<<"<"<<index2<<" get("<<it.first[0]<<","<<it.second[0]<<")\n";
    for(;index1<=index2&&index1<buckets.size();++index1){
      for(auto const& v:buckets[index1])
        result.insert(v.value);
    }
  }

  void remove(std::pair<T,T> const& it){
    size_t index1;
    size_t index2;
    indices(it,index1,index2);
    //std::cout <<" "<< index1<<"<"<<index2<<" remove("<<it.first[0]<<","<<it.second[0]<<")\n";
    for(;index1<=index2; ++index1){
      container c(it);
      auto v(buckets[index1].find(c));
      if(v->count<=1){
        buckets[index1].erase(c);
      }else{
        v->count--;
      }
    }
  }

  private:
  static double bucketWidth;
  std::vector<std::set<container>> buckets;
};

template <typename T, unsigned dim>
double KDBucketHash<T,dim>::bucketWidth=1.0;
template <typename T>
double KDBucketHash<T,1u>::bucketWidth=1.0;
#endif
