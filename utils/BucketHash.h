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


template <typename T, unsigned hashIntervalHundredths>
class BucketHash {
    public:
        BucketHash() :count(0),bucketWidth(double(hashIntervalHundredths)/100.0) {}

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
// hashIntervalHundredths specifies teh bucket width. (This is necessary since float/double are not allowed as template parameters)
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
template <typename T, unsigned hashIntervalHundredths, unsigned dim>
class KDBucketHash {
    public:
        KDBucketHash():bucketWidth(double(hashIntervalHundredths)/100.0) {}

        void insert(std::pair<T,T> const& it){
          size_t index1(it.first[dim-1]/bucketWidth);
          size_t index2(it.second[dim-1]/bucketWidth);
          assert(index1<=index2);
          std::cout << std::string(dim,' ') << index1<<"<"<<index2<<" insert("<<it.first[dim-1]<<","<<it.second[dim-1]<<")\n";
          if(buckets.size()<index2+1){
            buckets.resize(index2+1);
          }
          for(;index1<index2+1; ++index1){
            buckets[index1].insert(it);
          }
        }

        void get(std::pair<T,T> const& it, std::set<std::pair<T,T>>& result)const{
          size_t index1(it.first[dim-1]/bucketWidth);
          size_t index2(it.second[dim-1]/bucketWidth);
          assert(index1<=index2);
          std::cout << std::string(dim,' ') << index1<<"<"<<index2<<" get("<<it.first[dim-1]<<","<<it.second[dim-1]<<")\n";
          for(;index1<index2+1&&index1<buckets.size();++index1){
            buckets[index1].get(it,result);
          }
        }

        void remove(std::pair<T,T> const& it){
          size_t index1(it.first[dim-1]/bucketWidth);
          size_t index2(it.second[dim-1]/bucketWidth);
          assert(index1<=index2);
          std::cout << std::string(dim,' ') << index1<<"<"<<index2<<" remove("<<it.first[dim-1]<<","<<it.second[dim-1]<<")\n";
          for(;index1<index2+1; ++index1){
            buckets[index1].remove(it);
          }
        }

    private:
        double bucketWidth;
        std::vector<KDBucketHash<T,hashIntervalHundredths,dim-1>> buckets;
};

// Specialize the first dimension
template <typename T, unsigned hashIntervalHundredths>
class KDBucketHash<T,hashIntervalHundredths,1> {
    struct container{
      container(std::pair<T,T> const&v):count(1),value(v){}
      container(std::pair<T,T> const&v, unsigned c):count(c),value(v){}
      mutable unsigned count; // Changeable for even const copies; use with care
      std::pair<T,T> value;
      bool operator<(container const& other)const{return value<other.value;}
    };
    public:
        KDBucketHash() :bucketWidth(double(hashIntervalHundredths)/100.0) {}

        void insert(std::pair<T,T> const& it){
          size_t index1(it.first[0]/bucketWidth);
          size_t index2(it.second[0]/bucketWidth);
          assert(index1<=index2);
          std::cout <<" "<< index1<<"<"<<index2<<" insert("<<it.first[0]<<","<<it.second[0]<<")\n";
          if(buckets.size()<index2+1){
            buckets.resize(index2+1);
          }
          for(;index1<index2+1; ++index1){
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
          size_t index1(it.first[0]/bucketWidth);
          size_t index2(it.second[0]/bucketWidth);
          assert(index1<=index2);
          std::cout <<" "<< index1<<"<"<<index2<<" get("<<it.first[0]<<","<<it.second[0]<<")\n";
          for(;index1<index2+1&&index1<buckets.size();++index1){
            for(auto const& v:buckets[index1])
              result.insert(v.value);
          }
        }

        void remove(std::pair<T,T> const& it){
          size_t index1(it.first[0]/bucketWidth);
          size_t index2(it.second[0]/bucketWidth);
          assert(index1<=index2);
          std::cout <<" "<< index1<<"<"<<index2<<" remove("<<it.first[0]<<","<<it.second[0]<<")\n";
          for(;index1<index2+1; ++index1){
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
        double bucketWidth;
        std::vector<std::set<container>> buckets;
};

#endif
