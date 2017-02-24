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

#endif
