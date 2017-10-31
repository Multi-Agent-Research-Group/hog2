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
#ifndef MULTIOBJECTIVE_H
#define MULTIOBJECTIVE_H

#include <algorithm>
#include <vector>

template<unsigned dim>
struct cost{
  cost(){bzero(value,sizeof(float)*dim);}
  cost(std::vector<float> const&w){std::copy(w.begin(), w.end(), value);}
  float value[dim];
  float& operator[](size_t index){return value[index];}
  float const& operator[](size_t index)const{return value[index];}
  uint64_t Hash()const{
    static const uint64_t primes[10] = {98866927,15485863,87948617,32452843,34273271,87948617,27290279,70575433,17977291,16777619};
    // We assume at least 2 objectives, otherwise why are you using a vector?
    uint64_t hash(*(uint32_t*) &value[0] | ((*(uint32_t*) &value[1])<<32));
    for(int i(2); i<dim; i+=2){
      if(hash%2)
        hash ^= (((*(uint32_t*) &value[i])*primes[i%10])<<32);
      else
        hash ^= (*(uint32_t*) &value[i])*primes[i%10];
    }
    return hash;
  } 
  void operator+=(cost const& i2){
    for(int i(0); i<dim; ++i){
      value[i]+=i2.value[i];
    }
  }
  cost operator+(cost const& i2)const{
    cost total;
    for(int i(0); i<dim; ++i){
      total.value[i]=value[i]+i2.value[i];
    }
    return total;
  }
  cost operator-(cost const& i2)const{
    cost total;
    for(int i(0); i<dim; ++i){
      total.value[i]=value[i]-i2.value[i];
    }
    return total;
  }
  cost operator+(double i2)const{
    cost total;
    for(int i(0); i<dim; ++i){
      total.value[i]+=i2;
    }
    return total;
  }
  cost operator-(double i2)const{
    cost total;
    for(int i(0); i<dim; ++i){
      total.value[i]-=i2;
    }
    return total;
  }
  // Note that we can always set weights to 1 to get an unwieghted sum
  // Sum with coefficients
  double normalized()const{
    double total(0.0);
    for(int i(0); i<dim; ++i){
      total += weights[i]*value[i];
    }
    return total;
  }
  // Simple deviation
  double deviation()const{
    double total(0.0);
    for(int i(0); i<dim; ++i){
      //total += std::max(0.0f,goals[i]-value[i]); // Constrain leq
      total += fabs(goals[i]-value[i]); // Constrain fabs
    }
    return total;
  }
  // Weighted deviation
  double normalizedDeviation()const{
    double total(0.0);
    for(int i(0); i<dim; ++i){
      //total += std::max(0.0f,weights[i]*(goals[i]-value[i])); // Constrain leq
      total += weights[i]*fabs(goals[i]-value[i]); // Constrain fabs
    //std::cout << weights[i]*fabs(goals[i]-value[i]) << "+";
    }
    //std::cout << "="<<total<<" ";
    return total;
  }
  enum CompareType{
    PURE,
    GOAL_PURE,
    WEIGHTED_SUM,
    GOAL_WEIGHTED_SUM,
    LEXICOGRAPHIC,
    GOAL_LEXICOGRAPHIC
  };
  static CompareType compareType;
  static float goals[dim];
  static float weights[dim];
  // For grouping sets together to be compared lexicographically
  //static std::vector<std::vector<int>> groups;
  // Does this cost strictly dominate other?
  bool operator<(cost const& other)const{
    switch(compareType){
      case  PURE:
      default:
        {
          bool equal(true);
          // Are all values less?
          for(int i(0); i<dim; ++i){
            if(!fleq(value[i],other[i])){
              return false;
            }
            if(!fequal(value[i],other[i])){
              equal=false;
            }
          }
          // If we got here, then all values are either less or equal
          // If all equal, return false
          return !equal;
          break;
        }
      case GOAL_PURE:
        {
          bool equal(true);
          // Are all values less?
          for(int i(0); i<dim; ++i){
            float a(std::max(0.0f,goals[i]-value[i]));
            float b(std::max(0.0f,goals[i]-other.value[i]));
            
            if(!fleq(a,b)){
              return false;
            }
            if(!fequal(a,b)){
              equal=false;
            }
          }
          // If we got here, then all values are either less or equal
          // If all equal, return false
          return !equal;
          break;
        }
      case WEIGHTED_SUM:
        {
          return fless(normalized(),other.normalized());
          break;
        }
      case GOAL_WEIGHTED_SUM:
        {
          //std::cout << normalizedDeviation() << " < ?" << other.normalizedDeviation() << "\n";
          return fless(normalizedDeviation(),other.normalizedDeviation());
          break;
        }
      case LEXICOGRAPHIC:
        {
          for(int i(0); i<dim; ++i){
            if(!fequal(value[i], other.value[i]))
            {
              return fless(value[i], other.value[i]);
            }
          }
          return false;
          break;
        }
      case GOAL_LEXICOGRAPHIC:
        {
          for(int i(0); i<dim; ++i){
            float a(std::max(0.0f,goals[i]-value[i]));
            float b(std::max(0.0f,goals[i]-other.value[i]));
            if(!fequal(a,b)){
              return fless(a,b);
            }
          }
          return false;
          break;
        }
    }
  }
  // This is not equivalence! It simply means neither label dominates the other
  bool operator==(cost const& other)const{
    switch(compareType){
      case  PURE:
      case GOAL_PURE:
      default:
        {
          return !operator<(other) && !operator>(other);
          break;
        }
      case WEIGHTED_SUM:
        {
          return fequal(normalized(),other.normalized());
          break;
        }
      case GOAL_WEIGHTED_SUM:
        {
          return fequal(normalizedDeviation(),other.normalizedDeviation());
          break;
        }
      case LEXICOGRAPHIC:
        {
          for(int i(0); i<dim; ++i){
            if(!fequal(value[i], other.value[i])){
              return false;
            }
          }
          return true;
          break;
        }
      case GOAL_LEXICOGRAPHIC:
        {
          for(int i(0); i<dim; ++i){
            float a(std::max(0.0f,goals[i]-value[i]));
            float b(std::max(0.0f,goals[i]-other.value[i]));
            if(!fequal(a,b)){
              return false;
            }
          }
          return true;
          break;
        }
    }
  }
  bool operator!=(cost const& other)const{
    return !(*this==other);
  }
  bool operator>(cost const& other)const{
    return other<*this;
  }
};

namespace std
{
    template <unsigned dim>
    struct hash<cost<dim>>
    {
        size_t operator()(cost<dim>* const & x) const noexcept
        {
            return x->Hash();
        }
        size_t operator()(cost<dim> const & x) const noexcept
        {
            return x.Hash();
        }
    };
}


#endif
