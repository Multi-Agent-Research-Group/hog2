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
#ifndef UTILITIES_H
#define UTILITIES_H

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <sys/resource.h>

namespace Util{

template <typename I>
constexpr std::reverse_iterator<I> make_reverse_iter(I i){
  return std::reverse_iterator<I>(i);
}

// Compute max over a vector
template <typename T>
T max(std::vector<T> const& v){
	T m(0);
	for(auto const& vv:v){
		m=std::max(m,vv);
	}
	return m;
}

// Compute max over a vector of vectors
template <typename T>
T max(std::vector<std::vector<T>> const& v){
	T m(0);
	for(auto const& vv:v){
		m=std::max(m,max(vv));
	}
	return m;
}

  // Set upper limit on memory consumption
  void setmemlimit(rlim_t megabytes)
  {
    struct rlimit memlimit;

    megabytes*=(1024*1024);
    memlimit.rlim_cur = memlimit.rlim_max = megabytes;
    setrlimit(RLIMIT_AS, &memlimit);
  }

  // swaps two values
  template <typename T>
    void swap(T* a , T*b)
    {
      T temp(*a);
      *a = *b;
      *b = temp;
    }

  //returns integer part of a floating point number
  int iPartOfNumber(float x)
  {
    return (int)x;
  }

  //returns fractional part of a number
  float fPartOfNumber(float x)
  {
    if (x>0) return x - iPartOfNumber(x);
    else return x - (iPartOfNumber(x)+1);

  }

  //returns 1 - fractional part of number
  float rfPartOfNumber(float x)
  {
    return 1 - fPartOfNumber(x);
  }

  template<typename Out>
  void split(const std::string &s, char delim, Out result);
  std::vector<std::string> split(const std::string &s, char delim);

template<typename T>
bool contains(std::vector<T> const& haystack, T const& needle){
  return std::find(haystack.begin(),haystack.end(),needle)==haystack.end();
}

template<typename T>
signed indexOf(std::vector<T> const& haystack, T const& needle){
  auto index(std::find(haystack.begin(),haystack.end(),needle));
  return index==haystack.end()?-1:index-haystack.begin();
}

};

template<typename Out>
void Util::split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> Util::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}


#endif
