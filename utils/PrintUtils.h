/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2019 All rights reserved.
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
#ifndef PRINT_UTILS__
#define PRINT_UTILS__

#include <array>
#include <vector>
#include <utility>

template <typename T, size_t N>
inline std::ostream& operator <<(std::ostream & out, std::array<T,N> const& v){
  if(v.empty())return out;
  out<<"["<<*v.cbegin();
  for(auto e(v.cbegin()+1); e!=v.end(); ++e)
    out << "," << *e;
  out<<"]";
  return out;
}

template <typename T>
inline std::ostream& operator <<(std::ostream & out, std::vector<T> const& v){
  if(v.empty())return out;
  out<<"["<<*v.cbegin();
  for(auto e(v.cbegin()+1); e!=v.end(); ++e)
    out << "," << *e;
  out<<"]";
  return out;
}

template <typename T, typename U>
inline std::ostream& operator <<(std::ostream & out, std::pair<T,U> const& v){
  out<<"("<<v.first << "," << v.second << ")";
  return out;
}

template <typename T>
inline std::ostream& operator <<(std::ostream & out, std::unique_ptr<T> const& v){
  out << *v.get();
  return out;
}

#endif
