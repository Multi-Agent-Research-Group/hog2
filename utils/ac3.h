// AC3 algorithm
#ifndef AC3_H
#define AC3_H

#include <math.h>
#include <limits.h>
#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <ostream>
#include "PrintUtils.h"
#include "PairMap.h"

bool revise(std::pair<unsigned,unsigned>const& v,
    std::vector<std::vector<unsigned>>& d,
    std::vector<std::vector<unsigned>> const& c,
    std::map<std::pair<unsigned,unsigned>,unsigned>& r){
  bool revised(false);
  for(unsigned i(0); i<d[v.first].size(); ++i){
    auto x(d[v.first][i]);
    bool satisfied(false);
    for(auto const& y:d[v.second]){
      if(x+y == c[v.first][v.second]){
        std::cout << v << ":"<<c[v.first][v.second]<<" is satisfied by: " << x << "+" <<y << "\n";
        satisfied=true;
        //break;
      }else if(x+y > c[v.first][v.second] &&
          (r.find(v)==r.end() || (x+y)-c[v.first][v.second] < r[v])){
        r[v]=(x+y)-c[v.first][v.second];
      }
    }
    if(!satisfied){
      std::cout << v << ":"<<c[v.first][v.second]<<" is not satisfied by: " << x << "+" <<d[v.second] << "\n";
      std::cout << "erasing " << x << " from " << v.first << "\n";
      d[v.first].erase(d[v.first].begin()+i);
      std::cout << "could increase by " << r[v] << "\n";
      revised=true;
    }
  }
  std::cout << d << "\n";
  return revised;
}

void minSatAssignments(std::vector<std::set<unsigned>>& d,
    PairMap<unsigned> const& c,
    std::vector<std::vector<unsigned>>& assignments){ // Complete graph weights
  static std::vector<std::vector<unsigned>> combinations;
  combinations = {{}};
  combinations.reserve(d.size());
  static std::vector<std::vector<unsigned>> r;
  r.reserve(c.vals.size());
  signed i(0);
  for (const auto &u : d)
  {
    r.clear();
    for (const auto &x : combinations)
    {
      for (const auto y : u)
      {
        auto z(x);
        z.push_back(y);
        bool valid(true);
        for(signed j(0); j<i; ++j){
          if(!valid)break;
          for(signed k(i); k<i+1; ++k){
            if(z[j] + z[k] < c.get(j,k))
            {
              valid=false;
              break;
            }
          }
        }
        if(valid){
          r.push_back(z);
        }
      }
    }
    combinations = std::move(r);
    ++i;
  }

  if (combinations.size())
  {
    unsigned minsum(Util::sum(combinations[0]));
    assignments.push_back(combinations[0]);
    for (unsigned i(1); i < combinations.size(); ++i)
    {
      auto sum(Util::sum(combinations[i]));
      if (sum < minsum)
      {
        minsum = sum;
        assignments.clear();
        assignments.push_back(combinations[i]);
      }
      else if (sum == minsum)
      {
        assignments.push_back(combinations[i]);
      }
    }
  }
}

bool revise(std::pair<unsigned,unsigned>const& v,
    std::vector<std::vector<unsigned>>& d,
    PairMap<unsigned> const& c)
{
  bool revised(false);
  for(unsigned i(0); i<d[v.first].size(); ++i){
    auto x(d[v.first][i]);
    bool satisfied(false);
    for(auto const& y:d[v.second]){
      // Check if x+y >= min value
      if(x+y >= c.get(v.first,v.second)){
        std::cout << v << ":"<<c.get(v.first,v.second)<<" is satisfied by: " << x << "+" <<y << "\n";
        satisfied=true;
        break;
      }
    }
    if(!satisfied){
      std::cout << v << ":"<<c.get(v.first,v.second)<<" is not satisfied by: " << x << "+" <<d[v.second] << "\n";
      std::cout << "erasing " << x << " from " << v.first << "\n";
      d[v.first].erase(d[v.first].begin()+i);
      revised=true;
    }
  }
  std::cout << d << "\n";
  return revised;
}

bool ac3(std::vector<std::vector<unsigned>>& d,
    PairMap<unsigned> const& c) // Complete graph weights
{
  unsigned n(d.size());
  std::deque<std::pair<unsigned,unsigned>> q;
  // Test all arcs (agent pairs)
  for(unsigned i(0); i<n-1; ++i){
    for(unsigned j(i+1); j<n; ++j){
      q.emplace_back(i,j);
    }
  }
  while(q.size()){
    auto v(q.front());
    q.pop_front();
    if(revise(v,d,c)){
      if(d[v.first].empty()){
        return false;
      }
      for(unsigned i(0); i<v.first; ++i){
        q.emplace_back(i,v.first);
      }
      for(unsigned i(v.first+1); i<n; ++i){
        if(i!=v.second) q.emplace_back(v.first,i);
      }
    }
  }
  return true;
}

bool AC3(std::vector<std::vector<unsigned>>& d,
    std::vector<std::vector<unsigned>> const& c,
    std::map<std::pair<unsigned,unsigned>,unsigned>& r){
  unsigned n(d.size());
  std::deque<std::pair<unsigned,unsigned>> q;
  // Test all arcs (agent pairs)
  for(unsigned i(0); i<n-1; ++i){
    for(unsigned j(i+1); j<n; ++j){
      q.emplace_back(i,j);
    }
  }
  while(q.size()){
    auto v(q.front());
    q.pop_front();
    if(revise(v,d,c,r)){
      if(d[v.first].empty()){
        return false;
      }
      for(unsigned i(0); i<v.first; ++i){
        q.emplace_back(i,v.first);
      }
      for(unsigned i(v.first+1); i<n; ++i){
        if(i!=v.second) q.emplace_back(v.first,i);
      }
    }
  }
  return true;
}

// Assumes domains are in sorted order
bool satisfy(std::vector<std::vector<unsigned>>& d,
    std::vector<std::vector<unsigned>> const& c){
  std::map<std::pair<unsigned,unsigned>,unsigned> r;
  auto dcopy(d);
  auto ccopy(c);
  bool satisfied(AC3(dcopy,c,r));

  if(!satisfied){
    for(auto const& [k,v]:r){
      dcopy=d;
      dcopy[k.first].push_back(dcopy[k.first].back()+v);
      dcopy[k.second].push_back(dcopy[k.second].back()+v);
      ccopy=c;
      ccopy[k.first][k.second]+=v;
      if(AC3(dcopy,ccopy,r)){
        satisfied=true;
        break;
      }
    }
  }
  std::cout << "done\n" << dcopy << "\n" << ccopy << "\n";
  return satisfied;
}

#endif
