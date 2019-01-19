#ifndef BICLIQUE_H_
#define BICLIQUE_H_

#include<iostream>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <deque>
#include <algorithm>

namespace BiClique{

  // Perform set intersection for pre-sorted vectors
  template<typename T>
    unsigned intersection(std::vector<T>& v1, std::vector<T>& v2, std::vector<T>& out){
      out.reserve(std::min(v1.size(),v2.size()));
      for (auto it1 = v1.begin(), it2 = v2.begin();
          it1 != v1.end() && it2 != v2.end();
          ++it2) {
        while (it1 != v1.end() && *it1 < *it2) ++it1;
        if (it1 != v1.end() && *it1 == *it2) {
          out.push_back(*it1);
        }
      }
      return out.size();
    }

  // A utility function for testing and returning bicliques
  // of specified size
  bool testFeasibility(
      std::vector<std::vector<unsigned>> L,
      std::vector<std::vector<unsigned>> R,
      std::pair<unsigned,unsigned> const& start,
      unsigned lDegree,
      unsigned rDegree,
      std::vector<unsigned> const& histL,
      std::vector<unsigned> const& histR,
      std::vector<unsigned>& left,
      std::vector<unsigned>& right){
//std::cout << "Trying " << lDegree << " " << rDegree << "\n";
    assert(lDegree>0);
    assert(rDegree>0);
    if(histL[lDegree]<rDegree || histR[rDegree]<lDegree){
      return false;
    }
    if(rDegree==1){
      if(lDegree == 1){
        left.push_back(start.first);
        right.push_back(start.second);
        return true;
      }else{
        left.push_back(start.first);
        right.reserve(L[start.first].size());
        for(auto const& a:L[start.first]){
          right.push_back(a);
        }
        return true;
      }
    }
    if(lDegree==1){
      right.push_back(start.second);
      left.reserve(R[start.second].size());
      for(auto const& a:R[start.second]){
        left.push_back(a);
      }
      return true;
    }

    // Histograms for the count of nodes that  are referenced by index
    bool changed(true);
    unsigned lNodes(0);
    unsigned rNodes(0);
    while(changed){
      changed=false;
      lNodes=0;
      rNodes=0;
      // Prune out all edges that have a degree that is too small
      for(unsigned i(0); i<L.size(); ++i){
        if(L[i].size()){
          if(L[i].size()<lDegree){
            changed=true;
            for(auto j:L[i]){
              R[j].erase(std::remove(R[j].begin(),R[j].end(),i),R[j].end());
            }
            L[i].clear();
          }else{
            lNodes++;
          }
        }
      }
      for(unsigned i(0); i<R.size(); ++i){
        if(R[i].size()){
          if(R[i].size()<rDegree){
            changed=true;
            for(auto j:R[i]){
              L[j].erase(std::remove(L[j].begin(),L[j].end(),i),L[j].end());
            }
            R[i].clear();
          }else{
            rNodes++;
          }
        }
      }
    }
    if(lNodes==rDegree && rNodes==lDegree){
      left.reserve(R[start.second].size());
      for(auto const& a:R[start.second]){
        left.push_back(a);
      }
      right.reserve(L[start.first].size());
      for(auto const& a:L[start.first]){
        right.push_back(a);
      }
      return true;
    }
    if(lNodes<rDegree)return false;
    if(rNodes<lDegree)return false;

    // If we got here, the degree is too high on some vertices
    // TODO, this would likely run faster if we compute permuatations
    // based on the smaller of L or R... (instead of always using L)
    std::vector<unsigned> d;
    d.reserve(L.size());
    for(unsigned j(0); j<L.size(); ++j){
      if(j!=start.first && L[j].size()){
        d.push_back(j);
      }
    }
    // Find a subset that is of size lDegree and is common for
    // rDegree nodes
    // Test up to all N choose K combinations
    unsigned k(rDegree-1); // minus 1 because we always include the start vertex
    do {
      std::vector<unsigned> cmn(L[start.first]); // Start test with this...
      for(unsigned i(0); i<k; ++i){
        std::vector<unsigned> tmp;
        unsigned size(intersection(cmn,L[d[i]],tmp)); // Set intersection
        if(size<lDegree)break;
        cmn=tmp;
        if(size==lDegree && i+1==k &&
            std::find(cmn.begin(),cmn.end(),start.second)!=cmn.end()){
          // We have an answer
          right.reserve(cmn.size());
          for(auto const& a:cmn){
            right.push_back(a);
          }
          left.reserve(k);
          for(unsigned i(0); i<k; ++i){
            left.push_back(d[i]);
          }
          return true;
        }
      }
      std::reverse(d.begin()+k,d.end());
    } while (std::next_permutation(d.begin(),d.end()));
    return false;
  }

  // Finds the largest bi-connected clique in a bipartite graph G. This algorithm
  // Will also require the clique to contain an edge e, which must exist in G.
  // G is represented with two adjacency lists, one for the lefthand side and one
  // for the right-hand side of the graph. Numbers in each adjacency list must
  // match the indices of vertices in the respective adjacency list.
  // the result will be a list of edges in the biclique.
  void findBiClique(
      std::vector<std::vector<unsigned>> L,
      std::vector<std::vector<unsigned>> R,
      std::pair<unsigned,unsigned> const& start,
      std::vector<unsigned>& left,
      std::vector<unsigned>& right){
    // Sort adj lists for easier comparison ops
    for(auto& l:L){
      std::sort(l.begin(),l.end());
    }
    for(auto& r:R){
      std::sort(r.begin(),r.end());
    }
    /*for(auto const& f:L){
      unsigned j(0);
      for(unsigned i(0); i<R.size(); ++i){
      if(j>=f.size() || i<f[j]){std::cout << "0 ";}
      else{std::cout << "1 "; ++j;}
      }
      std::cout << "\n";
      }
      {
      std::cout << "L:\n";
      for(auto const& a:L){
      for(auto const& b:a){
      std::cout << b << " ";
      }
      std::cout << "\n";
      }
      std::cout << "R:\n";
      for(auto const& a:R){
      for(auto const& b:a){
      std::cout << b << " ";
      }
      std::cout << "\n";
      }
      std::cout << "\n";
      }*/

// Omitting this part because we assume that no nodes are disconnected from start
/*
    // Get rid of any node that start.first is not connected to
    unsigned i(0);
    unsigned k(0);
    unsigned j(L[start.first][k]);
    while(i<R.size()){
      if(i<j){
        R[i].clear();
        for(auto& l:L){
          // Remove references to i
          l.erase(std::remove(l.begin(),l.end(),i),l.end());
        }
        ++i;
      }else if(i==j){
        ++i;
        if(++k>=L[start.first].size()) break;
        j=L[start.first][k];
      }
    }
 {
  std::cout << "L:\n";
  for(auto const& a:L){
    for(auto const& b:a){
      std::cout << b << " ";
    }
    std::cout << "\n";
  }
  std::cout << "R:\n";
  for(auto const& a:R){
    for(auto const& b:a){
      std::cout << b << " ";
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}

    // Get rid of any node that start.second is not connected to
    i=0;
    k=0;
    j=R[start.second][k];
    while(i<L.size()){
      if(i<j){
        L[i].clear();
        for(auto& l:R){
          // Remove references to i
          l.erase(std::remove(l.begin(),l.end(),i),l.end());
        }
        ++i;
      }else if(i==j){
        ++i;
        if(++k>=R[start.second].size()) break;
        j=R[start.second][k];
      }
    }
 {
  std::cout << "L:\n";
  for(auto const& a:L){
    for(auto const& b:a){
      std::cout << b << " ";
    }
    std::cout << "\n";
  }
  std::cout << "R:\n";
  for(auto const& a:R){
    for(auto const& b:a){
      std::cout << b << " ";
    }
    std::cout << "\n";
  }
  std::cout << "\n";
}
*/
    std::vector<unsigned> histL(R.size()+1);
    std::vector<unsigned> histR(L.size()+1);
    unsigned lDegree(L[start.first].size());
    unsigned rDegree(R[start.second].size());
    // Special easy case...
    if(rDegree==1){
      if(lDegree == 1){
        left.push_back(start.first);
        right.push_back(start.second);
        return;
      }else{
        left.push_back(start.first);
        right.reserve(L[start.first].size());
        for(auto const& a:L[start.first]){
          right.push_back(a);
        }
        return;
      }
    }

    // Cumulative histogram - the number of vertices with degX or above.
    for(auto const& l:L){
      histL[l.size()]++;
    }
    for(int i(histL.size()-1); i>1; --i){
      histL[i-1]+=histL[i];
    }
    for(auto const& r:R){
      histR[r.size()]++;
    }
    for(int i(histR.size()-1); i>1; --i){
      histR[i-1]+=histR[i];
    }

    std::unordered_set<uint64_t> dupe;
    auto cmp = [](std::pair<unsigned,unsigned> left, std::pair<unsigned,unsigned> right) { return left.first*left.second<right.first*right.second;};
    std::priority_queue<std::pair<unsigned,unsigned>, std::vector<std::pair<unsigned,unsigned>>, decltype(cmp)> q(cmp);
    q.push({lDegree,rDegree});
    while(!q.empty()){
      lDegree=q.top().first;
      rDegree=q.top().second;
      q.pop();
      if(!testFeasibility(L,R,start,lDegree,rDegree,histL,histR,left,right)){
        // minus one from l
        unsigned n(lDegree-1);
        uint64_t key(n*L.size()+rDegree);
        if(n && dupe.find(key)==dupe.end()){
          dupe.insert(key);
          q.push({n,rDegree});
        }
        // minus one from r
        n=rDegree-1;
        key=lDegree*L.size()+n;
        if(n && dupe.find(key)==dupe.end()){
          dupe.insert(key);
          q.push({lDegree,n});
        }
      }else{
        break;
      }
    }
  }

};

#endif
