#ifndef BICLIQUE_H_
#define BICLIQUE_H_

#include <iostream>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <set>
#include <queue>
#include <algorithm>
#include <numeric>
#include <string.h>

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

  static bool testFeasibility(
      std::vector<std::vector<unsigned>> L,
      std::vector<std::vector<unsigned>> R,
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

    
    std::cout << "Before prune\n";
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

    // Histograms for the count of nodes that  are referenced by index
    bool changed(true);
    unsigned lNodes(0);
    unsigned rNodes(0);
    unsigned ixL=0, ixR=0;
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
            ixL=i;
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
            ixR=i;
          }
        }
      }
    }
    
    std::cout << "After prune\n";
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

    if(lNodes==rDegree && rNodes==lDegree){
      right=R[ixR];
      //right.reserve(R[ixR].size());
      //for(auto const& a:R[ixR]){
        //right.push_back(a);
      //}
      left=L[ixL];
      //left.reserve(L[ixL].size());
      //for(auto const& a:L[ixL]){
        //left.push_back(a);
      //}
      return true;
    }
    if(lNodes<rDegree)return false;
    if(rNodes<lDegree)return false;

    // If we got here, the degree is too high on some vertices
    static std::vector<unsigned> d;
    d.resize(0);
    d.reserve(R[ixR].size());
    for(unsigned j(0); j<L.size(); ++j){
      if(j!=ixL && L[j].size()){
        d.push_back(j);
      }
    }
    // Find a subset that is of size lDegree and is common for
    // rDegree nodes
    // Test up to all N choose K combinations
    unsigned k(rDegree-1); // minus 1 because we always include the start vertex
    static std::vector<unsigned> cmn(L[ixL]); // Start test with this...
    static std::vector<unsigned> tmp;

    // Punt!
    //right=L[start.first];
    //left.push_back(start.first);
    return false;

    // The following might be required, but is too expensive.
    do {
      //for(auto const& g:d)
        //std::cout << g << " ";
      //std::cout << "\n";
      cmn=L[ixL]; // Start test with this...
      for(unsigned i(0); i<k; ++i){
        tmp.resize(0);
        unsigned size(intersection(cmn,L[d[i]],tmp)); // Set intersection
        if(size<lDegree)break;
        cmn=tmp;
        if(size==lDegree && i+1==k &&
            std::find(cmn.begin(),cmn.end(),ixR)!=cmn.end()){
          // We have an answer
          right.reserve(cmn.size());
          for(auto const& a:cmn){
            right.push_back(a);
          }
          left.reserve(k);
          left.push_back(ixL);
          for(unsigned i(0); i<k; ++i){
            left.push_back(d[i]);
          }
          //std::cout << "\n";
          return true;
        }
      }
      std::reverse(d.begin()+k,d.end());
    } while (std::next_permutation(d.begin(),d.end()));
    return false;
  }
  // A utility function for testing and returning bicliques
  // of specified size
  static bool testFeasibility(
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
    if(histL[lDegree]<rDegree || histR[rDegree]<lDegree || L[start.first].size()<lDegree || R[start.second].size()<rDegree){
      return false;
    }

    
    /*std::cout << "Before prune\n";
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
            if(i==start.first)return false;
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
            if(i==start.second)return false;
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
    
    /*std::cout << "After prune\n";
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
    static std::vector<unsigned> d;
    d.resize(0);
    d.reserve(R[start.second].size());
    for(unsigned j(0); j<L.size(); ++j){
      if(j!=start.first && L[j].size()){
        d.push_back(j);
      }
    }
    // Find a subset that is of size lDegree and is common for
    // rDegree nodes
    // Test up to all N choose K combinations
    unsigned k(rDegree-1); // minus 1 because we always include the start vertex
    static std::vector<unsigned> cmn(L[start.first]); // Start test with this...
    static std::vector<unsigned> tmp;

    // Punt!
    //right=L[start.first];
    //left.push_back(start.first);
    return false;

    // The following might be required, but is too expensive.
    do {
      //for(auto const& g:d)
        //std::cout << g << " ";
      //std::cout << "\n";
      cmn=L[start.first]; // Start test with this...
      for(unsigned i(0); i<k; ++i){
        tmp.resize(0);
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
          left.push_back(start.first);
          for(unsigned i(0); i<k; ++i){
            left.push_back(d[i]);
          }
          //std::cout << "\n";
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
  static void findBiClique(
      std::vector<std::vector<unsigned>> const& L,
      std::vector<std::vector<unsigned>> const& R,
      std::pair<unsigned,unsigned> const& start,
      std::vector<unsigned>& left,
      std::vector<unsigned>& right){
    unsigned minimum(std::max(L[start.first].size()+1,R[start.first].size()+1));

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
*/
    unsigned lDegree(L[start.first].size());
/*
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
    if(lDegree==1){
      right.push_back(start.second);
      left.reserve(R[start.second].size());
      for(auto const& a:R[start.second]){
        left.push_back(a);
      }
      return;
    }

    static std::vector<unsigned> histL;
    histL.resize(R.size()+1);
    memset(&histL[0], 0, sizeof(histL[0]) * histL.size());
    static std::vector<unsigned> histR;
    histR.resize(L.size()+1);
    memset(&histR[0], 0, sizeof(histR[0]) * histR.size());
    auto comp([](std::pair<unsigned,unsigned> const& a, std::pair<unsigned,unsigned> const& b) { return (a.first*a.second > b.first*b.second);});
    std::set<std::pair<unsigned,unsigned>,decltype(comp)> q(comp);
    // Cumulative histogram - the number of vertices with degX or above.
    for(unsigned l(0); l<L.size(); ++l){
      //if(l.size()>=R.size())
        //std::cout << "bad "<<l.size()<<">="<<R.size()<<"\n";
      histL[L[l].size()]++;
    }
    for(int i(histL.size()-1); i>1; --i){
      histL[i-1]+=histL[i];
    }
    for(unsigned r(0); r<R.size(); ++r){
      //if(r.size()>=L.size())
        //std::cout << "bad "<<r.size()<<">="<<L.size()<<"\n";
      histR[R[r].size()]++;
    }
    for(int i(histR.size()-1); i>1; --i){
      histR[i-1]+=histR[i];
      //unsigned num(histR[i]);
      //while((i)+num>minimum && histL[num]>=i){
        //q.emplace(num--,i);
      //}
    }
    //if(q.empty()){
      q.emplace(L[start.first].size(),R[start.second].size());
    //}
    //if(R[start.second].size()>L[start.first].size()){
      //q.emplace(1,R[start.second].size());
    //}else if(R[start.second].size()<L[start.first].size()){
      //q.emplace(L[start.first].size(),1);
    //}
    //q.emplace(1,1);
    //q.emplace(R[start.second].size()>L[start.first].size()?1:L[start.first].size(),R[start.second].size()>L[start.first].size()?1:R[start.second].size());
    //std::cout << "q:\n";
    //for(auto const& p:possibilities){
      //std::cout << p.first << "," << p.second << "\n";
    //}

    // Sort adj lists for easier comparison ops
    // Skip this - we assume they are initially sorted
    /*for(auto& l:L){
      std::sort(l.begin(),l.end());
    }
    for(auto& r:R){
      std::sort(r.begin(),r.end());
    }*/

    //std::unordered_set<uint64_t> dupe;
    //auto cmp = [](std::pair<unsigned,unsigned> left, std::pair<unsigned,unsigned> right) { return left.first*left.second<right.first*right.second;};
    //std::priority_queue<std::pair<unsigned,unsigned>, std::vector<std::pair<unsigned,unsigned>>, decltype(cmp)> q(cmp);
    while(!q.empty()){
      lDegree=q.begin()->first;
      rDegree=q.begin()->second;

      if(lDegree+rDegree <= minimum){
        if(L[start.first].size()>R[start.second].size()){
          lDegree=1; rDegree=R[start.second].size();
        }else{
          lDegree=L[start.first].size(); rDegree=1;
        }
      }else{
        q.erase(q.begin());
        if(!testFeasibility(L,R,start,lDegree,rDegree,histL,histR,left,right)){
          // minus one from l
          unsigned n(lDegree-1);
          if(n>0)
            q.emplace(n,rDegree);
          // minus one from r
          n=rDegree-1;
          if(n>0)
            q.emplace(lDegree,n);
        }else{
          break;
        }
      }
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
      if(lDegree==1){
        right.push_back(start.second);
        left.reserve(R[start.second].size());
        for(auto const& a:R[start.second]){
          left.push_back(a);
        }
        return;
      }
    }
  }


// A class to represent Bipartite graph for Hopcroft 
// Karp implementation 
class BipGraph { 
	public: 
	// m and n are number of vertices on left 
	// and right sides of Bipartite Graph 
	unsigned m, n; 

	// adj[u] stores adjacents of left side 
	// vertex 'u'. The value of u ranges from 1 to m. 
	// 0 is used for dummy vertex 
	std::vector<std::vector<unsigned>> adj; 

	// These are basically pointers to arrays needed 
	// for hopcroftKarp() 

	std::vector<unsigned> pairU, pairV, dist; 
	BipGraph(std::vector<std::vector<unsigned>> const& a, unsigned nn):m(a.size()),n(nn),adj(a){
		// Shift all ids by 1 in order to add in the dummy node
		for (auto &v : adj) {
			for (auto &r : v) {
				r += 1;
			}
		}
		adj.insert(adj.begin(),{INT_MAX});
	}

  // Returns true if there is an augmenting path, else returns 
  // false 
  bool bfs() { 
    std::queue<unsigned> Q; //an integer queue 

    // First layer of vertices (set distance as 0) 
    for (unsigned u=1; u<=m; u++) { 
      // If this is a free vertex, add it to queue 
      if (pairU[u]==0) { 
        // u is not matched 
        dist[u] = 0; 
        Q.push(u); 
      } 

      // Else set distance as infinite so that this vertex 
      // is considered next time 
      else dist[u] = INT_MAX; 
    } 

    // Initialize distance to 0 as infinite 
    dist[0] = INT_MAX; 

    // Q is going to contain vertices of left side only.  
    while (!Q.empty()) 
    { 
      // Dequeue a vertex 
      auto u = Q.front(); 
      Q.pop(); 

      // If this node is not 0 and can provide a shorter path to 0 
      if (dist[u] < dist[0]) 
      { 
        // Get all adjacent vertices of the dequeued vertex u 
        for (auto i=adj[u].begin(); i!=adj[u].end(); ++i) 
        { 
          auto v = *i; 

          // If pair of v is not considered so far 
          // (v, pairV[V]) is not yet explored edge. 
          if (dist[pairV[v]] == INT_MAX) 
          { 
            // Consider the pair and add it to queue 
            dist[pairV[v]] = dist[u] + 1; 
            Q.push(pairV[v]); 
          } 
        } 
      } 
    } 

    // If we could come back to 0 using alternating path of distinct 
    // vertices then there is an augmenting path 
    return (dist[0] != INT_MAX); 
  } 


	// Adds augmenting path if there is one beginning 
	// with u 
  // Returns true if there is an augmenting path beginning with free vertex u 
  bool dfs(unsigned u) 
  { 
    if (u != 0) 
    { 
      for (auto i=adj[u].begin(); i!=adj[u].end(); ++i) 
      { 
        // Adjacent to u 
        auto v = *i; 

        // Follow the distances set by BFS 
        if (dist[pairV[v]] == dist[u]+1) 
        { 
          // If dfs for pair of v also returns 
          // true 
          if (dfs(pairV[v]) == true) 
          { 
            pairV[v] = u; 
            pairU[u] = v; 
            return true; 
          } 
        } 
      } 

      // If there is no augmenting path beginning with u. 
      dist[u] = INT_MAX; 
      return false; 
    } 
    return true; 
  } 


	// Returns size of maximum matching and stores the matching internally
  // Runtime is O(|E|sqrt{|V|})
  unsigned hopcroftKarp() { 
    // pairU[u] stores pair of u in matching where u 
    // is a vertex on left side of Bipartite Graph. 
    // If u doesn't have any pair, then pairU[u] is 0 
    pairU.resize(m+1); 

    // pairV[v] stores pair of v in matching. If v 
    // doesn't have any pair, then pairU[v] is 0 
    pairV.resize(n+1); 

    // dist[u] stores distance of left side vertices 
    // dist[u] is one more than dist[u'] if u is next 
    // to u'in augmenting path 
    dist.resize(m+1);

    // Initialize result 
    unsigned result = 0; 

    // Keep updating the result while there is an 
    // augmenting path. 
    while (bfs()) { 
      // Find a free vertex 
      for (unsigned u=1; u<=m; u++){
          // If current vertex is free and there is 
          // an augmenting path from current vertex
          if (pairU[u] == 0 && dfs(u)) {
            std::cout << u-1 << "\n";
            result++;
          }
        }
    } 
    return result; 
  } 
}; 

// This is a subroutine for koenigs algorithm
static void alternate(unsigned u, std::vector<std::vector<unsigned>> const &bigraph,
			   std::vector<bool> &visitU,
			   std::vector<bool> &visitV,
			   std::vector<unsigned> const& matchV) {
	// extend alternating tree from free vertex u.
    // visitU, visitV marks all vertices covered by the tree.

    visitU[u] = true;
	for (auto const &v : bigraph[u]) {
		if (!visitV[v]) {
			visitV[v] = true;
			assert(matchV[v] != INT_MAX); // otherwise match not maximum
			alternate(matchV[v], bigraph, visitU, visitV, matchV);
		}
	}
}

// Bipartite minimum vertex cover by Koenig's theorem
//
// param bigraph: adjacency list, index = vertex in U,
//                              value = neighbor list in V
// assumption: U = V = {0, 1, 2, ..., n - 1} for n = len(bigraph)
// returns: boolean table for U, boolean table for V
// comment: selected vertices form a minimum vertex cover,
//        i.e. every edge is adjacent to at least one selected vertex
//        and number of selected vertices is minimum
// complexity: `O(|V|*|E|)`
static std::pair<std::vector<unsigned>, std::vector<unsigned>> koenig(
	std::vector<std::vector<unsigned>> const &bigraph, // Adj list of entire graph
	std::vector<unsigned> const& matchV,					   // Maximal matching
	std::vector<unsigned> const& matchU){					   // Maximal matching
	auto n(matchV.size());
	auto m(matchU.size());
	std::cout << "matchV: ";
	for(int i(0); i<n; ++i)
		std::cout << matchV[i] << " ";
	std::cout << "\n";

	std::cout << "matchU: ";
	for(int i(0); i<n; ++i)
		std::cout << matchU[i] << " ";
	std::cout << "\n";

	//std::vector<unsigned> V(m);
    //std::iota(V.begin(),V.end(),0U); // Fill with counting numbers
	std::vector<bool> visitU(m,false); // -- build max alternating forest
	std::vector<bool> visitV(n,false);
	for(int u(0); u<m; ++u) {
	//for (auto const &u : V) {
		    std::cout << "matchU["<<u<<"]=?INF("<<matchU[u]<<")"<<INT_MAX<<"\n";
		if(matchU[u] == INT_MAX){ //--starting with free vertices in U
		std::cout << "do alternate\n";
			alternate(u, bigraph, visitU, visitV, matchV);
		}
	}
	std::pair<std::vector<unsigned>,std::vector<unsigned>> result;
	for (unsigned i(0); i < visitU.size(); ++i) {
		//std::cout << "visitU[" << i << "]=" << visitU[i] << "\n";
		if (!visitU[i]) {
			result.first.push_back(i);
		}
	}
	for (unsigned i(0); i < visitV.size(); ++i) {
		//std::cout << "visitV[" << i << "]=" << visitV[i] << "\n";
		if (visitV[i]) {
			result.second.push_back(i);
		}
	}
  return result;
}

// Get a complement of the vector
// Returns false if empty
static bool complement(std::vector<unsigned> const &in,
				unsigned n,
				std::vector<unsigned> &out) {
	out.reserve(n - in.size());
	unsigned v(0);
	for (int j(0); j < in.size(); ++j) {
		while (v < in[j]) {
			out.push_back(v++);
		}
		++v;
	}
	while (v < n) {
		out.push_back(v++);
	}
	return !out.empty();
}

// Returns false if complement is empty (that is, input is fully-connected)
static bool complement(std::vector<std::vector<unsigned>> const& adj, // Adjacency list for U
unsigned const m, // Number of vertices in V
std::vector<std::vector<unsigned>>& comp){ // Output
	bool valid(false);
	// Precondition: all vertices are listed in ascending order
	comp.resize(adj.size());
	for(int i(0); i<adj.size(); ++i){
		valid |= complement(adj[i],m,comp[i]);
	}
	return valid;
}

// Koenigs theorem states that the size of the vertex cover in a bipartite grah
// is equivalent to the size of a maximal matching.
// Koenig's algorithm also finds a vertex cover starting from a maximal matching
// However, our purpose is to get a max biclique. This can be dome by taking the
// complement of a vertex cover. Unfortunately, Koenigs algorithm doesn't
// necessarily find a vertex cover that spans both the left and right side of
// the bipartite graph. Hence taking the complement doesn't give us a biclique.
// One would have to enumerate all vertex covers in order to find one whose
// complement makes a valid biclique.
// Instead of enumerating all vertex covers, we use the core finding of Koenig
// that the size of the vertex cover is equal to the size of the maximal matching.
// ------------------------------------------------------------------------------
// TL;DR: The number of vertices in the graph, minus the size of the maximal
// matching in its complement tells us the size of the biclique :)
static void getMaxBiclique(std::vector<std::vector<unsigned>> const& adj, // Adjacency list for U
unsigned m, // Number of vetices in V
std::vector<unsigned>& left, std::vector<unsigned>& right) { // vertices selected for U,V

  unsigned order(m+adj.size()); // Total number of vertices in the graph
	std::vector<std::vector<unsigned>> a;
  if(!complement(adj,m,a)){ // This is the complement of the input graph
	  // Special case: adj is fully connected.
	  // This means the entire graph is a biclique...
	  left.resize(adj.size());
	  std::iota(left.begin(),left.end(),0); // All vertices in U
	  right=adj[0]; // A complete list of vertices in V
	  return;
	}
  std::cout << "complement: " << a << "\n";
	
	BipGraph g(a, m);
	auto sz(g.hopcroftKarp());

	std::cout << "Size of maximum matching is " << sz << "\n"; 
	for(unsigned i(1); i<=g.m; ++i){
		std::cout << i-1 << "-->" << g.pairU[i]-1 << "\n";
	}
	for(unsigned i(1); i<=g.n; ++i){
		std::cout << i-1 << "-->" << g.pairV[i]-1 << "\n";
	}

	// Build a reverse adj list... (adj list for V)
  // Take note of any orphaned vertices...
  signed orphansl(0);
	std::vector<std::vector<unsigned>> rev(m);
	for (int i(0); i < adj.size(); ++i) {
    if(adj[i].size()==m){++orphansl;}
		for (auto const &v : adj[i]) {
			rev[v].push_back(i);
		}
	}

  signed orphansr(0);
	for (int i(0); i < rev.size(); ++i) {
    if(rev[i].size()==adj.size()){++orphansr;}
	}
  unsigned orphans(orphansr>orphansl?orphansr-orphansl:orphansl-orphansr);

  // Size of the biclique
  auto bs=order-sz;
  //auto bs=order-(sz+orphans);
  std::cout << "The biclique size is " << bs << "\n";
  
  // Build histogram...
  static std::vector<unsigned> histL;
  histL.resize(rev.size()+1);
  memset(&histL[0], 0, sizeof(histL[0]) * histL.size());
  static std::vector<unsigned> histR;
  histR.resize(adj.size()+1);
  memset(&histR[0], 0, sizeof(histR[0]) * histR.size());
  // Cumulative histogram - the number of vertices with degX or above.
  for(unsigned l(0); l<adj.size(); ++l){
    //if(l.size()>=rev.size())
      //std::cout << "bad "<<l.size()<<">="<<rev.size()<<"\n";
    histL[adj[l].size()]++;
  }
  for(int i(histL.size()-1); i>1; --i){
    histL[i-1]+=histL[i];
  }
  for(unsigned r(0); r<rev.size(); ++r){
    //if(r.size()>=adj.size())
      //std::cout << "bad "<<r.size()<<">="<<adj.size()<<"\n";
    histR[rev[r].size()]++;
  }
  auto comp([](std::pair<unsigned,unsigned> const& a, std::pair<unsigned,unsigned> const& b) {
    auto f=a.first*a.second; auto s=b.first*b.second;
    return f==s?a<b:f>s;});
  std::set<std::pair<unsigned,unsigned>,decltype(comp)> q(comp);
  for (int i(histR.size() - 1); i > 1; --i) {
    histR[i - 1] += histR[i];
    if (histR[i]) {
      auto nL(bs - i);
      if (nL < histL.size() && histL[nL] >= bs - histR[i]) {
        q.emplace(nL, i);
      }
    }
  }
  for (int i(histL.size() - 1); i >= histR.size(); --i) {
    if (histL[i]) {
      auto nR(bs - i);
      if (nR < histR.size() && histR[nR] >= bs - histL[i]) {
        q.emplace(i, nR);
      }
    }
  }
  while (q.empty()&& bs>1) {
    --bs;
    for (int i(histR.size() - 1); i > 1; --i) {
      if (histR[i]) {
        auto nL(bs - i);
        if (nL < histL.size() && histL[nL] >= bs - histR[i]) {
          q.emplace(nL, i);
        }
      }
    }
    for (int i(histL.size() - 1); i >= histR.size(); --i) {
      if (histL[i]) {
        auto nR(bs - i);
        if (nR < histR.size() && histR[nR] >= bs - histL[i]) {
          q.emplace(i, nR);
        }
      }
    }
  }
  while (!q.empty()) {
    if(testFeasibility(adj, rev, q.begin()->first, q.begin()->second, histL, histR, left, right)) {
      break;
    } else {
      q.erase(q.begin());
      while (q.empty()&& bs>1) {
        --bs;
        for (int i(histR.size() - 1); i > 1; --i) {
          if (histR[i]) {
            auto nL(bs - i);
            if (nL < histL.size() && histL[nL] >= bs - histR[i]) {
              q.emplace(nL, i);
            }
          }
        }
        for (int i(histL.size() - 1); i >= histR.size(); --i) {
          if (histL[i]) {
            auto nR(bs - i);
            if (nR < histR.size() && histR[nR] >= bs - histL[i]) {
              q.emplace(i, nR);
            }
          }
        }
      }
    }

  }

  assert(left.size()==q.begin()->first);
  assert(right.size()==q.begin()->second);
  return;

  // Add the initial node, starting with an even split
    // Koenigs algorithm assumes that there is at least one unmatched vertex in
	// either U or V. If this is not the case, we have a perfect matching and
	// an alternative matching must be used
	std::vector<unsigned> matchU(g.m);
	bool Uok(false);
	for (int i(0); i < matchU.size(); ++i) {
		if (g.pairU[i+1]) {
			matchU[i] = g.pairU[i + 1] - 1;
		} else {
			matchU[i] = INT_MAX;
			Uok=true;
		}
	}
	std::vector<unsigned> matchV(g.n);
	bool Vok(false);
    for(int i(0); i<matchV.size(); ++i){
		if (g.pairV[i+1]) {
			matchV[i]=g.pairV[i+1]-1;
		} else {
			matchV[i] = INT_MAX;
			Vok=true;
		}
	}
	std::pair<std::vector<unsigned>, std::vector<unsigned>> k;
	if (Uok) {
		k = koenig(a, matchV, matchU);
	}else if(Vok){
		k = koenig(rev, matchU, matchV);
		// Swap
		auto tmp(k.first);
		k.first=k.second;
		k.second=tmp;
	} else {
		// There was a perfect matching. This means |U|=|V| and there is a vertex cover
		// of size |U|=|V| and the complement of the graph has a clique of size |U|=|V|. 
		// First, select all of one side and swap sides for any vertex with degree 1
		bool fixed(false);
		k.first.resize(a.size());
		std::iota(k.first.begin(), k.first.end(), 0); // All vertices in U
		for (auto v(k.first.begin()); v != k.first.end(); /*++v*/) {
			if (a[*v].size() == 1) {
				k.second.push_back(a[*v][0]);
				k.first.erase(v);
				fixed=true;
			} else {
				++v;
			}
		}
		if (!fixed) {
			// Try the other side
			k.first.clear();
			k.second.resize(rev.size());
			std::iota(k.second.begin(), k.second.end(), 0); // All vertices in U
			for (auto v(k.second.begin()); v != k.second.end(); /*++v*/) {
				if (rev[*v].size() == 1) {
					k.first.push_back(rev[*v][0]);
					k.second.erase(v);
					fixed = true;
				} else {
					++v;
				}
			}
		}
		if(!fixed){
			// What to do now???
			assert(false);
		}
	}

	for(auto const& v:k.first){
		std::cout << v << " ";
	}
	std::cout << "\n";

    complement(k.first,adj.size(),left);
    complement(k.second,m,right);
}

};

#endif
