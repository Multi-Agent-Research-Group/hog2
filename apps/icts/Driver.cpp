/*
 * $Id: sample.h,v 1.6 2006/09/18 06:23:39 nathanst Exp $
 *
 *  Driver.h
 *  hog
 *
 *  Created by Thayne Walker on 3/17/17.
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <memory>
#include <iostream>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include "Map2DEnvironment.h"

int maxDepth;

struct Hashable{
  virtual uint64_t Hash()const=0;
};

struct NodePtrComp
{
  bool operator()(const Hashable* lhs, const Hashable* rhs) const  { return lhs->Hash()<rhs->Hash(); }
};

namespace std
{
    template <>
    struct hash<Hashable>
    {
        size_t operator()(Hashable* const & x) const noexcept
        {
            return x->Hash();
        }
    };
}

struct Node : public Hashable{
	static MapEnvironment* env;
	Node(){}
	Node(xyLoc a, int d):n(a),depth(d),optimal(false){}
	xyLoc n;
	uint16_t depth;
        bool optimal;
        bool connected()const{return parents.size()+successors.size();}
	std::unordered_set<Node*> parents;
	std::unordered_set<Node*> successors;
	virtual uint64_t Hash()const{return (env->GetStateHash(n)<<16) | depth;}
};

typedef std::vector<std::unordered_set<Node*>> MDD;
typedef std::vector<Node*> MultiState; // rank=agent num
typedef std::unordered_map<uint64_t,Node> DAG;

std::ostream& operator << (std::ostream& ss, MultiState const& n){
  for(auto const& a: n)
    ss << " " << a->n;
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const& n){
  ss << std::string(n.depth,' ')<<n.n;
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const* n){
  ss << std::string(n->depth,' ')<<n->n << n->successors.size()<< "\n";
  for(auto const m: n->successors)
    ss << m;
  return ss;
}

MapEnvironment* Node::env=nullptr;

bool LimitedDFS(xyLoc const& start, xyLoc const& end, DAG& dag, MDD& mdd, int depth, int maxDepth){
  //std::cout << start << " g:" << (maxDepth-depth) << " h:" << Node::env->HCost(start,end) << " f:" << ((maxDepth-depth)+Node::env->HCost(start,end)) << "\n";
  if(depth<0 ||
      (maxDepth-depth)+Node::env->HCost(start,end)>maxDepth){ // Note - this only works for a perfect heuristic.
    //std::cout << "pruned\n";
    return false;
  }

  if(Node::env->GoalTest(end,start)){
    Node n(start,depth);
    uint64_t hash(n.Hash());
    dag[hash]=n;
    Node* parent(&dag[hash]);
    int dorig(depth);
    while(depth++<maxDepth){
      // Wait at goal
      Node current(start,depth);
      uint64_t chash(current.Hash());
      dag[chash]=current;
      parent->successors.insert(&dag[chash]);
      dag[chash].parents.insert(parent);
      parent=&dag[chash];
    }
    std::unordered_set<Node*>& s=mdd[maxDepth-dorig];
    s.insert(parent);
    //std::cout << "found " << start << "\n";
    return true;
  }

  std::vector<xyLoc> successors;
  Node::env->GetSuccessors(start,successors);
  bool result(false);
  for(auto const& node: successors){
    if(LimitedDFS(node,end,dag,mdd,depth-1,maxDepth)){
      Node n(start,depth);
      uint64_t hash(n.Hash());
      if(dag.find(hash)==dag.end()){
        dag[hash]=n;
        std::unordered_set<Node*>& s=mdd[maxDepth-(depth)];
        s.insert(&dag[hash]);
      }else if(dag[hash].optimal){
        return true; // Already found a solution from search at this depth
      }

      Node* parent(&dag[hash]);

      //std::cout << "found " << start << "\n";
      std::unique_ptr<Node> c(new Node(node,depth-1));
      Node* current(dag.find(c->Hash())==dag.end()?c.release():&dag[c->Hash()]);
      current->optimal = result = true;
      //std::cout << *parent << " parent of " << *current << "\n";
      dag[current->Hash()].parents.insert(parent);
      //std::cout << *current << " child of " << *parent << "\n";
      dag[parent->Hash()].successors.insert(current);
    }
  }
  return result;
}

// TODO
// Allocate DAG outside of this function
// Perform conflict check by moving forward in time at increments of the smallest time step
// Test the efficiency of VO vs. time-vector approach
void GetMDD(xyLoc const& start, xyLoc const& end, DAG& dag, MDD& mdd, int depth){
  // TODO: make this contain pointers or maybe combine into a structure with MDD.
  //       as-is, memory will be deallocated when this function exits.
  mdd.resize(depth+1);
  LimitedDFS(start,end,dag,mdd,depth,depth);
  //for(auto const& a: mdd){
    //for(auto const& n: a){
      //std::cout << *n;
    //}
    //std::cout << "\n";
  //}
  // Fill in any extra levels necessary to get to depth
  for(int i(1); i<mdd.size(); ++i){
    if(mdd[i].size()==0){
      Node* n(new Node((*mdd[i-1].begin())->n,(*mdd[i-1].begin())->depth+1));
      (*mdd[i-1].begin())->successors.insert(n);
      n->parents.insert(*mdd[i-1].begin());
      mdd[i].insert(n);
    }
  }
}

void generatePermutations2(std::vector<MultiState>& positions, std::vector<MultiState>& result, int depth, MultiState const& parent, MultiState const& current) {
  if(depth == positions.size()) {
    for(int i(0); i<current.size(); ++i){
      for(int j(i+1); j<current.size(); ++j){
        Vector2D A(parent[i]->n.x,parent[i]->n.y);
        Vector2D B(parent[j]->n.x,parent[j]->n.y);
        Vector2D VA(current[i]->n.x-parent[i]->n.x,current[i]->n.y-parent[i]->n.y);
        Vector2D VB(current[j]->n.x-parent[j]->n.x,current[j]->n.y-parent[j]->n.y);
        if(collisionImminent(A,VA,.25,parent[i]->n.depth,current[i]->n.depth,B,VB,.25,parent[j]->n.depth,current[j]->n.depth))
          return; // This is an edge collision
      }
    }
    result.push_back(current);
    return;
  }

  for(int i = 0; i < positions[depth].size(); ++i) {
    MultiState copy(current);
    bool found(false);
    for(auto const& p: current){
      if(positions[depth][i]->n.x==p->n.x && positions[depth][i]->n.y==p->n.y){
        found=true;
        break;
      }
    }
    if(found) continue;
    copy.push_back(positions[depth][i]);
    generatePermutations2(positions, result, depth + 1, parent, copy);
  }
}

void generatePermutations(std::vector<MultiState>& positions, std::vector<MultiState>& result, int depth, MultiState const& parent, MultiState const& current) {
  if(depth == positions.size()) {
    for(int i(0); i<current.size(); ++i){
      for(int j(i+1); j<current.size(); ++j){
        if(current[j]->n.x==parent[i]->n.x and parent[j]->n.y==current[i]->n.y
            && current[i]->n.x==parent[j]->n.x and parent[i]->n.y==current[j]->n.y)
          return; // This is an edge collision
      }
    }
    result.push_back(current);
    return;
  }

  for(int i = 0; i < positions[depth].size(); ++i) {
    MultiState copy(current);
    bool found(false);
    for(auto const& p: current){
      if(positions[depth][i]->n.x==p->n.x && positions[depth][i]->n.y==p->n.y){
        found=true;
        break;
      }
    }
    if(found) continue;
    copy.push_back(positions[depth][i]);
    generatePermutations(positions, result, depth + 1, parent, copy);
  }
}

// Return true if we get to the desired depth
bool jointDFS(MultiState const& s, int d, int term){
  //std::cout << std::string(d,' ')<< " s " << s << "\n";
  if(d==term){
    return true;
  }
  //Get successors into a vector
  std::vector<MultiState> successors;
  for(auto const& a: s){
    MultiState output(a->successors.begin(), a->successors.end()); 
    successors.push_back(output);
  }
  std::vector<MultiState> crossProduct;
  generatePermutations(successors,crossProduct,0,s,MultiState());
  bool value(false);
  for(auto const& a: crossProduct){
    value |= jointDFS(a,d+1,term);
  }
  return value;
}

struct ICTSNode{
  ICTSNode(int agents):mdd(agents){}
  ICTSNode(std::vector<int> s):dag(s.size()),mdd(s.size()),sizes(s){
    for(int i(0); i<starts.size(); ++i){
      //std::cout << "agent " << i << " GetMDD()\n";
      GetMDD(starts[i],ends[i],dag[i],mdd[i],Node::env->HCost(starts[i],ends[i])+sizes[i]);
    }
  }
  std::vector<DAG> dag;
  std::vector<MDD> mdd;
  std::vector<int> sizes;
  static std::vector<xyLoc> starts;
  static std::vector<xyLoc> ends;

  bool isValid(){
    // Extend all goal nodes out to the same depth by adding wait actions.
    int maxmdd(INT_MIN);
    MultiState root;
    for(auto const& m: mdd){
      if(m.empty()){return false;} // This is an invalid solution
      maxmdd=max(maxmdd,m.size());
      root.push_back(*m[0].begin());
    }
    //std::cout << "maxmdd " << maxmdd << "\n";
    //std::cout << "root " << root << "\n";
    // Lengthen out MDDs to the same length
    int agent(0);
    for(auto & m: mdd){
      //std::cout << "agent " << agent++ << " sz " << m.size() << "\n";
      while(m.size()<maxmdd){
        std::unordered_set<Node*> b;
        std::cout << "appending " << m.back().size() << "\n";
        Node* n(new Node((*m.back().begin())->n,(*m.back().begin())->depth+1));
        (*m.back().begin())->successors.insert(n);
        n->parents.insert(*m.back().begin());
        b.insert(n);
        m.push_back(b);
      }
    }
    // Do a depth-first search; if the search terminates at a goal, its valid.
    bool result(jointDFS(root,1,maxmdd));
    if(result){
      std::cout << "Found:\n";
      int i(0);
      for(auto & m: mdd){
        std::cout << ++i << "\n";
        for(auto & t: m){
          for(auto & p: t)
            std::cout << *p;
          std::cout << "\n";
        }
        std::cout << "\n";
      }
    }
    return result;
  }

  int SIC()const{
    int total(0);
    for(auto const& s:sizes){
      total += s;
    }
    return total;
  }
};

struct ICTSNodePtrComp
{
  bool operator()(const ICTSNode* lhs, const ICTSNode* rhs) const  { return lhs->SIC()>rhs->SIC(); }
};

std::vector<xyLoc> ICTSNode::starts;
std::vector<xyLoc> ICTSNode::ends;

std::priority_queue<ICTSNode*,std::vector<ICTSNode*>,ICTSNodePtrComp> q;
std::unordered_set<std::string> deconf;

int main(){
  MapEnvironment env(new Map(8,8));
  env.SetFiveConnected();
  Node::env=&env;
  int numAgents(15);
  std::set<xyLoc> st;
  std::set<xyLoc> en;
  int seed(1234456);
  srand(seed);
  for(int i(0);i<numAgents;++i){
    while(!st.emplace(rand()%8,rand()%8).second);
    while(!en.emplace(rand()%8,rand()%8).second);
  }
  ICTSNode::starts=std::vector<xyLoc>(st.begin(),st.end());
  ICTSNode::ends=std::vector<xyLoc>(en.begin(),en.end());
  std::vector<int> sizes(ICTSNode::ends.size());

  q.push(new ICTSNode(sizes));
  while(q.size()){
    ICTSNode* parent(q.top());
    std::cout << "pop ";
    for(auto const& a: parent->sizes){
      std::cout << a << " ";
    }
    std::cout << "\n";
    q.pop();
    std::cout << parent->SIC() << "\n";
    if(parent->isValid()){
      return 0;
    }
    for(int i(0); i<parent->sizes.size(); ++i){
      std::vector<int> s(parent->sizes);
      s[i]++;
      //std::cout << "push ";
      //for(auto const& a: s){
        //std::cout << a << " ";
      //}
      //std::cout << "\n";
      std::string sv = std::accumulate(s.begin()+1, s.end(), std::to_string(s[0]),
                     [](const std::string& a, int b){
                           return a + ',' + std::to_string(b);
                     });
      if(deconf.find(sv)==deconf.end()){
        q.push(new ICTSNode(s));
        deconf.insert(sv);
      }else{
        //std::cout << "duplicate: " << sv << "\n";
      }
    }
  }
  return 1;
}
