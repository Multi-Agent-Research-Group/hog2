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
	std::unordered_set<Node*> parents;
	std::unordered_set<Node*> successors;
	virtual uint64_t Hash()const{return (env->GetStateHash(n)<<16) | depth;}
};

typedef std::vector<std::unordered_set<Node*>> MDD;
typedef std::unordered_map<uint64_t,Node> DAG;

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

bool LimitedDFS(xyLoc const& start, xyLoc const& end, DAG& dag, int depth){
  //std::cout << start << " " << depth << "\n";
  if(depth==0)
    return false;

  Node* parent(nullptr);
  uint64_t hash(Node(start,depth).Hash());
  if(dag.find(hash)==dag.end())
    dag[hash]=Node(start,depth);
  else if(dag[hash].optimal)
    return true; // Already found a solution from search at this depth
  parent=&dag[hash];
    
  if(Node::env->GoalTest(end,start)){
    //std::cout << "found " << start << "\n";
    return true;
  }

  std::vector<xyLoc> successors;
  Node::env->GetSuccessors(start,successors);
  bool result(false);
  for(auto const& node: successors){
    if(LimitedDFS(node,end,dag,depth-1)){
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

/*void print(xyLoc const& s, uint64_t sdepth)
{
  std::queue<Node*> q;
  q.push(&dag[Node(s,sdepth).Hash()]);
  int d(sdepth);
  while(q.size()){
    Node* n(q.front());
    if(n->depth<d){
      d--;
      std::cout << "\n";
    }
    q.pop();
    std::cout << *n;
    for(auto const m: n->successors)
      q.push(m);
  }
}*/

// Reverse the search depths and store in the mdd
void buildMDD(Node* n, MDD& mdd){
  //std::cout << "depth " << (n->depth-1) << "\n";
  std::unordered_set<Node*>& s=mdd[n->depth-1];
  //std::cout << *n << "\n";
  s.insert(n);
  for(auto const m: n->successors)
    buildMDD(m,mdd);
}

void getMDD(xyLoc const& s, xyLoc const& e, DAG& dag, MDD& mdd){
  int sdepth(Node::env->HCost(s,e)+1);
  LimitedDFS(s,e,dag,sdepth);
  mdd.resize(sdepth);
  buildMDD(&dag[Node(s,sdepth).Hash()],mdd);
}

int main(){
  DAG dag;
  MDD mdd;

  MapEnvironment env(new Map(8,8));
  env.SetFiveConnected();
  Node::env=&env;

  xyLoc s(1,1);
  xyLoc e(5,5);
  getMDD(s,e,dag,mdd);
  
  for(auto const& a: mdd){
    for(auto const& n: a){
      std::cout << *n;
    }
    std::cout << "\n";
  }
  //std::cout << &dag[Node(s,sdepth).Hash()];
  //std::cout << dag[Node(s,sdepth).Hash()].successors.size();
  return 0;
}
