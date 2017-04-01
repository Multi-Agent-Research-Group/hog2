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
        bool connected()const{return parents.size()+successors.size();}
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
    std::unordered_set<Node*>& s=mdd[maxDepth-(depth)];
    s.insert(&dag[hash]);
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

void GetMDD(xyLoc const& start, xyLoc const& end, MDD& mdd, int depth){
  // TODO: make this contain pointers or maybe combine into a structure with MDD.
  //       as-is, memory will be deallocated when this function exits.
  DAG* dag=new DAG();
  mdd.resize(depth+1);
  LimitedDFS(start,end,*dag,mdd,depth,depth);
  for(auto const& a: mdd){
    for(auto const& n: a){
      std::cout << *n;
    }
    std::cout << "\n";
  }
}

void eraseDown(Node* n){
  // If no parent points here, then we must disconnect this node
  if(n->parents.empty()){
    // Erase children's parent pointers terminating at this node
    for(auto& m: n->successors){
      auto p(m->parents.find(n));
      if(p!=m->parents.end()){
        m->parents.erase(p);
      }
      // recurse
      eraseDown(m);
    }
    n->successors.clear();
  }
}

void eraseUp(Node* n){
  // If this node has no successors, it cannot connect us to the goal
  if(n->successors.empty()){
    // Erase parent's successor pointers terminating at this node
    for(auto& m: n->parents){
      auto p(m->successors.find(n));
      if(p!=m->successors.end()){
        m->successors.erase(p);
      }
      // recurse
      eraseUp(m);
    }
    n->parents.clear();
  }
}

void disconnect(Node* n){
  // Erase edges pointing to parents, and their edges pointing at me.
  for(auto& m: n->parents){
    auto p(m->successors.find(n));
    if(p!=m->successors.end()){
      m->successors.erase(p);
    }
    eraseUp(m);
  }
  n->parents.clear();

  // Erase edges to/from successors
  for(auto& m: n->successors){
    auto p(m->parents.find(n));
    if(p!=m->parents.end()){
      m->parents.erase(p);
    }
    // recurse
    eraseDown(m);
  }
  n->successors.clear();
}

struct ICTSNode{
  ICTSNode(int agents):mdd(agents){}
  std::vector<MDD> mdd;
  // Returns true and sets a1, a2 on conflict
  bool CheckConflicts(int& a1, int& a2){
    std::vector<MDD> temp(mdd);
    // Check all pairs
    for(int i(0); i<temp.size(); ++i){
      for(int j(i+1); j<temp.size(); ++j){
        // Check all times
        for(int t(0); t<std::min(temp[i].size(),temp[j].size()); ++t){
          // Check for node and edge conflicts
          for(auto const m: temp[i][t]){
            if(!m->connected()) continue; // Can't get here
            for(auto const n: temp[j][t]){
              if(!n->connected()) continue; // Can't get here
              if(m->Hash() == n->Hash()){
                disconnect(m);
                disconnect(n);
                continue;
              }
              if(t!=0){ // Check edges
                for(auto mp(m->parents.begin()); mp!=m->parents.end();/*++mp*/){
                  bool conflict(false);
                  for(auto np(n->parents.begin()); np!=n->parents.end();/*++np*/){
                    if((*mp)->n==n->n && (*np)->n==m->n){ // Same edge traversal
                      // Remove edge
                      // 1. Erase my pointer to parent
                      n->parents.erase(np);
                      // 2. Erase parent's pointer to me
                      auto p((*np)->successors.find(n));
                      if(p!=(*np)->successors.end()){
                        (*np)->successors.erase(p);
                      }
                      eraseUp(*np); // If severed, propagate
                      eraseDown(n);
                      conflict=true;
                      break;
                    }else{
                      ++np;
                    }
                  }
                  if(conflict){
                      m->parents.erase(mp);
                      auto p((*mp)->successors.find(m));
                      if(p!=(*mp)->successors.end()){
                        (*mp)->successors.erase(p);
                      }
                      // recurse
                      eraseUp(*mp); // If severed, propagate
                      eraseDown(m);
                  }else{
                    ++mp;
                  }
                }
              }
            }
          }
          // Finally, check for severed connectivity by checking across this time slice.
          // If all nodes have no parents or children, then this ICTS node is rendered infeasible
          bool connected(false);
          for(auto const& m: temp[i][t]){
            connected|=m->connected();
          }
          if(!connected){
            a1=i;
            a2=j;
            return false;
          }
          connected=false;
          for(auto const& n: temp[j][t]){
            connected|=n->connected();
          }
          if(!connected){
            a1=i;
            a2=j;
            return false;
          }
        }
      }
    }
    a1=-1;
    a2=-1;
    return true;
  }
};

void testErasers(){
  Node a(xyLoc(1,1),1);
  Node b(xyLoc(2,1),2);
  Node c(xyLoc(3,1),2);
  Node d(xyLoc(4,1),2);
  Node b1(xyLoc(5,1),3);
  Node c1(xyLoc(6,1),3);
  Node d1(xyLoc(7,1),3);
  Node e(xyLoc(8,1),4);

  a.successors.insert(&b);
  a.successors.insert(&c);
  a.successors.insert(&d);
  b.parents.insert(&a);
  c.parents.insert(&a);
  d.parents.insert(&a);
  b.successors.insert(&b1);
  c.successors.insert(&c1);
  d.successors.insert(&d1);
  b1.parents.insert(&b);
  c1.parents.insert(&c);
  d1.parents.insert(&d);
  b1.successors.insert(&e);
  c1.successors.insert(&e);
  d1.successors.insert(&e);
  e.parents.insert(&b1);
  e.parents.insert(&c1);
  e.parents.insert(&d1);

  std::cout << &a << "\n";

  // Disconnect b
  disconnect(&b);
  std::cout << &a << "\n";

  std::cout << "b " << b.connected() << "\n";
  std::cout << "b1 " << b1.connected() << "\n";
  
}

int main(){
  MapEnvironment env(new Map(8,8));
  env.SetFiveConnected();
  Node::env=&env;

  //testErasers();

  xyLoc s1(1,1);
  xyLoc e1(1,4);
  xyLoc s2(1,4);
  xyLoc e2(3,1);
  int agents(2);
  ICTSNode n(agents);
  GetMDD(s1,e1,n.mdd[0],Node::env->HCost(s1,e1));
  GetMDD(s2,e2,n.mdd[1],Node::env->HCost(s2,e2));
  //std::cout << &(n.mdd[0][0]) << "\n"; 
  //std::cout << &(n.mdd[1][0]) << "\n"; 
  int a1(-1);
  int a2(-1);
  std::cout << "satisfiable=" << (n.CheckConflicts(a1,a2)?"true":"false") << "\n";
  return 0;
}
