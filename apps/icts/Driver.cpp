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
#include "VelocityObstacle.h"


struct Hashable{
  virtual uint64_t Hash()const=0;
  virtual float Depth()const=0;
};

// Used for std::set
struct NodePtrComp
{
  bool operator()(const Hashable* lhs, const Hashable* rhs) const  { return fless(lhs->Depth(),rhs->Depth()); }
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
	Node(xyLoc a, float d):n(a),depth(d),optimal(false){}
	xyLoc n;
	float depth;
        bool optimal;
        //bool connected()const{return parents.size()+successors.size();}
	//std::unordered_set<Node*> parents;
	std::unordered_set<Node*> successors;
	virtual uint64_t Hash()const{return (env->GetStateHash(n)<<32) | ((uint32_t)floor(depth*1000.));}
	virtual float Depth()const{return depth; }
};

typedef std::vector<Node*> MultiState; // rank=agent num
typedef std::unordered_map<uint64_t,Node> DAG;

std::ostream& operator << (std::ostream& ss, MultiState const& n){
  int i(0);
  for(auto const& a: n)
    ss << " "<<++i<<"." << a->n << "@" << a->depth;
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const& n){
  ss << n.n << "@" << n.depth;
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const* n){
  ss << std::string(n->depth,' ')<<n->n << "_" << n->depth << "\n";
  for(auto const& m: n->successors)
    ss << m;
  return ss;
}

MapEnvironment* Node::env=nullptr;

bool LimitedDFS(xyLoc const& start, xyLoc const& end, DAG& dag, MultiState& root, float depth, float maxDepth){
  //std::cout << start << "-->" << end << " g:" << (maxDepth-depth) << " h:" << Node::env->HCost(start,end) << " f:" << ((maxDepth-depth)+Node::env->HCost(start,end)) << "\n";
  if(fless(depth,0) ||
      fgreater(maxDepth-depth+Node::env->HCost(start,end),maxDepth)){ // Note - this only works for a perfect heuristic.
    //std::cout << "pruned\n";
    return false;
  }

  if(Node::env->GoalTest(end,start)){
    Node n(start,maxDepth-depth);
    uint64_t hash(n.Hash());
    dag[hash]=n;
    // This may happen if the agent starts at the goal
    if(fleq(maxDepth-depth,0)){
      root.push_back(&dag[hash]);
      //std::cout << "root_ " << &dag[hash];
    }
    Node* parent(&dag[hash]);
    float d(maxDepth-depth);
    while(fless(d++,maxDepth)){
      // Wait at goal
      Node current(start,d);
      uint64_t chash(current.Hash());
      dag[chash]=current;
      //std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      parent->successors.insert(&dag[chash]);
      //dag[chash].parents.insert(parent);
      parent=&dag[chash];
    }
    return true;
  }

  std::vector<xyLoc> successors;
  Node::env->GetSuccessors(start,successors);
  bool result(false);
  for(auto const& node: successors){
    float ddiff(1.0);
    if(abs(node.x-start.x)>=1 && abs(node.y-start.y)>=1){
      ddiff = M_SQRT2;
    }
    if(LimitedDFS(node,end,dag,root,depth-ddiff,maxDepth)){
      Node n(start,maxDepth-depth);
      uint64_t hash(n.Hash());
      if(dag.find(hash)==dag.end()){
        dag[hash]=n;
        // This is the root if depth=0
        if(fleq(maxDepth-depth,0)){
          root.push_back(&dag[hash]);
          //std::cout << "_root " << &dag[hash];
        }
        //if(fequal(maxDepth-depth,0.0))root.push_back(&dag[hash]);
      }else if(dag[hash].optimal){
        return true; // Already found a solution from search at this depth
      }

      Node* parent(&dag[hash]);

      //std::cout << "found " << start << "\n";
      uint64_t chash(Node(node,maxDepth-depth+ddiff).Hash());
      if(dag.find(chash)==dag.end()){
        assert(!"Uh oh, node not already in the DAG!");
        //std::cout << "Add new.\n";
        Node c(node,maxDepth-depth+ddiff);
        dag[chash]=c;
      }
      Node* current(&dag[chash]);
      current->optimal = result = true;
      //std::cout << *parent << " parent of " << *current << "\n";
      //dag[current->Hash()].parents.insert(parent);
      //std::cout << *current << " child of " << *parent << "\n";
      //std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      dag[parent->Hash()].successors.insert(&dag[current->Hash()]);
      //std::cout << "at" << &dag[parent->Hash()] << "\n";
    }
  }
  return result;
}

// Perform conflict check by moving forward in time at increments of the smallest time step
// Test the efficiency of VO vs. time-vector approach
void GetMDD(xyLoc const& start, xyLoc const& end, DAG& dag, MultiState& root, float depth){
  LimitedDFS(start,end,dag,root,depth,depth);
}

void generatePermutations(std::vector<MultiState>& positions, std::vector<MultiState>& result, int agent, MultiState const& parent, MultiState const& current) {
  if(agent == positions.size()) {
    result.push_back(current);
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    //std::cout << "AGENT "<< i<<":\n";
    MultiState copy(current);
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      Vector2D A(parent[agent]->n.x,parent[agent]->n.y);
      Vector2D B(parent[j]->n.x,parent[j]->n.y);
      Vector2D VA(positions[agent][i]->n.x-parent[agent]->n.x,positions[agent][i]->n.y-parent[agent]->n.y);
      Vector2D VB(current[j]->n.x-parent[j]->n.x,current[j]->n.y-parent[j]->n.y);
      if(collisionImminent(A,VA,.25,parent[agent]->depth,positions[agent][i]->depth,B,VB,.25,parent[j]->depth,current[j]->depth)){
        found=true;
        break;
      }
    }
    if(found) continue;
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, parent, copy);
  }
}

// Return true if we get to the desired depth
bool jointDFS(MultiState const& s, float d, float term, std::vector<std::set<Node*,NodePtrComp>>& answer){
  //std::cout << d << std::string((int)d,' ')<< " s " << s << "\n";
  if(fgeq(d,term)){
    for(int a(0); a<s.size(); ++a){
      //std::cout << "push " << *s[a] << " " << s.size() << "\n";
      answer[a].insert(s[a]);
    }
    return true;
  }
  //Get successors into a vector
  std::vector<MultiState> successors;
  float sd(9999999.0);
  for(auto const& a: s){
    sd=min(sd,a->depth);
  }

  float md(-9999999.0);
  //Add in successors for parents who are equal to the min
  for(auto const& a: s){
    MultiState output;
    if(fleq(a->depth,sd)){
      for(auto const& b: a->successors){
        output.push_back(b);
        md=max(md,b->depth);
      }
    }else{
      output.push_back(a);
      md=max(md,a->depth);
    }
    if(output.empty()){
      // Stay at state...
      output.push_back(new Node(a->n,a->depth+1.0));
      md=max(md,a->depth+1.0);
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
  }
  std::vector<MultiState> crossProduct;
  generatePermutations(successors,crossProduct,0,s,MultiState());
  bool value(false);
  for(auto const& a: crossProduct){
    //std::cout << "eval " << a << "\n";
    value = jointDFS(a,md,term,answer);
    if(value){
      for(int a(0); a<s.size(); ++a){
        //std::cout << "push " << *s[a] << "\n";
        answer[a].insert(s[a]);
      }
      return true;
    }
  }
  return value;
}

// Not part of the algorithm... just for validating the answers
bool checkAnswer(std::vector<std::set<Node*,NodePtrComp>> const& answer){
  for(int i(0);i<answer.size();++i){
    for(int j(i+1);j<answer.size();++j){
      auto ap(answer[i].begin());
      auto bp(answer[j].begin());
      auto a(answer[i].begin());
      auto b(answer[j].begin());
      a++;b++;
      while(a!=answer[i].end() && b!=answer[j].end()){
        Vector2D A((*ap)->n.x,(*ap)->n.y);
        Vector2D B((*bp)->n.x,(*bp)->n.y);
        Vector2D VA((*a)->n.x-(*ap)->n.x,(*a)->n.y-(*ap)->n.y);
        Vector2D VB((*b)->n.x-(*bp)->n.x,(*b)->n.y-(*bp)->n.y);
        if(collisionImminent(A,VA,.25,(*ap)->depth,(*a)->depth,B,VB,.25,(*bp)->depth,(*b)->depth)){
          std::cout << "Collision: " << i << ":" << **ap << "-->" << **a << "," << j << ":" << **bp << "-->" << **b << "\n";
          return false;
        }
        if(fless((*a)->depth,(*b)->depth)){
          ++a;
          ++ap;
        }else if(fgreater((*a)->depth,(*b)->depth)){
          ++b;
          ++bp;
        }else{
          ++a;++b;
          ++ap;++bp;
        }
      }
    }
  }
}

struct ICTSNode{
  ICTSNode(std::vector<int> s):dag(s.size()),sizes(s),maxdepth(-99999999){
    root.reserve(s.size());
    for(int i(0); i<starts.size(); ++i){
      maxdepth=max(maxdepth,Node::env->HCost(starts[i],ends[i])+sizes[i]);
      //std::cout << "agent " << i << " GetMDD()\n";
      GetMDD(starts[i],ends[i],dag[i],root,Node::env->HCost(starts[i],ends[i])+sizes[i]);
      //std::cout << i << ":\n" << root[i] << "\n";
    }
  }
  std::vector<DAG> dag;
  std::vector<int> sizes;
  MultiState root;
  float maxdepth;
  static std::vector<xyLoc> starts;
  static std::vector<xyLoc> ends;

  bool isValid(){
    // Do a depth-first search; if the search terminates at a goal, its valid.
    std::vector<std::set<Node*,NodePtrComp>> answer(sizes.size());
    float sd(1.0);
    for(auto const& n:root){
      for(auto const& m:n->successors){
        sd=min(sd,m->depth);
      }
    }

    if(jointDFS(root,sd,maxdepth,answer)){// && checkAnswer(answer)){
      checkAnswer(answer);
      std::cout << "Answer:\n";
      for(int agent(0); agent<answer.size(); ++agent){
        std::cout << agent << ":\n";
        for(auto a(answer[agent].begin()); a!=answer[agent].end(); ++a){
          std::cout << std::string((*a)->depth,' ') << **a << "\n";
        }
        std::cout << "\n";
      }
      std::cout << std::endl;
      return true;
    }
    
    return false;
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
  env.SetNineConnected();
  Node::env=&env;
  int numAgents(6);
  std::set<xyLoc> st;
  std::set<xyLoc> en;
  std::vector<xyLoc> s;
  std::vector<xyLoc> e;
  int seed(123456);
  srand(seed);
  // Get disjoint start and goal locations
  for(int i(0);i<numAgents;++i){
    auto a(st.emplace(rand()%8,rand()%8));
    while(!a.second){
      a=st.emplace(rand()%8,rand()%8);
    }
    s.push_back(*a.first);

    a=en.emplace(rand()%8,rand()%8);
    while(!a.second){
      a=en.emplace(rand()%8,rand()%8);
    }
    while(!a.second);
    e.push_back(*a.first);
  }
  ICTSNode::starts=std::vector<xyLoc>(s.begin(),s.end());
  ICTSNode::ends=std::vector<xyLoc>(e.begin(),e.end());
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
    std::cout << "SIC" << parent->SIC() << "\n";
    if(parent->isValid()){
      return 0;
    }
    for(int i(0); i<parent->sizes.size(); ++i){
      std::vector<int> s(parent->sizes);
      s[i]++;
      std::string sv = std::accumulate(s.begin()+1, s.end(), std::to_string(s[0]),
                     [](const std::string& a, int b){
                           return a + ',' + std::to_string(b);
                     });
      if(deconf.find(sv)==deconf.end()){
        q.push(new ICTSNode(s));
        deconf.insert(sv);
      }
    }
  }
  return 1;
}
