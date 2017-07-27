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
#include <iomanip>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include "Map2DEnvironment.h"
#include "VelocityObstacle.h"
#include "TemplateAStar.h"

int renderScene(){}

struct Group{
  Group(int i){agents.insert(i);}
  std::unordered_set<int> agents;
  //~Group(){std::cout << "Destroy " << this << "\n";}
};

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

//std::unordered_set<uint64_t> checked;
//uint64_t EdgeHash(std::pair<Node*,Node*> const& edge){
  //return (edge.first->Hash() * 16777619) ^ edge.second->Hash();
//}
//uint64_t EdgePairHash(std::pair<Node*,Node*> const& edge1, std::pair<Node*,Node*> const& edge2){
  //return (EdgeHash(edge1) * 16777619) ^ EdgeHash(edge2);
//}

typedef std::vector<xyLoc> Points;
typedef std::pair<Points,Points> Instance;
typedef std::vector<std::pair<xyLoc,float>> TimePath;
typedef std::vector<TimePath> Solution;

typedef std::vector<Node*> MultiState; // rank=agent num
typedef std::vector<std::pair<Node*,Node*>> MultiEdge; // rank=agent num
typedef std::unordered_map<uint64_t,Node> DAG;

std::ostream& operator << (std::ostream& ss, Group const* n){
  std::string sep("{");
  for(auto const& a: n->agents){
    ss << sep << a;
    sep=",";
  }
  ss << "}";
  return ss;
}

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
  ss << std::string(n->depth,' ')<<n->n << "_" << n->depth << std::endl;
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

  Points successors;
  Node::env->GetSuccessors(start,successors);
  bool result(false);
  for(auto const& node: successors){
    float ddiff(std::max(Util::distance(node.x,node.y,start.x,start.y),1.0));
    //if(abs(node.x-start.x)>=1 && abs(node.y-start.y)>=1){
      //ddiff = M_SQRT2;
    //}
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

void generatePermutations(std::vector<MultiEdge>& positions, std::vector<MultiEdge>& result, int agent, MultiEdge const& current) {
  if(agent == positions.size()) {
    result.push_back(current);
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    //std::cout << "AGENT "<< i<<":\n";
    MultiEdge copy(current);
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      //uint64_t hash(EdgePairHash(positions[agent][i],current[j]));
      //if(checked.find(hash)!=checked.end())
      //{std::cout << "SKIPPED " << *positions[agent][i].second << " " << *current[j].second << "\n"; continue; /*No collision check necessary; checked already*/}
      //std::cout << "COMPARE " << *positions[agent][i].second << " " << *current[j].second << "\n";
      Vector2D A(positions[agent][i].first->n.x,positions[agent][i].first->n.y);
      Vector2D B(current[j].first->n.x,current[j].first->n.y);
      Vector2D VA(positions[agent][i].second->n.x-positions[agent][i].first->n.x,positions[agent][i].second->n.y-positions[agent][i].first->n.y);
      Vector2D VB(current[j].second->n.x-current[j].first->n.x,current[j].second->n.y-current[j].first->n.y);
      //std::cout << "Test for collision: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
      if(collisionImminent(A,VA,.25,positions[agent][i].first->depth,positions[agent][i].second->depth,B,VB,.25,current[j].first->depth,current[j].second->depth)){
        //std::cout << "Collision averted: " << A << " " << B << "\n";
        found=true;
        //checked.insert(hash);
        break;
      }
    }
    if(found) continue;
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy);
  }
}

// In order for this to work, we cannot generate sets of positions, we must generate sets of actions, since at time 1.0 an action from parent A at time 0.0 may have finished, while another action from the same parent A may still be in progress. 

// Return true if we get to the desired depth
bool jointDFS(MultiEdge const& s, float d, float term, std::vector<std::set<Node*,NodePtrComp>>& answer,std::vector<Node*>& toDelete,double increment=1.0){
  //std::cout << d << std::string((int)d,' ');
  //for(int a(0); a<s.size(); ++a){
    //std::cout << " s " << *s[a].second << "\n";
  //}
  if(fgreater(d,term-increment)){
    for(int a(0); a<s.size(); ++a){
      //std::cout << "push " << *s[a].second << " " << s.size() << "\n";
      answer[a].insert(s[a].second);
    }
    return true;
  }
  //Get successors into a vector
  std::vector<MultiEdge> successors;

  // Find minimum depth of current edges
  float sd(9999999.0);
  for(auto const& a: s){
    sd=min(sd,a.second->depth);
  }
  //std::cout << "min-depth: " << sd << "\n";

  float md(9999999.0);
  //Add in successors for parents who are equal to the min
  for(auto const& a: s){
    MultiEdge output;
    if(fleq(a.second->depth,sd)){
      //std::cout << "Keep Successors of " << *a.second << "\n";
      for(auto const& b: a.second->successors){
        output.emplace_back(a.second,b);
        md=min(md,b->depth);
      }
    }else{
      //std::cout << "Keep Just " << *a.second << "\n";
      output.push_back(a);
      md=min(md,a.second->depth);
    }
    if(output.empty()){
      // Stay at state...
      output.emplace_back(a.second,new Node(a.second->n,a.second->depth+increment));
      toDelete.push_back(output.back().second);
      md=min(md,a.second->depth+increment); // Amount of time to wait
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
  }
  //for(int agent(0); agent<successors.size(); ++agent){
    //std::cout << "Agent: " << agent << "\n\t";
    //for(int succ(0); succ<successors[agent].size(); ++succ)
      //std::cout << *successors[agent][succ].second << ",";
    //std::cout << std::endl;
  //}
  std::vector<MultiEdge> crossProduct;
  generatePermutations(successors,crossProduct,0,MultiEdge());
  bool value(false);
  for(auto const& a: crossProduct){
    //std::cout << "eval " << a << "\n";
    value = jointDFS(a,md,term,answer,toDelete,increment);
    if(value){
      for(int a(0); a<s.size(); ++a){
        //std::cout << "push " << *s[a] << "\n";
        answer[a].insert(s[a].second);
      }
      return true;
    }
  }
  return value;
}

bool jointDFS(MultiState const& s, float maxdepth, std::vector<std::set<Node*,NodePtrComp>>& answer, std::vector<Node*>& toDelete, double increment=1.0){
  MultiEdge act;
  float sd(1.0);
  for(auto const& n:s){ // Add null parents for the initial movements
    act.emplace_back(nullptr,n);
    /*for(auto const& m:n->successors){
      sd=min(sd,m->depth);
    }*/
    //act.push_back(a);
  }

  return jointDFS(act,0.0,maxdepth,answer,toDelete,increment);
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
          //std::cout << "Collision: " << i << ":" << **ap << "-->" << **a << "," << j << ":" << **bp << "-->" << **b << "\n";
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
  return true;
}

struct ICTSNode{
  ICTSNode(ICTSNode* parent,int agent, int size):instance(parent->instance),dag(parent->dag),sizes(parent->sizes),root(parent->root),maxdepth(parent->maxdepth),increment(parent->increment){
    sizes[agent]=size;
    maxdepth=max(maxdepth,Node::env->HCost(instance.first[agent],instance.second[agent])+sizes[agent]);
    //std::cout << "agent " << agent << " GetMDD("<<(Node::env->HCost(instance.first[agent],instance.second[agent])+sizes[agent])<<")\n";
    dag[agent].clear();
    GetMDD(instance.first[agent],instance.second[agent],dag[agent],root,Node::env->HCost(instance.first[agent],instance.second[agent])+sizes[agent]);
    // Replace new root node on top of old.
    std::swap(root[agent],root[root.size()-1]);
    root.resize(root.size()-1);
  }

  ICTSNode(Instance const& inst, std::vector<float> const& s, double inc=1.0):instance(inst),dag(s.size()),sizes(s),maxdepth(-99999999),increment(inc){
    root.reserve(s.size());
    for(int i(0); i<instance.first.size(); ++i){
      maxdepth=max(maxdepth,Node::env->HCost(instance.first[i],instance.second[i])+sizes[i]);
      //std::cout << "agent " << i << " GetMDD("<<(Node::env->HCost(instance.first[i],instance.second[i])+sizes[i])<<")\n";
      GetMDD(instance.first[i],instance.second[i],dag[i],root,Node::env->HCost(instance.first[i],instance.second[i])+sizes[i]);
      //std::cout << i << ":\n" << root[i] << "\n";
    }
  }

  ~ICTSNode(){for(auto d:toDelete){delete d;}}

  Instance instance;
  std::vector<DAG> dag;
  std::vector<float> sizes;
  MultiState root;
  float maxdepth;
  double increment;
  Instance points;
  std::vector<Node*> toDelete;

  bool isValid(std::vector<std::set<Node*,NodePtrComp>>& answer){
    // Do a depth-first search; if the search terminates at a goal, its valid.
    answer.resize(sizes.size());
    if(jointDFS(root,maxdepth,answer,toDelete,increment)){// && checkAnswer(answer)){
      //assert(checkAnswer(answer));
      //std::cout << "Answer:\n";
      //for(int agent(0); agent<answer.size(); ++agent){
        //std::cout << agent << ":\n";
        //for(auto a(answer[agent].begin()); a!=answer[agent].end(); ++a){
          //std::cout << std::string((*a)->depth,' ') << **a << "\n";
        //}
        //std::cout << "\n";
      //}
      //std::cout << std::endl;
      return true;
    }
    
    return false;
  }

  float SIC()const{
    float total(0);
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

void join(std::stringstream& s, std::vector<float> const& x){
  copy(x.begin(),x.end(), std::ostream_iterator<float>(s,","));
}

bool detectIndependence(Solution const& solution, std::vector<Group*>& group, std::unordered_set<Group*>& groups){
  bool independent(true);
  // Check all pairs for collision
  for(int i(0); i<solution.size(); ++i){
    for(int j(i+1); j<solution.size(); ++j){
      if(group[i]==group[j]) continue; // This can happen if both collide with a common agent
      // check collision between i and j
      int a(1);
      int b(1);
      if(solution[i].size() > a && solution[j].size() > b){
        float t(min(solution[i][a].second,solution[j][b].second));
        bool collision(false);
        while(1){
          if(a==solution[i].size() || b==solution[j].size()){break;}
          Vector2D A(solution[i][a-1].first.x,solution[i][a-1].first.y);
          Vector2D B(solution[j][b-1].first.x,solution[j][b-1].first.y);
          Vector2D VA(solution[i][a].first.x-solution[i][a-1].first.x,solution[i][a].first.y-solution[i][a-1].first.y);
          Vector2D VB(solution[j][b].first.x-solution[j][b-1].first.x,solution[j][b].first.y-solution[j][b-1].first.y);
          if(collisionImminent(A,VA,.25,solution[i][a-1].second,solution[i][a].second,B,VB,.25,solution[j][b-1].second,solution[j][b].second)){
            collision=true;
            //std::cout << i << " and " << j << " collide at " << solution[i][a-1].second << "~" << solution[i][a].second << solution[i][a-1].first << "-->" << solution[i][a].first << " X " << solution[j][b-1].first << "-->" << solution[j][b].first << "\n";
            // Combine groups i and j
            
            Group* toDelete(group[j]);
            groups.erase(group[j]);
            for(auto a:group[j]->agents){
              group[i]->agents.insert(a);
              group[a]=group[i];
            }
            delete toDelete;
            
            independent=false;
            break;
          }
          if(fequal(solution[i][a].second,solution[j][b].second)){
            ++a;++b;
          }else if(fless(solution[i][a].second,solution[j][b].second)){
            ++a;
          }else{++b;}
        }
      }
    }
  }
  return independent;
}

int main(int argc, char ** argv){
  MapEnvironment env(new Map(8,8));
  env.SetFiveConnected();
  if(argc>1){
    switch(atoi(argv[1])){
      case 4:
        env.SetFourConnected();
        break;
      case 8:
        env.SetEightConnected();
        break;
      case 9:
        env.SetNineConnected();
        break;
      case 24:
        env.SetTwentyFourConnected();
        break;
      case 25:
        env.SetTwentyFiveConnected();
        break;
      case 48:
        env.SetFortyEightConnected();
        break;
      case 49:
        env.SetFortyNineConnected();
        break;
      case 5:
      default:
        env.SetFiveConnected();
        break;
    }
  }
  int n(6);
  if(argc>2){
    n=atoi(argv[2]);
  }
  Node::env=&env;
  int seed(123456);
  double timeout(300);
  srand(seed);
  {
    std::cout << "N="<<n<<"\n";
    double total(0.0);
    double length(0.0);
    int numAgents(n);
    int failed(0);
    double cost(0);
    for(int t(0); t<100; ++t){
      //checked.clear();
      std::cout << "Trial #"<<t<<std::endl;
      std::set<xyLoc> st;
      std::set<xyLoc> en;
      Points s;
      Points e;
      // Get disjoint start and goal locations
      for(int i(0);i<numAgents;++i){
        auto a(st.emplace(rand()%8,rand()%8));
        while(!a.second){
          a=st.emplace(rand()%8,rand()%8);
        }
        s.push_back(*a.first);

        a=en.emplace(rand()%8,rand()%8);
        while(!a.second || s[i]==*a.first){
          a=en.emplace(rand()%8,rand()%8);
        }
        e.push_back(*a.first);
      }
      /*
         s.emplace_back(1,1);
         e.emplace_back(4,4);
         s.emplace_back(3,0);
         e.emplace_back(3,4);
         s.emplace_back(0,2);
         e.emplace_back(3,1);
         s.emplace_back(2,0);
         e.emplace_back(1,3);
         */
      //s.emplace_back(0,5);
      //e.emplace_back(3,0);
      //s.emplace_back(1,1);
      //e.emplace_back(6,1);
      //s.emplace_back(3,0);
      //e.emplace_back(4,2);
      TemplateAStar<xyLoc,tDirection,MapEnvironment> astar;
      Solution solution;
      //std::cout << "Init groups\n";
      std::vector<Group*> group(s.size()); // Agent#-->group#
      std::unordered_set<Group*> groups;
      // Add a singleton group for all groups
      for(int i(0); i<s.size(); ++i){
        group[i]=new Group(i); // Initially in its own group
        groups.insert(group[i]);
      }

      // Start timing
      //std::cout << std::setprecision(3);
      Timer tmr;
      tmr.StartTimer();

      // Initial individual paths.
      for(int i(0); i<s.size(); ++i){
        Points path;
        astar.GetPath(&env,s[i],e[i],path);
        TimePath timePath;
        //std::cout << s[i] << "-->" << e[i] << std::endl;
        timePath.emplace_back(path[0],0.0);
        for(int i(1); i<path.size(); ++i){
          timePath.emplace_back(path[i],timePath[timePath.size()-1].second+Util::distance(path[i-1].x,path[i-1].y,path[i].x,path[i].y));
        }
        solution.push_back(timePath);
      }
      //std::cout << std::endl;
      //std::cout << "Initial solution:\n";
      //int ii(0);
      //for(auto const& p:solution){
        //std::cout << ii++ << "\n";
        //for(auto const& t: p){
          // Print solution
          //std::cout << t.first << "," << t.second << "\n";
        //}
      //}

      double elapsed(timeout);
      while(!detectIndependence(solution,group,groups)){
      std::unordered_map<int,Instance> G;
      std::unordered_map<int,std::vector<int>> Gid;
        //std::cout << "There are " << groups.size() << " groups" << std::endl;
        // Create groups
        int g(0);
        for(auto const& grp:groups){
          for(auto a:grp->agents){
            G[g].first.push_back(s[a]);
            G[g].second.push_back(e[a]);
            Gid[g].push_back(a);
          }
          ++g;
        }
        for(int j(0); j<G.size(); ++j){
          auto g(G[j]);
            //std::cout << "Group " << j <<":\n";
            //for(int i(0); i<g.first.size(); ++i){
              //std::cout << Gid[j][i] << ":" << g.first[i] << "-->" << g.second[i] << "\n";
            //}
          if(g.first.size()>1){
            std::vector<float> sizes(g.first.size());
            std::priority_queue<ICTSNode*,std::vector<ICTSNode*>,ICTSNodePtrComp> q;
            std::unordered_set<std::string> deconf;

            q.push(new ICTSNode(g,sizes));

            std::vector<std::set<Node*,NodePtrComp>> answer;
            std::vector<ICTSNode*> toDelete;
            while(q.size()){
              ICTSNode* parent(q.top());
              //std::cout << "pop ";
              //for(auto const& a: parent->sizes){
                //std::cout << a << " ";
              //}
              //std::cout << "\n";
              q.pop();
              //std::cout << "SIC: " << parent->SIC() << "\n";
              if(parent->isValid(answer)){
                int i(0);
                for(auto const& b:answer){
                  TimePath p;
                  for(auto const& a:b){
                    p.emplace_back(a->n,a->depth);
                  }
                  solution[Gid[j][i]]=p;
                  ++i;
                }
                break;
              }
              for(int i(0); i<parent->sizes.size(); ++i){
                std::vector<float> sz(parent->sizes);
                sz[i]++;
                std::stringstream sv;
                join(sv,sz);
                if(deconf.find(sv.str())==deconf.end()){
                  q.push(new ICTSNode(parent,i,sz[i]));
                  deconf.insert(sv.str());
                }
              }
              if(tmr.TimeCut() > timeout){
                elapsed = timeout;
                while(q.size()) q.pop();
                failed++;
                std::cout << "failed" << std::endl;
              }
              toDelete.push_back(parent);
            }
            for(auto r:toDelete){
              delete r;
            }
            while(!q.empty()){
              delete q.top();
              q.pop();
            }
          }
        }
        for(auto const& path:solution){
          length += path.size();
          for(int j(1); j<path.size(); ++j){
            if(path[j-1].first!=path[j].first)
              cost += env.GCost(path[j-1].first,path[j].first);
          }
        }
        //std::cout << "Solution:\n";
        //int ii(0);
        //for(auto const& p:solution){
        //std::cout << ii++ << "\n";
        //for(auto const& t: p){
        //// Print solution
        //std::cout << t.first << "," << t.second << "\n";
        //}
        //}
        //std::cout << "Re-init groups\n";
        //group.resize(s.size()); // Agent#-->group#
        // Add a singleton group for all groups
        //for(int i(0); i<s.size(); ++i){
          //group[i]=i; // In its own group
        //}
      }

      for(auto y:groups){
        delete y;
      }

      elapsed=tmr.EndTimer();
      //std::cout << elapsed << " elapsed";
      //std::cout << std::endl;
     total += elapsed;
    }
    std::cout << "Average time: " << total/100. << std::endl;
    std::cout << "Average path: " << length/(100.*n) << std::endl;
    std::cout << "Average cost: " << cost/(100.) << std::endl;
    std::cout << failed << " failures" << std::endl;
  }
  return 1;
}
