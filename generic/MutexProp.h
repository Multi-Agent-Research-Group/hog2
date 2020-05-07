/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2020 All rights reserved.
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
#include <iostream>
#include <iomanip>
#include <set>
#include <numeric>
#include <map>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>
#include "CollisionDetection.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "MultiAgentStructures.h"
#include "Utilities.h"

extern bool verbose;

template <typename state>
struct Node;

template <typename state>
struct Node{
  static uint64_t count;

  Node(){count++;}
  template <typename action>
  Node(state a, uint64_t d, ConstrainedEnvironment<state, action> const* env):n(a),hash(env->GetStateHash(a)),id(0),optimal(false){assert(d==a.t);count++;}
  //Node(state a, float d):n(a),depth(d*state::TIME_RESOLUTION_U),optimal(false),unified(false),nogood(false){count++;}
  state n;
  uint64_t hash;
  uint32_t id;
  bool optimal;
  //bool connected()const{return parents.size()+successors.size();}
  std::set<Node*> parents;
  std::set<Node*> successors;
  std::map<Node*,std::set<std::pair<Node*,Node*>>> mutexes; // [op.s1,op.s2]=(self)-->succ[i]
  inline uint64_t Hash()const{return hash;}
  inline uint32_t Depth()const{return n.t;}
  inline void Print(std::ostream& ss, int d=0) const {
    ss << std::string(d,' ')<<n << "_" << Depth()<<":"<<id << std::endl;
    for(auto const& m: successors)
      m->Print(ss,d+1);
  }
  bool operator==(Node const& other)const{return n.sameLoc(other.n)&&Depth()==other.Depth();}
};

template <typename state>
uint64_t Node<state>::count = 0;
template <typename state>
using MultiState = std::vector<Node<state>*>; // rank=agent num
template <typename state>
using Edge = std::pair<Node<state>*,Node<state>*>;

template <typename state>
struct MultiEdge{
  MultiEdge():feasible(true){}
  void resize(size_t s){e.resize(s);}
  Edge<state> operator[](unsigned i)const{return e[i];}
  Edge<state>& operator[](unsigned i){return e[i];}
  typename std::vector<Edge<state>>::const_iterator begin()const{return e.begin();}
  typename std::vector<Edge<state>>::const_iterator end()const{return e.end();}
  typename std::vector<Edge<state>>::iterator begin(){return e.begin();}
  typename std::vector<Edge<state>>::iterator end(){return e.end();}
  size_t size()const{return e.size();}
  bool empty()const{return e.empty();}
  void clear(){e.clear();}
  void emplace_back(Node<state>* a,Node<state>* b){e.emplace_back(a,b);}
  void push_back(Edge<state> const& a){e.push_back(a);}
  Edge<state>& back(){return e.back();}
  std::vector<Edge<state>> e;
  bool feasible;
};

template <typename state>
static inline bool operator<(Edge<state> const& lhs, Edge<state> const& rhs){
  return lhs.first->Depth()==rhs.first->Depth()?
    lhs.second->Depth()==rhs.second->Depth()?
    lhs.first->n.x==rhs.first->n.x?
    lhs.first->n.y==rhs.first->n.y?
    lhs.second->n.x==rhs.second->n.x?
    lhs.second->n.y<rhs.second->n.y:
    lhs.second->n.x<rhs.second->n.x:
    lhs.first->n.y<rhs.first->n.y:
    lhs.first->n.x<rhs.first->n.x:
    lhs.second->Depth()<rhs.second->Depth():
    lhs.first->Depth()<rhs.first->Depth();}

template <typename state>
struct NodeRevCmp{
  inline bool operator()(Node<state> const* lhs, Node<state> const* rhs) {
    return lhs->Depth()==rhs->Depth()? // Tie-break by time first
    lhs->n.x==rhs->n.x?
    lhs->n.y<rhs->n.y:
    lhs->n.x<rhs->n.x:
    lhs->Depth()<rhs->Depth(); }
};

template <typename state>
using ActionSet = std::set<Node<state>*, NodeRevCmp<state>>;
template <typename state>
using EdgeSet = std::set<Edge<state>>;

template <typename state>
unsigned MinValue(MultiEdge<state> const& m){
  unsigned v(INT_MAX);
  for(auto const& n:m){
    v=std::min(v,n.first->Depth());
  }
  return v;
}
template <typename state>
struct MultiEdgeCmp
{
  bool operator()(MultiEdge<state> const& lhs, MultiEdge<state> const& rhs) const  { return MinValue(lhs)>MinValue(rhs);}
};

template <typename state>
using DAG = std::unordered_map<uint64_t,Node<state>>;

template <typename state>
static inline std::ostream& operator << (std::ostream& ss, Node<state> const& n){
  ss << n.n.x << "," << n.n.y << "," << double(n.Depth())/state::TIME_RESOLUTION_D<<":"<<n.id;
  return ss;
}

template <typename state>
static inline std::ostream& operator << (std::ostream& ss, Node<state> const* n){
  n->Print(ss);
  return ss;
}


template <typename state, typename action>
bool LimitedDFS(state const& start,
state const& end,
DAG<state>& dag,
Node<state>*& root,
EdgeSet<state>& terminals,
uint32_t depth,
uint32_t minDepth,
uint32_t maxDepth,
uint32_t& best,
ConstrainedEnvironment<state,action> const* env,
std::map<uint64_t,bool>& singleTransTable,
bool& blocked, // Goal couldn't be reached because of no successors (not depth limits)
unsigned recursions=1, bool disappear=true){
  if(verbose)std::cout << std::string(recursions,' ') << start << "g:" << (maxDepth-depth) << " h:" << (int)(env->HCost(start,end)) << " f:" << ((maxDepth-depth)+(int)(env->HCost(start,end))) << "\n";
  if(depth<0 || maxDepth-depth+(int)(env->HCost(start,end))>maxDepth){ // Note - this only works for an admissible heuristic.
    //if(verbose)std::cout << "pruned " << start << depth <<" "<< (maxDepth-depth+(int)(env->HCost(start,end)))<<">"<<maxDepth<<"\n";
    return false;
  }
    //if(verbose)std::cout << " OK " << start << depth <<" "<< (maxDepth-depth+(int)(env->HCost(start,end)))<<"!>"<<maxDepth<<"\n";

  Node<state> n(start,(maxDepth-depth),env);
  uint64_t hash(n.Hash());
  if(singleTransTable.find(hash)!=singleTransTable.end()){return singleTransTable[hash];}
  //std::cout << "\n";

  if(env->GoalTest(start,end)){
    singleTransTable[hash]=true;
      //std::cout << n<<"\n";
    n.id=dag.size()+1;
    dag[hash]=n;
    // This may happen if the agent starts at the goal
    if(maxDepth-depth<=0){
      root=&dag[hash];
      //std::cout << "root_ " << &dag[hash];
    }
    Node<state>* parent(&dag[hash]);
    int d(maxDepth-depth);
    if(!disappear && d<maxDepth){ // Insert one long wait action at goal
      // Wait at goal
      Node<state> current(start,maxDepth,env);
      uint64_t chash(current.Hash());
      //std::cout << current<<"\n";
      current.id=dag.size()+1;
      dag[chash]=current;
      if(verbose)std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      parent->successors.insert(&dag[chash]);
      dag[chash].parents.insert(parent);
      parent=&dag[chash];
    }
    if(parent->Depth()>minDepth){
      best=std::min(best,parent->Depth());
    }
    if(verbose)std::cout << "BEST "<<best<< ">" << minDepth << "\n";
    blocked=false;
    return true;
  }

  std::vector<state> successors(64);
  unsigned sz(env->GetSuccessors(start,&successors[0]));
  if(sz==0){
    blocked=true;
    return false;
  }
  successors.resize(sz);
  bool result(false);
  for(auto const& node: successors){
    int ddiff(std::max(Util::distance(node.x,node.y,start.x,start.y),1.0)*state::TIME_RESOLUTION_U);
    //std::cout << std::string(std::max(0,(maxDepth-(depth-ddiff))),' ') << "MDDEVAL " << start << "-->" << node << "\n";
    //if(verbose)std::cout<<node<<": --\n";
    if(LimitedDFS(node,end,dag,root,terminals,depth-ddiff,minDepth,maxDepth,best,env,singleTransTable,blocked,recursions+1,disappear)){
      singleTransTable[hash]=true;
      if(dag.find(hash)==dag.end()){
        //std::cout << n<<"\n";

        n.id=dag.size()+1;
        dag[hash]=n;
        // This is the root if depth=0
        if(maxDepth-depth<=0){
          root=&dag[hash];
          if(verbose)std::cout << "Set root to: " << (uint64_t)root << "\n";
          //std::cout << "_root " << &dag[hash];
        }
        //if(maxDepth-depth==0.0)root.push_back(&dag[hash]);
      }else if(dag[hash].optimal){
        return true; // Already found a solution from search at this depth
      }

      Node<state>* parent(&dag[hash]);

      //std::cout << "found " << start << "\n";
      uint64_t chash(Node<state>(node,(maxDepth-depth+ddiff),env).Hash());
      if(dag.find(chash)==dag.end()&&dag.find(chash+1)==dag.end()&&dag.find(chash-1)==dag.end()){
        std::cout << "Expected " << Node<state>(node,maxDepth-depth+ddiff,env) << " " << chash << " to be in the dag\n";
        assert(!"Uh oh, node not already in the DAG!");
        //std::cout << "Add new.\n";
        //Node<state> c(node,(maxDepth-depth+ddiff));
        //dag[chash]=c;
      }
      Node<state>* current(&dag[chash]);
      current->optimal = result = true;
      //std::cout << *parent << " parent of " << *current << "\n";
      dag[current->Hash()].parents.insert(&dag[parent->Hash()]);
      //std::cout << *current << " child of " << *parent << " " << parent->Hash() << "\n";
      //std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      dag[parent->Hash()].successors.insert(&dag[current->Hash()]);
      //std::cout << "at" << &dag[parent->Hash()] << "\n";
      if (env->GoalTest(current->n, end)){
        terminals.emplace(parent,current);
      }
    }
  }
  singleTransTable[hash]=result;
  if(!result){
    dag.erase(hash);
  }
  return result;
}


template <typename state, typename action>
bool getMDD(state const& start,
state const& end,
DAG<state>& dag,
Node<state> *& root,
EdgeSet<state>& terminals,
uint32_t minDepth,
uint32_t depth,
uint32_t& best,
ConstrainedEnvironment<state,action>* env,
bool& blocked,
unsigned offset=0){
  blocked=false;
  if(verbose)std::cout << "MDD up to depth: " << depth << start << "-->" << end << "\n";
  static std::map<uint64_t,bool> singleTransTable;
  singleTransTable.clear();
  auto result = LimitedDFS(start,end,dag,root,terminals,depth,minDepth,depth,best,env,singleTransTable,blocked);
  std::map<uint64_t,unsigned> m;
  std::map<unsigned,std::vector<uint64_t>> xs;
  std::vector<std::pair<float,float>> pos(dag.size());
  std::vector<std::string> lab(dag.size());
  std::cout << "g=Graph([";
  for(auto const& n:dag){
    if(m.find(n.first)==m.end()){
      m[n.first]=m.size();
    }
    auto ix(n.second.Depth()-n.second.Depth()%state::TIME_RESOLUTION_U);
    xs[ix].push_back(n.first);
    lab[m[n.first]].append("\"") 
    .append(std::to_string(n.second.n.x))
    .append(",")
    .append(std::to_string(n.second.n.y))
    .append("\"");
    for(auto const& s:n.second.successors){
      if (m.find(s->hash) == m.end())
      {
        m[s->hash] = m.size();
      }
      std::cout<<"("<<m[n.first]+offset<<","<<m[s->hash]+offset<<"), ";
    }
  }
  std::cout << "],directed=True)\n";

  std::cout << "vertex_label="<<lab<<"\n";

  for(auto const& x:xs){
    unsigned ss(0);
    for(auto const& q:x.second){
      auto v=m[q];
      pos[v].first=dag[q].Depth();
      pos[v].second=ss*5;
      ++ss;
    }
  }
  std::cout << "vertex_size="<<std::vector<int>(lab.size(),50) << "\n";
  //std::cout << "vertex_label_dist="<<std::vector<float>(lab.size(),-.5) << "\n";
  std::cout << "layout="<<pos<<"\n";
  std::cout << "fmt={}\n"
            << "fmt['layout']=layout\n"
            << "fmt['vertex_label']=vertex_label\n"
            //<< "fmt['vertex_label_dist']=vertex_label_dist\n"
            << "fmt['vertex_size']=vertex_size\n"
            << "plot(g,'mdd.png',**fmt)\n";
  return result;
  //if(verbose)std::cout << "Finally set root to: " << (uint64_t)root[agent] << "\n";
  //if(verbose)std::cout << root << "\n";
}

template <typename state>
void generatePermutations(std::vector<MultiEdge<state>>& positions,
std::vector<MultiEdge<state>>& result,
int agent,
MultiEdge<state> const& current,
uint32_t lastTime,
std::vector<float> const& radii,
//std::vector<ActionSet<state>>& acts,
bool update=true) {
  if(agent == positions.size()) {
    result.push_back(current);
    if(verbose)std::cout << "Generated joint move:\n";
    if(verbose)for(auto edge:current){
      std::cout << *edge.first << "-->" << *edge.second << "\n";
    }
    std::cout << "CrossProduct:\n";
    for(auto const& c:result){
      if(verbose)for(auto edge:c){
        std::cout << *edge.first << "-->" << *edge.second << " ";
      }
      std::cout <<"\n";
    }
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    //std::cout << "AGENT "<< i<<":\n";
    MultiEdge<state> copy(current);
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      bool conflict(false);
      // Sometimes, we add an instantaneous action at the goal to represent the
      // agent disappearing at the goal. If we see this, the agent did not come into conflict
      if (positions[agent][i].first != positions[agent][i].second &&
          current[j].first != current[j].second) {
            // Check easy case: agents crossing an edge in opposite directions, or
            // leaving or arriving at a vertex at the same time
        if ((positions[agent][i].first->Depth() == current[j].first->Depth() &&
             positions[agent][i].first->n == current[j].first->n) ||
            (positions[agent][i].second->Depth() == current[j].second->Depth() &&
             positions[agent][i].second->n == current[j].second->n) ||
            (positions[agent][i].first->n.sameLoc(current[j].second->n) &&
             current[j].first->n.sameLoc(positions[agent][i].second->n))) {
          found = true;
          conflict = true;
        } else {
          // Check general case - Agents in "free" motion
          Vector2D A(positions[agent][i].first->n.x, positions[agent][i].first->n.y);
          Vector2D B(current[j].first->n.x, current[j].first->n.y);
          Vector2D VA(positions[agent][i].second->n.x - positions[agent][i].first->n.x, positions[agent][i].second->n.y - positions[agent][i].first->n.y);
          VA.Normalize();
          Vector2D VB(current[j].second->n.x - current[j].first->n.x, current[j].second->n.y - current[j].first->n.y);
          VB.Normalize();
          std::cout<<"Checking:"<<current[j].first->n << "-->"<< current[j].second->n <<", " << positions[agent][i].first->n << "-->"<< positions[agent][i].second->n << "\n";
          if (collisionImminent(A, VA, radii[agent], positions[agent][i].first->Depth() / state::TIME_RESOLUTION_D, positions[agent][i].second->Depth() / state::TIME_RESOLUTION_D, B, VB, radii[j], current[j].first->Depth() / state::TIME_RESOLUTION_D, current[j].second->Depth() / state::TIME_RESOLUTION_D)) {
            found = true;
            conflict = true;
            //checked.insert(hash);
          }
        }
      }
      if(conflict){
        if(update){
          std::cout << "Initial mutex: " << current[j].first->n << "-->"<< current[j].second->n <<", " << positions[agent][i].first->n << "-->"<< positions[agent][i].second->n << "\n";
          //acts[j].insert(current[j].first);
          //acts[agent].insert(positions[agent][i].first);
          positions[agent][i].first->mutexes[positions[agent][i].second].emplace(current[j].first,current[j].second);
          current[j].first->mutexes[current[j].second].emplace(positions[agent][i].first,positions[agent][i].second);
        }
        if(verbose)std::cout << "Collision averted: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
      } else if(verbose)std::cout << "generating: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
    }
    if(found){
      copy.feasible=false;
      if(!update) continue; // Don't record pair if it was infeasible...
    }
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy,lastTime,radii,/*acts,*/update);
  }
}

// Assumes sets are ordered
template <typename T>
void inplace_intersection(T& set_1, T const& set_2){
auto it1 = set_1.begin();
auto it2 = set_2.begin();
while ( (it1 != set_1.end()) && (it2 != set_2.end()) ) {
    if (*it1 < *it2) {
        it1=set_1.erase(it1);
    } else if (*it2 < *it1) {
        ++it2;
    } else { // *it1 == *it2
            ++it1;
            ++it2;
    }
}
// Anything left in set_1 from here on did not appear in set_2,
// so we remove it.
set_1.erase(it1, set_1.end());
 
}

template<class T, class C, class Cmp=std::less<typename C::value_type>>
struct ClearablePQ:public std::priority_queue<T,C,Cmp>{
  void clear(){
    //std::cout << "Clearing pq\n";
    //while(this->size()){std::cout<<this->size()<<"\n";this->pop();}
    this->c.resize(0);
  }
  C& getContainer() { return this->c; }
};


template <typename state, typename action>
bool getMutexes(MultiEdge<state> const& n,
std::vector<state> const& goal,
std::vector<ConstrainedEnvironment<state,action>*> const& env,
std::vector<Node<state>*>& toDelete,
//std::vector<std::vector<Edge<state>>>& actions,
//std::vector<std::vector<std::vector<unsigned>>>& edges,
std::vector<EdgeSet<state>>const& terminals,
std::vector<EdgeSet<state>>& mutexes,
std::vector<float> const& radii,
bool disappear=true, bool OD=false){
  bool result(false);
  static const int MAXTIME(1000*state::TIME_RESOLUTION_U);
  static std::unordered_map<std::string,bool> visited;
  visited.clear();
  // Sort actions in reverse time order
  struct edgeCmp{
    inline bool operator()(Node<state>* lhs, Node<state>* rhs){return rhs->Depth()<lhs->Depth();}
  };
  //static std::vector<ActionSet<state>> acts(n.size());
  //for(auto& a:acts){a.clear();}
  static ClearablePQ<MultiEdge<state>,std::vector<MultiEdge<state>>,MultiEdgeCmp<state>> q;
  q.clear();
  q.push(n);
  static std::vector<Edge<state>> intersection;
  static std::vector<Edge<state>> stuff;

  while(q.size()){
    std::cout << "q:\n";
    auto t(q);
    while(t.size()){
      std::cout <<"  ";
      for (auto const &u : t.top()) {
        std::cout << "  " << u.first->n.x << "," << u.first->n.y << "-->" << u.second->n.x << "," << u.second->n.y << " ";
      }
      std::cout <<"\n";
      t.pop();
    }
    auto s(q.top());
    q.pop();
    std::cout << "s:\n";
    for(auto const& g:s){
      std::cout << g.first->n << "-->" << g.second->n << " ";
    }
    std::cout << "(feasible=" << s.feasible << ")\n";
    
    bool done(s.feasible);
    unsigned agent(0);
    if(done){
      for(auto const& g:s){
        if(!env[agent]->GoalTest(g.second->n,goal[agent])){
          done=false;
          std::cout << " no goal...\n";
          break;
        }
        agent++;
      }
    }
    if(done){result=true;}

    // Find minimum depth of current edges
    uint32_t sd(INT_MAX);
    unsigned minindex(0);
    int k(0);
    for(auto const& a: s){
      if(a.second!=a.first && // Ignore disappeared agents
          a.second->Depth()<sd){
        sd=a.second->Depth();
        minindex=k;
      }
      k++;
      //sd=min(sd,a.second->Depth());
    }
    if(sd==INT_MAX){sd=s[minindex].second->Depth();} // Can happen at root node
    //std::cout << "min-depth: " << sd << "\n";

    //Get successors into a vector
    std::vector<MultiEdge<state>> successors;
    successors.reserve(s.size());

    uint32_t md(INT_MAX); // Min depth of successors
    //Add in successors for parents who are equal to the min
    k=0;
    for(auto const& a: s){
      static MultiEdge<state> output;
      output.clear();
      if((OD && (k==minindex /* || a.second->Depth()==0*/)) || (!OD && a.second->Depth()<=sd)){
        //std::cout << "Keep Successors of " << *a.second << "\n";
        for(auto const& b: a.second->successors){
          output.emplace_back(a.second,b);
          md=min(md,b->Depth());
        }
      }else{
        //std::cout << "Keep Just " << *a.second << "\n";
        output.push_back(a);
        md=min(md,a.second->Depth());
      }
      if(output.empty()){
        // This means that this agent has reached its goal.
        // Stay at state...
        if(disappear){
          output.emplace_back(a.second,a.second); // Stay, but don't increase time
        }else{
          output.emplace_back(a.second,new Node<state>(a.second->n,MAXTIME,env[k]));
          //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
          toDelete.push_back(output.back().second);
        }
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
      ++k;
    }
    if(verbose){
      std::cout << "Move set\n";
      for(int a(0);a<successors.size(); ++a){
        std::cout << "agent: " << a << "\n";
        for(auto const& m:successors[a]){
          std::cout << "  " << *m.first << "-->" << *m.second << "\n";
        }
      }
    }
    static std::vector<MultiEdge<state>> crossProduct;
    crossProduct.clear();
    static MultiEdge<state> tmp; tmp.clear();
    tmp.feasible=s.feasible;

    // This call also computes initial mutexes
    generatePermutations(successors,crossProduct,0,tmp,sd,radii);//,acts);
    std::cout << "cross product size: " << crossProduct.size() << "\n";
    // Since we're visiting these in time-order, all parent nodes of this node
    // have been seen and their initial mutexes have been computed. Therefore
    // we can compute propagated mutexes and inherited mutexes at the same time.

    // Look for propagated mutexes...
    // For each pair of actions in s:
    static std::vector<Edge<state>> mpj;
    for(int i(0); i<s.size()-1; ++i){
      if(s[i].first==s[i].second){continue;} // Ignore disappearing at goal
      if(s[i].first->parents.size()){
        std::cout << s[i].first->n << " has " << s[i].first->parents.size() << " parents\n";
        for(int j(i+1); j<s.size(); ++j){
          std::cout << "Check versus " << s[j].first->n << "\n";
          mpj.clear();
          // Get list of pi's mutexes with pjs
          for(auto const& pi:s[i].first->parents){
            std::cout << "Parent has " << pi->mutexes.size() << " mutexes\n";

            for(auto const& mi:pi->mutexes){
              //acts[i].insert(pi); // Add to set of states which have mutexed actions
              if(mi.first==s[i].first){ // Does the edge from the parent end at this vertex?
                for(auto const& mu:mi.second){
                  if(mu.second==s[j].first){ // Does the mutexed edge end at the appropriate vertex?
                    std::cout << "This parent (" << pi->n << ") has a mutex that coincides: " << mu.first->n << "-->" << mu.second->n << "\n";
                    mpj.emplace_back(mu); // Add pointers to pjs
                  }else{
                    std::cout << "This mutex: " << mu.first->n << "-->" << mu.second->n << " does not coincide\n";
                  }
                }
              }else{
                std::cout << pi->n << "-->" << mi.first->n << " is not the right mutexed action\n";
              }
            }
          }
          // Now check if all pjs are mutexed with the set
          if(s[j].first->parents.size()==mpj.size() && mpj.size()){
            std::cout << "Parents size matches the number of mutexes ("<<mpj.size()<<")\n";
            bool found(true);
            for(auto const& m:mpj){
              if(std::find(s[j].first->parents.begin(),s[j].first->parents.end(),m.first)==s[j].first->parents.end()){
                found=false;
                break;
              }
            }
            // Because all of the parents of i and j are mutexed, i and j
            // get a propagated mutex :)
            if(found){
              std::cout << "Propagated mutex: " << s[i].first->n << "-->"<< s[i].second->n <<", " << s[j].first->n << "-->"<< s[j].second->n << "\n";
              s[i].first->mutexes[s[i].second].insert(s[j]);
              s[j].first->mutexes[s[j].second].insert(s[i]);
              //acts[i].insert(s[i].first); // Add to set of states which have mutexed actions
              //acts[j].insert(s[j].first); // Add to set of states which have mutexed actions
            }
          }
          intersection.clear();
          // Finally, see if we can add inherited mutexes
          // TODO: Might need to be a set union for non-cardinals
          std::cout << "check inh: " << s[i].first->n.x << "," << s[i].first->n.y
                    << "-->" << s[i].second->n.x << "," << s[i].second->n.y << "\n";
          if(s[i].first->parents.size()){
            // Get mutexes from first parent
            auto parent(s[i].first->parents.begin());
            std::cout << "  p: " << (*parent)->n.x << "," << (*parent)->n.y
                      << "-->" << s[i].first->n.x << "," << s[i].first->n.y << "\n";
            intersection.reserve((*parent)->mutexes.size());
            for(auto const& mi:(*parent)->mutexes){
              if(mi.first==s[i].first){
                for(auto const& mu:mi.second){
          std::cout << "    m: " << mu.first->n.x << "," << mu.first->n.y
                    << "-->" << mu.second->n.x << "," << mu.second->n.y << "\n";
                  intersection.push_back(mu);
                }
              }
            }
            ++parent;
            if(parent!=s[i].first->parents.end())
            std::cout << "  p: " << (*parent)->n.x << "," << (*parent)->n.y
                      << "-->" << s[i].first->n.x << "," << s[i].first->n.y << "\n";
            // Now intersect the remaining parents' sets
            while(parent!=s[i].first->parents.end()){
              stuff.clear();
              for(auto const& mi:(*parent)->mutexes){
                if(mi.first==s[i].first){
                  for(auto const& mu:mi.second){
                    std::cout << "    m: " << mu.first->n.x << "," << mu.first->n.y
                              << "-->" << mu.second->n.x << "," << mu.second->n.y << "\n";
                    stuff.push_back(mu);
                  }
                }
              }
              inplace_intersection(intersection,stuff);

              std::cout << "  intersection:" << "\n";
              for (auto const &mu : intersection) {
                std::cout << "    m: " << mu.first->n.x << "," << mu.first->n.y
                          << "-->" << mu.second->n.x << "," << mu.second->n.y << "\n";
              }
              ++parent;
            }
          }
          // Add inherited mutexes
          for(auto const& mu:intersection){
            std::cout << "Inherited mutex [i]: " << s[i].first->n << "-->"<< s[i].second->n <<", " << mu.first->n << "-->"<< mu.second->n << "\n";
            s[i].first->mutexes[s[i].second].insert(mu);
            mu.first->mutexes[mu.second].insert(s[i]);
            //acts[i].insert(s[i].first); // Add to set of states which have mutexed actions
            //acts[j].insert(mu.first); // Add to set of states which have mutexed actions
          }
          // Do the same for agent j
          intersection.clear();
          if(s[j].first->parents.size()){
            // Get mutexes from first parent
            auto parent(s[j].first->parents.begin());
            intersection.reserve((*parent)->mutexes.size());
            for(auto const& mj:(*parent)->mutexes){
              if(mj.first==s[j].first){
                for(auto const& mu:mj.second){
                  intersection.push_back(mu);
                }
              }
            }
            ++parent;
            // Now intersect the remaining parents' sets
            while(parent!=s[j].first->parents.end()){
              stuff.clear();
              for(auto const& mj:(*parent)->mutexes){
                if(mj.first==s[j].first){
                  for(auto const& mu:mj.second){
                    stuff.push_back(mu);
                  }
                }
              }
              inplace_intersection(intersection,stuff);
              ++parent;
            }
          }
          // Add inherited mutexes
          for(auto const& mu:intersection){
            std::cout << "Inherited mutex [j]: " << s[j].first->n << "-->"<< s[j].second->n <<", " << mu.first->n << "-->"<< mu.second->n << "\n";
            s[j].first->mutexes[s[j].second].insert(mu);
            mu.first->mutexes[mu.second].insert(s[j]);
            //acts[i].insert(mu.first); // Add to set of states which have mutexed actions
            //acts[j].insert(s[j].first); // Add to set of states which have mutexed actions
          }
        }
      }
    }

    for(auto& a: crossProduct){
      k=0;
      // Compute hash for transposition table
      std::string hash(a.size()*2*sizeof(uint64_t)+1,1);
      for(auto v:a){
        uint64_t h0(v.first->Hash());
        uint64_t h1(v.second->Hash());
        uint8_t c[sizeof(uint64_t)*2];
        memcpy(c,&h0,sizeof(uint64_t));
        memcpy(&c[sizeof(uint64_t)],&h1,sizeof(uint64_t));
        for(unsigned j(0); j<sizeof(uint64_t)*2; ++j){
          hash[k*sizeof(uint64_t)*2+j]=((int)c[j])?c[j]:0xff; // Replace null-terminators in the middle of the string
        }
        ++k;
      }
      hash.push_back(a.feasible?'f':'i');
      // Have we visited this node already?
      if(visited.find(hash)==visited.end()){
        visited[hash]=true;
        std::cout << "pushing:\n";
        for(auto const& g:a){
          std::cout << g.first->n << "-->" << g.second->n << " ";
        }
        q.push(a);
      }else{
        std::cout << "NOT pushing:\n";
        for(auto const& g:a){
          std::cout << g.first->n << "-->" << g.second->n << " ";
        }
      }
      std::cout << "HASH: ";
      for(auto const& c:hash){
        std::cout << +c << ",";
      }
      std::cout << "\n";
    }
  }
  // Create mutexes as intersection of mutexes on terminal actions
  unsigned j(1);
  for(auto const& t:terminals){
    intersection.clear();
    stuff.clear();
    auto term(t.begin());
    std::cout << "term: " << term->first->n.x << "," << term->first->n.y
     << "-->" << term->second->n.x << "," << term->second->n.y << "\n";
    intersection.reserve(term->first->mutexes.size());
    for (auto const &mi : term->first->mutexes) {
      if (term->second == mi.first){
        for (auto const &mu : mi.second) {
    std::cout << "  m: " << mu.first->n.x << "," << mu.first->n.y
     << "-->" << mu.second->n.x << "," << mu.second->n.y << "\n";
          intersection.push_back(mu);
        }
      }
    }
    ++term;
    // Now intersect the remaining parents' sets
    while(term!=t.end()){
    std::cout << "term: " << term->first->n.x << "," << term->first->n.y
     << "-->" << term->second->n.x << "," << term->second->n.y << "\n";
      stuff.clear();
      stuff.reserve(term->first->mutexes.size());
      for(auto const& mi:term->first->mutexes){
        if (term->second==mi.first){
          for(auto const& mu:mi.second){
    std::cout << "  m: " << mu.first->n.x << "," << mu.first->n.y
     << "-->" << mu.second->n.x << "," << mu.second->n.y << "\n";
            stuff.push_back(mu);
          }
        }
      }
      inplace_intersection(intersection,stuff);
      ++term;
    }

    std::cout << "mutexes for agent " << j << "\n";
    for (auto const &i : intersection) {
      if (i.first->n != i.second->n) {
        std::cout << i.first->n << "-->" << i.second->n << "\n";
        mutexes[j].insert(i);
      }
    }
    --j;
  }
  // Create k-partite graph
  /*for (int k(0); k < acts.size(); ++k) {
    for (auto const& a : acts[k]) {
      if (a->mutexes.size() == 0) {
        std::cout << "ERROR: Action recorded without mutexes!\n";
      }
      for (auto const& m : a->mutexes) {
        Edge<state> act(a, m.first);
        unsigned ix1(0);
        auto itr(std::find(actions[k].begin(), actions[k].end(), act));
        if (itr == actions[k].end()) {
          ix1 = actions[k].size();
          std::cout << "Add action[" << k << "]=" << act.first->n << "-->" << act.second->n << "=[" << ix1 << "]\n";
          actions[k].push_back(act);
        } else {
          ix1 = itr - actions[k].begin();
        }
        //if (act.second->Depth() == costs[k] &&         // Has the right goal cost
            //act.second->n.sameLoc(goal[k]) &&          // action terminates at the goal
            //(!act.second->n.sameLoc(act.first->n))) {  // does not wait at goal (otherwise, act.first would be a better soln)
          //goalEdges[k].push_back(ix1);
        //}
      }
    }
  }
  edges[0].resize(actions[0].size());
  edges[1].resize(actions[1].size());
  for (unsigned a(0); a < actions[0].size(); ++a) {
    for (auto const& m : actions[0][a].first->mutexes) {
      if (m.first == actions[0][a].second) {
        for (auto const& mu : m.second) {
        auto ix2(std::find(actions[1].begin(),
                           actions[1].end(), mu) -
                 actions[1].begin());
        if (ix2 < actions[1].size()) {
          edges[0][a].push_back(ix2);
          edges[1][ix2].push_back(a);
        }
      }
      }
    }

  }
  std::cout << "edges: " << edges << "\n"
            << "e=" << edges[0] << "\n"
            << "g=Graph(edges=[(i,j+len(e)) for i in range(len(e)) for j in e[i]])\n"
            << "g.vs['color']=['gold' if v.index<len(e) else 'red' for v in g.vs]\n"
            << "fmt={}\n"
            << "fmt['vertex_label']=\n"
            << "fmt['vertex_size']=vertex_size\n"
            << "fmt['layout']=g.layout_bipartite([True if v.index<len(e) else False for v in g.vs])\n"
            << "plot(g,'b.png',**fmt)\n";
  */
  return result;
}

// Return true if we get to the desired depth
/*
template <typename state, typename action>
bool getInitialMutexes(MultiEdge<state> const& s, uint32_t d, std::vector<state> const& goal, std::vector<ConstrainedEnvironment<state,action>> const& env, Mutexes& mutexes, MutexMap& mutexmap, std::vector<Node*>& toDelete, unsigned recursions=1){
  // Compute hash for transposition table
  std::string hash(s.size()*sizeof(uint64_t),1);
  int k(0);
  for(auto v:s){
    uint64_t h1(v.second->Hash());
    uint8_t c[sizeof(uint64_t)];
    memcpy(c,&h1,sizeof(uint64_t));
    for(unsigned j(0); j<sizeof(uint64_t); ++j){
      hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
    }
    ++k;
  }

  if(verbose)std::cout << "saw " << s << " hash ";
  if(verbose)for(unsigned int i(0); i<hash.size(); ++i){
    std::cout << (unsigned)hash[i]<<" ";
  }
  if(verbose)std::cout <<"\n";
  if(transTable.find(hash)!=transTable.end()){
    //std::cout << "AGAIN!\n";
    return transTable[hash];
    //if(!transTable[hash]){return false;}
  }

  bool done(true);
  unsigned agent(0);
  for(auto const& g:s){
  if(!env[agent]->GoalTest(g.second->n,goal[agent++])){
      done=false;
      break;
    }
  }
  if(done){return true;}

  //Get successors into a vector
  static std::vector<MultiEdge<state>> successors;
  successors.clear();

  // Find minimum depth of current edges
  uint32_t sd(INT_MAX);
  unsigned minindex(0);
  k=0;
  for(auto const& a: s){
    if(a.second!=a.first && // Ignore disappeared agents
        a.second->Depth()<sd){
      sd=a.second->Depth();
      minindex=k;
    }
    k++;
    //sd=min(sd,a.second->Depth());
  }
  //std::cout << "min-depth: " << sd << "\n";

  uint32_t md(INT_MAX); // Min depth of successors
  //Add in successors for parents who are equal to the min
  k=0;
  for(auto const& a: s){
    MultiEdge<state> output;
    if((OD && (k==minindex )) || (!OD && a.second->Depth()<=sd)){
      //std::cout << "Keep Successors of " << *a.second << "\n";
      for(auto const& b: a.second->successors){
        output.emplace_back(a.second,b);
        md=min(md,b->Depth());
      }
    }else{
      //std::cout << "Keep Just " << *a.second << "\n";
      output.push_back(a);
      md=min(md,a.second->Depth());
    }
    if(output.empty()){
      // This means that this agent has reached its goal.
      // Stay at state...
      if(disappear){
        output.emplace_back(a.second,a.second); // Stay, but don't increase time
      }else{
        output.emplace_back(a.second,new Node(a.second->n,MAXTIME,env));
        //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
        toDelete.push_back(output.back().second);
      }
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
    ++k;
  }
  if(verbose){
    std::cout << "Move set\n";
    for(int a(0);a<successors.size(); ++a){
      std::cout << "agent: " << a << "\n";
      for(auto const& m:successors[a]){
        std::cout << "  " << *m.first << "-->" << *m.second << "\n";
      }
    }
  }
  static std::vector<MultiEdge<state>> crossProduct;
  crossProduct.clear();
  MultiEdge<state> tmp;
  generatePermutations(successors,crossProduct,mutexes,mutexmap,0,tmp,sd);

  bool value(false);
  for(auto& a: crossProduct){
    if(verbose)std::cout << "EVAL " << s << "-->" << a << "\n";
    if(getMutexes(a,md,goal,env,mutexes,mutexmap,toDelete,recursions+1)){
      value=true;
      transTable[hash]=value;
    }
  }
  transTable[hash]=value;
  return value;
}
*/
