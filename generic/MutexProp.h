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
#ifndef MUTEX_PROP_H
#define MUTEX_PROP_H
#include <iostream>
#include <iomanip>
#include <set>
#include <numeric>
#include <map>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>
#include "PairMap.h"
#include "CollisionDetection.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "MultiAgentStructures.h"
#include "Utilities.h"
#include "sorted_vector.h"
#include "TemporalAStarPair.h"
#include "ICTSAlgorithm.h"

template<typename T>
void clear(std::vector<T>& v){
  for (auto &a : v)
  {
    a.clear();
  }
}


extern bool verbose;

template <typename state>
struct Node;

template <typename state>
struct PtrCmp{
  bool operator()(Node<state> const*const a, Node<state> const*const b){
  return a->Depth()==b->Depth()?a->Hash()<b->Hash():a->Depth()<b->Depth();
}
};

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
  sorted_vector<Node*,true,PtrCmp<state>> parents;
  sorted_vector<Node*,true,PtrCmp<state>> successors;
  std::map<Node*,std::set<std::tuple<Node*,Node*,unsigned>>> mutexes; // [self->successor]->[op.s1,op.s2,agent]
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
using Mutex = std::tuple<Node<state>*,Node<state>*,unsigned>;

template <typename state>
struct MultiEdge{
  MultiEdge():parent(nullptr),feasible(true){}
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
  MultiEdge* parent;
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
using DAG = std::map<uint64_t,Node<state>>;

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
  //if(verbose)std::cout << std::string(recursions,' ') << start << "g:" << (maxDepth-depth) << " h:" << (int)(env->HCost(start,end)) << " f:" << ((maxDepth-depth)+(int)(env->HCost(start,end))) << "\n";
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
      //if(verbose)std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      parent->successors.insert(&dag[chash]);
      dag[chash].parents.insert(parent);
      parent=&dag[chash];
    }
    if(parent->Depth()>=minDepth){
      best=std::min(best,parent->Depth());
    }
    //if(verbose)std::cout << "BEST "<<best<< ">" << minDepth << "\n";
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
    bool atGoal(env->GoalTest(node, end) && node.t>=minDepth);
    //std::cout << std::string(std::max(0,(maxDepth-(depth-ddiff))),' ') << "MDDEVAL " << start << "-->" << node << "\n";
    //if(verbose)std::cout<<node<<": --\n";
    if(atGoal && start.sameLoc(node)){
      continue; // Never allow waiting at the goal for the final action
      // The reason for this is because while waiting at or moving in and out of the goal location is ok,
      // We don't want to confuse with a solution of the next lowest cost.
      // (Adding cost to the end of the MDD doesn't help us discover new constraints)
    }
    if(LimitedDFS(node,end,dag,root,terminals,depth-ddiff,minDepth,maxDepth,best,env,singleTransTable,blocked,recursions+1,disappear)){
      singleTransTable[hash]=true;
      if(dag.find(hash)==dag.end()){
        //std::cout << n<<"\n";

        n.id=dag.size()+1;
        dag[hash]=n;
        // This is the root if depth=0
        if(maxDepth-depth<=0){
          root=&dag[hash];
          //if(verbose)std::cout << "Set root to: " << (uint64_t)root << "\n";
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
      if(atGoal){
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
  //if(verbose)std::cout << "MDD up to depth: " << depth << start << "-->" << end << "\n";
  static std::map<uint64_t,bool> singleTransTable;
  singleTransTable.clear();
  auto result = LimitedDFS(start,end,dag,root,terminals,depth,minDepth,depth,best,env,singleTransTable,blocked);
  //std::map<uint64_t,unsigned> m;
  //std::map<unsigned,std::vector<uint64_t>> xs;
  //std::vector<std::pair<float,float>> pos(dag.size());
  //std::vector<std::string> lab(dag.size());
  //std::cout << "g=Graph([";
  /*for(auto const& n:dag){
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
      //std::cout<<"("<<m[n.first]+offset<<","<<m[s->hash]+offset<<"), ";
    }
  }*/
  //std::cout << "],directed=True)\n";

  //std::cout << "vertex_label="<<lab<<"\n";

  /*for(auto const& x:xs){
    unsigned ss(0);
    for(auto const& q:x.second){
      auto v=m[q];
      pos[v].first=dag[q].Depth();
      pos[v].second=ss*5;
      ++ss;
    }
  }*/
  //std::cout << "vertex_size="<<std::vector<int>(lab.size(),50) << "\n";
  //std::cout << "vertex_label_dist="<<std::vector<float>(lab.size(),-.5) << "\n";
  //std::cout << "layout="<<pos<<"\n";
  //std::cout << "fmt={}\n"
            //<< "fmt['layout']=layout\n"
            //<< "fmt['vertex_label']=vertex_label\n"
            //<< "fmt['vertex_label_dist']=vertex_label_dist\n"
            //<< "fmt['vertex_size']=vertex_size\n"
            //<< "plot(g,'mdd.png',**fmt)\n";
  return result;
  //if(verbose)std::cout << "Finally set root to: " << (uint64_t)root[agent] << "\n";
  //if(verbose)std::cout << root << "\n";
}

template<typename state>
class Action: public std::pair<state,state>{
  public:
    Action(state const& a, state const& b):std::pair<state,state>(a,b),t(b.t){}
    Action():t(0){}
    uint32_t t;
    bool operator<(Action const& other)const{
      return this->first==other.first?this->second<other.second:this->first<other.first;
    }
};

template <typename state> class ActionPairIter;
template <typename state> class ActionPairCIter;

template<typename state>
class ActionPair: public std::pair<Action<state>,Action<state>>{
  public:
    ActionPair():std::pair<Action<state>,Action<state>>(),t(0),sz(0),feasible(true),nc(-1){}
    ActionPair(Action<state>const&a,Action<state> const&b,bool f=true):std::pair<Action<state>,Action<state>>(a,b),t(std::min(a.t,b.t)),nc(-1),sz(2),feasible(f){}
    ActionPair(state const& a, state const& b, state const& c, state const& d):std::pair<Action<state>,Action<state>>(Action<state>(a,b),Action<state>(c,d)),t(std::min(b.t,d.t)),nc(-1),sz(2),feasible(true){}
    friend class ActionPairIter<state>;
    friend class ActionPairCIter<state>;

    typedef ActionPairIter<state> iterator;
    typedef ActionPairCIter<state> const_iterator;
    typedef state value_type;
    typedef state *pointer;
    typedef state &reference;
    size_t size()const{return sz;}
    iterator begin(){return iterator(*this,0);}
    const_iterator begin()const{return const_iterator(*this,0);}
    iterator end(){return iterator(*this,2);}
    const_iterator end()const{return const_iterator(*this,2);}
    Action<state>& operator[](unsigned i){assert(i<2);return i?this->second:this->first;}
    Action<state>const& operator[](unsigned i)const{assert(i<2);return i?this->second:this->first;}
    void clear(){sz=0;}
    bool empty(){return sz==0;}
    void emplace_back(state const& a, state const& b){assert(sz<2); if(sz){this->second=Action<state>(a,b);}else{this->first=Action<state>(a,b);} ++sz;}
    void push_back(Action<state> const& a){assert(sz<2); if(sz){this->second=a;}else{this->first=a;} ++sz;}
    bool operator<(ActionPair const& other)const{
      return this->first==other.first?this->second<other.second:this->first<other.first;
    }

    size_t sz;
    uint32_t t;
    int32_t nc;
    bool feasible; // Infeasible because of itself or parents
};

template <typename state>
class ActionPairIter{
  private:
    ActionPair<state> & myPair;
    unsigned offset;
    public:
    ActionPairIter(ActionPair<state> & ap, int size):myPair(ap),offset(size)
    {
    }
    bool operator!=(ActionPairIter const& itr){ return offset != itr.offset;}
    ActionPairIter& operator++(){ ++offset; return *this; }
    ActionPairIter operator++(int){ ActionPair<state> clone(*this); ++clone.offset; return clone; }
    Action<state> & operator*(){ return offset?myPair.second:myPair.first; }
    Action<state> & operator->(){ return offset?myPair.second:myPair.first; }
};

template <typename state>
class ActionPairCIter{
  private:
    ActionPair<state> const& myPair;
    unsigned offset;
    public:
    ActionPairCIter(ActionPair<state> const& ap, int size):myPair(ap),offset(size)
    {
    }
    bool operator!=(ActionPairCIter const& itr){ return offset != itr.offset;}
    ActionPairCIter& operator++(){ ++offset; return *this; }
    ActionPairCIter operator++(int){ ActionPair<state> clone(*this); ++clone.offset; return clone; }
    Action<state> const& operator*(){ return offset?myPair.second:myPair.first; }
    Action<state> const& operator->(){ return offset?myPair.second:myPair.first; }
};


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
    //if(verbose)std::cout << "Generated joint move:\n";
    //if(verbose)for(auto edge:current){
      //std::cout << *edge.first << "-->" << *edge.second << "\n";
    //}
    //std::cout << "CrossProduct:\n";
    //for(auto const& c:result){
      //if(verbose)for(auto edge:c){
        //std::cout << *edge.first << "-->" << *edge.second << " ";
      //}
      //std::cout <<"\n";
    //}
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
          //std::cout<<"Checking:"<<current[j].first->n << "-->"<< current[j].second->n <<", " << positions[agent][i].first->n << "-->"<< positions[agent][i].second->n << "\n";
          if (collisionImminent(A, VA, radii[agent], positions[agent][i].first->Depth() / state::TIME_RESOLUTION_D, positions[agent][i].second->Depth() / state::TIME_RESOLUTION_D, B, VB, radii[j], current[j].first->Depth() / state::TIME_RESOLUTION_D, current[j].second->Depth() / state::TIME_RESOLUTION_D)) {
            found = true;
            conflict = true;
            //checked.insert(hash);
          }
        }
      }
      if(conflict){
        if(update){
          //acts[j].insert(current[j].first);
          //acts[agent].insert(positions[agent][i].first);
          positions[agent][i].first->mutexes[positions[agent][i].second].emplace(current[j].first,current[j].second,j);
          current[j].first->mutexes[current[j].second].emplace(positions[agent][i].first,positions[agent][i].second,agent);
          //if(current[j].first->mutexes[current[j].second].emplace(positions[agent][i].first,positions[agent][i].second,agent).second)
          //std::cout << "Initial mutex: " << current[j].first->n << "-->"<< current[j].second->n << "," << j
          //<< " " << positions[agent][i].first->n << "-->"<< positions[agent][i].second->n << "," << agent << "\n";
        }
        //if(verbose)std::cout << "Collision averted: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
      } //else if(verbose)std::cout << "generating: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
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
bool getTerminals(MultiEdge<state> const &n,
                  std::vector<state> const &goal,
                  std::vector<EnvironmentContainer<state, action> *> const &env,
                  std::vector<Node<state> *> &toDelete,
                  std::vector<EdgeSet<state>> &terminals,
                  std::vector<float> const &radii,
                  std::vector<unsigned> const &minCost,
                  std::vector<unsigned> const& unboundedAgents,
                  bool disappear = true, bool OD = false)
{
  // We are going to fill in terminals for these agents. Clear them for now...
  for (auto const &a : unboundedAgents)
  {
    terminals[a].clear();
  }
  bool result(false);
  static const int MAXTIME(1000 * state::TIME_RESOLUTION_U);
  static std::map<std::string, bool> visited;
  visited.clear();
  // Sort actions in reverse time order
  struct edgeCmp
  {
    inline bool operator()(Node<state> *lhs, Node<state> *rhs) { return rhs->Depth() < lhs->Depth(); }
  };
  static ClearablePQ<MultiEdge<state>, std::vector<MultiEdge<state>>, MultiEdgeCmp<state>> q;
  q.clear();
  q.push(n);
  static std::vector<Mutex<state>> intersection;
  static std::vector<Mutex<state>> stuff;

  while (q.size())
  {
    //std::cout << "q:\n";
    auto s(q.top());
    q.pop();
    /*std::cout << "s:\n";
    for (auto const &g : s)
    {
      std::cout << g.first->n << "-->" << g.second->n << " ";
    }
    std::cout << (s.feasible ? "feasible" : "infeasible") << "\n";
    */
    bool done(true);
    unsigned agent(0);
    if (done)
    {
      for (auto const &g : s)
      {
        // Only check for the goal if this agent isn't unbounded cost
        if (std::find(unboundedAgents.begin(), unboundedAgents.end(), agent) == unboundedAgents.end())
        {
          if ((!env[agent]->environment->GoalTest(g.second->n, goal[agent])) || g.second->Depth() < minCost[agent])
          {
            done = false;
            //std::cout << " no goal...\n";
            break;
          }
        }
        agent++;
      }
    }
    if (done)
    {
      for (auto const &a : unboundedAgents)
      {
        terminals[a].insert(s[a]);
      }
      result = true;
      continue; // We found this combo, no further processing req'd
    }

    // Find minimum depth of current edges
    uint32_t sd(INT_MAX);
    unsigned minindex(0);
    int k(0);
    for (auto const &a : s)
    {
      if (a.second != a.first && // Ignore disappeared agents
          a.second->Depth() < sd)
      {
        sd = a.second->Depth();
        minindex = k;
      }
      k++;
      //sd=min(sd,a.second->Depth());
    }
    if (sd == INT_MAX)
    {
      sd = s[minindex].second->Depth();
    } // Can happen at root node
    //std::cout << "min-depth: " << sd << "\n";

    //Get successors into a vector
    std::vector<MultiEdge<state>> successors;
    successors.reserve(s.size());

    uint32_t md(INT_MAX); // Min depth of successors
    //Add in successors for parents who are equal to the min
    k = 0;
    for (auto const &a : s)
    {
      static MultiEdge<state> output;
      output.clear();
      if ((OD && (k == minindex /* || a.second->Depth()==0*/)) || (!OD && a.second->Depth() <= sd))
      {
        //std::cout << "Keep Successors of " << *a.second << "\n";
        for (auto const &b : a.second->successors)
        {
          output.emplace_back(a.second, b);
          md = min(md, b->Depth());
        }
      }
      else
      {
        //std::cout << "Keep Just " << *a.second << "\n";
        output.push_back(a);
        md = min(md, a.second->Depth());
      }
      if (output.empty())
      {
        // This means that this agent has reached its goal.
        // Stay at state...
        if (disappear)
        {
          output.emplace_back(a.second, a.second); // Stay, but don't increase time
        }
        else
        {
          output.emplace_back(a.second, new Node<state>(a.second->n, MAXTIME, env[k]->environment.get()));
          //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
          toDelete.push_back(output.back().second);
        }
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
      ++k;
    }
    //if(verbose){
    //std::cout << "Move set\n";
    //for(int a(0);a<successors.size(); ++a){
    //std::cout << "agent: " << a << "\n";
    //for(auto const& m:successors[a]){
    //std::cout << "  " << *m.first << "-->" << *m.second << "\n";
    //}
    //}
    //}
    static std::vector<MultiEdge<state>> crossProduct;
    crossProduct.clear();
    static MultiEdge<state> tmp;
    tmp.clear();
    tmp.feasible = s.feasible;

    // This call also computes initial mutexes
    generatePermutations(successors, crossProduct, 0, tmp, sd, radii, false); // ignore infeasible combinations

    for (auto &a : crossProduct)
    {
      a.feasible &= s.feasible;
      k = 0;
      // Compute hash for transposition table
      std::string hash(a.size() * 2 * sizeof(uint64_t) + 1, 1);
      for (auto v : a)
      {
        uint64_t h0(v.first->Hash());
        uint64_t h1(v.second->Hash());
        uint8_t c[sizeof(uint64_t) * 2];
        memcpy(c, &h0, sizeof(uint64_t));
        memcpy(&c[sizeof(uint64_t)], &h1, sizeof(uint64_t));
        for (unsigned j(0); j < sizeof(uint64_t) * 2; ++j)
        {
          hash[k * sizeof(uint64_t) * 2 + j] = ((int)c[j]) ? c[j] : 0xff; // Replace null-terminators in the middle of the string
        }
        ++k;
      }
      hash.push_back(a.feasible ? 'f' : 'i');
      // Have we visited this node already?
      if (visited.find(hash) == visited.end())
      {
        visited[hash] = true;
        //std::cout << "  pushing: ";
        //for(auto const& g:a){
        //std::cout << g.first->n << "-->" << g.second->n << " ";
        //}
        if (a.feasible)
        {
          //a.parent = &storage.back();
          //std::cout << "(prnt: ";
          //for (auto const &g : storage.back())
          //{
          //std::cout << g.first->n << "-->" << g.second->n << " ";
          //}
          //std::cout << ")";
        }
        q.push(a);
      }
      else
      {
        //std::cout << "  NOT pushing: ";
        //for(auto const& g:a){
        //std::cout << g.first->n << "-->" << g.second->n << " ";
        //}
      }
      //std::cout << "HASH: ";
      //for(auto const& c:hash){
      //std::cout << +c << ",";
      //}
      //std::cout << " " << (a.feasible?"feasible":"infeasible");
      //std::cout << "\n";
    }
  }
  return result;
}

template <typename state, typename action>
bool getMutexes(MultiEdge<state> const& n,
std::vector<state> const& goal,
std::vector<EnvironmentContainer<state,action>*> const& env,
std::vector<Node<state>*>& toDelete,
//std::vector<std::vector<Edge<state>>>& actions,
//std::vector<std::vector<std::vector<unsigned>>>& edges,
std::vector<EdgeSet<state>>const& terminals,
std::vector<EdgeSet<state>>& mutexes,
std::vector<float> const& radii,
std::vector<unsigned> const& minCost,
Solution<state>& fixed, unsigned minCostLimit,
bool disappear=true, bool OD=false){
  bool result(false);
  static const int MAXTIME(1000*state::TIME_RESOLUTION_U);
  static std::map<std::string,bool> visited;
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
  static std::vector<Mutex<state>> intersection;
  static std::vector<Mutex<state>> stuff;
  std::deque<MultiEdge<state>> storage;
  MultiEdge<state>* goalref(nullptr);
  unsigned bestCost(INT_MAX);

  while(q.size()){
    //std::cout << "q:\n";
    auto s(q.top());
    if(s.feasible)
        storage.push_back(s);
    q.pop();
    /*std::cout << "s:\n";
    for(auto const& g:s){
      std::cout << g.first->n << "-->" << g.second->n << " ";
    }
    std::cout << (s.feasible?"feasible":"infeasible") << "\n";
    */

    bool done(s.feasible);
    unsigned agent(0);
    unsigned cost(0);
    if(done){
      for(auto const& g:s){
        cost+=g.second->Depth();
        if((!env[agent]->environment->GoalTest(g.second->n,goal[agent])) || g.second->Depth()<minCost[agent]){
          done=false;
          //std::cout << " no goal...\n";
          break;
        }
        agent++;
      }
      done&=cost>=minCostLimit; // Total cost must be high enough
    }
    if (done&&cost<bestCost)
    {
      bestCost=cost;
      //std::cout << "GOAL: ";
      //for (auto const &g : storage.back())
      //{
        //std::cout << g.first->n << "-->" << g.second->n << " ";
      //}
      //std::cout << (s.feasible ? "feasible" : "infeasible") << "\n";
      goalref = &storage.back();
      result = true;
    }

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
          output.emplace_back(a.second,new Node<state>(a.second->n,MAXTIME,env[k]->environment.get()));
          //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
          toDelete.push_back(output.back().second);
        }
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
      ++k;
    }
    //if(verbose){
      //std::cout << "Move set\n";
      //for(int a(0);a<successors.size(); ++a){
        //std::cout << "agent: " << a << "\n";
        //for(auto const& m:successors[a]){
          //std::cout << "  " << *m.first << "-->" << *m.second << "\n";
        //}
      //}
    //}
    static std::vector<MultiEdge<state>> crossProduct;
    crossProduct.clear();
    static MultiEdge<state> tmp; tmp.clear();
    tmp.feasible=s.feasible;

    // This call also computes initial mutexes
    generatePermutations(successors,crossProduct,0,tmp,sd,radii);//,acts);
    //std::cout << "cross product size: " << crossProduct.size() << "\n";
    // Since we're visiting these in time-order, all parent nodes of this node
    // have been seen and their initial mutexes have been computed. Therefore
    // we can compute propagated mutexes and inherited mutexes at the same time.

    // Look for propagated mutexes...
    // For each pair of actions in s:
    static std::vector<Edge<state>> mpj;
    for(int i(0); i<s.size()-1; ++i){
      if(s[i].first==s[i].second){continue;} // Ignore disappearing at goal
      if(s[i].first->parents.size()){
        //std::cout << s[i].first->n << " has " << s[i].first->parents.size() << " parents\n";
        for(int j(i+1); j<s.size(); ++j){
          if (s[j].first == s[j].second) { continue; } // Ignore disappearing at goal
          //std::cout << "Check versus " << s[j].first->n << "\n";
          mpj.clear();
          // Get list of pi's mutexes with pjs
          for(auto const& pi:s[i].first->parents){
            //std::cout << "Parent has " << pi->mutexes.size() << " mutexes\n";

            for(auto const& mi:pi->mutexes){
              //acts[i].insert(pi); // Add to set of states which have mutexed actions
              if(mi.first==s[i].first){ // Does the edge from the parent end at this vertex?
                for(auto const& mu:mi.second){
                  // Does the mutexed edge end at the appropriate vertex?
                  // Also - is the mutex for the right agent?
                  if(std::get<1>(mu)==s[j].first&&std::get<2>(mu)==j){
                    //std::cout << "This parent (" << pi->n << ") has a mutex that coincides: " << std::get<0>(mu)->n << "-->" << std::get<1>(mu)->n << "\n";
                    mpj.emplace_back(std::get<0>(mu),std::get<1>(mu)); // Add pointers to pjs
                  }else{
                    //std::cout << "This mutex: " << std::get<0>(mu)->n << "-->" << std::get<1>(mu)->n << " does not coincide\n";
                  }
                }
              }else{
                //std::cout << pi->n << "-->" << mi.first->n << " is not the right mutexed action\n";
              }
            }
          }
          // Now check if all pjs are mutexed with the set
          if(s[j].first->parents.size()==mpj.size() && mpj.size()){
            //std::cout << "Parents size matches the number of mutexes ("<<mpj.size()<<")\n";
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
              if(s[i].first->mutexes[s[i].second].emplace(s[j].first,s[j].second,j).second)
              //std::cout << "Propagated mutex: " << s[i].first->n << "-->"<< s[i].second->n <<","<<i<< " " << s[j].first->n << "-->"<< s[j].second->n <<","<<j<<"\n";
              s[j].first->mutexes[s[j].second].emplace(s[i].first,s[i].second,i);
              //acts[i].insert(s[i].first); // Add to set of states which have mutexed actions
              //acts[j].insert(s[j].first); // Add to set of states which have mutexed actions
            }
          }
          intersection.clear();
          // Finally, see if we can add inherited mutexes
          // TODO: Might need to be a set union for non-cardinals
          //std::cout << "check inh: " <<i<<" "<< s[i].first->n.x << "," << s[i].first->n.y
                    //<< "-->" << s[i].second->n.x << "," << s[i].second->n.y << "\n";
          if(s[i].first->parents.size()){
            // Get mutexes from first parent
            auto parent(s[i].first->parents.begin());
            //std::cout << "  p: " << (*parent)->n.x << "," << (*parent)->n.y
                      //<< "-->" << s[i].first->n.x << "," << s[i].first->n.y << "\n";
            intersection.reserve((*parent)->mutexes.size());
            for(auto const& mi:(*parent)->mutexes){
              if(mi.first==s[i].first){
                for(auto const& mu:mi.second){
                  if(std::get<2>(mu)==i){
                  std::cout << "Error! agent "<< i <<" is mutexed with itself!\n";
                  }
                  //std::cout << "    mi: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
                            //<< "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << "," << std::get<2>(mu) << "\n";
                  intersection.push_back(mu);
                }
              }
            }
            ++parent;
            if(parent!=s[i].first->parents.end())
            // Now intersect the remaining parents' sets
            while(parent!=s[i].first->parents.end()){
            //std::cout << "  p: " << (*parent)->n.x << "," << (*parent)->n.y
                      //<< "-->" << s[i].first->n.x << "," << s[i].first->n.y << "\n";
              stuff.clear();
              for(auto const& mi:(*parent)->mutexes){
                if(mi.first==s[i].first){
                  for(auto const& mu:mi.second){
                  if(std::get<2>(mu)==i){
                  std::cout << "Error! agent "<< i <<" is mutexed with itself!\n";
                  }
                    /*std::cout << "    mi+: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
                              << "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y<< "," << std::get<2>(mu) << "\n";
                              */
                    stuff.push_back(mu);
                  }
                }
              }
              inplace_intersection(intersection,stuff);

              /*std::cout << "  intersection:" << "\n";
              for (auto const &mu : intersection) {
                std::cout << "    m: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
                          << "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << "\n";
              }*/
              ++parent;
            }
          }
          // Add inherited mutexes
          for(auto const& mu:intersection){
            s[i].first->mutexes[s[i].second].insert(mu);
            /*if (s[i].first->mutexes[s[i].second].insert(mu).second)
            {
              std::cout << "Inherited mutex: " << s[i].first->n << "-->" << s[i].second->n << "," << i << " " << std::get<0>(mu)->n << "-->" << std::get<1>(mu)->n << "," << std::get<2>(mu) << "\n";
              std::cout << "  FROM: ";
              for (auto const &pp : s[i].first->parents)
              {
                std::cout << pp->n.x << "," << pp->n.y << "-->" << s[i].first->n.x << "," << s[i].first->n.y << "\n";
              }
            }*/
            // Do we add a mutex in the reverse dir? - I don't think so.
            //std::get<0>(mu)->mutexes[std::get<1>(mu)].emplace(s[i].first,s[i].second,i);
            //acts[i].insert(s[i].first); // Add to set of states which have mutexed actions
            //acts[j].insert(std::get<0>(mu)); // Add to set of states which have mutexed actions
          }
          // Do the same for agent j
          intersection.clear();
          //std::cout << "check inh: " << j << " " << s[j].first->n.x << "," << s[j].first->n.y
                    //<< "-->" << s[j].second->n.x << "," << s[j].second->n.y << "\n";
          if(s[j].first->parents.size()){
            // Get mutexes from first parent
            auto parent(s[j].first->parents.begin());
            //std::cout << "  p: " << (*parent)->n.x << "," << (*parent)->n.y
                      //<< "-->" << s[j].first->n.x << "," << s[j].first->n.y << "\n";
            intersection.reserve((*parent)->mutexes.size());
            for(auto const& mj:(*parent)->mutexes){
              if(mj.first==s[j].first){
                for(auto const& mu:mj.second){
                  if(std::get<2>(mu)==j){
                  std::cout << "Error! agent "<< j <<" is mutexed with itself!\n";
                  }
                  //std::cout << "    mj: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
                            //<< "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << "," << std::get<2>(mu) << "\n";
                  intersection.push_back(mu);
                }
              }
            }
            ++parent;
            // Now intersect the remaining parents' sets
            while(parent!=s[j].first->parents.end()){
              //std::cout << "  p: " << (*parent)->n.x << "," << (*parent)->n.y
                        //<< "-->" << s[j].first->n.x << "," << s[j].first->n.y << "\n";
              stuff.clear();
              for(auto const& mj:(*parent)->mutexes){
                if(mj.first==s[j].first){
                  for(auto const& mu:mj.second){
                  if(std::get<2>(mu)==j){
                  std::cout << "Error! agent "<< j <<" is mutexed with itself!\n";
                  }
                  //std::cout << "    mj+: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
                            //<< "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << "," << std::get<2>(mu) << "\n";
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
            s[j].first->mutexes[s[j].second].insert(mu);
            /*if (s[j].first->mutexes[s[j].second].insert(mu).second)
            {
              std::cout << "Inherited mutex: " << std::get<0>(mu)->n << "-->" << std::get<1>(mu)->n << ", " << s[j].first->n << "-->" << s[j].second->n << "\n";
              std::cout << "  FROM: ";
              for (auto const &pp : s[j].first->parents)
              {
                std::cout << pp->n.x << "," << pp->n.y << "-->" << s[j].first->n.x << "," << s[j].first->n.y << "\n";
              }
            }*/
            // Do we add a mutex in the reverse dir? - I don't think so.
            //std::get<0>(mu)->mutexes[std::get<1>(mu)].emplace(s[j].first,s[j].second,j);
            //acts[i].insert(std::get<0>(mu)); // Add to set of states which have mutexed actions
            //acts[j].insert(s[j].first); // Add to set of states which have mutexed actions
          }
        }
      }
    }

    for(auto& a: crossProduct){
      a.feasible&=s.feasible;
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
        //std::cout << "  pushing: ";
        //for(auto const& g:a){
          //std::cout << g.first->n << "-->" << g.second->n << " ";
        //}
        if (a.feasible)
        {
          a.parent = &storage.back();
          //std::cout << "(prnt: ";
          //for (auto const &g : storage.back())
          //{
            //std::cout << g.first->n << "-->" << g.second->n << " ";
          //}
          //std::cout << ")";
        }
        q.push(a);
      }else{
        //std::cout << "  NOT pushing: ";
        //for(auto const& g:a){
          //std::cout << g.first->n << "-->" << g.second->n << " ";
        //}
      }
      //std::cout << "HASH: ";
      //for(auto const& c:hash){
        //std::cout << +c << ",";
      //}
        //std::cout << " " << (a.feasible?"feasible":"infeasible");
      //std::cout << "\n";
    }
  }

  // Extract path from goal
  if(goalref){
    fixed.resize(goal.size());
    for(int i(0);i<goalref->e.size();++i){
      fixed[i].push_back(goalref->e[i].second->n);
    }
    while (goalref)
    {
      for (int i(0); i < goalref->e.size(); ++i)
      {
        if (fixed[i].back() != goalref->e[i].first->n)
        {
          fixed[i].push_back(goalref->e[i].first->n);
        }
      }
      goalref=goalref->parent;
    }
    for(auto& f:fixed){
      std::reverse(f.begin(),f.end());
    }
  }

  // Create mutexes as intersection of mutexes on terminal actions
  std::vector<std::vector<Mutex<state>>> mutex(terminals.size());
  std::vector<std::vector<Mutex<state>>> tmpmu(terminals.size());

  unsigned j(0);
  // For each agent (j), determine mutexes with it's goal-terminated actions
  // in the case of multiple goal-terminated actions, those have to be intersected.
  // In the case of >2 agents, mutex sets from multiple agents are unioned together
  for(auto const& t:terminals){
    clear(mutex);
    if(t.empty()){
      if(verbose)std::cout << "Error: No terminals\n";
      return false;
    }
    auto term(t.begin());
    //std::cout << "term: " << term->first->n.x << "," << term->first->n.y
     //<< "-->" << term->second->n.x << "," << term->second->n.y << "\n";
    for (auto const &mi : term->first->mutexes) {
      if (term->second == mi.first){
        for (auto const &mu : mi.second) {
    //std::cout << "  m: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
     //<< "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << ": " << std::get<2>(mu) << "\n";
          mutex[std::get<2>(mu)].push_back(mu);
        }
      }
    }
    ++term;
    // Now intersect the remaining terminals' sets
    while(term!=t.end()){
    clear(tmpmu);
    //std::cout << "term: " << term->first->n.x << "," << term->first->n.y
     //<< "-->" << term->second->n.x << "," << term->second->n.y << "\n";
      for(auto const& mi:term->first->mutexes){
        if (term->second==mi.first){
          for(auto const& mu:mi.second){
    //std::cout << "  m+: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
     //<< "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << ": " << std::get<2>(mu) << "\n";
            tmpmu[std::get<2>(mu)].push_back(mu);
          }
        }
      }
      // Take intersection of sets (these are from the goal-terminated actions of
      // an opposing agent)
      for (unsigned i(0); i < mutex.size(); ++i)
      {
        inplace_intersection(mutex[i], tmpmu[i]);
        //for (auto const &mu : mutex[i])
        //{
          //std::cout << "  m=: " << std::get<0>(mu)->n.x << "," << std::get<0>(mu)->n.y
                    //<< "-->" << std::get<1>(mu)->n.x << "," << std::get<1>(mu)->n.y << ": " << std::get<2>(mu) << "\n";
        //}
      }
      ++term;
    }

    //std::cout << "mutexes for agent " << j << "\n";
    for (unsigned k(0); k < mutex.size(); ++k)
    {
      for (auto const &i : mutex[k])
      {
        if (std::get<0>(i)->n != std::get<1>(i)->n)
        {
          //std::cout << std::get<0>(i)->n << "-->" << std::get<1>(i)->n << "," << std::get<2>(i) << "\n";
          // Take union of sets (these are of the intersected sets)
          mutexes[k].emplace(std::get<0>(i), std::get<1>(i));
        }
      }
    }
    ++j;
  }
  return result;
}

template <typename state>
class MultiAgentAStarOpenClosedData : public AStarOpenClosedData<ActionPair<state>>
{
  public:
    MultiAgentAStarOpenClosedData() : AStarOpenClosedData<ActionPair<state>>() {}
    MultiAgentAStarOpenClosedData(const ActionPair<state> &theData,
                                  double gg1,
                                  double gg2,
                                  double hh1,
                                  double hh2,
                                  uint64_t parent,
                                  uint64_t openLoc,
                                  dataLocation location)
        : AStarOpenClosedData<ActionPair<state>>(theData, gg1 + gg2, hh1 + hh2, parent, openLoc, location), g1(gg1), g2(gg2), h1(hh1), h2(hh2) {}
    MultiAgentAStarOpenClosedData(const ActionPair<state> &theData,
                                  double g,
                                  double h,
                                  uint64_t parent,
                                  uint64_t openLoc,
                                  dataLocation location)
         {assert(!"ERROR!: 6-argument ctor is not allowed! Use the 8-arg ctor.");}
    double g1, g2, h1, h2;
  };


template <typename state>
class MultiAgentIDAData
{
public:
  MultiAgentIDAData() {}
  MultiAgentIDAData(const ActionPair<state> &theData,
                    double gg1,
                    double gg2,
                    double hh1,
                    double hh2,
                    MultiAgentIDAData* parent,
                    unsigned location)
      : data(theData),
        g1(gg1),
        g2(gg2),
        h1(hh1),
        h2(hh2),
        id(location),
        marked(false)
  {
    parents.push_back(parent);
  }
  state data;
  std::vector<MultiAgentIDAData*> parents;
  unsigned g1, g2, h1, h2, id;
  bool marked;
};

  // This comparison function will push candidates that are above the limits to the back.
  // This is tricky because:
  //    waiting at the goal adds no additional cost
  //    we have to watch whether individual f-costs go above the limit, if they do
  //    then there is no feasible solution inside the limits
  // What about re-using the open list? (continuing the search where we left off)
  //    1) the primary prioritization has now changed because of the limits so
  //       we have to ensure that new limits are geq old limits.
  //    2) we may be working in an old plateau still - that means that we have to ignore
  //       everything until we reach a new plateau
  // So we need both individual limits (for prioritization) and SOC limit (for plateau)
  // individual lower-bounds do not sort open, but serve as goal test criteria.
  /*struct AStarCompare
  {
    static double ub1;
    static double ub2;
    bool operator()(const MultiAgentAStarOpenClosedData &i1,
                    const MultiAgentAStarOpenClosedData &i2) const
    {
      if (fequal(i1.g + i1.h, i2.g + i2.h))
      {
        if (i1.data.feasible == i2.data.feasible)
        {
          return (fless(i1.g, i2.g));
        }
        return i1.data.feasible < i2.data.feasible;
      }
      return (fgreater(i1.g + i1.h, i2.g + i2.h));
    }
  };*/

// Check if an openlist node conflicts with a node from an existing path
template<typename state>
unsigned checkForTheConflict(state const*const parent, state const*const node, state const*const pathParent, state const*const pathNode, float rad1){
  if (collisionCheck2D(*parent, *node, *pathParent, *pathNode, rad1, rad1))
  {
    return 1;
  }
  return 0;
}

template <typename state>
class CATTieBreaking
{
public:
  bool operator()(const MultiAgentAStarOpenClosedData<state> &ci1, const MultiAgentAStarOpenClosedData<state> &ci2) const
  {
    // Sort according to F-cost
    if (fequal(ci1.g + ci1.h, ci2.g + ci2.h))
    {
      // Tie break toward feasible solutions
      if (ci1.data.feasible == ci2.data.feasible)
      {
        // Tie break toward fewer number of conflicts
        if (ci1.data.nc == ci2.data.nc)
        {
          // Tie break toward greater gcost
          if (fequal(ci1.g, ci2.g))
          {
            if (randomalg && ci1.data.t == ci2.data.t)
            {
              return rand() % 2;
            }
            // Tie-break toward greater time (relevant for waiting at goal)
            return ci1.data.t < ci2.data.t;
          }
          return (fless(ci1.g, ci2.g));
        }
        return ci1.data.nc > ci2.data.nc;
      }
      return ci1.data.feasible < ci2.data.feasible;
    }
    return fgreater(ci1.g + ci1.h, ci2.g + ci2.h);
  }
  static AStarOpenClosed<state, TemporalAStarPairCompare<state>, AStarOpenClosedData<state>> *openList;
  //static BucketOpenClosed<state, TemporalAStarPairCompare<state>, AStarOpenClosedData<state>> *openList;
  static std::vector<uint32_t> currentAgents;
  static unsigned collchecks;
  static bool randomalg;
  static bool useCAT;
  static UniversalConflictAvoidanceTable<state> CAT; // Conflict Avoidance Table
  static double agentRadius;
};

template <typename state>
AStarOpenClosed<state,TemporalAStarPairCompare<state>,AStarOpenClosedData<state>>* CATTieBreaking<state>::openList=0;
template <typename state>
std::vector<uint32_t> CATTieBreaking<state>::currentAgents(2);
template <typename state>
unsigned CATTieBreaking<state>::collchecks=0;
template <typename state>
bool CATTieBreaking<state>::randomalg=false;
template <typename state>
bool CATTieBreaking<state>::useCAT=false;
template <typename state>
double CATTieBreaking<state>::agentRadius=0.25;
template <typename state>
UniversalConflictAvoidanceTable<state> CATTieBreaking<state>::CAT;

// A structure which represents the current state of a pairwise search
// It is clonable so that the search can be continued where left off
// but with different parameters
template <typename environ, typename state, typename action>
struct PairwiseConstrainedSearch
{
  //typedef MultiCostLimitedEnvironment<Action<state>, unsigned, environ> MultiEnv;

  TemporalAStarPair<state,action,environ> astar;
  // Initial constructor
  // Adds a single state to the open list and sets the lower bound on cost to 0
  PairwiseConstrainedSearch(environ *env1,
                            environ *env2,
                            Heuristic<state> *heu1,
                            Heuristic<state> *heu2,
                            double lower1,
                            double upper1,
                            double lower2,
                            double upper2,
                            state const &start1,
                            state const &goal1,
                            double r1,
                            state const &start2,
                            state const &goal2,
                            double r2,
                            double socLb,
                            bool timeReasoning=false,
                            bool storeAll=false)
                            //bool waitAtGoalNeverCosts=false)
      : intervals(2),
        firstCost(true),
        validivls(2),
        ivix(2),
        ids(2),
        radii(2),
        socLim(socLb),
        lastPlateau(0),
        firstsoc(INT_MAX),
        soc(INT_MAX),
        bf(1),
        timerange(timeReasoning),
        all(storeAll)
        //waitcosts(!waitAtGoalNeverCosts)
  {
    lb={lower1,lower2};
    ub={upper1,upper2};
    start={start1,start2};
    goal={goal1,goal2};
    radii[0]=r1;
    radii[1]=r2;
    envs = {env1, env2};
    heus = {heu1, heu2};
    //env = MultiEnv(envs);
    //open.Reset(env->GetMaxHash());
    ActionPair<state> s(start[0],start[0], start[1],start[1]);
    MultiAgentAStarOpenClosedData<state> data(s,0,0,
                                   heus[0]->HCost(start[0], goal[0]),
                                   heus[1]->HCost(start[1], goal[1]),
                         0,open.theHeap.size(), kOpenList);
    data.data.nc = 0; // We assume that the root has no conflicts with other agents
    open.Reset();
    open.AddOpenNode(data,GetHash(s,0,0));
    for (auto const &e : envs)
    {
      bf *= e->branchingFactor();
    }
    intervals[0].push_back({{0,INT_MAX-1,INT_MAX-1}, Action<state>(start[0],start[0])});
    intervals[1].push_back({{0,INT_MAX-1,INT_MAX-1}, Action<state>(start[1],start[1])});
  }

  void reset()
  {
    firstCost = true;
    finalcost.clear();
    seen.clear();
    open.Reset();
    ActionPair<state> s(start[0],start[0], start[1],start[1]);
    MultiAgentAStarOpenClosedData<state> data(s,0,0,
                                   heus[0]->HCost(start[0], goal[0]),
                                   heus[1]->HCost(start[1], goal[1]),
                         0,open.theHeap.size(), kOpenList);
    data.data.nc = 0; // We assume that the root has no conflicts with other agents
    intervals[0].push_back({{0,INT_MAX-1,INT_MAX-1}, Action<state>(start[0],start[0])});
    intervals[1].push_back({{0,INT_MAX-1,INT_MAX-1}, Action<state>(start[1],start[1])});
    open.AddOpenNode(data,GetHash(s,0,0));
  }

  //Timer t;
  static bool cardinalcheck;
  std::array<environ *,2> envs; // environments for agents
  std::array<Heuristic<state> *,2> heus; // environments for agents
  //MultiEnv env;
  double lastPlateau;
  double socLim;             // Plateau for sum of costs
  double firstsoc;                // soc of first solution
  double soc;                // soc of first solution
  std::array<double,2> lb;
  std::array<double,2> ub;
  std::array<state,2> goal;
  std::array<state,2> start;

  double maxTotal;
  unsigned bf; // branching factor
  bool timerange; // Whether to reason about mutually conflicting time ranges
  bool all; // Whether to store all solutions of cost
  //bool waitcosts; // Whether waiting at the goal costs if you move off again

  std::vector<std::deque<std::pair<std::array<unsigned,5>,Action<state>>>> intervals;
  std::vector<std::unordered_map<uint64_t,unsigned>> ivix;
  std::vector<std::set<uint64_t>> validivls;
  std::vector<std::vector<unsigned>> finalcost;
  bool firstCost;
  //PairTree<ActionPair<state>> mutexes;
  const int MAXTIME=1000 * state::TIME_RESOLUTION_U;
  std::vector<double> radii;

  static AStarOpenClosed<ActionPair<state>, CATTieBreaking<state>, MultiAgentAStarOpenClosedData<state>> open;
  //BucketOpenClosed<ActionPair<state>, CATTieBreaking<state>, MultiAgentAStarOpenClosedData<state>> open;


  // Get hash for a single two-agent state
  uint64_t GetHash(state const &first,
                   state const &second,
                   uint64_t gg1,
                   uint64_t gg2,
                   bool feasible)
  {
    // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
    uint64_t h(14695981039346656037UL); // Offset basis
    uint64_t h1(0);
    unsigned i(0);
    uint8_t c[sizeof(uint64_t)];
    h1 = envs[i]->GetStateHash(first);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }

    h1 = envs[i++]->GetStateHash(second);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    h1 = gg1*0x100000000 + gg2*2 +feasible;
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    return h;
  }

  // Get hash for a single-agent action
  uint64_t GetHash(Action<state> const &v)//, uint64_t gg1)
  {
    // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
    uint64_t h(14695981039346656037UL); // Offset basis
    uint64_t h1(0);
    unsigned i(0);
    uint8_t c[sizeof(uint64_t)];
    h1 = envs[i]->GetStateHash(v.first);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }

    h1 = envs[i++]->GetStateHash(v.second);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    /*memcpy(c, &gg1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }*/
    return h;
  }

  // Get hash for a two-agent action
  uint64_t GetHash(ActionPair<state> const &node, uint32_t gg1, uint32_t gg2)
  {
    // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
    uint64_t h(14695981039346656037UL); // Offset basis
    uint64_t h1(0);
    unsigned i(0);
    uint8_t c[sizeof(uint64_t)];
    for (auto const &v : node)
    {
      h1=envs[i]->GetStateHash(v.first);
      memcpy(c, &h1, sizeof(uint64_t));
      for (unsigned j(0); j < sizeof(uint64_t); ++j)
      {
        //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
        h = h ^ c[j];          // Xor with octet
        h = h * 1099511628211; // multiply by the FNV prime
      }

      h1=envs[i++]->GetStateHash(v.second);
      memcpy(c, &h1, sizeof(uint64_t));
      for (unsigned j(0); j < sizeof(uint64_t); ++j)
      {
        //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
        h = h ^ c[j];          // Xor with octet
        h = h * 1099511628211; // multiply by the FNV prime
      }
    }
//#pragma GCC push_options
//#pragma GCC optimize ("O0")
   h1 = (gg1*0xffffffff) + gg2;
//#pragma GCC pop_options
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    return h;
  }

  // Get cost ranges. No mutex propagation.
  void getRanges(std::vector<EnvironmentContainer<state, action> *> const &ec,
                 std::vector<unsigned> const &origCosts)
  {
    static std::vector<std::vector<uint32_t>> somecosts;
    somecosts.clear();
    somecosts.reserve(2);
    somecosts.resize(1);
    somecosts[0].push_back(lb[0] - origCosts[0]);
    somecosts[0].push_back(lb[1] - origCosts[1]);

    std::vector<state> starts(start.begin(),start.end()); // Cast from array to vector
    std::vector<state> goals(goal.begin(),goal.end());
    //static std::vector<unsigned> maxs;
    //maxs = origCost;
    static std::vector<unsigned> best;
    best.assign(2, INT_MAX);

    ICTSAlgorithm<state, action> ictsalgo(std::vector<float>(radii.begin(),radii.end()),ceil(ec[0]->environment->WaitTime()/2.0));
    ictsalgo.SetVerbose(false);
    ictsalgo.SetQuiet(true); //!Params::unquiet);
    paths.resize(0);
    ictsalgo.GetSolutionCosts(ec, starts, goals, somecosts, &paths);
    finalcost=somecosts;
  }
  void getRanges()
  {
    paths.resize(0);
    paths.push_back(std::vector<std::vector<state>>(2,std::vector<state>()));
    paths[0][0].clear();
    auto tmpgoal(goal[0]);
    tmpgoal.t=lb[0];
    astar.SetHeuristic(heus[0]);
    astar.SetUpperLimit(ub[0]);
    if(ub[0]!=lb[0])
    {
    astar.SetUpperLimit(ub[0]-1);
    }
    astar.SetLowerLimit(lb[0]);
    //Timer tmr;
    //tmr.StartTimer();
    //astar.SetVerbose(true);
    astar.GetPath(envs[0],start[0],tmpgoal,paths[0][0],lb[0]);
    //std::cout << "s1 took: " << tmr.EndTimer() << "\n";
    if(paths[0][0].empty())
    {
      return;
    }
    auto len1(envs[0]->GetPathLength(paths[0][0]));
    paths[0][0].clear();

    paths[0][1].clear();
    tmpgoal=goal[1];
    tmpgoal.t=lb[1];
    astar.SetHeuristic(heus[1]);
    astar.SetUpperLimit(ub[1]);
    if(ub[1]!=lb[1])
    {
      astar.SetUpperLimit(ub[1] - 1);
    }
    astar.SetLowerLimit(lb[1]);
    //astar.SetVerbose(true);
    //tmr.StartTimer();
    astar.GetPath(envs[1],start[1],tmpgoal,paths[0][1],lb[1]);
    //std::cout << "s2 took: " << tmr.EndTimer() << "\n";
    if(paths[0][1].empty())
    {
      return;
    }
    auto len2(envs[1]->GetPathLength(paths[0][1]));
    paths[0][1].clear();
    paths.clear();

    // If either of the upper bounds is unlimited, we could run forever
    // As a termination heuristic, we will assume that if an agent were
    // to make it to its goal, it may have to wait for the other agent
    // to traverse its entire path first.
    maxTotal=envs[0]->GetMapSize()*state::TIME_RESOLUTION_U;
    if(maxTotal<socLim)
    {
      maxTotal=socLim;
    }
    //maxTotal=len1+len2;
    //cd maxTotal*=maxTotal;

    // =======================
    // Cost limit Feasibility check...
    // We only have to do this if one of the agents has an upper
    // bound.
    // =======================
    //tmr.StartTimer();
    //t.StartTimer();
    bool go(doSingleSearchStep());
    while (go)
    {
      go = doSingleSearchStep();
    }

    if(finalcost.empty())return;
    //std::cout << finalcost << "\n";
    // TODO - If we ever plan to re-use this class, we won't be able
    //        to delete the intervals inline like this...

    // ===============================
    // Finally, filter down the list of mprop constraints
    // Only those which were completely infeasible for one
    // or more complete cost levels are retained
    // ===============================
    auto finalsoc(finalcost[0][0]+finalcost[0][1]);
  }

  void getRangesAndConstraints()
  {
    // =======================
    // Constraint-based feasibility checks.
    // Plan each agent singly.
    // If the search fails, the problem is unsolvable,
    // either due to being overconstrained, or just an
    // unsolvable instance
    // =======================
    paths.resize(0);
    paths.push_back(std::vector<std::vector<state>>(2, std::vector<state>()));
    paths[0][0].clear();
    auto tmpgoal(goal[0]);
    tmpgoal.t = lb[0];
    astar.SetHeuristic(heus[0]);
    astar.SetUpperLimit(ub[0]);
    if (ub[0] != lb[0])
    {
      astar.SetUpperLimit(ub[0] - 1);
    }
    astar.SetLowerLimit(lb[0]);
    //Timer tmr;
    //tmr.StartTimer();
    //astar.SetVerbose(true);
    astar.GetPath(envs[0], start[0], tmpgoal, paths[0][0], lb[0]);
    //std::cout << "s1 took: " << tmr.EndTimer() << "\n";
    if (paths[0][0].empty())
    {
      return;
    }
    auto len1(envs[0]->GetPathLength(paths[0][0]));
    paths[0][0].clear();

    paths[0][1].clear();
    tmpgoal = goal[1];
    tmpgoal.t = lb[1];
    astar.SetHeuristic(heus[1]);
    astar.SetUpperLimit(ub[1]);
    if (ub[1] != lb[1])
    {
      astar.SetUpperLimit(ub[1] - 1);
    }
    astar.SetLowerLimit(lb[1]);
    //astar.SetVerbose(true);
    //tmr.StartTimer();
    astar.GetPath(envs[1], start[1], tmpgoal, paths[0][1], lb[1]);
    //std::cout << "s2 took: " << tmr.EndTimer() << "\n";
    if (paths[0][1].empty())
    {
      return;
    }
    auto len2(envs[1]->GetPathLength(paths[0][1]));
    paths[0][1].clear();
    paths.clear();

    // If either of the upper bounds is unlimited, we could run forever
    // As a termination heuristic, we will assume that if an agent were
    // to make it to its goal, it may have to wait for the other agent
    // to traverse its entire path first.
    maxTotal = envs[0]->GetMapSize() * state::TIME_RESOLUTION_U;
    if (maxTotal < socLim)
    {
      maxTotal = socLim;
    }
    //maxTotal=len1+len2;
    //cd maxTotal*=maxTotal;

    // =======================
    // Cost limit Feasibility check...
    // We only have to do this if one of the agents has an upper
    // bound.
    // =======================
    //tmr.StartTimer();
    //t.StartTimer();
    if (cardinalcheck)
    {
      auto origall(all);
      all = false;
      // Check if cost increase is necessary.
      bool go(doSingleSearchStep());
      while (go)
      {
        go = doSingleSearchStep();
      }
      //std::cout << "p1 took: " << tmr.EndTimer() << "\n";
      if (finalcost.empty())
      {
        return;
      }
      all = origall;
      if (finalcost[0][0] + finalcost[0][1] <= socLim)
      {
        reset();
        go = doSingleSearchStep();
        while (go)
        {
          go = doSingleSearchStep();
        }
        return;
      }
      reset();
    }

    // =======================
    // Do full joint-agent planning and mutex-propagation
    // at the same time
    // =======================
    //tmr.StartTimer();
    bool go(doSingleSearchStepWithMutexes());
    while (go)
    {
      go = doSingleSearchStepWithMutexes();
    }
    //std::cout << "p2 took: " << tmr.EndTimer() << "\n";
    // Now that we're finished, search the mutexes to find constraints
    //for( auto const& m:mutexes){
    //std::cout << m << "\n";
    //}
    /*for (auto const &sln : paths)
    {
      finalcost.resize(finalcost.size()+1);
      unsigned i(0);
      for (auto const &p : sln)
      {
        if (p.size())
          finalcost.back().push_back(envs[i]->GetPathLength(p));
        ++i;
      }
    }*/
    if (finalcost.empty())
      return;
    //std::cout << finalcost << "\n";
    // TODO - If we ever plan to re-use this class, we won't be able
    //        to delete the intervals inline like this...

    // ===============================
    // Finally, filter down the list of mprop constraints
    // Only those which were completely infeasible for one
    // or more complete cost levels are retained
    // ===============================
    auto finalsoc(finalcost[0][0] + finalcost[0][1]);
    std::array<unsigned, 2> mins = {UINT_MAX, UINT_MAX};
    for (unsigned i(0); i < finalcost.size() - 1; ++i)
    {
      if (finalcost[i][0] < mins[0])
      {
        mins[0] = finalcost[i][0];
      }
      if (finalcost[i][1] < mins[1])
      {
        mins[1] = finalcost[i][1];
      }
    }
    if (validivls.size() < intervals.size())
    {
      decltype(intervals) good(2);

      for (unsigned i(0); i < intervals.size(); ++i)
      {
        unsigned other(i ? 0 : 1);
        for (auto const &ix : validivls[i])
        {
          auto ivl(ivix[i][ix]);
          if (ivl)
          {
            auto j(intervals[i][ivl]);
            assert(GetHash(j.second) == ix);
            if (j.first[0] < j.first[2] &&
                j.first[1] <= j.first[0] &&
                j.first[0] >= mins[other])
            {
              //std::cout << " included:" << GetHash(j.second) << "\n";
              good[i].push_back(j);
            }
            //else
            //{
            //std::cout << "Pruning " << j << "\n";
            //}
          }
        }
      }
      intervals = good;
    }
    else
    {
      //std::cout << "valid ivls " << validivls[0].size() << " intervals " << intervals[0].size() << "\n";
      //std::cout << "valid ivls " << validivls[1].size() << " intervals " << intervals[1].size() << "\n";
      //std::cout << "good " << good[0].size() << "\n";
      //std::cout << "good " << good[1].size() << "\n";

      //decltype(intervals) bad_time(2);
      //decltype(intervals) bad(2);
      for (unsigned i(0); i < intervals.size(); ++i)
      {
        unsigned other(i ? 0 : 1);
        //std::cout << "Constraints for agent " << i << ":\n";
        //std::cout << intervals[i][0] << "dummy\n";
        intervals[i].erase(intervals[i].begin());
        for (auto j(intervals[i].begin()); j != intervals[i].end(); /*++j*/)
        {
          // We can only trust cost combinations that make sense
          // if the upper bound on the constraint minus the lb on the other agent
          // is more than the sum of costs, it can't be trusted, because we never
          // looked deep enough into the structure.
          if (j->first[0] < j->first[2] &&
              j->first[1] <= j->first[0] &&
              j->first[0] >= mins[other])
          {
            auto hash(GetHash(j->second));
            if (validivls[i].find(hash) == validivls[i].end())
            {
              //std::cout << " Pruning (" << i << ") " << *j << "\n";
              //bad[i].push_back(*j);
              j = intervals[i].erase(j);
            }
            else
            {
              //std::cout << " Including (" << i << ") " << *j << "\n";
              ++j;
            }
          }
          else
          {
            //bad_time[i].push_back(*j);
            //std::cout << "ignore "<< i << " " << *j << "\n";
            j = intervals[i].erase(j);
          }
        }
      }
      //std::cout << "filtered intervals " << intervals[0].size() << "\n";
      //std::cout << "filtered intervals " << intervals[1].size() << "\n";
      //assert(intervals[0].size() == good[0].size());
      //assert(intervals[1].size() == good[1].size());
      //std::cout << "bad-time: " << bad_time << "\n";
      //std::cout << "bad: " << bad << "\n";
      /*for (auto const &o : open.elements)
      {
        //auto o(open.elements[n]);
        if (o.data.feasible == false)
        {
          for (unsigned i(0); i < 2; ++i)
          {
            for (auto const &b : bad[i])
            {
              bool found(true);
              // Found a node in the open list (there may be more than one).
              if (b.second == o.data[i])
              {
                found = true;
                // Check whether this has any parents in the "good" list
                auto tmpnode(o.parentID);
                while (true)
                {
                  auto const &nn(open.Lookup(tmpnode));
                  if (nn.data.feasible || nn.parentID == tmpnode)
                  {
                    break;
                  }
                  for (auto const &g : good[i])
                  {
                    if (g.second == nn.data[i])
                    {
                      found = true;
                      break;
                    }
                  }
                  if (found)
                    break;
                }
              }
              if (!found)
              {
                std::cout << "Counter example found: " << b << "\n";
                assert(!"Counter example found");
              }
            }
          }
        }
      }*/
    }
  }

  //From https://www.geeksforgeeks.org/longest-repeating-and-non-overlapping-substring/
  std::string longestRepeatedSubstring(std::string str)
  {
    int n = str.length();
    int LCSRe[n + 1][n + 1];

    // Setting all to 0
    memset(LCSRe, 0, sizeof(LCSRe));

    string res;         // To store result
    int res_length = 0; // To store length of result

    // building table in bottom-up manner
    int i, index = 0;
    for (i = 1; i <= n; i++)
    {
      for (int j = i + 1; j <= n; j++)
      {
        // (j-i) > LCSRe[i-1][j-1] to remove
        // overlapping
        if (str[i - 1] == str[j - 1] &&
            LCSRe[i - 1][j - 1] < (j - i))
        {
          LCSRe[i][j] = LCSRe[i - 1][j - 1] + 1;

          // updating maximum length of the
          // substring and updating the finishing
          // index of the suffix
          if (LCSRe[i][j] > res_length)
          {
            res_length = LCSRe[i][j];
            index = max(i, index);
          }
        }
        else
          LCSRe[i][j] = 0;
      }
    }

    // If we have non-empty result, then insert all
    // characters from first character to last
    // character of string
    if (res_length > 0)
      for (i = index - res_length + 1; i <= index; i++)
        res.push_back(str[i - 1]);

    return res;
  }

  bool foundInitialSolution() const{
    return paths.size();
  }

  // Simply finds the first solution within the constraints
  // Does not retain info required for mutex propagation
  bool doSingleSearchStep()
  {
    if (!open.OpenSize())
    {
      return false;
    }

    // Get next candidate off open
    uint64_t nodeid = open.Close();
    // Note - this reference becomes invalid once we insert anything into open.
    auto &node(open.Lookup(nodeid));

    std::array<double, 2> G = {node.g1, node.g2};
    //auto f1(G[0] + node.h1);
    //auto f2(G[1] + node.h2);
    //if(!all)soc=f1+f2;
    // Check that we're below the bounds
    //if (ub[0] < f1 || ub[1] < f2)
    //{
    //return true;
    //}
    auto s(node.data);
    if(node.g > firstsoc)
    {
        //if (firstCost)
        //{
          //t.EndTimer();
          //std::cout << "Took " << t.GetElapsedTime() << " " << open.size() << " to find next cost\n";
          //t.StartTimer();
          //firstCost = false;
        //}
      if (s.feasible)
      {
        if (envs[0]->GoalTest(s[0].second, goal[0]) &&
            envs[1]->GoalTest(s[1].second, goal[1]))
        {
          bool a1wait(s[0].first.sameLoc(s[0].second));
          bool a2wait(s[1].first.sameLoc(s[1].second));

          // Make sure both aren't waiting at the goal
          if (!(a1wait && a2wait))
          {
            //t.EndTimer();
            //std::cout << "Took " << t.GetElapsedTime() << " " << open.size() << " to find last goal\n";
            // Add paths and return
            paths.push_back(std::vector<std::vector<state>>(2));
            // Extract the path back to the root.
            auto tmpnode(nodeid);
            do
            {
              auto const &tmpn(open.Lookup(tmpnode));
              //std::cout << open.Lookup(tmpnode).data << "\n";
              for (unsigned q(0); q < tmpn.data.size(); ++q)
              {
                auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                if (paths.back()[q].empty() || paths.back()[q].back() != tmpn.data[q].second)
                {
                  paths.back()[q].push_back(tmpn.data[q].second);
                  //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                }
                else
                {
                  //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                }
              }
              tmpnode = tmpn.parentID;
            } while (open.Lookup(tmpnode).parentID != tmpnode);
            auto const &tmpn(open.Lookup(tmpnode));
            for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
            {
              auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
              if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
              {
                paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
              }
              else
              {
                //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
              }
              std::reverse(paths.back()[q].begin(), paths.back()[q].end());
            }
            std::vector<unsigned> fincosts(2);
            fincosts[0] = envs[0]->GetPathLength(paths.back()[0]);
            fincosts[1] = envs[1]->GetPathLength(paths.back()[1]);
            finalcost.push_back(fincosts);

            return false;
          }
        }
      }
    }
    // Early termination criteria so we don't run forever.
    if (G[0] + node.h1 > maxTotal + 1 || G[1] + node.h2 > maxTotal + 1)
    {
      return true;
    }

    // Add this node to the interval tree
    auto shash(GetHash(s.first.second, s.second.second, G[0], G[1], s.feasible));
    if (seen.find(shash) == seen.end())
    {
      seen.insert(shash);
    }
    else
    {
      return true;
    }
    //std::cout <<std::fixed <<"pop " << s << " f: " << node.h+node.g << "\n";

    //auto G(node.g);

    // First:
    // Find minimum time of current edges
    double sd(DBL_MAX);
    unsigned minindex(0);
    double k(0);
    for (auto const &a : s)
    {
      if (a.second.t < sd)
      {
        sd = a.second.t;
        minindex = k;
      }
      k++;
    }
    if (sd == DBL_MAX)
    {
      sd = s[minindex].second.t;
    } // Can happen at root node

    //Get successors into a vector
    successors.clear();
    successors.reserve(s.size());

    //Add in successors for parents who are equal to the min
    std::vector<std::vector<unsigned>> ecs(2);
    k = 0;
    for (auto const &a : s)
    {
      static std::vector<Action<state>> output;
      output.clear();
      if (a.second.t <= sd)
      {
        //std::cout << "Keep Successors of " << *a.second << "\n";
        std::vector<state> succ(envs[k]->branchingFactor()*2);
        auto sz(envs[k]->GetSuccessors(a.second, &succ[0]));
        for (unsigned j(0); j < sz; ++j)
        {
          // We assume here that GCost will never charge for waiting at the goal
          // Wait actions at the goal cost (unless we're at the end)
          //if (waitcosts && G[k] < lb[k] && envs[k]->GoalTest(a.second,goal[k]) && envs[k]->GoalTest(succ[j],goal[k]))
          //{
          //ecs[k].push_back(succ[j].t - a.second.t);
          //}
          //else
          //{
          ecs[k].push_back(envs[k]->GCost(a.second, succ[j]));
          //}
          output.emplace_back(a.second, succ[j]);
        }
      }
      else
      {
        // We assume here that GCost will never charge for waiting at the goal
        // Wait actions at the goal cost (unless we're at the end)
        //if (waitcosts && G[k] < lb[k] && envs[k]->GoalTest(a.first,goal[k]) && envs[k]->GoalTest(a.second,goal[k]))
        //{
        //ecs[k].push_back(a.second.t-a.first.t);
        //}
        //else
        //{
        // We are re-using this, so don't add the edge cost again...
        ecs[k].push_back(0); //envs[k]->GCost(a.first, a.second));
        //ecs[k].push_back(envs[k]->GCost(a.first, a.second));
        //}
        output.push_back(a);
      }
      if (output.empty())
      {
        // All movements from this position at this time are blocked
        return true;
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
      ++k;
    }
    static std::vector<ActionPair<state>> crossProduct;
    crossProduct.clear();
    crossProduct.reserve(successors[0].size() * successors[1].size());
    unsigned i(0);
    for (auto const &a1 : successors[0])
    {
      auto h1(heus[0]->HCost(a1.second, goal[0]));
      unsigned j(0);
      auto const &ec1(ecs[0][i]);
      for (auto const &a2 : successors[1])
      {
        auto h2(heus[1]->HCost(a2.second, goal[1]));
        auto const &ec2(ecs[1][j]);
        crossProduct.emplace_back(a1, a2, s.feasible);
        auto &n(crossProduct.back());
        if (CATTieBreaking<state>::useCAT)
        {
          // Compute cumulative conflicts
          static std::vector<state const *> matches;
          matches.clear();
          //std::cout << "Getting NC for " << i1.data << ":\n";
          CATTieBreaking<state>::CAT.get(n.t, std::max(n.first.second.t, n.second.second.t), matches, CATTieBreaking<state>::currentAgents);

          // Get number of conflicts in the parent
          unsigned nc1(s.nc);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for (unsigned m(1); m < matches.size(); ++m)
          {
            nc1 += collisionCheck2D(n.first.first, n.first.second, *matches[m - 1], *matches[m], radii[0], radii[0]);
            nc1 += collisionCheck2D(n.second.first, n.second.second, *matches[m - 1], *matches[m], radii[1], radii[1]);
            //if(!nc2){std::cout << "NO ";}
            //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          n.nc = nc1;
        }

        if (s.feasible)
        {
          // Check for conflict...
          if ((a1.first == a2.first) ||
              (a1.second == a2.second) ||
              (a1.first.sameLoc(a2.second) &&
               a2.first.sameLoc(a1.second)))
          {
            n.feasible = false;
          }
          else
          {
            //std::cout<<"Checking:"<<current[j].first << "-->"<< current[j].second <<", " << positions[agent][i].first << "-->"<< positions[agent][i].second << "\n";
            if (collisionCheck2D(a1.first, a1.second,
                                 a2.first, a2.second,
                                 radii[0], radii[1]))
            {
              n.feasible = false;
            }
          }
        }
        if(!n.feasible){++j;continue;}
        //std::cout << "  succ: " << n << " "<<n.feasible << "\n";
        auto gg1(G[0] + ec1);
        auto gg2(G[1] + ec2);
        // Make sure this node will come in under the upper bound
        if (gg1 + h1 < ub[0] && gg2 + h2 < ub[1])
        {
          bool a1done(envs[0]->GoalTest(n[0].second, goal[0]));
          bool a2done(envs[1]->GoalTest(n[1].second, goal[1]));
          bool a1wait(n[0].first.sameLoc(n[0].second));
          bool a2wait(n[1].first.sameLoc(n[1].second));
          // Make sure both aren't waiting at the goal
          if (!(a2done && a1done && a1wait && a2wait))
          {
            //if(n.feasible)
            //std::cout << "a0 lb:" << lb[0] <<": " << ub[0] << "/a1 lb:" << lb[1] <<": " << ub[1] << " " << n[0]
            //<< "(f="<< (gg1+h1) << ") feasible with " << n[1]  << "(f="<< (gg2+h2) << ")\n";
            // Legit arrival at goal
            // check goal
            //if(soc>=gg1+gg2)
            //{
            if (a1done && a2done) // At goal
            {
              //std::cout << "  {g:" << n << "f1:" << gg1 << "+" << h1 << "=" << (gg1 + h1) << ",f2:" << gg2 << "+" << h2 << "=" << (gg2+ h2)  << " feasible: " << n.feasible << "}\n";
              if (G[0] + G[1] + ec1 + ec2 >= socLim && // Must be above the frontier
                  gg1 >= lb[0] &&                      // Must be above lower bound
                  gg2 >= lb[1] &&                      // Must be above lower bound
                  gg1 < ub[0] &&                       // Must be below upper bound
                  gg2 < ub[1])                         // Must be below upper bound
              {
                if (firstsoc == INT_MAX || gg1 + gg2 == firstsoc)
                {
                  if (n.feasible) // Reachable
                  {
                    //if (firstsoc == INT_MAX)
                    ////{
                      //t.EndTimer();
                      //std::cout << "Took " << t.GetElapsedTime() << " " << open.size() << " to find first goal\n";
                      //t.StartTimer();
                    //}
                    //std::cout << "Solution found: " << gg1 << "+" << gg2 << "=" << gg1+gg2 << "\n";
                    // TODO: CAT check (if found duplicate)
                    paths.push_back(std::vector<std::vector<state>>(2));
                    // Extract the path back to the root.
                    auto tmpnode(nodeid);
                    paths.back()[0].push_back(n[0].second);
                    //std::cout << "*" << n[0].second << "\n";
                    paths.back()[1].push_back(n[1].second);
                    //std::cout << "      *" << n[1].second << "\n";
                    do
                    {
                      auto const &tmpn(open.Lookup(tmpnode));
                      //std::cout << open.Lookup(tmpnode).data << "\n";
                      for (unsigned q(0); q < tmpn.data.size(); ++q)
                      {
                        auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                        if (paths.back()[q].back() != tmpn.data[q].second)
                        {
                          paths.back()[q].push_back(tmpn.data[q].second);
                          //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                        }
                        else
                        {
                          //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                        }
                      }
                      tmpnode = tmpn.parentID;
                    } while (open.Lookup(tmpnode).parentID != tmpnode);
                    auto const &tmpn(open.Lookup(tmpnode));
                    for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
                    {
                      auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                      if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
                      {
                        paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                        //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                      }
                      else
                      {
                        //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                      }
                      std::reverse(paths.back()[q].begin(), paths.back()[q].end());
                    }
                    std::vector<unsigned> fincosts(2);
                    fincosts[0] = envs[0]->GetPathLength(paths.back()[0]);
                    fincosts[1] = envs[1]->GetPathLength(paths.back()[1]);
                    if (all)
                    {
                      auto ix(std::find(finalcost.begin(), finalcost.end(), fincosts));
                      if (ix == finalcost.end())
                      {
                        finalcost.push_back(fincosts);
                      }
                      else
                      {
                        //TODO: here is where we would perform the CAT check.
                        // For now, this is a duplicate, so we throw it out. :(
                        paths.pop_back();
                      }
                    }
                    else
                    {
                      finalcost.push_back(fincosts);
                    }

                    if (firstsoc == INT_MAX)
                    {
                      firstsoc = soc = gg1 + gg2;
                    }
                    if (!all)
                    {
                      return false;
                    }
                  }
                }
              }
            }
            //}
            uint64_t hash(GetHash(n, gg1, gg2));
            //std::cout << "hash " << hash << "\n";
            uint64_t theID(0);
            switch (open.Lookup(hash, theID))
            {
            case kClosedList:
              // Closed list guy is not feasible but this one is!
              {
                // This should never happen in this context
                auto &cand(open.Lookup(theID));
                if (gg1 + gg2 <= cand.g)
                {
                  cand.parentID = nodeid;
                  cand.g = gg1 + gg2;
                  cand.h = h1 + h2;
                  cand.g1 = gg1;
                  cand.h1 = h1;
                  cand.g2 = gg2;
                  cand.h2 = h2;
                  cand.data.feasible = true;

                  open.Reopen(theID);
                  //std::cout << "Update " << n << " id:" << theID << " to feasible\n";
                }
              }
              break;
            case kOpenList:
              // previously generated node is not feasible but this one is!
              {
                // Replace if infeasible or has better cost
                auto &cand(open.Lookup(theID));
                if (gg1 + gg2 <= cand.g)
                {
                  cand.parentID = nodeid;
                  cand.g = gg1 + gg2;
                  cand.h = h1 + h2;
                  cand.g1 = gg1;
                  cand.h1 = h1;
                  cand.g2 = gg2;
                  cand.h2 = h2;
                  cand.data.feasible = true;
                  open.KeyChanged(theID);
                  //std::cout << "Update " << n << " id:" << theID << " to feasible\n";
                }
              }
              break;
            case kNotFound:
              // Add to open :)
              if (gg1 + h1 < ub[0] && gg2 + h2 < ub[1])
              {
                //MultiAgentAStarOpenClosedData<state> data(n, gg1, gg2, h1, h2,
                                                          //nodeid, open.theHeap.size(), kOpenList);
                open.EmplaceOpenNode(hash, n, gg1, gg2, h1, h2,nodeid, open.theHeap.size(), kOpenList);
                //open.Lookup(hash, theID);
              }
              //std::cout << std::fixed << "  Add " << n << " id:" << theID << " f: " << (gg1+h1+gg2+h2) << " g1:" << gg1 << " g2:" << gg2 << " h1:" << h1
              //<< " h2:" << h2 << " f1:"<< gg1+h1 << " f2:" << gg2+h2 << "\n";
              break;
            }
          }
        }
        ++j;
      }
      ++i;
    }
    return true;
  }

  // This search includes infeasible nodes and tracks mutexes
  // It is exponential in nature - running doSingleSearchStep() to completion
  // before this is recommended

  bool doSingleSearchStepWithMutexes()
  {
    if (!open.OpenSize())
    {
      return false;
    }

    // Get next candidate off open
    uint64_t nodeid = open.Close();
    // Note - this reference becomes invalid once we insert anything into open.
    auto &node(open.Lookup(nodeid));
    auto F(node.g+node.h);
    //std::cout << "{d:" << node.data << "f1:" << node.g1 << "+" << node.h1 << "=" << (node.g1+node.h1) << ",f2:" << node.g2 << "+" << node.h2 << "=" << (node.g2+node.h2) << ",id:" << nodeid << ",p:" << node.parentID << " feasible: "<< node.data.feasible <<"}\n";
    if (soc>firstsoc)
    {
      return false; // Return because a solution was already found and we have finished the plateau
    }

    std::array<double,2> G={node.g1,node.g2};
    //auto f1(G[0] + node.h1);
    //auto f2(G[1] + node.h2);
    //if(!all)soc=f1+f2;
    // Check that we're below the bounds
    //if (ub[0] < f1 || ub[1] < f2)
    //{
      //return true;
    //}
    auto s(node.data);
    if(node.g > firstsoc)
    {
      //if (firstCost)
      //{
        //t.EndTimer();
        //std::cout << "(mutexed)Took " << t.GetElapsedTime() << " " << open.size() << " to find next cost\n";
        //t.StartTimer();
        //firstCost = false;
      //}
      if (s.feasible)
      {
        if (envs[0]->GoalTest(s[0].second, goal[0]) &&
            envs[1]->GoalTest(s[1].second, goal[1]))
        {
          bool a1wait(s[0].first.sameLoc(s[0].second));
          bool a2wait(s[1].first.sameLoc(s[1].second));

          // Make sure both aren't waiting at the goal
          if (!(a1wait && a2wait))
          {
            //t.EndTimer();
            //std::cout << "(mutexed)Took " << t.GetElapsedTime() << " " << open.size() << " to find last goal\n";
            // Add paths and return
            paths.push_back(std::vector<std::vector<state>>(2));
            // Extract the path back to the root.
            auto tmpnode(nodeid);
            do
            {
              auto const &tmpn(open.Lookup(tmpnode));
              //std::cout << open.Lookup(tmpnode).data << "\n";
              for (unsigned q(0); q < tmpn.data.size(); ++q)
              {
                auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                if (paths.back()[q].empty() || paths.back()[q].back() != tmpn.data[q].second)
                {
                  paths.back()[q].push_back(tmpn.data[q].second);
                  //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                }
                else
                {
                  //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                }
              }
              tmpnode = tmpn.parentID;
            } while (open.Lookup(tmpnode).parentID != tmpnode);
            auto const &tmpn(open.Lookup(tmpnode));
            for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
            {
              auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
              if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
              {
                paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
              }
              else
              {
                //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
              }
              std::reverse(paths.back()[q].begin(), paths.back()[q].end());
            }
            std::vector<unsigned> fincosts(2);
            fincosts[0] = envs[0]->GetPathLength(paths.back()[0]);
            fincosts[1] = envs[1]->GetPathLength(paths.back()[1]);
            finalcost.push_back(fincosts);

            return false;
          }
        }
      }
    }
    // Early termination criteria so we don't run forever.
    if(G[0]+node.h1>maxTotal+1 || G[1]+node.h2>maxTotal+1)
    {
      return true;
    }

    // Add this node to the interval tree
    auto shash(GetHash(s.first.second,s.second.second,G[0],G[1],s.feasible));
    if(seen.find(shash)==seen.end()){
      seen.insert(shash);
    }else{
      //std::cout << std::fixed << "Already saw equivalent " << shash << s << "feasible: " << s.feasible << " f: " << node.h+node.g << " g1:" << node.g1 << " g2:" << node.g2 << " h1:" << node.h1
                //<< " h2:" << node.h2 << " f1:" << node.g1 + node.h1 << " f2:" << node.g2 + node.h2 << "\n";
      return true;
    }
    //std::cout << std::fixed << "Pop " << shash << s << "feasible: " << s.feasible << " f: " << node.h+node.g << " g1:" << node.g1 << " g2:" << node.g2 << " h1:" << node.h1
              //<< " h2:" << node.h2 << " f1:" << node.g1 + node.h1 << " f2:" << node.g2 + node.h2 << "\n";

    //auto G(node.g);

    // First:
    // Find minimum time of current edges
    double sd(DBL_MAX);
    unsigned minindex(0);
    double k(0);
    bool doCtime(!s.feasible && timerange);
    std::array<std::pair<double,double>,2> pivl;
    std::array<unsigned,2> pshift = {s.first.second.t-s.first.first.t,s.second.second.t-s.second.first.t};
    pivl[0] = pivl[1] = {0,INT_MAX};
    for (auto const &a : s)
    {
      if (a.second.t < sd)
      {
        sd = a.second.t;
        minindex = k;
      }
      if (doCtime)
      {
        auto phash(GetHash(a)); //,G[i]+ecs[i][a]));
        auto pivld(ivix[k].find(phash));
        if (pivld != ivix[k].end())
        {
          doCtime = false;
          pivl[k].first = intervals[k][pivld->second].first[3];
          pivl[k].second = intervals[k][pivld->second].first[4];
        }
      }
      k++;
    }
    if (doCtime)
    {
      auto intvl(getForbiddenInterval(
          s.first.first,
          s.first.second,
          s.first.first.t,
          s.first.second.t,
          radii[0],
          s.second.first,
          s.second.second,
          s.second.first.t,
          s.second.second.t,
          radii[1]));
      pivl[0] = {s.second.first.t + intvl.first, s.second.first.t + intvl.second};
      pivl[1] = {s.first.first.t - intvl.second, s.first.first.t - intvl.first};
    }
    if (sd == DBL_MAX)
    {
      sd = s[minindex].second.t;
    } // Can happen at root node

    //Get successors into a vector
    successors.clear();
    successors.reserve(s.size());

    //Add in successors for parents who are equal to the min
    std::vector<std::vector<unsigned>> ecs(2);
    k = 0;
    for (auto const &a : s)
    {
      static std::vector<Action<state>> output;
      output.clear();
      if (a.second.t <= sd)
      {
        //std::cout << "Keep Successors of " << *a.second << "\n";
        std::vector<state> succ(envs[k]->branchingFactor()*2);
        auto sz(envs[k]->GetSuccessors(a.second, &succ[0]));
        for (unsigned j(0); j < sz; ++j)
        {
          // We assume here that GCost will never charge for waiting at the goal
          // Wait actions at the goal cost (unless we're at the end)
          //if (waitcosts && G[k] < lb[k] && envs[k]->GoalTest(a.second,goal[k]) && envs[k]->GoalTest(succ[j],goal[k]))
          //{
            //ecs[k].push_back(succ[j].t - a.second.t);
          //}
          //else
          //{
            ecs[k].push_back(envs[k]->GCost(a.second, succ[j]));
          //}
          output.emplace_back(a.second, succ[j]);
        }
      }
      else
      {
        // We assume here that GCost will never charge for waiting at the goal
        // Wait actions at the goal cost (unless we're at the end)
        //if (waitcosts && G[k] < lb[k] && envs[k]->GoalTest(a.first,goal[k]) && envs[k]->GoalTest(a.second,goal[k]))
        //{
          //ecs[k].push_back(a.second.t-a.first.t);
        //}
        //else
        //{
          // We are re-using this, so don't add the edge cost again...
          ecs[k].push_back(0);//envs[k]->GCost(a.first, a.second));
          //ecs[k].push_back(envs[k]->GCost(a.first, a.second));
        //}
        pshift[k]=0; // This action is being reused, so the parent is itself
        output.push_back(a);
      }
      if (output.empty())
      {
        // All movements from this position at this time are blocked
        return true;
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
      ++k;
    }
    static std::vector<std::array<unsigned,4>> unsafeTimes;
    unsafeTimes.clear();
    unsafeTimes.reserve(successors[0].size()*successors[1].size());
    static std::vector<ActionPair<state>> crossProduct;
    crossProduct.clear();
    crossProduct.reserve(successors[0].size()*successors[1].size());
    ids[0].resize(successors[0].size());
    ids[1].resize(successors[1].size());
    for(auto& id:ids[1])
    {
      id.clear();
    }
    k=0;
    unsigned i(0);
    for (auto const &a1 : successors[0])
    {
      ids[0][i].clear();
      auto h1(heus[0]->HCost(a1.second, goal[0]));
      unsigned j(0);
      auto const& ec1(ecs[0][i]);
      for (auto const &a2 : successors[1])
      {
        auto h2(heus[1]->HCost(a2.second, goal[1]));
        auto const &ec2(ecs[1][j]);
        crossProduct.emplace_back(a1, a2, s.feasible);
        auto &n(crossProduct.back());
        if (CATTieBreaking<state>::useCAT)
        {
          // Compute cumulative conflicts
          static std::vector<state const *> matches;
          matches.clear();
          //std::cout << "Getting NC for " << i1.data << ":\n";
          CATTieBreaking<state>::CAT.get(n.t, std::max(n.first.second.t, n.second.second.t), matches, CATTieBreaking<state>::currentAgents);

          // Get number of conflicts in the parent
          unsigned nc1(s.nc);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for (unsigned m(1); m < matches.size(); ++m)
          {
            nc1 += collisionCheck2D(n.first.first, n.first.second, *matches[m - 1], *matches[m], radii[0], radii[0]);
            nc1 += collisionCheck2D(n.second.first, n.second.second, *matches[m - 1], *matches[m], radii[1], radii[1]);
            //if(!nc2){std::cout << "NO ";}
            //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          n.nc = nc1;
        }

        if (s.feasible)
        {
          // Check for conflict...
          if ((a1.first == a2.first) ||
              (a1.second == a2.second) ||
              (a1.first.sameLoc(a2.second) &&
               a2.first.sameLoc(a1.second)))
          {
            n.feasible = false;
          }
          else
          {
            //std::cout<<"Checking:"<<current[j].first << "-->"<< current[j].second <<", " << positions[agent][i].first << "-->"<< positions[agent][i].second << "\n";
            if (collisionCheck2D(a1.first, a1.second,
                                 a2.first, a2.second,
                                 radii[0], radii[1]))
            {
              n.feasible = false;
            }
          }
          if(timerange)
          {
            if (!n.feasible)
            {
              // Compute the forbidden interval
              auto intvl(getForbiddenInterval(
                  n.first.first,
                  n.first.second,
                  n.first.first.t,
                  n.first.second.t,
                  radii[0],
                  n.second.first,
                  n.second.second,
                  n.second.first.t,
                  n.second.second.t,
                  radii[1]));
              unsafeTimes.push_back({n.second.first.t + intvl.first,
                                     n.second.first.t + intvl.second,
                                     n.first.first.t - intvl.second,
                                     n.first.first.t - intvl.first});
            }
            else
            {
              // Placeholder...
              unsafeTimes.push_back({0, INT_MAX, 0, INT_MAX});
            }
          }
          else
          {
              unsafeTimes.push_back({0, INT_MAX, 0, INT_MAX});
          }
        }
        else
        {
          // Fill with time-offset parent conflict time
          if (timerange)
          {
            unsafeTimes.push_back({pivl[0].first + pshift[0],
                                   pivl[0].second + pshift[0],
                                   pivl[1].first + pshift[1],
                                   pivl[1].second + pshift[1]});
          }
          else
          {
            unsafeTimes.push_back({0, INT_MAX, 0, INT_MAX});
          }
        }

        //std::cout << "  " << n << " "<<n.feasible << "\n";
        auto gg1(G[0] + ec1);
        auto gg2(G[1] + ec2);
        // Make sure this node will come in under the upper bound
        if (gg1 + h1 < ub[0] && gg2 + h2 < ub[1])
        {
          bool a1done(envs[0]->GoalTest(n[0].second, goal[0]));
          bool a2done(envs[1]->GoalTest(n[1].second, goal[1]));
          bool a1wait(n[0].first.sameLoc(n[0].second));
          bool a2wait(n[1].first.sameLoc(n[1].second));
          // Make sure both aren't waiting at the goal
          if (!(a2done && a1done && a1wait && a2wait))
          {
            //if(n.feasible)
            //std::cout << "a0 lb:" << lb[0] <<": " << ub[0] << "/a1 lb:" << lb[1] <<": " << ub[1] << " " << n[0]
            //<< "(f="<< (gg1+h1) << ") feasible with " << n[1]  << "(f="<< (gg2+h2) << ")\n";
            // Legit arrival at goal
            // check goal
            //if(soc>=gg1+gg2)
            //{
            if (a1done && a2done) // At goal
            {
              //std::cout << "  {g:" << n << "f1:" << gg1 << "+" << h1 << "=" << (gg1 + h1) << ",f2:" << gg2 << "+" << h2 << "=" << (gg2+ h2)  << " feasible: " << n.feasible << "}\n";
              if (G[0] + G[1] + ec1 + ec2 >= socLim && // Must be above the frontier
                  gg1 < ub[0] &&                       // Must be below upper bound
                  gg2 < ub[1])                         // Must be below upper bound
              {
                if (firstsoc == INT_MAX || gg1+gg2 == firstsoc)
                {
                  if (n.feasible)   // Reachable
                  {
                    if (gg1 >= lb[0] && // Must be above lower bound
                        gg2 >= lb[1])   // Must be above lower bound
                    {
                      //if (firstsoc == INT_MAX)
                      //{
                        //t.EndTimer();
                        //std::cout << "(metexed)Took " << t.GetElapsedTime() << " " << open.size() << " to find first goal\n";
                        //t.StartTimer();
                      //}
                      //std::cout << "Solution found: " << gg1 << "+" << gg2 << "=" << gg1+gg2 << "\n";
                      // TODO: CAT check (if found duplicate)
                      paths.push_back(std::vector<std::vector<state>>(2));
                      // Extract the path back to the root.
                      auto tmpnode(nodeid);
                      paths.back()[0].push_back(n[0].second);
                      //std::cout << "*" << n[0].second << "\n";
                      paths.back()[1].push_back(n[1].second);
                      //std::cout << "      *" << n[1].second << "\n";
                      do
                      {
                        auto const &tmpn(open.Lookup(tmpnode));
                        //std::cout << open.Lookup(tmpnode).data << "\n";
                        for (unsigned q(0); q < tmpn.data.size(); ++q)
                        {
                          auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                          if (paths.back()[q].back() != tmpn.data[q].second)
                          {
                            paths.back()[q].push_back(tmpn.data[q].second);
                            //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                          }
                          else
                          {
                            //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                          }
                        }
                        tmpnode = tmpn.parentID;
                      } while (open.Lookup(tmpnode).parentID != tmpnode);
                      auto const &tmpn(open.Lookup(tmpnode));
                      for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
                      {
                        auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                        if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
                        {
                          paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                          //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                        }
                        else
                        {
                          //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                        }
                        std::reverse(paths.back()[q].begin(), paths.back()[q].end());
                      }
                      std::vector<unsigned> fincosts(2);
                      fincosts[0] = envs[0]->GetPathLength(paths.back()[0]);
                      fincosts[1] = envs[1]->GetPathLength(paths.back()[1]);
                      if (all)
                      {
                        auto ix(std::find(finalcost.begin(), finalcost.end(), fincosts));
                        if (ix == finalcost.end())
                        {
                          finalcost.push_back(fincosts);
                        }
                        else
                        {
                          //TODO: here is where we would perform the CAT check.
                          // For now, this is a duplicate, so we throw it out. :(
                          paths.pop_back();
                        }
                      }
                      else
                      {
                        finalcost.push_back(fincosts);
                      }

                      if (firstsoc == INT_MAX)
                      {
                        firstsoc = soc = gg1 + gg2;
                      }
                      if (!all)
                      {
                        return false;
                      }
                    }
                  }
                  else
                  {
                    // Add all infeasible nodes inside the cost bounds
                    // seen up to this point
                    auto tmpnode(nodeid);
                    while (true)
                    {
                      auto const &nn(open.Lookup(tmpnode));
                      if (nn.data.feasible || nn.parentID == tmpnode)
                      {
                        break;
                      }
                      uint64_t hash(GetHash(nn.data.first));
                      validivls[0].insert(hash);
                      hash = GetHash(nn.data.second);
                      validivls[1].insert(hash);
                      tmpnode = nn.parentID;
                    }
                  }
                }
              }
            }
            //}
            uint64_t hash(GetHash(n, gg1, gg2));
            //std::cout << "hash " << hash << "\n";
            uint64_t theID(0);
            switch (open.Lookup(hash, theID))
            {
            case kClosedList:
              // Closed list guy is not feasible but this one is!
              {
                if (n.feasible)
                {
                  auto &cand(open.Lookup(theID));
                  if (!cand.data.feasible || gg1 + gg2 <= cand.g)
                  {
                    cand.parentID = nodeid;
                    cand.g = gg1 + gg2;
                    cand.h = h1 + h2;
                    cand.g1 = gg1;
                    cand.h1 = h1;
                    cand.g2 = gg2;
                    cand.h2 = h2;
                    cand.data.feasible = true;

                    open.Reopen(theID);
                    //std::cout << "Update " << hash << n << " id:" << theID << " to feasible\n";
                  }
                }
              }
              break;
            case kOpenList:
              // previously generated node is not feasible but this one is!
              {
                // Replace if infeasible or has better cost
                if (n.feasible)
                {
                  auto &cand(open.Lookup(theID));
                  if (!cand.data.feasible || gg1 + gg2 <= cand.g)
                  {
                    cand.parentID = nodeid;
                    cand.g = gg1 + gg2;
                    cand.h = h1 + h2;
                    cand.g1 = gg1;
                    cand.h1 = h1;
                    cand.g2 = gg2;
                    cand.h2 = h2;
                    cand.data.feasible = true;
                    open.KeyChanged(theID);
                    //std::cout << "Update " << hash << n << " id:" << theID << " to feasible\n";
                  }
                }
              }
              break;
            case kNotFound:
              // Add to open :)
              if (gg1 + h1 < ub[0] && gg2 + h2 < ub[1])
              {
                //MultiAgentAStarOpenClosedData<state> data(n, gg1, gg2, h1, h2,
                //nodeid, open.theHeap.size(), kOpenList);
                //open.AddOpenNode(data, hash);
                open.EmplaceOpenNode(hash, n, gg1, gg2, h1, h2, nodeid, open.theHeap.size(), kOpenList);
                open.Lookup(hash, theID);
                //std::cout << std::fixed << "  Add " << hash << n << "feasible: " << n.feasible << " id:" << theID << " f: " << (gg1 + h1 + gg2 + h2) << " g1:" << gg1 << " g2:" << gg2 << " h1:" << h1
                          //<< " h2:" << h2 << " f1:" << gg1 + h1 << " f2:" << gg2 + h2 << "\n";
              //}
              //else
              //{
                //std::cout << std::fixed << "  Skip " << hash << n << "feasible: " << n.feasible << " id:" << theID << " f: " << (gg1 + h1 + gg2 + h2) << " g1:" << gg1 << " g2:" << gg2 << " h1:" << h1
                          //<< " h2:" << h2 << " f1:" << gg1 + h1 << " f2:" << gg2 + h2 << "\n";
              }
              break;
            }
          }
        }
        ids[0][i].emplace(gg2+h2,k); // other's f-cost and my action
        ids[1][j].emplace(gg1+h1,k); // other's f-cost and my action
        //std::cout << "a " << "0x" << n.first << "(" << ids[0][i].back().first << ")=" << n.feasible << "\n";
        //std::cout << "a " << "1x" << n.second << "(" << ids[1][j].back().first << ")=" << n.feasible << "\n";
        ++j;
        ++k;
      }
      ++i;
    }
        //std::cout << "Combined G: " << G << "\n-\n";
        /*unsigned max1(0);
        unsigned max2(0);
        for(auto const& ii:intervals[0])
        {
          if(ii.first[0] && ii.first[0]!=ii.first[2])
          {
            max1 = std::max(max1, ii.first[0]);
            //std::cout << ii.first << "\n";
          }
        }
        //std::cout << "-\n";
        for(auto const& ii:intervals[1])
        {
          if(ii.first[0] && ii.first[0]!=ii.first[2])
          {
            max2 = std::max(max1, ii.first[0]);
            //std::cout << ii.first << "\n";
          }
        }
        std::cout << "maxs: " << max1 << ", " << max2 << "\n";
        for(auto const& ii:intervals[0])
        {
          if(ii.first[0]==max1 && ii.first[0]!=ii.first[2])
          {
            std::cout << ii << "\n";
          }
        }
        for(auto const& ii:intervals[1])
        {
          if(ii.first[0]==max2 && ii.first[0]!=ii.first[2])
          {
            std::cout << ii << "\n";
          }
        }*/

    k=0;
    for (unsigned i(0); i < ids.size(); ++i) // Agent num
    {
      for (unsigned a(0); a < ids[i].size(); ++a) // Action num
      {
        unsigned minCost(INT_MAX);
        unsigned maxCost(0);
        unsigned minInclusive(INT_MAX);
        unsigned maxInclusive(0);
        // Get the current unsafe interval...
        unsigned minTime(unsafeTimes[ids[i][a].front().second][i*2]);
        unsigned maxTime(unsafeTimes[ids[i][a].front().second][i*2+1]);
        if(ids[i][a].empty())continue;
        auto hash(GetHash(successors[i][a]));//,G[i]+ecs[i][a]));
        auto ivl(ivix[i].find(hash));
        // This is a new interval
        if (ivl == ivix[i].end())
        {
          // These are sorted by f-cost. They must be inclusively conflicting
          // over an entire cost plateau in order to be a constraint.
          if (!crossProduct[ids[i][a].front().second].feasible) // lowest f-cost action must be infeasible
          {
            // Loop over all f-costs of opposing agent - record continuous intervals
            auto totalMax(ids[i][a].back().first);
            minCost = ids[i][a].front().first;
            bool allconflicting(true);
            for (auto const &id : ids[i][a])
            {
              if (id.first > minCost && minInclusive == INT_MAX)
              {
                minInclusive = minCost;
                //std::cout << "new lower bound: " << minCost << "\n";
              }
              if (id.first > maxCost)
              {
                maxInclusive = maxCost;
                //std::cout << "new upper bound: " << maxCost << "\n";
              }
              // NOTE: if f-cost increases too much, the interval goes to nothing.
              // We need to try and preserve the interval per the upper f-cost
              // Problem is, if the upper cost ever decreases, we can't retrieve
              // the interval for the  lower cost... We'll see what happens.
              if (!crossProduct[id.second].feasible)
              {
                // There are two cases:
                // (1) the pair is infeasible due to a direct conflict
                // (2) the pair is infeasible due to a propagated conflict
                //
                // Case 1: compute the unsafe interval (save if necessary)
                // Case 2: The unsafe interval is carried forward from the parent
                maxCost = id.first;
                if (allconflicting && timerange) // If everything so far was conflicting...
                {
                  minTime = std::max(unsafeTimes[id.second][i * 2], minTime);
                  maxTime = std::min(unsafeTimes[id.second][i * 2 + 1], maxTime);
                }
              }
              else
              {
                allconflicting = false;
                break;
              }
              ++k;
            }
            if (allconflicting)
            {
              maxInclusive = maxCost;
              if (minInclusive == INT_MAX)
              {
                minInclusive = minCost;
              }
              //std::cout << "new upper bound: " << maxCost << "\n";
            }

            //std::cout << "Add interval: (" << i << ")" << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
            std::array<unsigned,5> tmp={maxInclusive, minInclusive, totalMax, minTime, maxTime};
            ivix[i][hash]=intervals[i].size();
            intervals[i].emplace_back(tmp, successors[i][a]);
          }
          else
          {
            //std::cout << "No conflicts\n";
            //std::cout << "Add interval: (" << i << ")" << successors[i][a] << "(" << minInclusive << "," << 0 << "]\n";
            ivix[i][hash]=0;
            //std::array<unsigned,3> tmp={0, minInclusive-1, minInclusive-1};
            //ivix[i][hash]=intervals[i].size();
            //intervals[i].emplace_back(tmp, successors[i][a]);
          }
        }
        else if(ivl->second)
        { // have an existing interval
          // If this is no longer infeasible, (this action is reachable via another route)
          // we must remove the interval
          auto& val(intervals[i][ivl->second]);

          if (val.first[0] >= val.first[1])
          {
            minTime=std::max(minTime,val.first[3]);
            maxTime=std::min(maxTime,val.first[4]);
            //std::cout << "update " << *val << "(" << i << ") because " << successors[i?0:1] << "\n";
            // Otherwise, we must narrow the interval of the constraint if needed
            unsigned totalMax(ids[i][a].back().first);
            minCost = std::min(val.first[1],ids[i][a].front().first);
            maxCost=minCost;
            bool allconflicting(true);
            for (auto const &id : ids[i][a])
            {
              if (id.first > minCost && minInclusive == INT_MAX)
              {
                minInclusive = minCost;
                //std::cout << "new lower bound: " << minCost << "\n";
                maxInclusive = maxCost;
                //std::cout << "new upper bound: " << maxCost << "\n";
              }
              if (id.first > maxCost)
              {
                maxInclusive = maxCost;
                //std::cout << "new upper bound: " << maxCost << "\n";
              }
              if (!crossProduct[id.second].feasible)
              {
                maxCost = id.first;
                if (allconflicting) // If everything so far was conflicting...
                {
                  minTime = std::max(unsafeTimes[id.second][i * 2], minTime);
                  maxTime = std::min(unsafeTimes[id.second][i * 2 + 1], maxTime);
                }
              }
              else
              {
                allconflicting = false;
                break;
              }
              ++k;
            }
            if (allconflicting)
            {
              maxInclusive = maxCost;
              if (minInclusive == INT_MAX)
              {
                minInclusive = minCost;
              }
              //std::cout << "new upper bound: " << maxCost << "\n";
            }
            if(minInclusive<val.first[1]){
              // Update min
              //std::cout << "update lb of interval: " << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
              val.first[1]=minInclusive;
            }
            bool update(false);
            if(val.first[0]==val.first[2]){
              if (maxInclusive > val.first[0])
              {
                update = true;
                val.first[0] = maxInclusive;
              }
              if (totalMax > val.first[2])
              {
                val.first[2] = totalMax;
              }
            }
            if (maxInclusive < val.first[0] && maxInclusive!=totalMax)
            {
              update = true;
              val.first[0] = maxInclusive;
            }
            val.first[3] = minTime;
            val.first[4] = maxTime;
            //if (update)
            //{
              //std::cout << "adjust upper bound from " << val->first << " to " << maxInclusive << "\n";
              //std::cout << "update ub of interval: " << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
              //auto v(*val);
              //intervals[i].erase(val);
              //intervals[i].emplace(v.first, v.second);
            //}
          }
        }
      }
    }
    return true;
  }
  std::vector<std::vector<Action<state>>> successors;
  std::vector<std::vector<std::vector<state>>> paths;
  std::vector<std::vector<sorted_vector<std::pair<unsigned, unsigned>, true>>> ids;
  std::unordered_set<uint64_t> seen;

  void generatePermutations(std::vector<std::vector<Action<state>>> &positions,
                            std::vector<ActionPair<state>> &result,
                            int agent,
                            ActionPair<state> const &current,
                            uint32_t lastTime,
                            std::vector<double> const &radii)
  {
    if (agent == positions.size())
    {
      result.push_back(current);
      return;
    }

    for (int i = 0; i < positions[agent].size(); ++i)
    {
      //std::cout << "AGENT "<< i<<":\n";
      ActionPair<state> copy(current);
      bool found(false);
      for (int j(0); j < current.size(); ++j)
      {
        bool conflict(false);
        // Sometimes, we add an instantaneous action at the goal to represent the
        // agent disappearing at the goal. If we see this, the agent did not come into conflict
        if (positions[agent][i].first != positions[agent][i].second &&
            current[j].first != current[j].second)
        {
          // Check easy case: agents crossing an edge in opposite directions, or
          // leaving or arriving at a vertex at the same time
          if ((positions[agent][i].first == current[j].first) ||
              (positions[agent][i].second == current[j].second) ||
              (positions[agent][i].first.sameLoc(current[j].second) &&
               current[j].first.sameLoc(positions[agent][i].second)))
          {
            found = true;
            conflict = true;
          }
          else
          {
            // Check general case - Agents in "free" motion
            Vector2D A(positions[agent][i].first.x, positions[agent][i].first.y);
            Vector2D B(current[j].first.x, current[j].first.y);
            Vector2D VA(positions[agent][i].second.x - positions[agent][i].first.x, positions[agent][i].second.y - positions[agent][i].first.y);
            VA.Normalize();
            Vector2D VB(current[j].second.x - current[j].first.x, current[j].second.y - current[j].first.y);
            VB.Normalize();
            //std::cout<<"Checking:"<<current[j].first << "-->"<< current[j].second <<", " << positions[agent][i].first << "-->"<< positions[agent][i].second << "\n";
            if (collisionImminent(A, VA, radii[agent], positions[agent][i].first.t / state::TIME_RESOLUTION_D, positions[agent][i].second.t / state::TIME_RESOLUTION_D, B, VB, radii[j], current[j].first.t / state::TIME_RESOLUTION_D, current[j].second.t / state::TIME_RESOLUTION_D))
            {
              found = true;
              conflict = true;
              //checked.insert(hash);
            }
          }
        }
      }
      if (found)
      {
        copy.feasible = false;
      }
      copy.push_back(positions[agent][i]);
      generatePermutations(positions, result, agent + 1, copy, lastTime, radii);
    }
}

};
template <typename environ, typename state, typename action>
AStarOpenClosed<ActionPair<state>, CATTieBreaking<state>, MultiAgentAStarOpenClosedData<state>> PairwiseConstrainedSearch<environ,state,action>::open;
template <typename environ, typename state, typename action>
bool PairwiseConstrainedSearch<environ,state,action>::cardinalcheck = false;

//template <typename environ, typename state>
//inline std::ostream &operator<<(std::ostream &os, typename PairwiseConstrainedSearch<environ, state>::MultiAgentAStarOpenClosedData const &ma)
//{
  //return os;
//}
#ifdef FOO
template <typename environ, typename state>
struct PairwiseConstrainedSearch2
{
  //typedef MultiCostLimitedEnvironment<Action<state>, unsigned, environ> MultiEnv;

  TemporalAStarPair<state,int,environ> astar;
  // Initial constructor
  // Adds a single state to the open list and sets the lower bound on cost to 0
  PairwiseConstrainedSearch2(environ *env1,
                             environ *env2,
                             Heuristic<state> *heu1,
                             Heuristic<state> *heu2,
                             double lower1,
                             double upper1,
                             double lower2,
                             double upper2,
                             state const &start1,
                             state const &goal1,
                             double r1,
                             state const &start2,
                             state const &goal2,
                             double r2,
                             double socLb,
                             bool storeAll = false)
      //bool waitAtGoalNeverCosts=false)
      : intervals(2),
        ivix(2),
        ids(2),
        radii(2),
        socLim(socLb),
        firstsoc(INT_MAX),
        soc(INT_MAX),
        bf(1),
        all(storeAll),
        currList(new std::vector<MultiAgentIDAData<state>>()),
        nextList(new std::vector<MultiAgentIDAData<state>>())
  //waitcosts(!waitAtGoalNeverCosts)
  {
    lb={lower1,lower2};
    ub={upper1,upper2};
    start={start1,start2};
    goal={goal1,goal2};
    radii[0]=r1;
    radii[1]=r2;
    envs = {env1, env2};
    heus = {heu1, heu2};
    //env = MultiEnv(envs);
    //open.Reset(env->GetMaxHash());
    for (auto const &e : envs)
    {
      bf *= e->branchingFactor();
    }
  }

  void reset()
  {
    finalcost.clear();
    seen.clear();
    intervals[0].push_back({{0,INT_MAX-1,INT_MAX-1}, Action<state>(start[0],start[0])});
    intervals[1].push_back({{0,INT_MAX-1,INT_MAX-1}, Action<state>(start[1],start[1])});
  }

  std::array<environ *,2> envs; // environments for agents
  std::array<Heuristic<state> *,2> heus; // environments for agents
  //MultiEnv env;
  unsigned currFLimit;
  unsigned nextFLimit;
  std::vector<MultiAgentIDAData<state>*>* currList;
  std::vector<MultiAgentIDAData<state>*>* nextList;
  std::vector<MultiAgentIDAData<state>*> closedList;
  std::vector<MultiAgentIDAData<state>*> costList;
  std::vector<MultiAgentIDAData<state>> elements;

  double socLim;             // Plateau for sum of costs
  double firstsoc;                // soc of first solution
  double soc;                // soc of first solution
  std::array<double,2> lb;
  std::array<double,2> ub;
  std::array<state,2> goal;
  std::array<state,2> start;

  double maxTotal;
  unsigned bf; // branching factor
  bool all; // Whether to store all solutions of cost
  //bool waitcosts; // Whether waiting at the goal costs if you move off again

  std::vector<std::deque<std::pair<std::array<unsigned,3>,Action<state>>>> intervals;
  std::vector<std::unordered_map<uint64_t,unsigned>> ivix;
  std::vector<std::vector<unsigned>> finalcost;
  //PairTree<ActionPair<state>> mutexes;
  const int MAXTIME=1000 * state::TIME_RESOLUTION_U;
  std::vector<double> radii;

  static AStarOpenClosed<ActionPair<state>, CATTieBreaking<state>, MultiAgentAStarOpenClosedData<state>> open;
  //BucketOpenClosed<ActionPair<state>, CATTieBreaking<state>, MultiAgentAStarOpenClosedData<state>> open;


  // Get hash for a single two-agent state
  uint64_t GetHash(state const &first,
                   state const &second,
                   uint64_t gg1,
                   uint64_t gg2,
                   bool feasible)
  {
    // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
    uint64_t h(14695981039346656037UL); // Offset basis
    uint64_t h1(0);
    unsigned i(0);
    uint8_t c[sizeof(uint64_t)];
    h1 = envs[i]->GetStateHash(first);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }

    h1 = envs[i++]->GetStateHash(second);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    h1 = (gg1<<33 + gg2<<1) +feasible;
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    return h;
  }

  // Get hash for a single-agent action
  uint64_t GetHash(Action<state> const &v)//, uint64_t gg1)
  {
    // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
    uint64_t h(14695981039346656037UL); // Offset basis
    uint64_t h1(0);
    unsigned i(0);
    uint8_t c[sizeof(uint64_t)];
    h1 = envs[i]->GetStateHash(v.first);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }

    h1 = envs[i++]->GetStateHash(v.second);
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    /*memcpy(c, &gg1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }*/
    return h;
  }

  // Get hash for a two-agent action
  uint64_t GetHash(ActionPair<state> const &node, uint32_t gg1, uint32_t gg2)
  {
    // Implement the FNV-1a hash http://www.isthe.com/chongo/tech/comp/fnv/index.html
    uint64_t h(14695981039346656037UL); // Offset basis
    uint64_t h1(0);
    unsigned i(0);
    uint8_t c[sizeof(uint64_t)];
    for (auto const &v : node)
    {
      h1=envs[i]->GetStateHash(v.first);
      memcpy(c, &h1, sizeof(uint64_t));
      for (unsigned j(0); j < sizeof(uint64_t); ++j)
      {
        //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
        h = h ^ c[j];          // Xor with octet
        h = h * 1099511628211; // multiply by the FNV prime
      }

      h1=envs[i++]->GetStateHash(v.second);
      memcpy(c, &h1, sizeof(uint64_t));
      for (unsigned j(0); j < sizeof(uint64_t); ++j)
      {
        //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
        h = h ^ c[j];          // Xor with octet
        h = h * 1099511628211; // multiply by the FNV prime
      }
    }
//#pragma GCC push_options
//#pragma GCC optimize ("O0")
   h1 = (gg1*0xffffffff) + gg2;
//#pragma GCC pop_options
    memcpy(c, &h1, sizeof(uint64_t));
    for (unsigned j(0); j < sizeof(uint64_t); ++j)
    {
      //hash[k*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
      h = h ^ c[j];          // Xor with octet
      h = h * 1099511628211; // multiply by the FNV prime
    }
    return h;
  }

  void getRangesAndConstraints()
  {
    // =======================
    // Constraint-based feasibility checks.
    // Plan each agent singly.
    // If the search fails, the problem is unsolvable,
    // either due to being overconstrained, or just an
    // unsolvable instance
    // =======================
    paths.resize(0);
    paths.push_back(std::vector<std::vector<state>>(2, std::vector<state>()));
    paths[0][0].clear();
    auto tmpgoal(goal[0]);
    tmpgoal.t = lb[0];
    astar.SetHeuristic(heus[0]);
    astar.SetUpperLimit(ub[0]);
    if (ub[0] != lb[0])
    {
      astar.SetUpperLimit(ub[0] - 1);
    }
    astar.SetLowerLimit(lb[0]);
    //Timer tmr;
    //tmr.StartTimer();
    //astar.SetVerbose(true);
    astar.GetPath(envs[0], start[0], tmpgoal, paths[0][0], lb[0]);
    //std::cout << "s1 took: " << tmr.EndTimer() << "\n";
    if (paths[0][0].empty())
    {
      return;
    }
    auto len1(envs[0]->GetPathLength(paths[0][0]));
    paths[0][0].clear();

    paths[0][1].clear();
    tmpgoal = goal[1];
    tmpgoal.t = lb[1];
    astar.SetHeuristic(heus[1]);
    astar.SetUpperLimit(ub[1]);
    if (ub[1] != lb[1])
    {
      astar.SetUpperLimit(ub[1] - 1);
    }
    astar.SetLowerLimit(lb[1]);
    //astar.SetVerbose(true);
    //tmr.StartTimer();
    astar.GetPath(envs[1], start[1], tmpgoal, paths[0][1], lb[1]);
    //std::cout << "s2 took: " << tmr.EndTimer() << "\n";
    if (paths[0][1].empty())
    {
      return;
    }
    auto len2(envs[1]->GetPathLength(paths[0][1]));
    paths[0][1].clear();
    paths.clear();

    // If either of the upper bounds is unlimited, we could run forever
    // As a termination heuristic, we will assume that if an agent were
    // to make it to its goal, it may have to wait for the other agent
    // to traverse its entire path first.
    maxTotal = envs[0]->GetMapSize() * state::TIME_RESOLUTION_U;
    if (maxTotal < socLim)
    {
      maxTotal = socLim;
    }
    //maxTotal=len1+len2;
    //cd maxTotal*=maxTotal;

    // =======================
    // Do full joint-agent planning and mutex-propagation
    // at the same time
    // =======================
    //tmr.StartTimer();
    ActionPair<state> s(start[0], start[0], start[1], start[1]);
    MultiAgentIDAData<state> data(s, 0, 0,
                                  heus[0]->HCost(start[0], goal[0]),
                                  heus[1]->HCost(start[1], goal[1]),
                                  0, 0);
    data.data.nc = 0; // We assume that the root has no conflicts with other agents
    currList->clear();
    currList->push_back(data);
    {
      while (currList.size() > 0)
      {
        auto currOpenNode(currList->back());
        currList->pop_back();
        if (currOpenNode == 0)
        {
          //checkIteration();
          if (currList->empty())
          {
            auto tmp(currList);
            currList = nextList;
            nextList = tmp;
            currFLimit = nextFLimit;
            nextFLimit = INT_MAX;
          }
          continue;
        }
        auto currF(currOpenNode->f());
        if (currF > currFLimit)
        {
          nextFLimit = std::min(nextFLimit, currF);
          nextList->push_back(currOpenNode);
          //checkIteration();
          if (currList->empty())
          {
            auto tmp(currList);
            currList = nextList;
            nextList = tmp;
            currFLimit = nextFLimit;
            nextFLimit = INT_MAX;
          }
          continue;
        }

        // Add to closed list
        auto ref(table.find(currOpenNode->id));
        assert(ref!=table.end()){
        }
        uint64_t hash(GetHash(*currOpenNode, currOpenNode->g1, currOpenNode->g2));
        currOpenNode->id=closedList.size();
        closedList.push_back(currOpenNode);

        // For each successor
    auto F(currOpenNode->g+currOpenNode->h);

    //std::cout << "{d:" << node.data << "f1:" << node.g1 << "+" << node.h1 << "=" << (node.g1+node.h1) << ",f2:" << node.g2 << "+" << node.h2 << "=" << (node.g2+node.h2) << ",id:" << nodeid << ",p:" << node.parentID << " feasible: "<< node.data.feasible <<"}\n";
    if (soc>firstsoc)
    {
      return false; // Return because a solution was already found and we have finished the plateau
    }

    std::array<double,2> G={currOpenNode->g1,currOpenNode->g2};
    //auto f1(G[0] + currOpenNode->h1);
    //auto f2(G[1] + currOpenNode->h2);
    //if(!all)soc=f1+f2;
    // Check that we're below the bounds
    //if (ub[0] < f1 || ub[1] < f2)
    //{
      //return true;
    //}
    auto s(currOpenNode->data);
    // Early termination criteria so we don't run forever.
    if(G[0]+currOpenNode->h1>maxTotal+1 || G[1]+currOpenNode->h2>maxTotal+1)
    {
      return true;
    }

    // Add this node to the interval tree
    auto shash(GetHash(s.first.second,s.second.second,G[0],G[1],s.feasible));
    if(seen.find(shash)==seen.end()){
      seen.insert(shash);
    }else{
      return true;
    }
    //std::cout <<std::fixed <<"pop " << s << " f: " << currOpenNode->h+currOpenNode->g << "\n";


    //auto G(currOpenNode->g);

    // First:
    // Find minimum time of current edges
    double sd(DBL_MAX);
    unsigned minindex(0);
    double k(0);
    for (auto const &a : s)
    {
      if (a.second.t < sd)
      {
        sd = a.second.t;
        minindex = k;
      }
      k++;
    }
    if (sd == DBL_MAX)
    {
      sd = s[minindex].second.t;
    } // Can happen at root node

    //Get successors into a vector
    successors.clear();
    successors.reserve(s.size());

    //Add in successors for parents who are equal to the min
    std::vector<std::vector<unsigned>> ecs(2);
    k = 0;
    for (auto const &a : s)
    {
      static std::vector<Action<state>> output;
      output.clear();
      if (a.second.t <= sd)
      {
        //std::cout << "Keep Successors of " << *a.second << "\n";
        std::vector<state> succ(envs[k]->branchingFactor()*2);
        auto sz(envs[k]->GetSuccessors(a.second, &succ[0]));
        for (unsigned j(0); j < sz; ++j)
        {
          // We assume here that GCost will never charge for waiting at the goal
          // Wait actions at the goal cost (unless we're at the end)
          //if (waitcosts && G[k] < lb[k] && envs[k]->GoalTest(a.second,goal[k]) && envs[k]->GoalTest(succ[j],goal[k]))
          //{
            //ecs[k].push_back(succ[j].t - a.second.t);
          //}
          //else
          //{
            ecs[k].push_back(envs[k]->GCost(a.second, succ[j]));
          //}
          output.emplace_back(a.second, succ[j]);
        }
      }
      else
      {
        // We assume here that GCost will never charge for waiting at the goal
        // Wait actions at the goal cost (unless we're at the end)
        //if (waitcosts && G[k] < lb[k] && envs[k]->GoalTest(a.first,goal[k]) && envs[k]->GoalTest(a.second,goal[k]))
        //{
          //ecs[k].push_back(a.second.t-a.first.t);
        //}
        //else
        //{
          // We are re-using this, so don't add the edge cost again...
          ecs[k].push_back(0);//envs[k]->GCost(a.first, a.second));
          //ecs[k].push_back(envs[k]->GCost(a.first, a.second));
        //}
        output.push_back(a);
      }
      if (output.empty())
      {
        // All movements from this position at this time are blocked
        return true;
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
      ++k;
    }
    static std::vector<ActionPair<state>> crossProduct;
    crossProduct.clear();
    crossProduct.reserve(successors[0].size()*successors[1].size());
    ids[0].resize(successors[0].size());
    ids[1].resize(successors[1].size());
    for(auto& id:ids[1])
    {
      id.clear();
    }
    k=0;
    unsigned i(0);
    for (auto const &a1 : successors[0])
    {
      ids[0][i].clear();
      auto h1(heus[0]->HCost(a1.second, goal[0]));
      unsigned j(0);
      auto const& ec1(ecs[0][i]);
      for (auto const &a2 : successors[1])
      {
        auto h2(heus[1]->HCost(a2.second, goal[1]));
        auto const &ec2(ecs[1][j]);
        crossProduct.emplace_back(a1, a2, s.feasible);
        auto &n(crossProduct.back());
        if (CATTieBreaking<state>::useCAT)
        {
          // Compute cumulative conflicts
          static std::vector<state const *> matches;
          matches.clear();
          //std::cout << "Getting NC for " << i1.data << ":\n";
          CATTieBreaking<state>::CAT.get(n.t, std::max(n.first.second.t, n.second.second.t), matches, CATTieBreaking<state>::currentAgents);

          // Get number of conflicts in the parent
          unsigned nc1(s.nc);
          //std::cout << "  matches " << matches.size() << "\n";

          // Count number of conflicts
          for (unsigned m(1); m < matches.size(); ++m)
          {
            nc1 += collisionCheck2D(n.first.first, n.first.second, *matches[m - 1], *matches[m], radii[0], radii[0]);
            nc1 += collisionCheck2D(n.second.first, n.second.second, *matches[m - 1], *matches[m], radii[1], radii[1]);
            //if(!nc2){std::cout << "NO ";}
            //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
          }
          // Set the number of conflicts in the data object
          n.nc = nc1;
        }

        if (s.feasible)
        {
          // Check for conflict...
          if ((a1.first == a2.first) ||
              (a1.second == a2.second) ||
              (a1.first.sameLoc(a2.second) &&
               a2.first.sameLoc(a1.second)))
          {
            n.feasible = false;
          }
          else
          {
            //std::cout<<"Checking:"<<current[j].first << "-->"<< current[j].second <<", " << positions[agent][i].first << "-->"<< positions[agent][i].second << "\n";
            if (collisionCheck2D(a1.first, a1.second,
                                 a2.first, a2.second,
                                 radii[0], radii[1]))
            {
              n.feasible = false;
            }
          }
        }
        //std::cout << "  " << n << " "<<n.feasible << "\n";
        auto gg1(G[0] + ec1);
        auto gg2(G[1] + ec2);
        // Make sure this node will come in under the upper bound
        if (gg1 + h1 < ub[0] && gg2 + h2 < ub[1])
        {
          bool a1done(envs[0]->GoalTest(n[0].second, goal[0]));
          bool a2done(envs[1]->GoalTest(n[1].second, goal[1]));
          bool a1wait(n[0].first.sameLoc(n[0].second));
          bool a2wait(n[1].first.sameLoc(n[1].second));
          // Make sure both aren't waiting at the goal
          if (!(a2done && a1done && a1wait && a2wait))
          {
            //if(n.feasible)
            //std::cout << "a0 lb:" << lb[0] <<": " << ub[0] << "/a1 lb:" << lb[1] <<": " << ub[1] << " " << n[0]
            //<< "(f="<< (gg1+h1) << ") feasible with " << n[1]  << "(f="<< (gg2+h2) << ")\n";
            // Legit arrival at goal
            // check goal
            //if(soc>=gg1+gg2)
            //{
            if (a1done && a2done) // At goal
            {
              //std::cout << "  {g:" << n << "f1:" << gg1 << "+" << h1 << "=" << (gg1 + h1) << ",f2:" << gg2 << "+" << h2 << "=" << (gg2+ h2)  << " feasible: " << n.feasible << "}\n";
              if (G[0] + G[1] + ec1 + ec2 >= socLim && // Must be above the frontier
                  gg1 >= lb[0] &&                      // Must be above lower bound
                  gg2 >= lb[1] &&                      // Must be above lower bound
                  gg1 < ub[0] &&                       // Must be below upper bound
                  gg2 < ub[1])                         // Must be below upper bound
              {
                if (n.feasible) // Reachable
                {
                  if (all || firstsoc == INT_MAX)
                  {
                    // TODO: CAT check (if found duplicate)
                    paths.push_back(std::vector<std::vector<state>>(2));
                    // Extract the path back to the root.
                    auto tmpnode(nodeid);
                    paths.back()[0].push_back(n[0].second);
                    //std::cout << "*" << n[0].second << "\n";
                    paths.back()[1].push_back(n[1].second);
                    //std::cout << "      *" << n[1].second << "\n";
                    do
                    {
                      auto const &tmpn(open.Lookup(tmpnode));
                      //std::cout << open.Lookup(tmpnode).data << "\n";
                      for (unsigned q(0); q < tmpn.data.size(); ++q)
                      {
                        //auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                        if (paths.back()[q].back() != tmpn.data[q].second)
                        {
                          paths.back()[q].push_back(tmpn.data[q].second);
                          //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                        }
                        else
                        {
                          //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                        }
                      }
                      tmpnode = tmpn.parentID;
                    } while (open.Lookup(tmpnode).parentID != tmpnode);
                    auto const &tmpn(open.Lookup(tmpnode));
                    for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
                    {
                      //auto gg(envs[q]->GCost(tmpn.data[q].first, tmpn.data[q].second));
                      if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
                      {
                        paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                        //std::cout << (q ? "                  " : "") <<tmpnode<< "*" << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                      }
                      else
                      {
                        //std::cout << (q ? "                  " : "") <<tmpnode<< " " << tmpn.data[q] << " ng: " << (q?tmpn.g2:tmpn.g1) << " ec: " << gg << "\n";
                      }
                      std::reverse(paths.back()[q].begin(), paths.back()[q].end());
                    }
                    std::vector<unsigned> fincosts(2);
                    fincosts[0] = envs[0]->GetPathLength(paths.back()[0]);
                    fincosts[1] = envs[1]->GetPathLength(paths.back()[1]);
                    if (all)
                    {
                      auto ix(std::find(finalcost.begin(), finalcost.end(), fincosts));
                      if (ix == finalcost.end())
                      {
                        finalcost.push_back(fincosts);
                        if (gg1 + gg2 > soc)
                        {
                          return false;
                        }
                      }
                      else
                      {
                        //TODO: here is where we would perform the CAT check.
                        // For now, this is a duplicate, so we throw it out. :(
                        paths.pop_back();
                      }
                    }
                    else
                    {
                      finalcost.push_back(fincosts);
                    }
                  }
                  else
                  {
                    // Add all infeasible nodes inside the cost bounds
                    // seen up to this point
                    auto tmpnode(nodeid);
                    while (true)
                    {
                      auto const &nn(open.Lookup(tmpnode));
                      if (nn.data.feasible || nn.parentID == tmpnode)
                      {
                        break;
                      }
                      uint64_t hash(GetHash(nn.data.first));
                      validivls[0].insert(hash);
                      hash = GetHash(nn.data.second);
                      validivls[1].insert(hash);
                      tmpnode = nn.parentID;
                    }
                  }

                  if (firstsoc == INT_MAX)
                  {
                    firstsoc = soc = gg1 + gg2;
                  }
                  /*auto top(open.Lookup(open.Peek()));
            if (top.g + top.h > soc)
            {
              return false;
            }
            else
            {
              return true;
            }*/
                  if (!all)
                  {
                    return false;
                  }
                }
              }
            }
            //}
            uint64_t hash(GetHash(n, gg1, gg2));
            //std::cout << "hash " << hash << "\n";
            uint64_t theID(0);
            switch (open.Lookup(hash, theID))
            {
            case kClosedList:
              // Closed list guy is not feasible but this one is!
              {
                if (n.feasible)
                {
                  auto &cand(open.Lookup(theID));
                  if (!cand.data.feasible || gg1 + gg2 <= cand.g)
                  {
                    cand.parentID = nodeid;
                    cand.g = gg1 + gg2;
                    cand.h = h1 + h2;
                    cand.g1 = gg1;
                    cand.h1 = h1;
                    cand.g2 = gg2;
                    cand.h2 = h2;
                    cand.data.feasible = true;

                    open.Reopen(theID);
                    //std::cout << "Update " << n << " id:" << theID << " to feasible\n";
                  }
                }
              }
              break;
            case kOpenList:
              // previously generated node is not feasible but this one is!
              {
                // Replace if infeasible or has better cost
                if (n.feasible)
                {
                  auto &cand(open.Lookup(theID));
                  if (!cand.data.feasible || gg1 + gg2 <= cand.g)
                  {
                    cand.parentID = nodeid;
                    cand.g = gg1 + gg2;
                    cand.h = h1 + h2;
                    cand.g1 = gg1;
                    cand.h1 = h1;
                    cand.g2 = gg2;
                    cand.h2 = h2;
                    cand.data.feasible = true;
                    open.KeyChanged(theID);
                    //std::cout << "Update " << n << " id:" << theID << " to feasible\n";
                  }
                }
              }
              break;
            case kNotFound:
              // Add to open :)
              if (gg1 + h1 < ub[0] && gg2 + h2 < ub[1])
              {
                //MultiAgentAStarOpenClosedData<state> data(n, gg1, gg2, h1, h2,
                                                          //nodeid, open.theHeap.size(), kOpenList);
                //open.AddOpenNode(data, hash);
                open.EmplaceOpenNode(hash, n, gg1, gg2, h1, h2,nodeid, open.theHeap.size(), kOpenList);
                //open.Lookup(hash, theID);
              }
              //std::cout << std::fixed << "  Add " << n << " id:" << theID << " f: " << (gg1+h1+gg2+h2) << " g1:" << gg1 << " g2:" << gg2 << " h1:" << h1
              //<< " h2:" << h2 << " f1:"<< gg1+h1 << " f2:" << gg2+h2 << "\n";
              break;
            }
          }
        }
        ids[0][i].emplace(gg2+h2,k); // other's f-cost and my action
        ids[1][j].emplace(gg1+h1,k); // other's f-cost and my action
        //std::cout << "a " << "0x" << n.first << "(" << ids[0][i].back().first << ")=" << n.feasible << "\n";
        //std::cout << "a " << "1x" << n.second << "(" << ids[1][j].back().first << ")=" << n.feasible << "\n";
        ++j;
        ++k;
      }
      ++i;
    }


      }

    }
    if(finalcost.empty())return;
    //std::cout << finalcost << "\n";
    // TODO - If we ever plan to re-use this class, we won't be able
    //        to delete the intervals inline like this...

    // ===============================
    // Finally, filter down the list of mprop constraints
    // Only those which were completely infeasible for one
    // or more complete cost levels are retained
    // ===============================
    auto finalsoc(finalcost[0][0]+finalcost[0][1]);
    std::array<unsigned,2> mins={UINT_MAX,UINT_MAX};
    for (unsigned i(0); i < finalcost.size() - 1; ++i)
    {
      if (finalcost[i][0] < mins[0])
      {
        mins[0] = finalcost[i][0];
      }
      if (finalcost[i][1] < mins[1])
      {
        mins[1] = finalcost[i][1];
      }
    }
    for(unsigned i(0); i<intervals.size(); ++i){
      unsigned other(i?0:1);
      //std::cout << "Constraints for agent " << i << ":\n";
      for(auto j(intervals[i].begin()); j!=intervals[i].end(); /*++j*/){
        // We can only trust cost combinations that make sense
        // if the upper bound on the constraint minus the lb on the other agent
        // is more than the sum of costs, it can't be trusted, because we never
        // looked deep enough into the structure.
        if (j->first[0]<j->first[2] &&
        j->first[1]<=j->first[0] &&
        j->first[0]>=mins[other])
        {
          //std::cout << i << "  " << *j << "\n";
          ++j;
        }
        else
        {
          //std::cout << "ignore "<< i << " " << *j << "\n";
          j=intervals[i].erase(j);
        }
      }
    }
  }

  bool foundInitialSolution() const{
    return paths.size();
  }

  // This search includes infeasible nodes and tracks mutexes
  // It is exponential in nature - running doSingleSearchStep() to completion
  // before this is recommended

  bool doSingleSearchStepWithMutexes()
  {
    if (!open.OpenSize())
    {
      return false;
    }

    // Get next candidate off open
    uint64_t nodeid = open.Close();
    // Note - this reference becomes invalid once we insert anything into open.
    auto &node(open.Lookup(nodeid));
        //std::cout << "Combined G: " << G << "\n-\n";
        /*unsigned max1(0);
        unsigned max2(0);
        for(auto const& ii:intervals[0])
        {
          if(ii.first[0] && ii.first[0]!=ii.first[2])
          {
            max1 = std::max(max1, ii.first[0]);
            //std::cout << ii.first << "\n";
          }
        }
        //std::cout << "-\n";
        for(auto const& ii:intervals[1])
        {
          if(ii.first[0] && ii.first[0]!=ii.first[2])
          {
            max2 = std::max(max1, ii.first[0]);
            //std::cout << ii.first << "\n";
          }
        }
        std::cout << "maxs: " << max1 << ", " << max2 << "\n";
        for(auto const& ii:intervals[0])
        {
          if(ii.first[0]==max1 && ii.first[0]!=ii.first[2])
          {
            std::cout << ii << "\n";
          }
        }
        for(auto const& ii:intervals[1])
        {
          if(ii.first[0]==max2 && ii.first[0]!=ii.first[2])
          {
            std::cout << ii << "\n";
          }
        }*/

    k=0;
    for (unsigned i(0); i < ids.size(); ++i) // Agent num
    {
      for (unsigned a(0); a < ids[i].size(); ++a) // Action num
      {
        unsigned minCost(INT_MAX);
        unsigned maxCost(0);
        unsigned minInclusive(INT_MAX);
        unsigned maxInclusive(0);
        if(ids[i][a].empty())continue;
        auto hash(GetHash(successors[i][a]));//,G[i]+ecs[i][a]));
        auto ivl(ivix[i].find(hash));
        // This is a new interval
        if (ivl == ivix[i].end())
        {
          // These are sorted by f-cost. They must be inclusively conflicting
          // over an entire cost plateau in order to be a constraint.
          if (!crossProduct[ids[i][a].front().second].feasible) // lowest f-cost action must be infeasible
          {
            // Loop over all f-costs of opposing agent - record continuous intervals
            auto totalMax(ids[i][a].back().first);
            minCost = ids[i][a].front().first;
            bool allconflicting(true);
            for (auto const &id : ids[i][a])
            {
              if (id.first > minCost && minInclusive == INT_MAX)
              {
                minInclusive = minCost;
                //std::cout << "new lower bound: " << minCost << "\n";
              }
              if (id.first > maxCost)
              {
                maxInclusive = maxCost;
                //std::cout << "new upper bound: " << maxCost << "\n";
              }
              if (!crossProduct[id.second].feasible)
              {
                maxCost = id.first;
              }
              else
              {
                allconflicting = false;
                break;
              }
              ++k;
            }
            if (allconflicting)
            {
              maxInclusive = maxCost;
              if (minInclusive == INT_MAX)
              {
                minInclusive = minCost;
              }
              //std::cout << "new upper bound: " << maxCost << "\n";
            }

            //std::cout << "Add interval: (" << i << ")" << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
            std::array<unsigned,3> tmp={maxInclusive, minInclusive, totalMax};
            ivix[i][hash]=intervals[i].size();
            intervals[i].emplace_back(tmp, successors[i][a]);
          }
          else
          {
            //std::cout << "No conflicts\n";
            //std::cout << "Add interval: (" << i << ")" << successors[i][a] << "(" << minInclusive << "," << 0 << "]\n";
            ivix[i][hash]=0;
            //std::array<unsigned,3> tmp={0, minInclusive-1, minInclusive-1};
            //ivix[i][hash]=intervals[i].size();
            //intervals[i].emplace_back(tmp, successors[i][a]);
          }
        }
        else
        { // have an existing interval
          // If this is no longer infeasible, (this action is reachable via another route)
          // we must remove the interval
          auto& val(intervals[i][ivl->second]);

          if (val.first[0] >= val.first[1])
          {
            //std::cout << "update " << *val << "(" << i << ") because " << successors[i?0:1] << "\n";
            // Otherwise, we must narrow the interval of the constraint if needed
            unsigned totalMax(ids[i][a].back().first);
            minCost = std::min(val.first[1],ids[i][a].front().first);
            maxCost=minCost;
            bool allconflicting(true);
            for (auto const &id : ids[i][a])
            {
              if (id.first > minCost && minInclusive == INT_MAX)
              {
                minInclusive = minCost;
                //std::cout << "new lower bound: " << minCost << "\n";
                maxInclusive = maxCost;
                //std::cout << "new upper bound: " << maxCost << "\n";
              }
              if (id.first > maxCost)
              {
                maxInclusive = maxCost;
                //std::cout << "new upper bound: " << maxCost << "\n";
              }
              if (!crossProduct[id.second].feasible)
              {
                maxCost = id.first;
              }
              else
              {
                allconflicting = false;
                break;
              }
              ++k;
            }
            if (allconflicting)
            {
              maxInclusive = maxCost;
              if (minInclusive == INT_MAX)
              {
                minInclusive = minCost;
              }
              //std::cout << "new upper bound: " << maxCost << "\n";
            }
            if(minInclusive<val.first[1]){
              // Update min
              //std::cout << "update lb of interval: " << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
              val.first[1]=minInclusive;
            }
            bool update(false);
            if(val.first[0]==val.first[2]){
              if (maxInclusive > val.first[0])
              {
                update = true;
                val.first[0] = maxInclusive;
              }
              if (totalMax > val.first[2])
              {
                val.first[2] = totalMax;
              }
            }
            if (maxInclusive < val.first[0] && maxInclusive!=totalMax)
            {
              update = true;
              val.first[0] = maxInclusive;
            }
            //if (update)
            //{
              //std::cout << "adjust upper bound from " << val->first << " to " << maxInclusive << "\n";
              //std::cout << "update ub of interval: " << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
              //auto v(*val);
              //intervals[i].erase(val);
              //intervals[i].emplace(v.first, v.second);
            //}
          }
        }
      }
    }
    return true;
  }
  std::vector<std::vector<Action<state>>> successors;
  std::vector<std::vector<std::vector<state>>> paths;
  std::vector<std::vector<sorted_vector<std::pair<unsigned, unsigned>, true>>> ids;
  std::unordered_set<uint64_t> seen;

  void generatePermutations(std::vector<std::vector<Action<state>>> &positions,
                            std::vector<ActionPair<state>> &result,
                            int agent,
                            ActionPair<state> const &current,
                            uint32_t lastTime,
                            std::vector<double> const &radii)
  {
    if (agent == positions.size())
    {
      result.push_back(current);
      return;
    }

    for (int i = 0; i < positions[agent].size(); ++i)
    {
      //std::cout << "AGENT "<< i<<":\n";
      ActionPair<state> copy(current);
      bool found(false);
      for (int j(0); j < current.size(); ++j)
      {
        bool conflict(false);
        // Sometimes, we add an instantaneous action at the goal to represent the
        // agent disappearing at the goal. If we see this, the agent did not come into conflict
        if (positions[agent][i].first != positions[agent][i].second &&
            current[j].first != current[j].second)
        {
          // Check easy case: agents crossing an edge in opposite directions, or
          // leaving or arriving at a vertex at the same time
          if ((positions[agent][i].first == current[j].first) ||
              (positions[agent][i].second == current[j].second) ||
              (positions[agent][i].first.sameLoc(current[j].second) &&
               current[j].first.sameLoc(positions[agent][i].second)))
          {
            found = true;
            conflict = true;
          }
          else
          {
            // Check general case - Agents in "free" motion
            Vector2D A(positions[agent][i].first.x, positions[agent][i].first.y);
            Vector2D B(current[j].first.x, current[j].first.y);
            Vector2D VA(positions[agent][i].second.x - positions[agent][i].first.x, positions[agent][i].second.y - positions[agent][i].first.y);
            VA.Normalize();
            Vector2D VB(current[j].second.x - current[j].first.x, current[j].second.y - current[j].first.y);
            VB.Normalize();
            //std::cout<<"Checking:"<<current[j].first << "-->"<< current[j].second <<", " << positions[agent][i].first << "-->"<< positions[agent][i].second << "\n";
            if (collisionImminent(A, VA, radii[agent], positions[agent][i].first.t / state::TIME_RESOLUTION_D, positions[agent][i].second.t / state::TIME_RESOLUTION_D, B, VB, radii[j], current[j].first.t / state::TIME_RESOLUTION_D, current[j].second.t / state::TIME_RESOLUTION_D))
            {
              found = true;
              conflict = true;
              //checked.insert(hash);
            }
          }
        }
      }
      if (found)
      {
        copy.feasible = false;
      }
      copy.push_back(positions[agent][i]);
      generatePermutations(positions, result, agent + 1, copy, lastTime, radii);
    }
}

};
#endif
#endif