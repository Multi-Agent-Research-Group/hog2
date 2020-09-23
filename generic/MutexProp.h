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
#include "PairTree.h"
#include "CollisionDetection.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "MultiAgentStructures.h"
#include "Utilities.h"
#include "sorted_vector.h"
#include "TemporalAStar2.h"

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
    ActionPair():std::pair<Action<state>,Action<state>>(),t(0),sz(0),feasible(true){}
    ActionPair(Action<state>const&a,Action<state> const&b,bool f=true):std::pair<Action<state>,Action<state>>(a,b),t(std::min(a.t,b.t)),sz(2),feasible(f){}
    ActionPair(state const& a, state const& b, state const& c, state const& d):std::pair<Action<state>,Action<state>>(Action<state>(a,b),Action<state>(c,d)),t(std::min(b.t,d.t)),sz(2),feasible(true){}
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


// A structure which represents the current state of a pairwise search
// It is clonable so that the search can be continued where left off
// but with different parameters
template <typename environ, typename state, typename action>
struct PairwiseConstrainedSearch
{
  //typedef MultiCostLimitedEnvironment<Action<state>, unsigned, environ> MultiEnv;

  TemporalAStar<state,action,environ> astar;
  // Initial constructor
  // Adds a single state to the open list and sets the lower bound on cost to 0
  PairwiseConstrainedSearch(unsigned agent1,
                            unsigned agent2,
                            environ *env1,
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
                            bool storeAll=false)
      : a1(agent1),
        a2(agent2),
        lb1(lower1),
        ub1(upper1),
        lb2(lower2),
        ub2(upper2),
        s1(start1),
        g1(goal1),
        s2(start2),
        g2(goal2),
        intervals(2),
        ids(2),
        radii(2),
        socLim(socLb),
        firstsoc(INT_MAX),
        soc(INT_MAX),
        bf(1),
        all(storeAll)
  {
    radii[0]=r1;
    radii[1]=r2;
    envs = {env1, env2};
    heus = {heu1, heu2};
    //env = MultiEnv(envs);
    //open.Reset(env->GetMaxHash());
    ActionPair<state> s(s1,s1, s2,s2);
    MultiAgentAStarOpenClosedData data(s,0,0,
                                   heus[0]->HCost(s1, g1),
                                   heus[1]->HCost(s2, g2),
                         0,open.theHeap.size(), kOpenList);
    open.AddOpenNode(data,GetHash(s));
    for (auto const &e : envs)
    {
      bf *= e->branchingFactor();
    }
  }

  // Copy constructor
  PairwiseConstrainedSearch(PairwiseConstrainedSearch const &from,
                            double lower1,
                            double upper1,
                            double lower2,
                            double upper2,
                            double soclb)
      : a1(from.a1),
        a2(from.a2),
        lb1(lower1),
        ub1(upper1),
        lb2(lower2),
        ub2(upper2),
        s1(from.s1),
        g1(from.g1),
        s2(from.s2),
        g2(from.g2),
        envs(from.envs),
        heus(from.heus),
        bf(from.bf),
        open(from.open), // Make a full copy of open
        socLim(soclb),
        firstsoc(INT_MAX),
        soc(INT_MAX),
        all(from.all)
  {
    // Make sure that the originating open list makes sense
    if (ub1 < from.lb1 || // old lower bound is too high
        ub2 < from.lb2)   // old lower bound is too high
      assert(false);

    // Remove and re-insert all nodes which have changed priority...
    for (auto const &id : open.theHeap)
    {
      auto i1(open.Lookup(id));
      if (greater(from.ub1, i1.g1 + i1.h1) && greater(from.ub2, i1.g2 + i1.h2) &&
          fless(ub1, i1.g1 + i1.h1) && fless(ub2, i1.g2 + i1.h2))
      {
        open.KeyChanged(id);
      }
    }
  }

  unsigned a1, a2;             // global agent numbers
  std::vector<environ *> envs; // environments for agents
  std::vector<Heuristic<state> *> heus; // environments for agents
  //MultiEnv env;
  double socLim;             // Plateau for sum of costs
  double firstsoc;                // soc of first solution
  double soc;                // soc of first solution
  double lb1, ub1, lb2, ub2; // Ind. lower and upper cost bounds
  double maxTotal;
  state s1, g1, s2, g2;
  unsigned bf; // branching factor
  bool all; // Whether to store all solutions of cost
  //std::vector<PairTree<Action<state>>> intervals;
  std::vector<sorted_vector<std::pair<std::array<unsigned,3>,Action<state>>>> intervals;
  std::vector<std::vector<unsigned>> finalcost;
  //PairTree<ActionPair<state>> mutexes;
  const int MAXTIME=1000 * state::TIME_RESOLUTION_U;
  std::vector<double> radii;

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
  struct AStarCompare
  {
    static double ub1;
    static double ub2;
    bool operator()(const MultiAgentAStarOpenClosedData &i1,
                    const MultiAgentAStarOpenClosedData &i2) const
    {
      // Prioritize f-costs that are less than the bounds first
      /*bool f1l(fless(ub1, i1.g1 + i1.h1) && fless(ub2, i1.g2 + i1.h2));
      bool f2l(fless(ub1, i2.g1 + i2.h1) && fless(ub2, i2.g2 + i2.h2));
      if (f1l)
      {
        if (f2l)
        {
          if (fequal(i1.g + i1.h, i2.g + i2.h))
          {
            return (fless(i1.g, i2.g));
          }
          return (fgreater(i1.g + i1.h, i2.g + i2.h));
        }
        return true;
      }
      else if (f2l)
      {
        return false;
      }*/
      if (fequal(i1.g + i1.h, i2.g + i2.h))
      {
        return (fless(i1.g, i2.g));
      }
      return (fgreater(i1.g + i1.h, i2.g + i2.h));
    }
  };

  AStarOpenClosed<ActionPair<state>, AStarCompare, MultiAgentAStarOpenClosedData> open;

  uint64_t GetHash(ActionPair<state> const &node)
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
    return h;
  }

  void getRangesAndConstraints()
  {
    AStarCompare::ub1 = ub1;
    AStarCompare::ub2 = ub2;
    //check feasibility
    paths.resize(0);
    paths.push_back(std::vector<std::vector<state>>(2,std::vector<state>()));
    paths[0][0].clear();
    auto goal(g1);
    goal.t=lb1;
    astar.SetHeuristic(heus[0]);
    astar.SetUpperLimit(ub1);
    if(ub1!=lb1)
    {
    astar.SetUpperLimit(ub1-1);
    }
    astar.SetLowerLimit(lb1);
    astar.GetPath(envs[0],s1,goal,paths[0][0],lb1);
    if(paths[0][0].empty())
    {
      return;
    }
    auto len1(envs[0]->GetPathLength(paths[0][0]));
    paths[0][0].clear();

    paths[0][1].clear();
    goal=g2;
    goal.t=lb2;
    astar.SetHeuristic(heus[1]);
    astar.SetUpperLimit(ub2);
    if(ub2!=lb2)
    {
      astar.SetUpperLimit(ub2 - 1);
    }
    astar.SetLowerLimit(lb2);
    astar.GetPath(envs[1],s2,goal,paths[0][1],lb2);
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
    maxTotal=len1+len2;

    bool go(doSingleSearchStep());
    while(go){
      go = doSingleSearchStep();
    }
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
    if(finalcost.empty())return;
    //std::cout << finalcost << "\n";
    // TODO - If we ever plan to re-use this class, we won't be able
    //        to delete the intervals inline like this...
    auto finalsoc(finalcost[0][0]+finalcost[0][1]);
    for(unsigned i(0); i<intervals.size(); ++i){
      //std::cout << "Constraints for agent " << i << ":\n";
      for(auto j(intervals[i].begin()); j!=intervals[i].end(); /*++j*/){
        // We can only trust cost combinations that make sense
        // if the upper bound on the constraint minus the lb on the other agent
        // is more than the sum of costs, it can't be trusted, because we never
        // looked deep enough into the structure.
        auto lb(i ? lb2 : lb1);
        if (j->first[0]<j->first[2] && j->first[1]<=j->first[0])
        {
          //std::cout << "  " << *j << "\n";
          ++j;
        }
        else
        {
          //std::cout << "ignore " << *j << "\n";
          intervals[i].erase(j);
        }
      }
    }
    
  }

  bool foundInitialSolution() const{
    return paths.size();
  }

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
    //std::cout << "{d:" << node.data << "f1:" << node.g1 << "+" << node.h1 << "=" << (node.g1+node.h1) << ",f2:" << node.g2 << "+" << node.h2 << "=" << (node.g2+node.h2) << ",id:" << nodeid << ",p:" << node.parentID << " feasible: "<< node.data.feasible <<"}\n";
    if (soc>firstsoc)
    {
      return false; // Return because a solution was already found and we have finished the plateau
    }

    auto f1(node.g1 + node.h1);
    auto f2(node.g2 + node.h2);
    if(!all)soc=f1+f2;
    // Check that we're below the bounds
    //if (ub1 < f1 || ub2 < f2)
    //{
      //return true;
    //}
    // Early termination criteria so we don't run forever.
    if(f1>maxTotal+1 || f2>maxTotal+1)
    {
      return true;
    }

    // Add this node to the interval tree
    auto s(node.data);
    //std::cout << "pop " << s << "\n";


    auto G(node.g);
    auto G1(node.g1);
    auto G2(node.g2);

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
    k = 0;
    for (auto const &a : s)
    {
      static std::vector<Action<state>> output;
      output.clear();
      if (a.second.t <= sd)
      {
        //std::cout << "Keep Successors of " << *a.second << "\n";
        std::vector<state> succ(envs[k]->branchingFactor());
        auto sz(envs[k]->GetSuccessors(a.second, &succ[0]));
        for (unsigned j(0); j < sz; ++j)
        {
          output.emplace_back(a.second, succ[j]);
        }
      }
      else
      {
        //std::cout << "Keep Just " << *a.second << "\n";
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
      auto h1(heus[0]->HCost(a1.second, g1));
      auto ec1(s[0] == a1 ? 0.0 : envs[0]->GCost(a1.first, a1.second));
      unsigned j(0);
      for (auto const &a2 : successors[1])
      {
        auto h2(heus[1]->HCost(a2.second, g2));
        auto ec2(s[1] == a2 ? 0.0 : envs[1]->GCost(a2.first, a2.second));
        crossProduct.emplace_back(a1, a2, s.feasible);
        auto &n(crossProduct.back());
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
            // Check general case - Agents in "free" motion
            Vector2D A(a1.first.x, a1.first.y);
            Vector2D B(a2.first.x, a2.first.y);
            Vector2D VA(a1.second.x - a1.first.x, a1.second.y - a1.first.y);
            VA.Normalize();
            Vector2D VB(a2.second.x - a2.first.x, a2.second.y - a2.first.y);
            VB.Normalize();
            //std::cout<<"Checking:"<<current[j].first << "-->"<< current[j].second <<", " << positions[agent][i].first << "-->"<< positions[agent][i].second << "\n";
            if (collisionImminent(A, VA, radii[0],
                                  a1.first.t / state::TIME_RESOLUTION_D,
                                  a1.second.t / state::TIME_RESOLUTION_D,
                                  B, VB, radii[1],
                                  a2.first.t / state::TIME_RESOLUTION_D,
                                  a2.second.t / state::TIME_RESOLUTION_D))
            {
              n.feasible = false;
            }
          }
        }
        auto gg1(G1 + ec1);
        auto gg2(G2 + ec2);
        if(gg1+h1 < ub1 && gg2+h2 < ub2)
        {
          bool a1done(envs[0]->GoalTest(n[0].second, g1));
          bool a2done(envs[1]->GoalTest(n[1].second, g2));
          bool a1wait(n[0].first.sameLoc(n[0].second));
          bool a2wait(n[1].first.sameLoc(n[1].second));
          if (a2done && a1done && a1wait && a2wait)
          {
            continue; // Both agents can't just sit at their goal
          }
          //if(n.feasible)
          //std::cout << "a0 lb:" << lb1 <<": " << ub1 << "/a1 lb:" << lb2 <<": " << ub2 << " " << n[0]
          //<< "(f="<< (gg1+h1) << ") feasible with " << n[1]  << "(f="<< (gg2+h2) << ")\n";
          // Legit arrival at goal
          // check goal
          //if(soc>=gg1+gg2)
          //{
          if (a1done && a2done) // At goal
          {
            //std::cout << "  {g:" << n << "f1:" << gg1 << "+" << h1 << "=" << (gg1 + h1) << ",f2:" << gg2 << "+" << h2 << "=" << (gg2+ h2)  << " feasible: " << n.feasible << "}\n";
            if (n.feasible) // Reachable
              int fffff = 987;
            if (G + ec1 + ec2 >= socLim && // Must be above the frontier
                gg1 >= lb1 &&              // Must be above lower bound
                gg2 >= lb2 &&              // Must be above lower bound
                gg1 < ub1 &&               // Must be below upper bound
                gg2 < ub2)                 // Must be below upper bound
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
                    //std::cout << open.Lookup(tmpnode).data << "\n";
                    for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
                    {
                      if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
                      {
                        paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                        //std::cout << (q ? "      " : "") << "*" << open.Lookup(tmpnode).data[q].second << "\n";
                      }
                      else
                      {
                        //std::cout << (q ? "      " : "") << " " << open.Lookup(tmpnode).data[q].second << "\n";
                      }
                    }
                    tmpnode = open.Lookup(tmpnode).parentID;
                  } while (open.Lookup(tmpnode).parentID != tmpnode);
                  for (unsigned q(0); q < open.Lookup(tmpnode).data.size(); ++q)
                  {
                    if (paths.back()[q].back() != open.Lookup(tmpnode).data[q].second)
                    {
                      paths.back()[q].push_back(open.Lookup(tmpnode).data[q].second);
                      //std::cout << (q ? "      " : "") << "*" << open.Lookup(tmpnode).data[q].second << "\n";
                    }
                    else
                    {
                      //std::cout << (q ? "      " : "") << " " << open.Lookup(tmpnode).data[q].second << "\n";
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
                }

                if (firstsoc == INT_MAX)
                {
                  firstsoc = soc = gg1 + gg2;
                }
                else
                {
                  soc = gg1 + gg2;
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
              }
            }
          }
          //}
          uint64_t hash(GetHash(n));
          //std::cout << "hash " << hash << "\n";
          uint64_t theID(0);
          switch (open.Lookup(hash, theID))
          {
          case kClosedList:
            // Closed list guy is not feasible but this one is!
            {
              auto const &cand(open.Lookup(theID));
              if (n.feasible && (!open.Lookup(theID).data.feasible || gg1 + gg2 <= open.Lookup(theID).g))
              {
                open.Lookup(theID).parentID = nodeid;
                open.Lookup(theID).g = gg1 + gg2;
                open.Lookup(theID).h = h1 + h2;
                open.Lookup(theID).g1 = gg1;
                open.Lookup(theID).h1 = h1;
                open.Lookup(theID).g2 = gg2;
                open.Lookup(theID).h2 = h2;
                open.Lookup(theID).data.feasible = true;

                open.Reopen(theID);
                //std::cout << "Update " << n << " id:" << theID << " to feasible\n";
              }
            }
            break;
          case kOpenList:
            // previously generated node is not feasible but this one is!
            {
              auto const &cand1(open.Lookup(theID));
              // Replace if infeasible or has better cost
              if (n.feasible && (!open.Lookup(theID).data.feasible || gg1 + gg2 <= open.Lookup(theID).g))
              {
                open.Lookup(theID).parentID = nodeid;
                open.Lookup(theID).g = gg1 + gg2;
                open.Lookup(theID).h = h1 + h2;
                open.Lookup(theID).g1 = gg1;
                open.Lookup(theID).h1 = h1;
                open.Lookup(theID).g2 = gg2;
                open.Lookup(theID).h2 = h2;
                open.Lookup(theID).data.feasible = true;
                //std::cout << "Update " << n << " id:" << theID << " to feasible\n";
              }
            }
            break;
          case kNotFound:
            // Add to open :)
            MultiAgentAStarOpenClosedData data(n, gg1, gg2, h1, h2,
                                               nodeid, open.theHeap.size(), kOpenList);
            open.AddOpenNode(data, hash);
            open.Lookup(hash, theID);
            //std::cout << "Add " << n << " id:" << theID << " g1:" << gg1 << " g2:" << gg2 << " h1:" << h1
            //<< " h2:" << h2 << " f1:"<< gg1+h1 << " f2:" << gg2+h2 << "\n";
            break;
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
    k=0;
    for (unsigned i(0); i < ids.size(); ++i)
    {
      for (unsigned a(0); a < ids[i].size(); ++a)
      {
        unsigned minCost(INT_MAX);
        unsigned maxCost(0);
        unsigned minInclusive(INT_MAX);
        unsigned maxInclusive(0);
        if(ids[i][a].empty())continue;
        auto val(std::find_if(
            intervals[i].begin(), intervals[i].end(),
            [&](std::pair<std::array<unsigned,3>, Action<state>> const &x) { return x.second == successors[i][a]; }));
        // this is a new interval
        if (val == intervals[i].end())
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
            intervals[i].emplace(tmp, successors[i][a]);
          }
          else
          {
            //std::cout << "No conflicts\n";
            //std::cout << "Add interval: (" << i << ")" << successors[i][a] << "(" << minInclusive << "," << 0 << "]\n";
            std::array<unsigned,3> tmp={0, minInclusive-1, minInclusive-1};
            intervals[i].emplace(tmp, successors[i][a]);
          }
        }
        else
        { // have an existing interval
          // If this is no longer infeasible, (this action is reachable via another route)
          // we must remove the interval

          {
            //std::cout << "update " << *val << "(" << i << ") because " << successors[i?0:1] << "\n";
            // Otherwise, we must narrow the interval of the constraint if needed
            unsigned totalMax(ids[i][a].back().first);
            minCost = std::min(val->first[1],ids[i][a].front().first);
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
            if(minInclusive<val->first[1]){
              // Update min
              //std::cout << "update lb of interval: " << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
              val->first[1]=minInclusive;
            }
            bool update(false);
            if(val->first[0]==val->first[2]){
              if (maxInclusive > val->first[0])
              {
                update = true;
                val->first[0] = maxInclusive;
              }
              if (totalMax > val->first[2])
              {
                val->first[2] = totalMax;
              }
            }
            if (maxInclusive < val->first[0] && maxInclusive!=totalMax)
            {
              update = true;
              val->first[0] = maxInclusive;
            }
            if (update)
            {
              //std::cout << "adjust upper bound from " << val->first << " to " << maxInclusive << "\n";
              //std::cout << "update ub of interval: " << successors[i][a] << "(" << minInclusive << "," << maxInclusive << "]\n";
              auto v(*val);
              intervals[i].erase(val);
              intervals[i].emplace(v.first, v.second);
            }
          }
        }
      }
    }
    return true;
  }
  std::vector<std::vector<Action<state>>> successors;
  std::vector<std::vector<std::vector<state>>> paths;
  std::vector<std::vector<sorted_vector<std::pair<unsigned, unsigned>, true>>> ids;

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
double PairwiseConstrainedSearch<environ,state,action>::AStarCompare::ub1 = ub1;
template <typename environ, typename state, typename action>
double PairwiseConstrainedSearch<environ,state,action>::AStarCompare::ub2 = ub2;

//template <typename environ, typename state>
//inline std::ostream &operator<<(std::ostream &os, typename PairwiseConstrainedSearch<environ, state>::MultiAgentAStarOpenClosedData const &ma)
//{
  //return os;
//}