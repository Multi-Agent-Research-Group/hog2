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
#include "PEAStar.h"
#include "TemplateAStar.h"
#include "Heuristic.h"

bool verbose(false);
bool validate(false);
uint64_t jointnodes(0);
float step(1.0);

template<typename T, typename C>
class custom_priority_queue : public std::priority_queue<T, std::vector<T>, C>
{
  public:

    bool remove(const T& value) {
      toDelete.insert(value->key());
    }
    bool removeAll(int agent, int count){
      for(auto const& value:this->c){
        if(value->sizes[agent]<count){
          toDelete.insert(value->key());
        }
      }
      return true;
    }
    virtual T popTop(){
      T val(this->top());
      this->pop();
      while(toDelete.find(val->key()) != toDelete.end()){
        val=this->top();
        this->pop();
      }
      return val;
    }
    std::unordered_set<std::string> toDelete;
};

int renderScene(){return 1;}

struct Group{
  Group(int i){agents.insert(i);}
  std::unordered_set<int> agents;
  //~Group(){std::cout << "Destroy " << this << "\n";}
};

/*
struct Hashable{
  virtual uint64_t Hash()const=0;
  virtual float Depth()const=0;
};
*/

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
        static uint64_t count;
	Node(){count++;}
	Node(xyLoc a, float d):n(a),depth(d),optimal(false){count++;}
	xyLoc n;
	float depth;
        bool optimal;
        //bool connected()const{return parents.size()+successors.size();}
	//std::unordered_set<Node*> parents;
	std::unordered_set<Node*> successors;
	virtual uint64_t Hash()const{return (env->GetStateHash(n)<<32) | ((uint32_t)floor(depth*1000.));}
	virtual float Depth()const{return depth; }
        virtual void Print(std::ostream& ss, int d=0) const {
          ss << std::string(d,' ')<<n << "_" << depth << std::endl;
          for(auto const& m: successors)
            m->Print(ss,d+1);
        }
        bool operator==(Node const& other)const{return n.sameLoc(other.n)&&fequal(depth,other.depth);}
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
typedef std::vector<std::pair<Node*,Node*>> Multiedge; // rank=agent num
typedef std::unordered_map<uint64_t,Node> DAG;

class MultiEdge: public Multiedge{
  public:
  MultiEdge():Multiedge(),parent(nullptr){}
  MultiEdge(Multiedge const& other):Multiedge(other),parent(nullptr){} // Do not copy successors!
  std::vector<MultiEdge> successors;
  MultiEdge* parent;
  void Print(std::ostream& ss, int d=0) const {
    ss << std::string(d,' ');
    int i(0);
    for(auto const& a: *this)
      ss << " "<<++i<<"." << a.second->n << "@" << a.second->depth;
    ss << std::endl;
    for(auto const& m: successors)
      m.Print(ss,d+1);
  }
};

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

std::ostream& operator << (std::ostream& ss, MultiEdge const& n){
  /*int i(0);
  for(auto const& a: n)
    ss << " "<<++i<<"." << a.second->n << "@" << a.second->depth;
  ss << std::endl;
  for(auto const& m:n.successors)
    ss << "----"<<m;
  */
  n.Print(ss,0);
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const& n){
  ss << n.n << "@" << n.depth;
  return ss;
}

std::ostream& operator << (std::ostream& ss, Node const* n){
  n->Print(ss);
  //ss << std::string(n->depth,' ')<<n->n << "_" << n->depth << std::endl;
  //for(auto const& m: n->successors)
    //ss << m;
  return ss;
}

MapEnvironment* Node::env=nullptr;
uint64_t Node::count(0);

bool LimitedDFS(xyLoc const& start, xyLoc const& end, DAG& dag, MultiState& root, float depth, float maxDepth, float& best){
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
    while(fleq(d+1,maxDepth)){ // Increment depth by 1 for wait actions
      // Wait at goal
      Node current(start,++d);
      uint64_t chash(current.Hash());
      dag[chash]=current;
      //std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
      parent->successors.insert(&dag[chash]);
      //dag[chash].parents.insert(parent);
      parent=&dag[chash];
    }
    best=std::min(best,d);
    if(verbose)std::cout << "ABEST "<<best<<"\n";
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
    if(LimitedDFS(node,end,dag,root,depth-ddiff,maxDepth,best)){
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
void GetMDD(xyLoc const& start, xyLoc const& end, DAG& dag, MultiState& root, float depth, float& best){
  if(verbose)std::cout << "MDD up to depth: " << depth << "\n";
  LimitedDFS(start,end,dag,root,depth,depth,best);
}

void generatePermutations(std::vector<MultiEdge>& positions, std::vector<MultiEdge>& result, int agent, MultiEdge const& current, float lastTime) {
  if(agent == positions.size()) {
    result.push_back(current);
    if(verbose)std::cout << "Generated joint move:\n";
    if(verbose)for(auto edge:current){
      std::cout << *edge.first << "-->" << *edge.second << "\n";
    }
    jointnodes++;
    return;
  }

  for(int i = 0; i < positions[agent].size(); ++i) {
    //std::cout << "AGENT "<< i<<":\n";
    MultiEdge copy(current);
    bool found(false);
    for(int j(0); j<current.size(); ++j){
      // Make sure we don't do any checks that were already done
      if(positions[agent][i].first->depth!=lastTime&&current[j].first->depth!=lastTime)continue;
      //uint64_t hash(EdgePairHash(positions[agent][i],current[j]));
      //if(checked.find(hash)!=checked.end())
      //{std::cout << "SKIPPED " << *positions[agent][i].second << " " << *current[j].second << "\n"; continue; /*No collision check necessary; checked already*/}
      //std::cout << "COMPARE " << *positions[agent][i].second << " " << *current[j].second << "\n";
      Vector2D A(positions[agent][i].first->n.x,positions[agent][i].first->n.y);
      Vector2D B(current[j].first->n.x,current[j].first->n.y);
      Vector2D VA(positions[agent][i].second->n.x-positions[agent][i].first->n.x,positions[agent][i].second->n.y-positions[agent][i].first->n.y);
      VA.Normalize();
      Vector2D VB(current[j].second->n.x-current[j].first->n.x,current[j].second->n.y-current[j].first->n.y);
      VB.Normalize();
      //std::cout << "Test for collision: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
      if(collisionImminent(A,VA,.25,positions[agent][i].first->depth,positions[agent][i].second->depth,B,VB,.25,current[j].first->depth,current[j].second->depth)){
        if(verbose)std::cout << "Collision averted: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
        found=true;
        //checked.insert(hash);
        break;
      }else if(verbose){std::cout << "generating: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
      }
    }
    if(found) continue;
    copy.push_back(positions[agent][i]);
    generatePermutations(positions, result, agent + 1, copy,lastTime);
  }
}

// In order for this to work, we cannot generate sets of positions, we must generate sets of actions, since at time 1.0 an action from parent A at time 0.0 may have finished, while another action from the same parent A may still be in progress. 

// Return true if we get to the desired depth
//bool jointDFS(MultiEdge & s, float d, float term, std::vector<std::set<Node*,NodePtrComp>>& answer,std::vector<Node*>& toDelete, float& best, float bestSeen, float increment=1.0, bool suboptimal=false){
bool jointDFS(MultiEdge & s, float d, float term, std::vector<Node*>& toDelete, float& best, float bestSeen, float increment=1.0, bool suboptimal=false){
  //std::cout << "saw " << s << "\n";
  //std::cout << d << std::string((int)d,' ');
  //for(int a(0); a<s.size(); ++a){
    //std::cout << " s " << *s[a].second << "\n";
  //}
  if(fgreater(d,term-std::min(1.0f,increment))){
    if(fless(d,best)){
      best=d;
      if(verbose)std::cout << "BEST="<<best<<std::endl;
      if(verbose)std::cout << "TERM="<<term<<std::endl;
      if(verbose)std::cout << "BS="<<bestSeen<<std::endl;
      /*for(int a(0); a<s.size(); ++a){
        if(verbose)std::cout << "Found solution: "<<a<<":" << *s[a].second << "\n";
        answer[a].clear();
        //std::cout << "push " << *s[a].second << " " << s.size() << "\n";
        answer[a].insert(s[a].second);
      }*/
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
      output.emplace_back(a.second,new Node(a.second->n,a.second->depth+1.0/*increment*/));
      toDelete.push_back(output.back().second);
      md=min(md,a.second->depth+1.0/*increment*/); // Amount of time to wait
    }
    //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
    successors.push_back(output);
  }
  /*if(verbose)for(int agent(0); agent<successors.size(); ++agent){
    std::cout << "Agent joint successors: " << agent << "\n\t";
    for(int succ(0); succ<successors[agent].size(); ++succ)
      std::cout << *successors[agent][succ].second << ",";
    std::cout << std::endl;
  }*/
  std::vector<MultiEdge> crossProduct;
  generatePermutations(successors,crossProduct,0,MultiEdge(),sd);
  bool value(false);
  bool recorded(false);
  for(auto& a: crossProduct){
    a.parent=&s;
    //std::cout << "EVAL " << s << "-->" << a << "\n";
    if(jointDFS(a,md,term,toDelete,best,bestSeen,increment,suboptimal)){
    //std::cout << "found " << s << "-->" << a << "\n";
      auto*p=&s;
      while(true){
        bool valid(true);
        for(int i(0); i<p->size(); ++i){
          bool found(false);
          for(auto const& suc:(*p)[i].second->successors){
            if(*a[i].second==*suc){
              found=true;
              break;
            }
          }
          // Finally, check wait action
          if(!found && (!a[i].second->n.sameLoc((*p)[i].second->n) || !fequal(a[i].second->Depth(),(*p)[i].second->Depth()+1.0))){
            valid=false;
            break;
          }
        }
        if(valid){
          //std::cout << a << " is a successor of " << *p << "\n";
          a.parent=p;
          p->successors.push_back(a);
          break;
        }else{
          //std::cout << a << " is NOT a successor of " << *p << "\n";
          if(!p->parent){
            //std::cout << "ERROR! no valid parent found in progression!\n";
            break;
          }
          //std::cout << "Go from "<< *p << " to parent " << *p->parent << "\n";
          p=p->parent;
        }
      }
      // Note: this assumes we're only interested in a single optimal answer
      /*if(!recorded){
        // Check whether this joint-state is congruent with
        // previous answer
        for(int a(0); a<s.size(); ++a){
          answer[a].insert(s[a].second);
        }
        if(false)for(int a(0); a<s.size(); ++a){
          // Check wait action explicitly
          if(answer[a].empty()){
            if(verbose)std::cout << "push "<<a<<"("<<d<<")"<<":" << *s[a].second << "\n";
            answer[a].insert(s[a].second);
          }else if((*answer[a].begin())->n.sameLoc(s[a].second->n)){
            if(fequal((*answer[a].begin())->Depth(),s[a].second->Depth()+1)){
              //std::cout << "Move from " << *s[a].second << " to " << *(*answer[a].begin()) << " is legal\n";
              if(verbose)std::cout << "push "<<a<<"("<<d<<")"<<":" << *s[a].second << "\n";
              answer[a].insert(s[a].second);
            }else{
                //std::cout << "Move from " << *s[a].second << " to " << *(*answer[a].begin()) << " is not legal\n";
            }
          }else{
            Points successors;
            Node::env->GetSuccessors(s[a].second->n,successors);
            for(auto n:successors){
              if(n==(*answer[a].begin())->n && fequal((*answer[a].begin())->Depth(),s[a].second->Depth()+Node::env->GCost(s[a].second->n,n))){
                //std::cout << "Move from " << *s[a].second << " to " << *(*answer[a].begin()) << " is legal\n";
                if(verbose)std::cout << "push "<<a<<"("<<d<<")"<<":" << *s[a].second << "\n";
                answer[a].insert(s[a].second);
                break;
              }else{
                //std::cout << "Move from " << *s[a].second << " to " << *(*answer[a].begin()) << " is not legal ";
                //if(n==(*answer[a].begin())->n){std::cout << "because " << (*answer[a].begin())->Depth() << "!=" << s[a].second->Depth()+Node::env->GCost(s[a].second->n,n) << "\n";
                //}else{std::cout << "because " << n <<"!="<<(*answer[a].begin())->n<<"\n";}
              }
            }
            //answer[a].insert(s[a].second);
          }
        }
        recorded=true;
      }*/
      // Return first solution...
      if(suboptimal) return true;
      // Return if solution is as good as any MDD
      if(fequal(best,bestSeen))return true;
      value=true;
    }
  }
  return value;
}

class BaseOccupancyInterface : public OccupancyInterface<xyLoc,tDirection>
{
public:
	BaseOccupancyInterface(Map* m){};
	virtual ~BaseOccupancyInterface(){}
	virtual void SetStateOccupied(const MultiEdge&, bool){}
	virtual bool GetStateOccupied(const MultiEdge&){}
	virtual bool CanMove(const MultiEdge&, const MultiEdge&){}
	virtual void MoveUnitOccupancy(const MultiEdge &, const MultiEdge&){}

private:
	////BitVector *bitvec; /// For each map position, set if occupied
	//std::vector<bool> bitvec;
	//long mapWidth; /// Used to compute index into bitvector
	//long mapHeight; /// used to compute index into bitvector

	//long CalculateIndex(uint16_t x, uint16_t y);
};

class JointEnvironment : public Heuristic<MultiEdge>{
public:

  JointEnvironment(float i, float d, std::vector<float> const& b):increment(i),goalDepth(d),best(b){}

  uint64_t GetMaxHash(){return INT_MAX;}
  BaseOccupancyInterface *GetOccupancyInfo(){return nullptr;}
  void GetSuccessors(MultiEdge const& s,std::vector<MultiEdge>& crossProduct)const{
    //Get successors into a vector
    std::vector<MultiEdge> successors;

    // Find minimum depth of current edges
    float sd(9999999.0);
    for(auto const& a: s){
      sd=min(sd,a.second->depth);
    }
    //std::cout << "min-depth: " << sd << "\n";

    //float md(9999999.0);
    //Add in successors for parents who are equal to the min
    for(auto const& a: s){
      MultiEdge output;
      if(fleq(a.second->depth,sd)){
        //std::cout << "Keep Successors of " << *a.second << "\n";
        for(auto const& b: a.second->successors){
          output.emplace_back(a.second,b);
          //md=min(md,b->depth);
        }
      }else{
        //std::cout << "Keep Just " << *a.second << "\n";
        output.push_back(a);
        //md=min(md,a.second->depth);
      }
      if(output.empty()){
        // Stay at state...
        output.emplace_back(a.second,new Node(a.second->n,a.second->depth+increment));
        //toDelete.push_back(output.back().second);
        //md=min(md,a.second->depth+increment); // Amount of time to wait
      }
      //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
      successors.push_back(output);
    }
    /*if(verbose)for(int agent(0); agent<successors.size(); ++agent){
      std::cout << "Agent joint successors: " << agent << "\n\t";
      for(int succ(0); succ<successors[agent].size(); ++succ)
      std::cout << *successors[agent][succ].second << ",";
      std::cout << std::endl;
      }*/
    generatePermutations(successors,crossProduct,0,MultiEdge(),sd);
  }

  bool GoalTest(MultiEdge const& n, MultiEdge const& g)const{
    return fgreater(n[0].second->depth,goalDepth-increment);
    /*for(int i(0); i<n.size(); ++i){
      if(n[i].second->n != g[i].second->n){
        return false;
      }
    }
    return true;*/
  }

  double GCost(MultiEdge const& n, MultiEdge const& g)const{
    double total(0);
    for(int i(0); i<n.size(); ++i){
      total+=Node::env->GCost(n[i].second->n,g[i].second->n);
    }
    return total;
  }

  double HCost(MultiEdge const& n, MultiEdge const& g)const{
    double total(0);
    for(int i(0); i<n.size(); ++i){
      //total+=std::max((float)best[i],Node::env->HCost(n[i].second->n,g[i].second->n));
      total+=Node::env->HCost(n[i].second->n,g[i].second->n);
    }
    return total;
  }

  uint64_t GetStateHash(MultiEdge const& node)const{
    uint64_t h = 0;
    for(auto const& s : node){
      if(s.first)
        h = (h * 16777619) ^ Node::env->GetStateHash(s.first->n); // xor
      h = (h * 16777619) ^ Node::env->GetStateHash(s.second->n); // xor
    }
    return h;
  }

  void SetColor(float,float,float,float){}
  void OpenGLDraw(MultiEdge const&){}
  int GetAction(MultiEdge const&, MultiEdge const&)const{}
  float increment;
  float goalDepth;
  std::vector<float> best;
};

bool jointDFS(MultiState const& s, float maxdepth, std::vector<Node*>& toDelete, float bestSeen, float increment=1.0, bool suboptimal=false){
  MultiEdge act;
  float sd(1.0);
  for(auto const& n:s){ // Add null parents for the initial movements
    act.emplace_back(nullptr,n);
    /*for(auto const& m:n->successors){
      sd=min(sd,m->depth);
    }*/
    //act.push_back(a);
  }
  float best(9999999);


  bool ans = jointDFS(act,0.0,maxdepth,toDelete,best,bestSeen,increment,suboptimal);
  std::vector<std::vector<std::vector<Node*>>> solutions;

  if(ans)std::cout << "ANSWER " << act << "\n";
  else std::cout << "FAILED\n";
  return ans;
}

void parseSolution(MultiEdge& root, MultiEdge& dag){
  // Copy contents of root to dag node
  std::copy(root.begin(),root.end(),dag.begin());
  // Copy any successors of root that are valid to successors of dag
  for(auto& a:root.successors){
    a.parent=&root;
    for(int s(0); s<a.size(); ++s){
      //for
      //if(a[s].second
    }
  }
}

bool jointAStar(MultiState const& s, float maxdepth, std::vector<Node*>& toDelete, Instance const& inst, std::vector<float> const& best, float increment=1.0){
  if(verbose){std::cout << "JointAStar\n";}
  MultiEdge start;
  for(auto const& n:s){ // Add null parents for the initial movements
    start.emplace_back(nullptr,n);
    /*for(auto const& m:n->successors){
      sd=min(sd,m->depth);
    }*/
    //start.push_back(a);
  }
  MultiEdge goal;
  for(auto const& g:inst.second){
    goal.emplace_back(nullptr,new Node(g,0));
  }
  JointEnvironment env(increment,maxdepth,best);
  TemplateAStar<MultiEdge,int,JointEnvironment> peastar;
  //PEAStar<MultiEdge,int,JointEnvironment> peastar;
  //peastar.SetVerbose(verbose);
  std::vector<MultiEdge> solution;
  peastar.GetPath(&env,start,goal,solution);
  for(auto const& g:goal){
    delete g.second;
  }
  if(solution.size()==0){return false;}
  if(verbose)std::cout << "answer ---\n";
  auto* parent(&s);
  for(auto const& node(solution.begin()+1); node!=solution.end(); ++node){
    parent.successors.push_back(*node);
    parent.successors.back().parent=parent;
    parent=&parent.successors.back();
  }
  return true;
}

// Not part of the algorithm... just for validating the answers
bool checkAnswer(Solution const& answer){
  for(int i(0);i<answer.size();++i){
    for(int j(i+1);j<answer.size();++j){
      auto ap(answer[i].begin());
      auto bp(answer[j].begin());
      auto a(answer[i].begin());
      auto b(answer[j].begin());
      a++;b++;
      while(a!=answer[i].end() && b!=answer[j].end()){
        Vector2D A(ap.first.x,ap.first.y);
        Vector2D B(bp.first.x,bp.first.y);
        Vector2D VA(a.first.x-ap.first.x,(*a)->n.y-(*ap)->n.y);
        VA.Normalize();
        Vector2D VB(b.first.x-bp.first.x,(*b)->n.y-(*bp)->n.y);
        VB.Normalize();
        if(collisionImminent(A,VA,.25,ap.second,a.second,B,VB,.25,bp.second,b.second)){
          std::cout << "Collision: " << i << ":" << **ap << "-->" << **a << "," << j << ":" << **bp << "-->" << **b << "\n";
          return false;
        }
        if(fless(a.second,b.second)){
          ++a;
          ++ap;
        }else if(fgreater(a.second,b.second)){
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

void join(std::stringstream& s, std::vector<float> const& x){
  copy(x.begin(),x.end(), std::ostream_iterator<float>(s,","));
}

struct ICTSNode{
  ICTSNode(ICTSNode* parent,int agent, float size):instance(parent->instance),dag(parent->dag),best(parent->sizes.size()),bestSeen(parent->bestSeen),sizes(parent->sizes),root(parent->root),maxdepth(parent->maxdepth),increment(parent->increment){
    count++;
    sizes[agent]=size;
    best[agent]=9999999;
    maxdepth=max(maxdepth,Node::env->HCost(instance.first[agent],instance.second[agent])+sizes[agent]);
    if(verbose)std::cout << "replan agent " << agent << " GetMDD("<<(Node::env->HCost(instance.first[agent],instance.second[agent])+sizes[agent])<<")\n";
    dag[agent].clear();
    replanned.push_back(agent);
    GetMDD(instance.first[agent],instance.second[agent],dag[agent],root,Node::env->HCost(instance.first[agent],instance.second[agent])+sizes[agent],best[agent]);
    bestSeen=std::max(bestSeen,best[agent]);
    // Replace new root node on top of old.
    std::swap(root[agent],root[root.size()-1]);
    root.resize(root.size()-1);
    if(verbose)std::cout << agent << ":\n" << root[agent] << "\n";
  }

  ICTSNode(Instance const& inst, std::vector<float> const& s, float inc=1.0):instance(inst),dag(s.size()),best(s.size()),bestSeen(0),sizes(s),maxdepth(-99999999),increment(inc){
    count++;
    root.reserve(s.size());
    replanned.resize(s.size());
    for(int i(0); i<instance.first.size(); ++i){
      best[i]=9999999;
      replanned[i]=i;
      maxdepth=max(maxdepth,Node::env->HCost(instance.first[i],instance.second[i])+sizes[i]);
      if(verbose)std::cout << "plan agent " << i << " GetMDD("<<(Node::env->HCost(instance.first[i],instance.second[i])+sizes[i])<<")\n";
      GetMDD(instance.first[i],instance.second[i],dag[i],root,Node::env->HCost(instance.first[i],instance.second[i])+sizes[i],best[i]);
      bestSeen=std::max(bestSeen,best[i]);
      if(verbose)std::cout << i << ":\n" << root[i] << "\n";
    }
  }

  ~ICTSNode(){for(auto d:toDelete){delete d;}}

  // Get unique identifier for this node
  std::string key()const{
    std::stringstream sv;
    join(sv,sizes);
    return sv.str();
  }

  Instance instance;
  std::vector<DAG> dag;
  std::vector<float> sizes;
  std::vector<float> best;
  float bestSeen;
  MultiState root;
  float maxdepth;
  float increment;
  Instance points;
  std::vector<Node*> toDelete;
  static uint64_t count;
  static bool pairwise;
  static bool forwardjump;
  static bool suboptimal;
  static bool astar;
  std::vector<int> replanned; // Set of nodes that was just re-planned

  MultiState* isValid(){
    if(root.size()>2 && pairwise){
      // Perform pairwise check
      if(verbose)std::cout<<"Pairwise checks\n";
      if(replanned.size()>1){
        for(int i(0); i<root.size(); ++i){
          for(int j(i+1); j<root.size(); ++j){
            MultiState tmproot(2);
            tmproot[0]=root[i];
            tmproot[1]=root[j];
            std::vector<std::set<Node*,NodePtrComp>> tmpanswer(sizes.size());
            std::vector<Node*> toDeleteTmp;
            // This is a satisficing search, thus we only need to a sub-optimal check
            if(!jointDFS(tmproot,maxdepth,tmpanswer,toDeleteTmp,9999999,increment,true)){
              if(verbose)std::cout << "Pairwise failed\n";
              return nullptr;
            }
          }
        }
      }else{
        for(int i(0); i<root.size(); ++i){
          if(i==replanned[0]){continue;}
          MultiState tmproot(2);
          tmproot[0]=root[i];
          tmproot[1]=root[replanned[0]];
          std::vector<std::set<Node*,NodePtrComp>> tmpanswer(sizes.size());
          std::vector<Node*> toDeleteTmp;
          // This is a satisficing search, thus we only need to a sub-optimal check
          if(!jointDFS(tmproot,maxdepth,tmpanswer,toDeleteTmp,9999999,increment,true)){
            if(verbose)std::cout << "Pairwise failed\n";
            return nullptr;
          }
        }
      }
    }
    // Do a depth-first search; if the search terminates at a goal, its valid.
    if(verbose)std::cout<<"Full check\n";
    answer.resize(sizes.size());
    if(astar&&jointAStar(root,maxdepth,answer,toDelete,instance,best,increment)||
    !astar&&jointDFS(root,maxdepth,answer,toDelete,bestSeen,increment,suboptimal)){
      if(verbose){
        std::cout << "Answer:\n";
        for(int agent(0); agent<answer.size(); ++agent){
          std::cout << agent << ":\n";
          for(auto a(answer[agent].begin()); a!=answer[agent].end(); ++a){
            std::cout << std::string((*a)->depth,' ') << **a << "\n";
          }
          std::cout << "\n";
        }
        std::cout << std::endl;
      }
      if(validate)assert(checkAnswer(answer));
      return root;
    }
    
    if(verbose)std::cout << "Full check failed\n";
    return nullptr;
  }

  float SIC()const{
    float total(0);
    for(auto const& s:best){
      total += s;
    }
    return total;
  }
};

bool ICTSNode::suboptimal(false); // Sub-optimal variant
uint64_t ICTSNode::count(0);
bool ICTSNode::pairwise(false);
bool ICTSNode::forwardjump(false);
bool ICTSNode::astar(false);

struct ICTSNodePtrComp
{
  bool operator()(const ICTSNode* lhs, const ICTSNode* rhs) const  { return lhs->SIC()>rhs->SIC(); }
};

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
        while(1){
          if(a==solution[i].size() || b==solution[j].size()){break;}
          Vector2D A(solution[i][a-1].first.x,solution[i][a-1].first.y);
          Vector2D B(solution[j][b-1].first.x,solution[j][b-1].first.y);
          Vector2D VA(solution[i][a].first.x-solution[i][a-1].first.x,solution[i][a].first.y-solution[i][a-1].first.y);
          VA.Normalize();
          Vector2D VB(solution[j][b].first.x-solution[j][b-1].first.x,solution[j][b].first.y-solution[j][b-1].first.y);
          VB.Normalize();
          if(collisionImminent(A,VA,.25,solution[i][a-1].second,solution[i][a].second,B,VB,.25,solution[j][b-1].second,solution[j][b].second)){
            if(verbose)std::cout << i << " and " << j << " collide at " << solution[i][a-1].second << "~" << solution[i][a].second << solution[i][a-1].first << "-->" << solution[i][a].first << " X " << solution[j][b-1].first << "-->" << solution[j][b].first << "\n";
            // Combine groups i and j
            
            Group* toDelete(group[j]);
            groups.erase(group[j]);
            for(auto a:group[j]->agents){
              if(verbose){
                std::cout << "Inserting agent " << a << " into group for agent " << i << "\n";
              }
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

Solution solution;
//unsigned timeout(300);
unsigned timeout(7777777);
float total(0.0);
float length(0.0);
int failed(0);
float cost(0);
char* p2;
Timer tmr;

void printResults(){
  std::cout << "Solution:\n";
  int ii=0;
  for(auto const& p:solution){
    std::cout << ii++ << "\n";
    for(auto const& t: p){
      // Print solution
      std::cout << t.first << "," << t.second << "\n";
    }
  }
  for(auto const& path:solution){
    for(int j(path.size()-1); j>0; --j){
      if(path[j-1].first!=path[j].first){
        cost += path[j].second;
        length += j;
       if(verbose)std::cout << "Adding " << path[j].first<<","<<path[j].second<<"\n";
        break;
      }else if(j==1){
        cost += path[0].second;
        length += 1;
       if(verbose)std::cout << "Adding_" << path[0].first<<","<<path[0].second<<"\n";
      }
    }
  }
  total=tmr.EndTimer();
  //std::cout << elapsed << " elapsed";
  //std::cout << std::endl;
  //total += elapsed;
  std::cout << p2 << "," << int(Node::env->GetConnectedness()) << "," << ICTSNode::count << "," << jointnodes << "," << Node::count << "," << total << "," << length << "," << cost;
  if(total >= timeout)std::cout << " failure";
  std::cout << std::endl;
  exit(1);
}

bool extractSolutions(MultiEdge const* ans, Solution tmp, std::vector<Solution>& s){
  for(int i(0); i<tmp.size(); ++i){
    tmp[i].emplace_back(ans[i].second->n,ans[i].second->depth);
  }
  if(ans->successors.empty()){
    s.push_back(tmp);
  }else{
    for(auto const& ans->successors){
      extractSolution(ans,tmp,s);
    }
  }
}
bool extractSolution(MultiEdge const* ans, Solution& s, std::unordered_map<int,std::vector<int>> Gid, int theGroup){
  // Fill in solutions
  std::vector<Solution> result;
  extractSolution(ans,Solution(ans->size()),tmp);

  std::vector<int> outsiders;
  k=0;
  while(k<s.size()){
    bool found(false);
    for(int i(0); i<ans->size(); ++i){
      if(k==Gid[theGroup][i]){
        found=true;
        break;
      }
    }
    if(!found)
      outsiders.push_back(k);
  }
  // TODO: perform check of all outsiders paths with answer DAG.
  // If a chain from root to leaf can be attained without conflict, we're good
  for(int i:outsiders){
    
  }
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
  Points s;
  Points e;
  if(argc>2){
    p2=argv[2];
    if(argv[2][0]=='r'){ // Param is "r6" for random instance, 6-agents
      n=atoi(argv[2]+1);
    }else{
      std::cout << "Reading instance from file: \""<<argv[2]<<"\"\n";
      std::ifstream ss(argv[2]);
      int x,y;
      std::string line;
      while(std::getline(ss, line)){
        std::istringstream is(line);
        std::string field;
        bool goal(0);
        while(is >> field){
          sscanf(field.c_str(),"%d,%d", &x,&y);
          if(goal){
            e.emplace_back(x,y);
            //std::cout << e.back() << "\n";
          }else{
            s.emplace_back(x,y);
            //std::cout << s.back() << "-->";
          }
          goal=!goal;
        }
      }
      n=s.size();
      //std::cout << "read " << n << " instances\n";
    }
  }
  if(argc>3){ // Anything in the 3rd parameter indicates turn on pairwise
    if(argv[3][0]=='s'){
      ICTSNode::suboptimal=true;
      std::cout << "suboptimal\n";
    }
    else if(argv[3][0]=='p'){
      ICTSNode::pairwise=true;
      std::cout << "pairwise\n";
    }
    else if(argv[3][0]=='b'){
      ICTSNode::pairwise=true;
      ICTSNode::suboptimal=true;
      std::cout << "pairwise,suboptimal\n";
    }
    else if(argv[3][0]=='a'){
      ICTSNode::pairwise=true;
      ICTSNode::astar=true;
      std::cout << "pairwise,astar\n";
    }
    //if(argv[3][0]=='f'){
      //std::cout << "forwardjump\n";
      //ICTSNode::forwardjump=true;
    //}
  }
  if(argc>4){
    step=atof(argv[4]);
    std::cout << "Step " << step << "\n";
  }
  if(argc>5){
    verbose=true;
  }
  Node::env=&env;
  {
    //std::cout << "N="<<n<<"\n";
    int numAgents(n);
    //for(int t(0); t<100; ++t)
    //checked.clear();
    //std::cout << "Trial #"<<t<<"\n";
    if(s.empty()){
      int seed(123456);
      srand(seed);
      std::set<xyLoc> st;
      std::set<xyLoc> en;
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
    }

    //TemplateAStar<xyLoc,tDirection,MapEnvironment> astar;
    PEAStar<xyLoc,tDirection,MapEnvironment> astar;
    //std::cout << "Init groups\n";
    std::vector<Group*> group(s.size()); // Agent#-->group#
    std::unordered_set<Group*> groups;
    // Add a singleton group for all groups
    for(int i(0); i<s.size(); ++i){
      group[i]=new Group(i); // Initially in its own group
      groups.insert(group[i]);
    }

    // Initial individual paths.
    for(int i(0); i<s.size(); ++i){
      Points path;
      if(s[i]==e[i]){
        path.push_back(s[i]);
      }else{
        astar.GetPath(&env,s[i],e[i],path);
      }
      TimePath timePath;
      //std::cout << s[i] << "-->" << e[i] << std::endl;
      if(path.empty()){std::cout << "AStar failed on instance " << i << " - No solution\n"; return 0;}
      timePath.emplace_back(path[0],0.0);
      for(int i(1); i<path.size(); ++i){
        timePath.emplace_back(path[i],timePath[timePath.size()-1].second+Util::distance(path[i-1].x,path[i-1].y,path[i].x,path[i].y));
      }
      solution.push_back(timePath);
    }
    if(verbose){
      std::cout << std::endl;
      std::cout << "Initial solution:\n";
      int ii(0);
      for(auto const& p:solution){
        std::cout << ii++ << "\n";
        for(auto const& t: p){
          // Print solution
          std::cout << t.first << "," << t.second << "\n";
        }
      }
    }

    //float elapsed(timeout);
    // Start timing
    //std::cout << std::setprecision(3);
    tmr.StartTimer();

    Timer::Timeout func(std::bind(&printResults));
    tmr.StartTimeout(std::chrono::seconds(timeout),func);
    while(!detectIndependence(solution,group,groups)){
      std::unordered_map<int,Instance> G;
      std::unordered_map<int,std::vector<int>> Gid;
      if(verbose)std::cout << "There are " << groups.size() << " groups" << std::endl;
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
        if(verbose)std::cout << "Group " << j <<":\n";
        if(verbose)for(int i(0); i<g.first.size(); ++i){
          std::cout << Gid[j][i] << ":" << g.first[i] << "-->" << g.second[i] << "\n";
        }
        if(g.first.size()>1){
          std::vector<float> sizes(g.first.size());
          custom_priority_queue<ICTSNode*,ICTSNodePtrComp> q;
          std::unordered_set<std::string> deconf;

          q.push(new ICTSNode(g,sizes,step));

          std::vector<std::set<Node*,NodePtrComp>> answer;
          std::vector<ICTSNode*> toDelete;
          while(q.size()){
            ICTSNode* parent(q.popTop());
            if(verbose){
              std::cout << "pop ";
              for(auto const& a: parent->sizes){
                std::cout << a << " ";
              }
              std::cout << "\n";
              std::cout << "SIC: " << parent->SIC() << std::endl;
            }
            MultiEdge* root;
            if(root=parent->isValid()){
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
              sz[i]+=step;
              std::stringstream sv;
              join(sv,sz);
              if(deconf.find(sv.str())==deconf.end()){
                q.push(new ICTSNode(parent,i,sz[i]));
                deconf.insert(sv.str());
              }
            }
            toDelete.push_back(parent);
          }
          for(auto z:toDelete){
            delete z;
          }
          while(!q.empty()){
            delete q.top();
            q.pop();
          }
        }
      }
    }

    for(auto y:groups){
      delete y;
    }

  }
  printResults();
  return 0;
}
