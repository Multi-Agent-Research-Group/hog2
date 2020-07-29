/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
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

#ifndef __ICTSAlgorithm_H__
#define __ICTSAlgorithm_H__

#include "Common.h"
#include <memory>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <set>
#include <numeric>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>
#include "CollisionDetection.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "MultiAgentStructures.h"
#include "Utilities.h"

extern double agentRadius;

#define INFTY 999999999
// for agents to stay at goal
const uint32_t MAXTIME(0xfffff);

template<typename T, typename C>
class custom_priority_queue : public std::priority_queue<T, std::vector<T>, C>
{
  public:
    virtual T popTop(){
      T val(const_cast<T&>(this->top()).release());
      this->pop();
      return val;
    }
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

template<typename state, typename action>
class ICTSAlgorithm: public MAPFAlgorithm<state,action>{
  public:
    ICTSAlgorithm(std::vector<float> const& radi):radii(radi),jointTime(0),pairwiseTime(0),mddTime(0),verbose(false),quiet(false),verify(false),jointnodes(0),step(state::TIME_RESOLUTION_U),suboptimal(false),pairwise(true){}
    std::vector<float> radii;
    float jointTime;
    float pairwiseTime;
    float mddTime;

    bool verbose;
    bool quiet;
    bool verify;
    bool epsilon;
    bool suboptimal;
    bool pairwise;
    uint64_t jointnodes;
    uint32_t step;
    //int n;
    std::unordered_map<std::string,bool> transTable;
    std::unordered_map<uint64_t,bool> singleTransTable;

    std::vector<SearchEnvironment<state,action>*> envs;
    std::vector<Heuristic<state>*> heuristics;


    // Used for std::set
    struct NodePtrComp
    {
      bool operator()(const Hashable* lhs, const Hashable* rhs) const  { return lhs->Depth()<rhs->Depth(); }
    };

    struct Node : public Hashable{
      static uint64_t count;

      Node(){}
      Node(state a, uint64_t h):n(a),hash(h),id(0),optimal(false){count++;}
      virtual ~Node(){}
      state n;
      //uint32_t depth;
      uint64_t hash;
      uint32_t id;
      bool optimal;
      //bool connected()const{return parents.size()+successors.size();}
      //std::unordered_set<Node*> parents;
      std::unordered_set<Node*> successors;
      virtual uint64_t Hash()const{return hash;}//(env->GetStateHash(n)<<32) | ((uint32_t)(depth*state::TIME_RESOLUTION_U));
      virtual uint32_t Depth()const{return n.t;}
      virtual void Print(std::ostream& ss, int d=0) const {
        ss << std::string(d,' ')<<n <<"_"<<n.t<< std::endl;
        for(auto const& m: successors)
          m->Print(ss,d+1);
      }
      bool operator==(Node const& other)const{return n==other.n;}
    };

    typedef std::pair<MultiAgentState<state>,MultiAgentState<state>> Instance;
    typedef std::vector<Node*> Path;
    typedef std::vector<std::vector<Node*>> MDDSolution;

    typedef std::vector<Node*> MultiState; // rank=agent num
    typedef std::vector<std::pair<Node*,Node*>> MultiEdge; // rank=agent num
    typedef std::unordered_map<uint64_t,Node> DAG;
    std::unordered_map<uint64_t,Node*> mddcache;
    std::unordered_map<uint64_t,uint32_t> lbcache;
    std::unordered_map<uint64_t,uint32_t> mscache;

    /*class MultiEdge: public Multiedge{
      public:
      MultiEdge():Multiedge(),parent(nullptr){}
      MultiEdge(Multiedge const& other):Multiedge(other),parent(nullptr){} // Do not copy successors!
      std::vector<MultiEdge> successors;
      MultiEdge* parent;
      void Print(std::ostream& ss, int d=0) const {
      ss << std::string(d,' ');
      int i(0);
      for(auto const& a: *this)
      ss << " "<<++i<<"." << a.second->n << "@" << a.second->Depth();
      ss << std::endl;
      for(auto const& m: successors)
      m.Print(ss,d+1);
      }
      };*/

    // Compute path cost, ignoring actions that wait at the goal
    static uint64_t computePathCost(Path const& path, bool ignoreWaitAtGoal=true){
      uint64_t cost(0);
      if(ignoreWaitAtGoal){
        for(int j(path.size()-1); j>0; --j){
          if(!path[j-1]->n.sameLoc(path[j]->n)){
            cost+=path[j]->Depth();
            break;
          }else if(j==1){
            cost+=path[0]->Depth();
          }
        }
      }else{
        cost+=path.back()->Depth();
      }
      return cost;
    }

    static uint64_t computeSolutionCost(MDDSolution const& solution, bool ignoreWaitAtGoal=true){
      uint64_t cost(0);
      for(auto const& path:solution){
        cost+=computePathCost(path,ignoreWaitAtGoal);
      }
      return cost;
    }

    static inline bool get(uint64_t* bitarray, size_t idx) {
      return bitarray[idx / 64] & (1 << (idx % 64));
    }
    static inline void set(uint64_t* bitarray, size_t idx) {
      bitarray[idx / 64] |= (1 << (idx % 64));
    }

    // Big caveat - this implementation assumes that time IS a component of the state hash
    uint64_t GetHash(state const &n, unsigned agent)
    {
      return (envs[agent]->GetStateHash(n));
    }

    bool LimitedDFS(state const& start, state const& end,
    DAG& dag, Node*& root, uint32_t depth, uint32_t maxDepth,
    uint32_t& best, SearchEnvironment<state,action>* env,
    unsigned agent, unsigned margin){
      if(depth<0 || maxDepth-depth+(int)(HCost(start,end,agent))>maxDepth){ // Note - this only works for a perfect heuristic.
        //std::cout << " pruned " << depth <<" "<< (maxDepth-depth+(int)(env->HCost(start,end)))<<">"<<maxDepth<<"\n";
        return false;
      }
      Node n(start,GetHash(start,agent));
      uint64_t hash(n.Hash());
      if(singleTransTable.find(hash)!=singleTransTable.end()){return singleTransTable[hash];}
      //std::cout << "\n";

      // We're not 'at the goal' unless we can't wait here any longer...
      if(env->GoalTest(start,end) && depth<margin){ // Time resolution represents the shortest poss. action
        singleTransTable[hash]=true;
        n.id=dag.size();
        dag[hash]=n;
        // This may happen if the agent starts at the goal
        if(maxDepth-depth<=0){
          root=&dag[hash];
          //std::cout << "root_ " << &dag[hash];
        }
        Node* parent(&dag[hash]);
        uint32_t d(parent->Depth());
        //int d(maxDepth-depth);
        while(d+envs[agent]->WaitTime()<=maxDepth){ // Increment depth by 1 for wait actions
          // Wait at goal
          d+=envs[agent]->WaitTime();
          state tmp(start,d);
          Node current(tmp,GetHash(tmp,agent));
          uint64_t chash(current.Hash());
          current.id=dag.size();
          dag[chash]=current;
          if(verbose)std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
          parent->successors.insert(&dag[chash]);
          //dag[chash].parents.insert(parent);
          parent=&dag[chash];
        }
        best=std::min(best,parent->Depth());
        //std::cout << "found d\n";
        //if(verbose)std::cout << "ABEST "<<best<<"\n";
        return true;
      }

      MultiAgentState<state> successors(64);
      auto sz(env->GetSuccessors(n.n,&successors[0]));
      bool result(false);
      for(unsigned i(0);i<sz; ++i){
        auto const& node(successors[i]);
        unsigned ddiff(node.t-n.Depth());

        // ENFORCE NO WAIT AT GOAL on LAST STEP
        bool atGoal(env->GoalTest(node, end) && depth-ddiff<margin);
        if(atGoal && start.sameLoc(node))
          continue; // Never allow waiting at the goal for the final action
        // END ENFORCE NO WAIT...

        if(LimitedDFS(node,end,dag,root,depth-ddiff,maxDepth,best,env,agent,margin)){
          singleTransTable[hash]=true;
          if(dag.find(hash)==dag.end()){
            n.id=dag.size();
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

          Node* parent(&dag[hash]);

          //std::cout << "found " << start << "\n";
          uint32_t newDepth(maxDepth-depth+ddiff);
          state tmp(node,newDepth);
          uint64_t chash(GetHash(tmp,agent));
          if(dag.find(chash)==dag.end()&&dag.find(chash+1)==dag.end()&&dag.find(chash-1)==dag.end()){
            std::cout << "Expected " << Node(tmp,GetHash(tmp,agent)) << " " << chash << " to be in the dag\n";
            assert(!"Uh oh, node not already in the DAG!");
            //std::cout << "Add new.\n";
            //Node c(tmp,chash);
            //dag[chash]=c;
          }
          Node* current(&dag[chash]);
          current->optimal = result = true;
          //std::cout << *parent << " parent of " << *current << "\n";
          //dag[current->Hash()].parents.insert(parent);
          //std::cout << *current << " child of " << *parent << " " << parent->Hash() << "\n";
          //std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
          dag[parent->Hash()].successors.insert(&dag[current->Hash()]);
          //std::cout << "at" << &dag[parent->Hash()] << "\n";
        }
      }
      singleTransTable[hash]=result;
      if(!result){
        dag.erase(hash);
      }
      return result;
    }

    // Perform conflict check by moving forward in time at increments of the smallest time step
    // Test the efficiency of VO vs. time-vector approach
    bool GetMDD(unsigned agent,state const& start, state const& end, DAG& dag,
    MultiState& root, int depth, uint32_t& best, uint32_t& dagsize,
    SearchEnvironment<state,action>* env, unsigned size){
      root[agent]=nullptr;
      if(verbose)std::cout << "MDD up to depth: " << depth << start << "-->" << end << "\n";
      uint64_t hash(((uint32_t) size)<<8|agent);
      bool found(mddcache.find(hash)!=mddcache.end());
      if(verbose)std::cout << "lookup "<< (found?"found":"missed") << "\n";
      if (!found) {
        singleTransTable.clear();
        if (LimitedDFS(start, end, dag, root[agent], depth, depth, best, envs[agent], agent, size?state::TIME_RESOLUTION:1)) {
          mddcache[hash] = root[agent];
          lbcache[hash] = best;
          mscache[hash] = dagsize = dag.size();
          return true;
        }else{
          // No solution found for cost
          mddcache[hash] = 0;
          lbcache[hash] = INFTY;
          mscache[hash] = dagsize = 0;
          if(verbose)std::cout << "No solution: " << depth << start << "-->" << end << "\n";
        }
      }else{
        if(!mddcache[hash])return false;
        root[agent]=mddcache[hash];
        best=lbcache[hash];
        dagsize=mscache[hash];
        return true;
      }
      return false;
    }

    void generatePermutations(std::vector<MultiEdge>& positions, std::vector<MultiEdge>& result, int agent, MultiEdge const& current, uint32_t lastTime) {
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
          if(collisionCheck3D(positions[agent][i].first->n,positions[agent][i].second->n,current[j].first->n,current[j].second->n,radii[agent],radii[j])){
          // Make sure we don't do any checks that were already done
          //if(positions[agent][i].first->Depth()==lastTime&&current[j].first->Depth()==lastTime)continue;
          //uint64_t hash(EdgePairHash(positions[agent][i],current[j]));
          //if(checked.find(hash)!=checked.end())
          //{std::cout << "SKIPPED " << *positions[agent][i].second << " " << *current[j].second << "\n"; continue; /*No collision check necessary; checked already*/}
          //std::cout << "COMPARE " << *positions[agent][i].second << " " << *current[j].second << "\n";
            if(verbose)std::cout << "Collision averted: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
            found=true;
            //checked.insert(hash);
            break;
          }
          if(verbose)std::cout << "generating: " << *positions[agent][i].first << "-->" << *positions[agent][i].second << " " << *current[j].first << "-->" << *current[j].second << "\n";
        }
        if(found) continue;
        copy.push_back(positions[agent][i]);
        generatePermutations(positions, result, agent + 1, copy,lastTime);
      }
    }

    // In order for this to work, we cannot generate sets of positions, we must generate sets of actions, since at time 1.0 an action from parent A at time 0.0 may have finished, while another action from the same parent A may still be in progress. 

    // Return true if we get to the desired depth
    bool jointDFS(MultiEdge const &s, uint32_t d, MDDSolution solution,
                  std::vector<MDDSolution> &solutions, std::vector<Node *> &toDelete,
                  uint64_t bestSeen, uint64_t &best, std::vector<uint32_t> &bestCosts,
                  std::vector<uint32_t> const& minimum,
                  bool suboptimal = false, bool checkOnly = false)
    {
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

      if(!checkOnly&&d>0){
        // Copy solution so far, but only copy components if they are
        // a valid successor
        for(int i(0); i<solution.size(); ++i){
          auto const& p = solution[i];
          bool found(false);
          for(auto const& suc:p.back()->successors){
            // True successor or wait action...
            if(*s[i].second==*suc){
              found=true;
              break;
            }
          }
          if(found || (s[i].second->n.sameLoc(p.back()->n) && s[i].second->Depth()>p.back()->Depth())){
            solution[i].push_back(s[i].second);
          }
        }
      }

      bool done(true);
      for(auto const& g:s){
        if(g.second->Depth()!=MAXTIME){
          done=false;
          break;
        }
        //maxCost=std::max(maxCost,g.second->Depth());
      }
      if (done)
      {
        std::vector<uint32_t> costs(s.size());
        uint64_t cost(0);
        if (checkOnly)
        {
          k = 0;
          solution.resize(s.size());
          for (auto const &g : s)
          {
            solution[k].reserve(2);
            solution[k].push_back(g.first);
            solution[k].push_back(g.second);
            costs[k] = computePathCost(solution[k]);
            cost += costs[k];
            ++k;
          }
          solutions.clear();
          solutions.push_back(solution);
        }
        if (cost < best)
        {
          bestCosts = costs;
          best = cost;
          if (verbose)
            std::cout << "BEST=" << best << std::endl;
          if (verbose)
            std::cout << "BS=" << bestSeen << std::endl;

          // This is a leaf node
          // Copy the solution into the answer set
          if (!checkOnly)
          {
            // Shore up with wait actions
            for (int i(0); i < solution.size(); ++i)
            {
              if (solution[i].back()->Depth() < MAXTIME)
              {
                state tmp(solution[i].back()->n, MAXTIME);
                solution[i].emplace_back(new Node(tmp, GetHash(tmp, i)));
                toDelete.push_back(solution[i].back());
              }
            }
            solutions.clear();
            solutions.push_back(solution);
          }
        }
        return true;
      }
      //Get successors into a vector
      std::vector<MultiEdge> successors;
      successors.reserve(s.size());

      // Find minimum depth of current edges
      uint32_t sd(INFTY);
      for(auto const& a: s){
        sd=min(sd,a.second->Depth());
      }
      //std::cout << "min-depth: " << sd << "\n";

      uint32_t md(INFTY); // Min depth of successors
      //Add in successors for parents who are equal to the min
      k=0;
      for(auto const& a: s){
        MultiEdge output;
        if(a.second->Depth()<=sd){
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
          // Stay at state...
          // Only set time MAX if it is above the minimum cost (this is used to indicate a goal state)
          state tmp(a.second->n,(a.second->Depth()>minimum[k]?MAXTIME:a.second->Depth()+state::TIME_RESOLUTION));
          output.emplace_back(a.second,new Node(tmp,GetHash(tmp,successors.size())));
          //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
          toDelete.push_back(output.back().second);
          //md=min(md,a.second->Depth()+1.0); // Amount of time to wait
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
      std::vector<MultiEdge> crossProduct;
      MultiEdge tmp;
      generatePermutations(successors,crossProduct,0,tmp,sd);
      bool value(false);
      for(auto& a: crossProduct){
        if(verbose)std::cout << "EVAL " << s << "-->" << a << "\n";
        if(jointDFS(a,md,solution,solutions,toDelete,bestSeen,best,bestCosts,minimum,suboptimal,checkOnly)){
          value=true;
          transTable[hash]=value;
          // Return first solution... (unless this is a pairwise check with pruning)
          if(suboptimal&&!(checkOnly)) return true;
          // Return if solution is as good as any MDD
          if((!(checkOnly))&&best==bestSeen)return true;
        }
      }
      transTable[hash]=value;
      return value;
    }

    bool jointDFS(MultiState const &s, std::vector<MDDSolution> &solutions,
                  std::vector<Node *> &toDelete, uint64_t bestSeen, uint64_t &best,
                  std::vector<uint32_t> &bestCosts, std::vector<uint32_t> const& minimum,
                  bool suboptimal = false, bool checkOnly = false)
    {
      if(verbose)std::cout << "JointDFS\n";
      MultiEdge act;
      MDDSolution solution;
      std::unordered_set<std::string> ttable;
      // Add null parents for the initial movements
      for(auto const& n:s){
        act.emplace_back(nullptr,n);
        if(!checkOnly){
          // Add initial state for solution
          solution.push_back({n});
        }
      }
      best=0xffffffffffffffff;

      transTable.clear();
      return jointDFS(act,0.0,solution,solutions,toDelete,bestSeen,best,bestCosts,minimum,suboptimal,checkOnly);
    }

    // Check that two paths have no conflicts
    static bool checkPair(Path const& p1, Path const& p2, float r1, float r2, bool loud=false){
      auto ap(p1.begin());
      auto a(ap+1);
      auto bp(p2.begin());
      auto b(bp+1);
      while(a!=p1.end() && b!=p2.end()){
        if(collisionCheck3D((*ap)->n,(*a)->n,(*bp)->n,(*b)->n,r1,r2)){
          if(loud)std::cout << "Collision: " << **ap << "-->" << **a << "," << **bp << "-->" << **b;
          return false;
        }
        if((*a)->Depth()<(*b)->Depth()){
          ++a;
          ++ap;
        }else if((*a)->Depth()>(*b)->Depth()){
          ++b;
          ++bp;
        }else{
          ++a;++b;
          ++ap;++bp;
        }
      }
      return true;
    }

    // Not part of the algorithm... just for validating the answers
    bool checkAnswer(MDDSolution const& answer){
      for(int i(0);i<answer.size();++i){
        for(int j(i+1);j<answer.size();++j){
          if(!checkPair(answer[i],answer[j],radii[i],radii[j])){
            if(verify)std::cout<< "for agents: " << i << " and " << j << "\n";
            return false;
          }
        }
      }
      return true;
    }

    static void parse(std::string const& s, std::vector<uint32_t> &sizes){
        auto strings(Util::split(s,','));
        if(strings.size()!=sizes.size()){
          std::cout << "bad string: " << s << "expected length of " << sizes.size() << "but got " << strings.size() << "\n";
        }else{
          for(int i(0); i<sizes.size(); ++i){
            sizes[i]=atoi(strings[i].c_str());
          }
        }
    }
    static std::string join(std::vector<uint32_t> const& x){
      std::stringstream s;
      copy(x.begin(),x.end(), std::ostream_iterator<uint32_t>(s,","));
      return s.str().substr(0,s.str().size()-1);
    }

    uint32_t HCost(state const& a, state const& b, unsigned agent)const{
      return heuristics[agent]?round(heuristics[agent]->HCost(a,b)*state::TIME_RESOLUTION_D):round(envs[agent]->HCost(a,b)*state::TIME_RESOLUTION_D);
    }

    struct ICTSNode{
      ICTSNode(ICTSAlgorithm* alg, ICTSNode* parent,int agent, uint32_t size):
      ictsalg(alg),instance(parent->instance),dag(parent->dag.size()),
      best(parent->best),minimum(parent->minimum),dagsize(parent->dagsize),
      bestSeen(0),sizes(parent->sizes),root(parent->root),ok(true){

        sizes[agent]=size;
        // Make sure we're not combining with bogus MDDs
        for(unsigned i(0); i<sizes.size(); ++i){
          uint64_t hash(((uint32_t)sizes[i]) << 8 | i);
          if(alg->mddcache.find(hash) != alg->mddcache.end()&& !alg->mddcache[hash]){
            ok=false;
          }
        }
        count++;
        best[agent]=INFTY;
        unsigned newsz(unsigned(alg->HCost(instance.first[agent],instance.second[agent],agent))+unsigned(sizes[agent]));
        if(alg->verbose)std::cout << "replan agent " << agent << " GetMDD("<<newsz<<")\n";
        replanned.push_back(agent);
        Timer timer;
        timer.StartTimer();
        ok &= ictsalg->GetMDD(agent,instance.first[agent],
        instance.second[agent],
        dag[agent],root,newsz,
        best[agent],dagsize[agent],
        alg->envs[agent],size);
        ictsalg->mddTime+=timer.EndTimer();
        minimum[agent] = newsz - state::TIME_RESOLUTION; // lower bound
        bestSeen=std::accumulate(best.begin(),best.end(),0);
        // Replace new root node on top of old.
        //std::swap(root[agent],root[root.size()-1]);
        //root.resize(root.size()-1);
        //if(verbose)std::cout << agent << ":\n" << root[agent] << "\n";
      }

      ICTSNode(ICTSAlgorithm *alg,
               Instance const &inst,
               std::vector<uint32_t> const &s)
          : ictsalg(alg), instance(inst), dag(s.size()), best(s.size()), minimum(s.size()),
            dagsize(s.size()), bestSeen(0), sizes(s), root(s.size()), ok(true)
      {
        count++;
        replanned.resize(s.size());
        for(int i(0); i<instance.first.size(); ++i){
          best[i]=INFTY;
          replanned[i]=i;
          unsigned newsz(unsigned(alg->HCost(instance.first[i],instance.second[i],i))+unsigned(sizes[i]));
          if(alg->verbose)std::cout << "plan agent " << i << " GetMDD("<<newsz<<")\n";
          //std::cout.precision(17);
          std::cout.precision(6);

          Timer timer;
          timer.StartTimer();
          ok &= ictsalg->GetMDD(i,
          instance.first[i],
          instance.second[i],
          dag[i],root,newsz,
          best[i],dagsize[i],alg->envs[i],
          sizes[i]);
          ictsalg->mddTime+=timer.EndTimer();
          minimum[i]=newsz-1; // lower bound
          //if(verbose)std::cout << i << ":\n" << root[i] << "\n";
        }
        bestSeen=std::accumulate(best.begin(),best.end(),0);
      }

      virtual ~ICTSNode(){for(auto d:toDelete){delete d;}}

      // Get unique identifier for this node
      std::string key()const{
        return join(sizes);
      }

      ICTSAlgorithm* ictsalg;
      Instance instance;
      std::vector<DAG> dag;
      std::vector<uint32_t> sizes;
      std::vector<uint32_t> best;
      std::vector<uint32_t> minimum;
      std::vector<uint32_t> dagsize;
      ICTSNode* p;
      uint64_t bestSeen;
      MultiState root;
      Instance points;
      std::vector<Node*> toDelete;
      static uint64_t count;
      std::vector<int> replanned; // Set of nodes that was just re-planned
      bool ok; // Indicates whether all MDDs are valid

      // TODO add cardinal check enhancement from icts driver
      bool isValid(std::vector<MDDSolution> &answers,
                   uint64_t &bestJoint,
                   std::vector<uint32_t> &bestCosts,
                   bool costonly = false)
      {
        bestJoint=0xffffffffffffffff;
        if(!ok) return false;
        for(auto const& r:root){if(!r)return false;} // Make sure all MDDs have a solution
        uint64_t increase(0);
        if(root.size()>2 && ictsalg->pairwise){
          Timer timer;
          timer.StartTimer();
          // Perform pairwise check
          if(ictsalg->verbose)std::cout<<"Pairwise checks\n";
          std::vector<uint32_t> tmpBest(2);
          std::vector<uint32_t> ms(2);
          MultiState tmproot(2);
          for(int i(0); i<root.size(); ++i){
            tmpBest[0]=bestCosts[i];
            tmproot[0] = root[i];
            ms[0]=minimum[i];
            for(int j(i+1); j<root.size(); ++j){
              tmpBest[1] = bestCosts[j];
              tmproot[1]=root[j];
              ms[1]=minimum[j];
              std::vector<Node*> toDeleteTmp;

              // This is a satisficing search, thus we only need do a sub-optimal check
              if(ictsalg->verbose)std::cout<<"pairwise for " << i << ","<<j<<"\n";
              if(!ictsalg->jointDFS(tmproot,answers,toDeleteTmp,INFTY,bestJoint,tmpBest,ms,true,true)){
                if(ictsalg->verbose)std::cout << "Pairwise failed\n";
                return false;
              }else{
                increase=std::max(increase,bestJoint-(best[i]+best[j]));
              }
            }
          }
          ictsalg->pairwiseTime+=timer.EndTimer();
        }
        bestSeen+=increase;
        // Do a depth-first search; if the search terminates at a goal, its valid.
        if(ictsalg->verbose)std::cout<<"Pairwise passed\nFull check\n";
        Timer timer;
        timer.StartTimer();
        bestJoint=0xffffffffffffffff;
        if(ictsalg->jointDFS(root,answers,toDelete,lb(),bestJoint,bestCosts,minimum,ictsalg->suboptimal,costonly)){
          ictsalg->jointTime+=timer.EndTimer();
          if(ictsalg->verbose){
            std::cout << "Answer:\n";
            for(int num(0); num<answers.size(); ++num){
              std::cout << "number " << num << ":\n";
              for(int agent(0); agent<answers[num].size(); ++agent){
                std::cout << "  " << agent << ":\n";
                for(auto a(answers[num][agent].begin()); a!=answers[num][agent].end()-1; ++a){
                  std::cout  << "  " << std::string((*a)->Depth()/state::TIME_RESOLUTION_U,' ') << **a << "\n";
                }
                std::cout << "\n";
              }
              std::cout << "Cost: " << bestJoint << std::endl;
              if(ictsalg->verify&&!ictsalg->checkAnswer(answers[num]))std::cout<< "Failed in ICT node\n";
            }
          }
          if(ictsalg->verbose)std::cout << "Full check passed\n";
          return true;
        }

        if(ictsalg->verbose)std::cout << "Full check failed\n";
        return false;
      }

      bool operator<(ICTSNode const& other)const{
        uint32_t sic1(lb());
        uint32_t sic2(other.lb());
        if(sic1==sic2){
          uint32_t t1(0);
          for(auto const& s:sizes){
            t1 += s;
          }
          uint32_t t2(0);
          for(auto const& s:other.sizes){
            t2 += s;
          }
          return t1>t2;
        }else{
          return sic1>sic2;
        }
      }
      uint32_t ub()const{
        uint32_t total(0);
        for(auto const& s:sizes){
          total += s;
        }
        return total;
      }
      uint64_t lb()const{ return bestSeen; }
    };

    struct ICTSNodePtrComp
    {
      bool operator()(std::unique_ptr<ICTSNode> const& lhs, std::unique_ptr<ICTSNode> const& rhs) const  { return *lhs<*rhs; }
    };

    void SetVerbose(bool v){verbose=v;}
    void SetQuiet(bool v){quiet=v;}
    virtual unsigned GetNodesExpanded()const{return Node::count;}

    bool GetSolutionCosts(std::vector<EnvironmentContainer<state,action>*> const& env, MultiAgentState<state> const& start, MultiAgentState<state> const& goal, std::vector<std::vector<uint32_t>>& costs){
      bool found(false);
      for(int i(0);i<env.size(); ++i){
        envs.push_back(env[i]->environment.get());
        heuristics.push_back(env[i]->heuristic.get());
      }

      custom_priority_queue<std::unique_ptr<ICTSNode>,ICTSNodePtrComp> q;
      std::unordered_set<std::string> deconf;
      std::vector<std::set<Node*,NodePtrComp>> answer;
      std::vector<std::unique_ptr<ICTSNode>> toDelete;
      uint64_t bestCost(0xffffffffffffffff);
      Instance inst(start,goal);

      //costs.resize(1);
      auto sizes(&costs[0]);
      assert(sizes->size()==start.size());
      sizes->resize(start.size());
      q.emplace(new ICTSNode(this,inst,*sizes));

      while(q.size()){
        // Keep searching until we've found a candidate with greater cost than 'best'
        // To keep going on all plateaus <= the best is how we ensure optimality
        // It is impossible for a better solution to be further down in the tree - 
        // no node of cost>=best can possibly produce a child with better cost
        if(verbose)std::cout << "TOP: " << q.top()->lb() << " BEST: " << bestCost << "\n";
        if(q.top()->lb()>bestCost){
          found=true;
          break;
        }
        // This node could contain a solution since its lb is <=
        toDelete.push_back(q.popTop());
        ICTSNode* parent(toDelete.back().get());
        if(!quiet){
          std::cout << "pop ";
          for(auto const& a: parent->sizes){
            std::cout << a << " ";
          }
          std::cout << "\n";
          //best: 5.82843 6.65685 5.41421
          //best: 5.82843 6.65685 5.24264
          std::cout << "best: ";
          for(auto b:parent->best)std::cout << b << " ";
          std::cout << "\n";
          std::cout << "SIC: " << parent->lb() << std::endl;
        }
        std::vector<MDDSolution> answers;
        // If we found an answer set and any of the answers are not in conflict
        // with other paths in the solution, we can quit if the answer has a cost
        // less than the best SIC in the current plateau. Otherwise, we will
        // continue the ICT search until the next plateau
        std::vector<uint32_t> slnCosts(start.size(),INFTY);
        uint64_t cost(INFTY);
        if(parent->isValid(answers,cost,slnCosts,true)){
          auto cost(std::accumulate(slnCosts.begin(),slnCosts.end(),0));
          for(auto const& a:answers){
            if(cost<bestCost){
              costs.resize(1);
              sizes=&costs[0];
              bestCost=cost;
              sizes->resize(0);
              sizes->insert(sizes->begin(),slnCosts.begin(),slnCosts.end());
            }else if(cost==bestCost){
              costs.resize(costs.size()+1);
              sizes=&costs[costs.size()-1];
              sizes->resize(0);
              sizes->insert(sizes->begin(),slnCosts.begin(),slnCosts.end());
            }
          }
          // We can exit now if we've finished exploring the plateau
          if(suboptimal||q.empty()||q.top()->lb()>bestCost){
            found = true;
            /*for(auto & m:parent->root){
              std::cout << "icts-MDD:\n" << m << "\n";
            }*/
            break;
          }
        }else{
          // Split the ICTS node
          for(int i(0); i<parent->sizes.size(); ++i){
            std::vector<uint32_t> sz(parent->sizes);
            sz[i]+=step;
            std::string sv(join(sz));
            if(deconf.find(sv)==deconf.end()){
              ICTSNode* tmp(new ICTSNode(this,parent,i,sz[i]));
              if(verbose){
                std::cout << "push ";
                for(auto const& a: tmp->sizes){
                  std::cout << a << " ";
                }
                std::cout << "\n";
                //best: 5.82843 6.65685 5.41421
                //best: 5.82843 6.65685 5.24264
                std::cout << "  best: ";
                for(auto b:tmp->best)std::cout << b << " ";
                std::cout << "\n";
                std::cout << "  SIC: " << tmp->lb() << std::endl;
              }
              deconf.insert(sv);
              if(!tmp->ok &&
              this->HCost(inst.first[i],inst.second[i],i)/2>tmp->sizes[i]){
              continue;
                }
              q.emplace(tmp);
            }
          }
        }
      }

      // Clean up before exiting
      mddcache.clear();
      lbcache.clear();
      mscache.clear();
      if(!found)costs.resize(0);
      return found;
    }

    void GetSolution(std::vector<EnvironmentContainer<state,action>*> const& env, MultiAgentState<state> const& start, MultiAgentState<state> const& goal, Solution<state>& solution, std::string& hint){
      for(int i(0);i<env.size(); ++i){
        envs.push_back(env[i]->environment.get());
        heuristics.push_back(env[i]->heuristic.get());
      }

      custom_priority_queue<std::unique_ptr<ICTSNode>,ICTSNodePtrComp> q;
      std::unordered_set<std::string> deconf;
      std::vector<std::set<Node*,NodePtrComp>> answer;
      std::vector<std::unique_ptr<ICTSNode>> toDelete;
      uint64_t bestCost(0xffffffffffffffff);
      Instance inst(start,goal);
      std::string answerkey;

      std::vector<uint32_t> sizes(start.size());
      if(hint.size()>0){
        if(verbose)std::cout << "loading hint: " << hint << "\n";
        auto stuff(Util::split(hint,':'));
        uint64_t prevBest(atoi(stuff[0].c_str()));
        // Format = prevcost:<;-delimited queue stuff>:<;-delimited hash stuff>
        
        int i(0);
        for(auto h:Util::split(stuff[1],';')){
          parse(h,sizes);
          if(i++){
            q.emplace(new ICTSNode(this,inst,sizes));
          }else{
            // The first seed in the queue is the one most likely to yield an answer
            std::vector<MDDSolution> answers;
            ICTSNode* node(new ICTSNode(this,inst,sizes));
            toDelete.emplace_back(node);
            std::vector<uint32_t> slnCosts(start.size(), INFTY);
            uint64_t cost(INFTY);
            if (node->isValid(answers, cost, slnCosts))
            {
              for(auto const& a:answers){
                auto cost(computeSolutionCost(a));
                solution.resize(0);
                for(auto const& path:a){
                  std::vector<state> p;
                  for(auto const& s:path){
                    p.push_back(s->n);
                  }
                  solution.push_back(p);
                }
                bestCost=std::min(bestCost,cost);
                answerkey=node->key();
              }
              if(bestCost<=prevBest||suboptimal){return;}
            }
            else
            {
              // No solution... perform the split operation
              for(int i(0); i<node->sizes.size(); ++i){
                std::vector<uint32_t> sz(node->sizes);
                sz[i]+=step;
                std::string sv(join(sz));
                if(deconf.find(sv)==deconf.end()){
                  q.emplace(new ICTSNode(this,node,i,sz[i]));
                  deconf.insert(sv);
                }
              }
            }
          }
        }
        for(auto h:Util::split(stuff[2],';')){
          deconf.insert(h);
        }
      }else{
        q.emplace(new ICTSNode(this,inst,sizes));
      }

      while(q.size()){
        // Keep searching until we've found a candidate with greater cost than 'best'
        // To keep going on all plateaus <= the best is how we ensure optimality
        // It is impossible for a better solution to be further down in the tree - 
        // no node of cost>=best can possibly produce a child with better cost
        if(verbose)std::cout << "TOP: " << q.top()->lb() << " BEST: " << bestCost << "\n";
        if(q.top()->lb()>=bestCost){
          // Record the hint
          std::stringstream ss;
          ss << bestCost << ":"<<answerkey;
          hint=ss.str();
          while(q.size()){
            hint.append(";");
            hint.append(q.popTop()->key()); // this also deletes the contents of the queue
          }
          hint.append(":");
          hint.append(answerkey);
          for(auto const& d:deconf){
            if(d!=answerkey){
              hint.append(";");
              hint.append(d);
            }
          }
          break;
        }
        // This node could contain a solution since its lb is <=
        toDelete.push_back(q.popTop());
        ICTSNode* parent(toDelete.back().get());
        if(!quiet){
          std::cout << "pop ";
          for(auto const& a: parent->sizes){
            std::cout << a << " ";
          }
          std::cout << "\n";
          //best: 5.82843 6.65685 5.41421
          //best: 5.82843 6.65685 5.24264
          std::cout << "best: ";
          for(auto b:parent->best)std::cout << b << " ";
          std::cout << "\n";
          std::cout << "SIC: " << parent->lb() << std::endl;
        }
        std::vector<MDDSolution> answers;
        // If we found an answer set and any of the answers are not in conflict
        // with other paths in the solution, we can quit if the answer has a cost
        // equal to that of the best SIC in the current plateau. Otherwise, we will
        // continue the ICT search until the next plateau
        std::vector<uint32_t> slnCosts(start.size(), INFTY);
        uint64_t cost(INFTY);
        if (parent->isValid(answers, cost, slnCosts)){
          for(auto const& a:answers){
            auto cost(computeSolutionCost(a));
            if(cost<=bestCost){
              bestCost=cost;
              solution.resize(0);
              for(auto const& path:a){
                std::vector<state> p;
                for(auto const& s:path){
                  p.push_back(s->n);
                }
                solution.push_back(p);
              }
              answerkey=parent->key();
            }
            if(answerkey!=""&&(suboptimal||q.empty()||q.top()->lb()>bestCost)){
              // Record the hint
              std::stringstream ss;
              ss << bestCost << ":"<<answerkey;
              hint=ss.str();
              while(q.size()){
                hint.append(";");
                hint.append(q.popTop()->key()); // this also deletes the contents of the queue
              }
              hint.append(":");
              hint.append(answerkey);
              for(auto const& d:deconf){
                if(d!=answerkey){
                  hint.append(";");
                  hint.append(d);
                }
              }
              break;
            }
          }
        }else{
          // Split the ICTS node
          for(int i(0); i<parent->sizes.size(); ++i){
            std::vector<uint32_t> sz(parent->sizes);
            sz[i]+=step;
            std::string sv(join(sz));
            if(deconf.find(sv)==deconf.end()){
              ICTSNode* tmp(new ICTSNode(this,parent,i,sz[i]));
              if(verbose){
                std::cout << "push ";
                for(auto const& a: tmp->sizes){
                  std::cout << a << " ";
                }
                std::cout << "\n";
                //best: 5.82843 6.65685 5.41421
                //best: 5.82843 6.65685 5.24264
                std::cout << "  best: ";
                for(auto b:tmp->best)std::cout << b << " ";
                std::cout << "\n";
                std::cout << "  SIC: " << tmp->lb() << std::endl;
              }
              q.emplace(tmp);
              deconf.insert(sv);
            }
          }
        }
      }

      // Clean up before exiting
      mddcache.clear();
      lbcache.clear();
      mscache.clear();
    }

    friend std::ostream& operator << (std::ostream& ss, MultiState const& n){
      int i(0);
      for(auto const& a: n)
        ss << " "<<++i<<"." << a->n << "@" << a->Depth();
      return ss;
    }

    friend std::ostream& operator << (std::ostream& ss, MultiEdge const& n){
      int i(0);
      for(auto const& a: n)
        ss << " "<<(++i)<<"." << a.second->n << "@" << a.second->Depth();
      ss << std::endl;
      /*
         for(auto const& m:n.successors)
         ss << "----"<<m;
         */
      //n.Print(ss,0);
      return ss;
    }

    friend std::ostream& operator << (std::ostream& ss, Node const& n){
      ss << n.n << "@" << n.Depth();
      return ss;
    }

    friend std::ostream& operator << (std::ostream& ss, Node const* n){
      n->Print(ss);
      //ss << std::string(n->Depth(),' ')<<n->n << "_" << n->Depth() << std::endl;
      //for(auto const& m: n->successors)
      //ss << m;
      return ss;
    }
};


template<typename state, typename action>
uint64_t ICTSAlgorithm<state,action>::Node::count(0);
template<typename state, typename action>
uint64_t ICTSAlgorithm<state,action>::ICTSNode::count(0);

#endif
