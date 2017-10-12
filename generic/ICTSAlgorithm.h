/*
 *  ICTSAlgorithm.h
 *  hog
 *
 *  Created by Thayne Walker on 9/27/17.
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

#include "Common.h"
#include <memory>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <functional>
#include "VelocityObstacle.h"
#include "TemplateAStar.h"
#include "Heuristic.h"
#include "MultiAgentStructures.h"

extern double agentRadius;

#define INF 9999999.0f

template<typename T, typename C>
class custom_priority_queue : public std::priority_queue<T, std::vector<T>, C>
{
  public:

    /*bool remove(const T& value) {
     *       toDelete.insert(value->key());
     *           }
     *               bool removeAll(int agent, int count){
     *                     for(auto const& value:this->c){
     *                             if(value->sizes[agent]<count){
     *                                       toDelete.insert(value->key());
     *                                               }
     *                                                     }
     *                                                           return true;
     *                                                               }*/
    virtual T popTop(){
      T val(this->top());
      this->pop();
      /*while(toDelete.find(val->key()) != toDelete.end()){
       *         val=this->top();
       *                 this->pop();
       *                       }*/
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
    ICTSAlgorithm():jointTime(0),pairwiseTime(0),mddTime(0),nogoodTime(0),epp(false),verbose(false),quiet(false),verify(false),jointnodes(0),step(1.0),suboptimal(false),pairwise(true){}
    float jointTime;
    float pairwiseTime;
    float mddTime;
    float nogoodTime;

    bool epp;
    bool verbose;
    bool quiet;
    bool verify;
    bool epsilon;
    bool suboptimal;
    bool pairwise;
    uint64_t jointnodes;
    float step;
    //int n;
    std::unordered_map<std::string,bool> transTable;

    std::vector<SearchEnvironment<state,action>*> envs;
    std::vector<Heuristic<state>*> heuristics;

    // for agents to stay at goal
#define MAXTIME 300
    // for inflation of floats to avoid rounding errors
#define INFLATION 1000.0f

    // Used for std::set
    struct NodePtrComp
    {
      bool operator()(const Hashable* lhs, const Hashable* rhs) const  { return fless(lhs->Depth(),rhs->Depth()); }
    };

    struct Node : public Hashable{
      static uint64_t count;

      Node(){}
      Node(state a, float d, uint64_t h):n(a),depth(d),hash(h),optimal(false),unified(false),nogood(false){count++;}
      virtual ~Node(){}
      state n;
      float depth;
      uint64_t hash;
      bool optimal;
      bool unified;
      bool nogood;
      //bool connected()const{return parents.size()+successors.size();}
      //std::unordered_set<Node*> parents;
      std::unordered_set<Node*> successors;
      virtual uint64_t Hash()const{return hash;}//(env->GetStateHash(n)<<32) | ((uint32_t)(depth*INFLATION));}
      virtual float Depth()const{return depth; }
      virtual void Print(std::ostream& ss, int d=0) const {
        ss << std::string(d,' ')<<n << "_" << depth << std::endl;
        for(auto const& m: successors)
          m->Print(ss,d+1);
      }
      bool operator==(Node const& other)const{return n.sameLoc(other.n)&&fequal(depth,other.depth);}
    };

    typedef std::pair<MultiAgentState<state>,MultiAgentState<state>> Instance;
    typedef std::vector<Node*> Path;
    typedef std::vector<std::vector<Node*>> MDDSolution;

    typedef std::vector<Node*> MultiState; // rank=agent num
    typedef std::vector<std::pair<Node*,Node*>> MultiEdge; // rank=agent num
    typedef std::unordered_map<uint64_t,Node> DAG;
    std::unordered_map<uint64_t,Node*> mddcache;
    std::unordered_map<uint64_t,float> lbcache;

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
      ss << " "<<++i<<"." << a.second->n << "@" << a.second->depth;
      ss << std::endl;
      for(auto const& m: successors)
      m.Print(ss,d+1);
      }
      };*/

    // Compute path cost, ignoring actions that wait at the goal
    static float computeSolutionCost(MDDSolution const& solution, bool ignoreWaitAtGoal=true){
      float cost(0);
      if(ignoreWaitAtGoal){
        for(auto const& path:solution){
          for(int j(path.size()-1); j>0; --j){
            if(path[j-1]->n!=path[j]->n){
              cost += path[j]->depth;
              break;
            }else if(j==1){
              cost += path[0]->depth;
            }
          }
        }
      }else{
        for(auto const& path:solution){
          cost+=path.back()->depth;
        }
      }
      return cost;
    }

    // Big caveat - this implementation assumes that time is NOT a component of the state hash AND that it is <=32 bits in length.
    uint64_t GetHash(state const& n, uint32_t depth, unsigned agent){
      return (envs[agent]->GetStateHash(n)<<32) | (depth*uint32_t(INFLATION));}

    bool LimitedDFS(state const& start, state const& end, DAG& dag, Node*& root, int depth, int maxDepth, float& best, SearchEnvironment<state,action>* env, unsigned agent){
      if(depth<0 || maxDepth-depth+(int)(HCost(start,end,agent)*INFLATION)>maxDepth){ // Note - this only works for a perfect heuristic.
        //std::cout << " pruned " << depth <<" "<< (maxDepth-depth+(int)(env->HCost(start,end)*INFLATION))<<">"<<maxDepth<<"\n";
        return false;
      }
      //std::cout << "\n";

      if(env->GoalTest(start,end)){
        Node n(start,(maxDepth-depth)/INFLATION,GetHash(start,(maxDepth-depth)/INFLATION,agent));
        uint64_t hash(n.Hash());
        dag[hash]=n;
        // This may happen if the agent starts at the goal
        if(maxDepth-depth<=0){
          root=&dag[hash];
          //std::cout << "root_ " << &dag[hash];
        }
        Node* parent(&dag[hash]);
        int d(maxDepth-depth);
        while(d+INFLATION<=maxDepth){ // Increment depth by 1 for wait actions
          // Wait at goal
          d+=INFLATION;
          Node current(start,d/INFLATION,GetHash(start,d/INFLATION,agent));
          uint64_t chash(current.Hash());
          dag[chash]=current;
          if(verbose)std::cout << "inserting " << dag[chash] << " " << &dag[chash] << "under " << *parent << "\n";
          parent->successors.insert(&dag[chash]);
          //dag[chash].parents.insert(parent);
          parent=&dag[chash];
        }
        best=std::min(best,parent->depth);
        //std::cout << "found d\n";
        if(verbose)std::cout << "ABEST "<<best<<"\n";
        return true;
      }

      MultiAgentState<state> successors;
      env->GetSuccessors(start,successors);
      bool result(false);
      for(auto const& node: successors){
        int ddiff(std::max(Util::distance(node,start),1.0)*INFLATION);
        //std::cout << std::string(std::max(0,(maxDepth-(depth-ddiff)))/INFLATION,' ') << "MDDEVAL " << start << "-->" << node << "\n";
        //if(abs(node.x-start.x)>=1 && abs(node.y-start.y)>=1){
        //ddiff = M_SQRT2;
        //}
        if(LimitedDFS(node,end,dag,root,depth-ddiff,maxDepth,best,env,agent)){
          Node n(start,(maxDepth-depth)/INFLATION,GetHash(start,(maxDepth-depth)/INFLATION,agent));
          uint64_t hash(n.Hash());
          if(dag.find(hash)==dag.end()){
            dag[hash]=n;
            // This is the root if depth=0
            if(maxDepth-depth<=0){
              root=&dag[hash];
              if(verbose)std::cout << "Set root to: " << (uint64_t)root << "\n";
              //std::cout << "_root " << &dag[hash];
            }
            //if(fequal(maxDepth-depth,0.0))root.push_back(&dag[hash]);
          }else if(dag[hash].optimal){
            return true; // Already found a solution from search at this depth
          }

          Node* parent(&dag[hash]);

          //std::cout << "found " << start << "\n";
          uint64_t chash(GetHash(node,(maxDepth-depth+ddiff)/INFLATION,agent));
          if(dag.find(chash)==dag.end()&&dag.find(chash+1)==dag.end()&&dag.find(chash-1)==dag.end()){
            std::cout << "Expected " << Node(node,maxDepth-depth+ddiff,GetHash(node,(maxDepth-depth+ddiff)/INFLATION,agent)) << " " << chash << " to be in the dag\n";
            assert(!"Uh oh, node not already in the DAG!");
            //std::cout << "Add new.\n";
            Node c(node,(maxDepth-depth+ddiff)/INFLATION,chash);
            dag[chash]=c;
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
      return result;
    }

    // Perform conflict check by moving forward in time at increments of the smallest time step
    // Test the efficiency of VO vs. time-vector approach
    void GetMDD(unsigned agent,state const& start, state const& end, DAG& dag, MultiState& root, int depth, float& best, SearchEnvironment<state,action>* env){
      if(verbose)std::cout << "MDD up to depth: " << depth << start << "-->" << end << "\n";
      uint64_t hash(((uint32_t) depth)<<8|agent);
      bool found(mddcache.find(hash)!=mddcache.end());
      if(verbose)std::cout << "lookup "<< (found?"found":"missed") << "\n";
      if(!found){
        LimitedDFS(start,end,dag,root[agent],depth,depth,best,envs[agent],agent);
        mddcache[hash]=root[agent];
        lbcache[hash]=best;
      }else{
        root[agent]=mddcache[hash];
        best=lbcache[hash];
      }
      if(verbose)std::cout << "Finally set root to: " << (uint64_t)root[agent] << "\n";
      if(verbose)std::cout << root[agent] << "\n";
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
          if(collisionCheck(positions[agent][i].first->n,positions[agent][i].second->n,current[j].first->n,current[j].second->n,agentRadius)){
          // Make sure we don't do any checks that were already done
          //if(fequal(positions[agent][i].first->depth,lastTime)&&fequal(current[j].first->depth,lastTime))continue;
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
    bool jointDFS(MultiEdge const& s, float d, MDDSolution solution, std::vector<MDDSolution>& solutions, std::vector<Node*>& toDelete, float& best, float bestSeen, bool suboptimal=false, bool checkOnly=false){
      // Compute hash for transposition table
      std::string hash(s.size()*sizeof(uint64_t),1);
      int i(0);
      for(auto v:s){
        uint64_t h1(v.second->Hash());
        uint8_t c[sizeof(uint64_t)];
        memcpy(c,&h1,sizeof(uint64_t));
        for(unsigned j(0); j<sizeof(uint64_t); ++j){
          hash[i*sizeof(uint64_t)+j]=((int)c[j])?c[j]:1; // Replace null-terminators in the middle of the string
        }
        ++i;
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
          if(found || (s[i].second->n.sameLoc(p.back()->n) && fgreater(s[i].second->depth,p.back()->depth))){
            solution[i].push_back(s[i].second);
          }
        }
      }

      bool done(true);
      for(auto const& g:s){
        if(!fequal(g.second->depth,MAXTIME)){
          done=false;
          break;
        }
        //maxCost=std::max(maxCost,g.second->depth);
      }
      if(done){
        float cost(computeSolutionCost(solution));
        if(fless(cost,best)){
          best=cost;
          if(verbose)std::cout << "BEST="<<best<<std::endl;
          if(verbose)std::cout << "BS="<<bestSeen<<std::endl;

          // This is a leaf node
          // Copy the solution into the answer set
          if(!checkOnly){
            // Shore up with wait actions
            for(int i(0); i<solution.size(); ++i){
              if(fless(solution[i].back()->depth,MAXTIME)){
                solution[i].emplace_back(new Node(solution[i].back()->n,MAXTIME,GetHash(solution[i].back()->n,MAXTIME,i)));
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

      // Find minimum depth of current edges
      float sd(INF);
      for(auto const& a: s){
        sd=min(sd,a.second->depth);
      }
      //std::cout << "min-depth: " << sd << "\n";

      float md(INF); // Min depth of successors
      //Add in successors for parents who are equal to the min
      for(auto const& a: s){
        if(epp){
          if(a.second->nogood){
            if(verbose)std::cout << *a.second << " is no good.\n";
            return false; // If any  of these are "no good" just exit now
          }else{
            // The fact that we are evaluating this node means that all components were unified with something
            if(checkOnly)a.second->unified=true;
          }
        }
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
          output.emplace_back(a.second,new Node(a.second->n,MAXTIME,GetHash(a.second->n,MAXTIME,successors.size())));
          //if(verbose)std::cout << "Wait " << *output.back().second << "\n";
          toDelete.push_back(output.back().second);
          //md=min(md,a.second->depth+1.0); // Amount of time to wait
        }
        //std::cout << "successor  of " << s << "gets("<<*a<< "): " << output << "\n";
        successors.push_back(output);
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
        if(jointDFS(a,md,solution,solutions,toDelete,best,bestSeen,suboptimal,checkOnly)){
          value=true;
          transTable[hash]=value;
          // Return first solution... (unless this is a pairwise check with pruning)
          if(suboptimal&&!(epp&&checkOnly)) return true;
          // Return if solution is as good as any MDD
          if(!(epp&&checkOnly)&&fequal(best,bestSeen))return true;
        }
      }
      transTable[hash]=value;
      return value;
    }

    bool jointDFS(MultiState const& s, std::vector<MDDSolution>& solutions, std::vector<Node*>& toDelete, float bestSeen, bool suboptimal=false, bool checkOnly=false){
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
      float best(INF);

      transTable.clear();
      return jointDFS(act,0.0,solution,solutions,toDelete,best,bestSeen,suboptimal,checkOnly);
    }

    // Check that two paths have no collisions
    static bool checkPair(Path const& p1, Path const& p2, unsigned agent, bool loud=false){
      auto ap(p1.begin());
      auto a(ap+1);
      auto bp(p2.begin());
      auto b(bp+1);
      while(a!=p1.end() && b!=p2.end()){
        if(collisionCheck((*ap)->n,(*a)->n,(*bp)->n,(*b)->n,agentRadius)){
          if(loud)std::cout << "Collision: " << **ap << "-->" << **a << "," << **bp << "-->" << **b;
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
      return true;
    }

    // Not part of the algorithm... just for validating the answers
    bool checkAnswer(MDDSolution const& answer){
      for(int i(0);i<answer.size();++i){
        for(int j(i+1);j<answer.size();++j){
          if(!checkPair(answer[i],answer[j],verify)){
            if(verify)std::cout<< "for agents: " << i << " and " << j << "\n";
            return false;
          }
        }
      }
      return true;
    }

    static void join(std::stringstream& s, std::vector<float> const& x){
      copy(x.begin(),x.end(), std::ostream_iterator<float>(s,","));
    }

    float HCost(state const& a, state const& b, unsigned agent)const{
      return heuristics[agent]?heuristics[agent]->HCost(a,b):envs[agent]->HCost(a,b);
    }

    struct ICTSNode{
      ICTSNode(ICTSAlgorithm* alg, ICTSNode* parent,int agent, float size):ictsalg(alg),instance(parent->instance),dag(parent->dag),best(parent->best),bestSeen(0),sizes(parent->sizes),root(parent->root){
        count++;
        sizes[agent]=size;
        best[agent]=INF;
        if(alg->verbose)std::cout << "replan agent " << agent << " GetMDD("<<(alg->HCost(instance.first[agent],instance.second[agent],agent)+sizes[agent])<<")\n";
        dag[agent].clear();
        replanned.push_back(agent);
        Timer timer;
        timer.StartTimer();
        ictsalg->GetMDD(agent,instance.first[agent],instance.second[agent],dag[agent],root,(int)(alg->HCost(instance.first[agent],instance.second[agent],agent)*INFLATION)+(int)(sizes[agent]*INFLATION),best[agent],alg->envs[agent]);
        ictsalg->mddTime+=timer.EndTimer();
        bestSeen=std::accumulate(best.begin(),best.end(),0.0f);
        // Replace new root node on top of old.
        //std::swap(root[agent],root[root.size()-1]);
        //root.resize(root.size()-1);
        //if(verbose)std::cout << agent << ":\n" << root[agent] << "\n";
      }

      ICTSNode(ICTSAlgorithm* alg, Instance const& inst, std::vector<float> const& s):ictsalg(alg),instance(inst),dag(s.size()),best(s.size()),bestSeen(0),sizes(s),root(s.size()){
        count++;
        root.reserve(s.size());
        replanned.resize(s.size());
        for(int i(0); i<instance.first.size(); ++i){
          best[i]=INF;
          replanned[i]=i;
          if(alg->verbose)std::cout << "plan agent " << i << " GetMDD("<<(alg->HCost(instance.first[i],instance.second[i],i)+sizes[i])<<")\n";
          //std::cout.precision(17);
          std::cout.precision(6);

          Timer timer;
          timer.StartTimer();
          ictsalg->GetMDD(i,instance.first[i],instance.second[i],dag[i],root,(int)(alg->HCost(instance.first[i],instance.second[i],i)*INFLATION)+(int)(sizes[i]*INFLATION),best[i],alg->envs[i]);
          ictsalg->mddTime+=timer.EndTimer();
          bestSeen=std::accumulate(best.begin(),best.end(),0.0f);
          //if(verbose)std::cout << i << ":\n" << root[i] << "\n";
        }
      }

      virtual ~ICTSNode(){for(auto d:toDelete){delete d;}}

      // Get unique identifier for this node
      std::string key()const{
        std::stringstream sv;
        join(sv,sizes);
        return sv.str();
      }

      ICTSAlgorithm* ictsalg;
      Instance instance;
      std::vector<DAG> dag;
      std::vector<float> sizes;
      std::vector<float> best;
      float bestSeen;
      MultiState root;
      Instance points;
      std::vector<Node*> toDelete;
      static uint64_t count;
      std::vector<int> replanned; // Set of nodes that was just re-planned

      // Set all nodes that were never unified as "no good"
      void resetNoGoods(Node* root){
        if(!ictsalg->epp)return;
        root->nogood=false;
        root->unified=false;
        for(auto& a:root->successors)
          resetNoGoods(a);
      }
      void resetUnified(Node* root){
        root->unified=false;
        for(auto& a:root->successors)
          resetUnified(a);
      }
      void setNoGoods(Node* root){
        if(root->nogood)return;
        root->nogood=!root->unified;
        for(auto& a:root->successors)
          setNoGoods(a);
      }
      void updateNoGoods(Node* root){
        if(!ictsalg->epp)return;
        Timer timer;
        timer.StartTimer();
        setNoGoods(root); // Set them all first
        resetUnified(root); // Reset the unified flag
        ictsalg->nogoodTime+=timer.EndTimer();
      }
      bool isValid(std::vector<MDDSolution>& answers){
        if(root.size()>2 && ictsalg->pairwise){
          Timer timer;
          timer.StartTimer();
          // Perform pairwise check
          if(ictsalg->verbose)std::cout<<"Pairwise checks\n";
          if(replanned.size()>1){
            for(int i(0); i<root.size(); ++i){
              for(int j(i+1); j<root.size(); ++j){
                MultiState tmproot(2);
                tmproot[0]=root[i];
                tmproot[1]=root[j];
                std::vector<Node*> toDeleteTmp;
                // This is a satisficing search, thus we only need do a sub-optimal check
                if(ictsalg->verbose)std::cout<<"pairwise for " << i << ","<<j<<"\n";
                if(!ictsalg->jointDFS(tmproot,answers,toDeleteTmp,INF,true,true)){
                  if(ictsalg->verbose)std::cout << "Pairwise failed\n";
                  //clearNoGoods();
                  // Reset the MDDs
                  if(ictsalg->epp){
                    Timer timer;
                    timer.StartTimer();
                    for(int i(0); i<root.size(); ++i){
                      resetNoGoods(root[i]);
                    }
                    ictsalg->nogoodTime+=timer.EndTimer();
                  }
                  return false;
                }
                updateNoGoods(root[i]);
                updateNoGoods(root[j]);
              }
            }
          }else{
            for(int i(0); i<root.size(); ++i){
              if(i==replanned[0]){continue;}
              MultiState tmproot(2);
              tmproot[0]=root[i];
              tmproot[1]=root[replanned[0]];
              std::vector<Node*> toDeleteTmp;
              // This is a satisficing search, thus we only need do a sub-optimal check
              if(ictsalg->verbose)std::cout<<"pairwise for " << i << ","<<replanned[0]<<"\n";
              if(!ictsalg->jointDFS(tmproot,answers,toDeleteTmp,INF,true,true)){
                if(ictsalg->verbose)std::cout << "Pairwise failed\n";
                //clearNoGoods();
                // Reset the MDDs
                if(ictsalg->epp){
                  Timer timer;
                  timer.StartTimer();
                  for(int i(0); i<root.size(); ++i){
                    resetNoGoods(root[i]);
                  }
                  ictsalg->nogoodTime+=timer.EndTimer();
                }
                return false;
              }
              updateNoGoods(root[i]);
              updateNoGoods(root[replanned[0]]);
            }
          }
          ictsalg->pairwiseTime+=timer.EndTimer();
        }
        // Do a depth-first search; if the search terminates at a goal, its valid.
        if(ictsalg->verbose)std::cout<<"Pairwise passed\nFull check\n";
        Timer timer;
        timer.StartTimer();
        if(ictsalg->jointDFS(root,answers,toDelete,lb(),ictsalg->suboptimal)){
          ictsalg->jointTime+=timer.EndTimer();
          if(ictsalg->verbose){
            std::cout << "Answer:\n";
            for(int num(0); num<answers.size(); ++num){
              std::cout << "number " << num << ":\n";
              for(int agent(0); agent<answers[num].size(); ++agent){
                std::cout << "  " << agent << ":\n";
                for(auto a(answers[num][agent].begin()); a!=answers[num][agent].end(); ++a){
                  std::cout  << "  " << std::string((*a)->depth,' ') << **a << "\n";
                }
                std::cout << "\n";
              }
              std::cout << "Cost: " << computeSolutionCost(answers[num]) << std::endl;
              if(ictsalg->verify&&!ictsalg->checkAnswer(answers[num]))std::cout<< "Failed in ICT node\n";
            }
          }
          //clearNoGoods();
          if(ictsalg->verbose)std::cout << "Full check passed\n";
          return true;
        }
        // Reset the MDDs
        if(ictsalg->epp){
          Timer timer;
          timer.StartTimer();
          for(int i(0); i<root.size(); ++i){
            resetNoGoods(root[i]);
          }
          ictsalg->nogoodTime=timer.EndTimer();
        }

        if(ictsalg->verbose)std::cout << "Full check failed\n";
        //clearNoGoods();
        return false;
      }

      bool operator<(ICTSNode const& other)const{
        float sic1(lb());
        float sic2(other.lb());
        if(fequal(sic1,sic2)){
          float t1(0);
          for(auto const& s:sizes){
            t1 += s;
          }
          float t2(0);
          for(auto const& s:other.sizes){
            t2 += s;
          }
          return fgreater(t1,t2);
        }else{
          return fgreater(sic1,sic2);
        }
      }
      float ub()const{
        float total(0);
        for(auto const& s:sizes){
          total += s;
        }
        return total;
      }
      float lb()const{ return bestSeen; }
    };

    struct ICTSNodePtrComp
    {
      bool operator()(const ICTSNode* lhs, const ICTSNode* rhs) const  { return *lhs<*rhs; }
    };

    void SetVerbose(bool v){verbose=v;}
    virtual unsigned GetNodesExpanded()const{return Node::count;}

    void GetSolution(std::vector<EnvironmentContainer<state,action>*> const& env, MultiAgentState<state> const& start, MultiAgentState<state> const& goal, Solution<state>& solution){
      for(int i(0);i<env.size(); ++i){
        envs.push_back(env[i]->environment);
        heuristics.push_back(env[i]->heuristic);
      }

      std::vector<float> sizes(start.size());
      custom_priority_queue<ICTSNode*,ICTSNodePtrComp> q;
      std::unordered_set<std::string> deconf;

      Instance inst(start,goal);
      q.push(new ICTSNode(this,inst,sizes));

      std::vector<std::set<Node*,NodePtrComp>> answer;
      std::vector<ICTSNode*> toDelete;
      float bestCost(INF);
      //clearNoGoods();
      while(q.size()){
        // Keep searching until we've found a candidate with greater cost than 'best'
        // To keep going on all plateaus <= the best is how we ensure optimality
        // It is impossible for a better solution to be further down in the tree - 
        // no node of cost>=best can possibly produce a child with better cost
        if(verbose)std::cout << "TOP: " << q.top()->lb() << " BEST: " << bestCost << "\n";
        if(fgeq(q.top()->lb(),bestCost)){
          break;
        }
        // This node could contain a solution since its lb is <=
        ICTSNode* parent(q.popTop());
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
        if(parent->isValid(answers)){
          for(auto const& a:answers){
            auto cost(computeSolutionCost(a));
            if(fleq(cost,bestCost)){
              bestCost=cost;
              solution.resize(0);
              for(auto const& path:a){
                std::vector<state> p;
                for(auto const& s:path){
                  p.push_back(s->n);
                }
                solution.push_back(p);
              }
            }
          }
          if(suboptimal||fgreater(q.top()->lb(),bestCost)){break;}
        }

        // Split the ICTS node
        for(int i(0); i<parent->sizes.size(); ++i){
          std::vector<float> sz(parent->sizes);
          sz[i]+=step;
          std::stringstream sv;
          join(sv,sz);
          if(deconf.find(sv.str())==deconf.end()){
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
            q.push(tmp);
            deconf.insert(sv.str());
          }
        }
        toDelete.push_back(parent);
      }

      // Clean up before exiting
      for(auto z:toDelete){
        delete z;
      }
      mddcache.clear();
      lbcache.clear();
      while(!q.empty()){
        delete q.top();
        q.pop();
      }
    }

    friend std::ostream& operator << (std::ostream& ss, MultiState const& n){
      int i(0);
      for(auto const& a: n)
        ss << " "<<++i<<"." << a->n << "@" << a->depth;
      return ss;
    }

    friend std::ostream& operator << (std::ostream& ss, MultiEdge const& n){
      int i(0);
      for(auto const& a: n)
        ss << " "<<++i<<"." << a.second->n << "@" << a.second->depth;
      ss << std::endl;
      /*
         for(auto const& m:n.successors)
         ss << "----"<<m;
         */
      //n.Print(ss,0);
      return ss;
    }

    friend std::ostream& operator << (std::ostream& ss, Node const& n){
      ss << n.n << "@" << n.depth;
      return ss;
    }

    friend std::ostream& operator << (std::ostream& ss, Node const* n){
      n->Print(ss);
      //ss << std::string(n->depth,' ')<<n->n << "_" << n->depth << std::endl;
      //for(auto const& m: n->successors)
      //ss << m;
      return ss;
    }
};


template<typename state, typename action>
uint64_t ICTSAlgorithm<state,action>::Node::count(0);
template<typename state, typename action>
uint64_t ICTSAlgorithm<state,action>::ICTSNode::count(0);

