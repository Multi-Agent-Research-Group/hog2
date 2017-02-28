//==============================================================================
// AdmissibilityChecker
//==============================================================================

#ifndef __AdmissibilityChecker__
#define __AdmissibilityChecker__

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <limits>
#include <iostream>
#include <string.h>
#include "AStarOpenClosed.h"
#include "NN.h"


template <typename state, typename action, typename environment>
class AdmissibilityChecker
{
  private:
    struct Container{
      Container():depth(0){}
      Container(int d, state const& ss):depth(d),s(ss){}
      int depth;
      state s;
    };

    struct Comparator{
      bool operator()(const AStarOpenClosedData<Container> &ci1, const AStarOpenClosedData<Container> &ci2) const {
        return fgreater(ci1.g+ci1.h, ci2.g+ci2.h);
      }
    };

  public:
    bool check(environment& env, std::vector<state> startStates, double radius, int depthLimit){
      double gap(0.0);
      unsigned total(0);

      //std::cout << "x<-matrix(c(0,0,0,0,0),1,5);y<-c(0)\n";
      for(state const& start: startStates){
        AStarOpenClosed<Container,Comparator> q;
        env.setStart(start);
        uint64_t hash(env.GetStateHash(start));
        Container temp(0,start);
        q.AddOpenNode(temp,hash,0.0,0.0,0);
        while(q.OpenSize()){
          uint64_t nodeid(q.Close());
          double G(q.Lookup(nodeid).g);
          env.setGoal(q.Lookup(nodeid).data.s);
          Container node(q.Lookup(nodeid).data);
          double h(env.HCost(start,node.s));
          gap=std::max(gap,h);
          if(fgreater(h,G)){
            std::cout << "FAIL: HCost: " << start<<"-->"<<node.s <<"=" << h << " Greater than actual: "<< G << std::endl; 
            std::vector<state> thePath;
            do{
              thePath.push_back(q.Lookup(nodeid).data.s);
              nodeid=q.Lookup(nodeid).parentID;
            }while(q.Lookup(nodeid).parentID!=nodeid);

            state const* prev(0);
            double t(0.0);
            for(auto const& s: thePath){
              std::cout << s;
              if(prev){std::cout << " " << env.GCost(*prev,s)<<"="<<(t+=env.GCost(*prev,s));}
              std::cout << "\n";
              prev=&s;
            }
            return false;
          }
          if(node.depth > depthLimit || env.GetRadius(start,node.s)>radius) continue;
          std::vector<state> succ;
          env.GetSuccessors(node.s,succ);
          for(state s: succ){
            uint64_t hash(env.GetStateHash(s));
            uint64_t theID;
            switch(q.Lookup(hash,theID)){
              case kClosedList:
              case kOpenList:
                if(fgreater(q.Lookup(theID).g,G+env.GCost(node.s,s))){
                  q.Lookup(theID).g=G+env.GCost(node.s,s);
                }
                break;
              case kNotFound:
              default:
                Container current(node.depth+1,s);
                q.AddOpenNode(current,hash,G+env.GCost(node.s,s),0,nodeid);
                break;
            }
          }
        }
        total+=q.ClosedSize();
      }
      std::cout << "Forward admissibility check PASSED on depth/radius of " << depthLimit << "/" << radius << ", checked " << total << " states.\n Largest gap: " << gap <<std::endl;
      return true;
    }

    bool checkReverse(environment& env, std::vector<state> startStates, double radius, int depthLimit){
      double gap(0.0);
      unsigned total(0);

      //std::cout << "x<-matrix(c(0,0,0,0,0),1,5);y<-c(0)\n";
      for(state const& start: startStates){
        AStarOpenClosed<Container,Comparator> q;
        env.setGoal(start);
        uint64_t hash(env.GetStateHash(start));
        Container temp(0,start);
        q.AddOpenNode(temp,hash,0.0,0.0,0);
        while(q.OpenSize()){
          uint64_t nodeid(q.Close());
          env.setStart(q.Lookup(nodeid).data.s);
          double G(q.Lookup(nodeid).g);
          Container node(q.Lookup(nodeid).data);
          double h(env.HCost(node.s,start));
          if(fgreater(h,G)){
            std::cout << "FAIL: HCost: " << node.s<<"-->"<<start <<"=" << h << " Greater than actual: "<< G << std::endl; 
            std::vector<state> thePath;
            do{
              thePath.push_back(q.Lookup(nodeid).data.s);
              nodeid=q.Lookup(nodeid).parentID;
            }while(q.Lookup(nodeid).parentID!=nodeid);

            std::reverse(thePath.begin(),thePath.end());
            state const* prev(0);
            double t(0.0);
            for(auto const& s: thePath){
              std::cout << s;
              if(prev){std::cout << " " << env.GCost(*prev,s)<<"="<<(t+=env.GCost(*prev,s));}
              std::cout << "\n";
              prev=&s;
            }
            return false;
          }
          if(node.depth > depthLimit || env.GetRadius(node.s,start)>radius) continue;
          std::vector<state> succ;
          env.GetReverseSuccessors(node.s,succ);
          for(state s: succ){
            uint64_t hash(env.GetStateHash(s));
            uint64_t theID;
            switch(q.Lookup(hash,theID)){
              case kClosedList:
              case kOpenList:
                if(fgreater(q.Lookup(theID).g,G+env.GCost(s,node.s))){
                  q.Lookup(theID).g=G+env.GCost(s,node.s);
                }
                break;
              case kNotFound:
              default:
                Container current(node.depth+1,s);
                q.AddOpenNode(current,hash,G+env.GCost(s,node.s),0,nodeid);
                break;
            }
          }
        }
        total+=q.ClosedSize();
      }
      std::cout << "Reverse admissibility check PASSED on depth/radius of " << depthLimit << "/" << radius << ", checked " << total << " states.\n Largest gap: " << gap <<std::endl;
      return true;
    }

  private:

    NN* model=0;
};

# endif
