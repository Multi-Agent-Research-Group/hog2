//==============================================================================
// AirplanePerimeterBuilder
//==============================================================================

#ifndef __AirplanePerimeterBuilder__
#define __AirplanePerimeterBuilder__

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


template <typename state, typename action, typename environment, unsigned dims>
class AirplanePerimeterBuilder
{
  private:
    std::vector<int> sizes;
    uint64_t num_bits=0;
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
    void learnSpace(environment const& env, std::vector<state> startStates, int depthLimit){
      std::vector<double> data(dims);
      std::vector<double> target(1);
      model = new NN(dims,dims+30,1,.01);
      AStarOpenClosed<Container,Comparator> q;
      q.Reset();
      //std::cout << "x<-matrix(c(0,0,0,0,0),1,5);y<-c(0)\n";
      for(state const& start: startStates){
        env.GetDimensions(start,start,data);
        uint64_t hash(getHash(data,env.GetRanges()));
        Container temp(0,start);
        q.AddOpenNode(temp,hash,0.0,0.0,0);
        while(q.OpenSize()){
          uint64_t nodeid(q.Close());
          double G(q.Lookup(nodeid).g);
          Container node(q.Lookup(nodeid).data);
          if(node.depth > depthLimit) continue;
          //env.GetDimensions(node.s,start,data);
          /*char sep(' ');
          std::cout << "x<-rbind(x,c(";
          for(double d: data){
            std::cout << sep << d;
            sep=',';
          }
          std::cout << ")); y<-c(y," << G << ")\n";*/
          //target[0]=G;
          //model->train(data,target);
          std::vector<state> succ;
          env.GetReverseSuccessors(node.s,succ);
          for(state s: succ){
            uint64_t theID;
            uint64_t hash(env.GetStateHash(s));
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
        std::vector<int> myvector(q.size());
        for(int i(0);i<q.size();++i)
          myvector[i]=i;

        std::random_shuffle ( myvector.begin(), myvector.end() );

        for(int i: myvector){
          AStarOpenClosedData<Container> temp(q.Lookup(i));
          if(temp.where == kClosedList){
            target[0]=temp.g;
            env.GetDimensions(temp.data.s,start,data);
            model->train(data,target);
          }
        }
      }
      //model->save(env.name());
    }

    void loadDB(environment const& env, std::vector<state> startStates, int depthLimit, char const*const filename, bool replace=false){
      loadDB(env,startStates,depthLimit,true,filename,replace);
      loadDB(env,startStates,depthLimit,false,filename,replace);
    }

    void loadDB(environment const& env, std::vector<state> startStates, int depthLimit, bool forward, char const*const filename, bool replace=false){
      std::vector<double> data(dims);
      std::vector<double> target(1);

      AStarOpenClosed<Container,Comparator>& q(forward?fwd:rev);

      if(!replace){
        q.LoadFromFile(std::string(filename).append(forward?".forward":".reverse").c_str());
        if(q.size() > 0) return;
      }
      //std::cout << "x<-matrix(c(0,0,0,0,0),1,5);y<-c(0)\n";
      for(state const& start: startStates){
        env.GetDimensions(start,start,data);
        uint64_t hash(getHash(data,env.GetRanges()));
        Container temp(0,start);
        q.AddOpenNode(temp,hash,0.0,0.0,0);
        while(q.OpenSize()){
          uint64_t nodeid(q.Close());
          double G(q.Lookup(nodeid).g);
          Container node(q.Lookup(nodeid).data);
          if(node.depth > depthLimit) continue;
          std::vector<state> succ;
          if(forward)
            env.GetSuccessors(node.s,succ);
          else
            env.GetReverseSuccessors(node.s,succ);
          for(state s: succ){
            uint64_t theID;
            if(forward)
              env.GetDimensions(start,s,data);
            else
              env.GetDimensions(s,start,data);
            uint64_t hash(getHash(data,env.GetRanges()));
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
      }
      q.SaveToFile(std::string(filename).append(forward?".forward":".reverse").c_str());
    }

    // Input: dimensions: a vector of relative state values
    //        ranges:     a vector of range data; max value and quantization per unit (10,10 is max of 10, with ten increments between units, i.e. 0-10 by 0.1)
    uint64_t getHash(std::vector<double> const& dimensions, std::vector<std::pair<int,int>> const& ranges,bool relative=true)
    {
      if(num_bits==0){
        for(auto const& r:ranges){
          int bits(ceil(log2(r.first*r.second*(relative?2.0:1.0))));
          num_bits+=bits;
          sizes.push_back(bits);
        }
      }
      uint64_t hash(0);
      if(num_bits<=64){
        for(int i(0); i<dims; ++i){
          if(i>0){ hash = hash << sizes[i]; }
          hash |= unsigned((dimensions[i]+(relative?ranges[i].first:0))*ranges[i].second) & unsigned(pow(2,sizes[i])-1);
        }
      }
      return hash;
    }

    void fromHash(std::vector<double>& dimensions, uint64_t hash, std::vector<std::pair<int,int>> const& ranges, bool relative=true)
    {
      if(num_bits==0){
        for(auto const& r:ranges){
          int bits(ceil(log2(r.first*r.second*(relative?2.0:1.0))));
          num_bits+=bits;
          sizes.push_back(bits);
        }
      }
      if(num_bits<=64){
        for(int i(dims-1); i>-1; --i){
          if(i!=dims-1){ hash=hash >> sizes[i+1]; }
          dimensions[i]=double(hash & unsigned(pow(2,sizes[i])-1)) / ranges[i].second - (relative?ranges[i].first:0);
        }
      }
    }
    AirplanePerimeterBuilder() : model(0){}

    double GCost(state const& s, state const& g, environment const& env){
      double result(DBL_MAX);
      std::vector<double> data;
      env.GetDimensions(s,g,data);
      uint64_t hash(getHash(data,env.GetRanges()));
      uint64_t theID;
      dataLocation loc = rev.Lookup(hash,theID);
      if(loc==kNotFound){
        dataLocation loc = fwd.Lookup(hash,theID);
        if(loc==kNotFound){
          return DBL_MAX;
        }else{
          return fwd.Lookat(theID).g;
        }
      }else{
        return rev.Lookat(theID).g;
      }
    }

    // Regression-based G-Cost
    double GCostR(state const& s, state const& g, environment const& env){
      std::vector<double> input;
      env.GetDimensions(s,g,input);
      if(!model){model=new NN(env.name());}
      return *(model->test(input));
    }

  private:

    NN* model=0;
    AStarOpenClosed<Container,Comparator> fwd;
    AStarOpenClosed<Container,Comparator> rev;
};

# endif
