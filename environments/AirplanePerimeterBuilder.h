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


template <typename state, typename action, typename environment, unsigned xySize=3, unsigned zSize=3, unsigned numHeadings=8, unsigned numSpeeds=5>
class AirplanePerimeterBuilder
{
  private:
    struct Container{
      Container(int d,state ss):depth(d),s(ss){}
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
      std::vector<double> data;
      env.GetDimensions(startStates[0],startStates[0],data);
      std::vector<double> target(1);
      model = new NN(data.size(),data.size()+30,1,.01);
      //std::cout << "x<-matrix(c(0,0,0,0,0),1,5);y<-c(0)\n";
      for(state const& start: startStates){
        AStarOpenClosed<Container,Comparator> q;
        Container temp(0,start);
        q.AddOpenNode(temp,env.GetStateHash(start),0.0,0.0,0);
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
                break;
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
      model->save(env.name());
    }

    AirplanePerimeterBuilder() : model(0){}

    double GCost(state const& s, state const& g, environment const& env){
      std::vector<double> input;
      env.GetDimensions(s,g,input);
      if(!model){model=new NN(env.name());}
      return *(model->test(input));
    }

  private:

    NN* model=0;
};

# endif
