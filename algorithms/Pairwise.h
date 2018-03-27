// Sweep and Prune algorithm

#ifndef Pairwise_H_
#define Pairwise_H_
#include <vector>
#include "BroadPhase.h"

template<typename BB>
class Pairwise : public BroadPhase<BB>{
  public:
    Pairwise(unsigned n):BroadPhase<BB>(n){}

    Pairwise(std::vector<std::vector<BB>*>* p):BroadPhase<BB>(p),paths(p){}

    virtual void insert(BB const* v){
      ppaths.push_back(v);
    }

    virtual void getConflicts(BB const* v, std::vector<BB const*>& conflicting)const{
      for(auto const& p:ppaths){
        if(v->start.t>p->end.t || v->end.t<p->start.t || v->end.t==v->start.t || v->end.t == v->start.t){}else{
          conflicting.push_back(p);
        }
      }
    }

    virtual void replace(std::vector<BB>* o, std::vector<BB>* n){
      //paths->at(n->at(0).agent)=n;
    }
  
    virtual void getAllPairs(std::vector<std::pair<BB const*,BB const*>>& pairs)const{
      for(unsigned i(0); i<paths->size(); ++i){
        for(unsigned j(i+1); j<paths->size(); ++j){
          auto a(paths->at(i)->begin());
          auto b(paths->at(j)->begin());
          while(a!=paths->at(i)->end() && b!=paths->at(j)->end()){
            if(a->agent!=b->agent &&
                a->upperBound.x>=b->lowerBound.x &&
                a->lowerBound.x<=b->upperBound.x &&
                a->lowerBound.y<=b->upperBound.y &&
                a->upperBound.y>=b->lowerBound.y){
              pairs.emplace_back(&*a,&*b);
            }
            if(a->start.t<b->start.t){
              ++a;
            }else if(a->start.t>b->start.t){
              ++b;
            }else if(a->end.t>b->end.t){
              ++b;
            }else if(a->end.t<b->end.t){
              ++a;
            }else{
              ++a;++b;
            }
          }
        }
      }
    }

    virtual void clear(){ppaths.resize(0);}

    std::vector<BB const*> ppaths;
    std::vector<std::vector<BB>*>* paths;
};

#endif
