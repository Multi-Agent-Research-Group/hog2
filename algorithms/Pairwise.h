// Sweep and Prune algorithm

#ifndef Pairwise_H_
#define Pairwise_H_
#include <vector>
#include "BroadPhase.h"

template<typename BB>
class Pairwise : public BroadPhase<BB>{
  public:
    Pairwise(unsigned n):BroadPhase<BB>(n){}

    Pairwise(std::vector<std::vector<BB>*>* p, unsigned m):BroadPhase<BB>(p),paths(p),movelen(m){
    }

    virtual void insert(BB const* v){
      ppaths.push_back(v);
    }

    virtual void update(std::vector<std::vector<BB>*>* p){paths=p;}

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
      comparisons=0;
      for(unsigned i(0); i<paths->size(); ++i){
        for(unsigned j(i+1); j<paths->size(); ++j){
          auto a(paths->at(i)->begin());
          auto b(paths->at(j)->begin());
          while(a<paths->at(i)->end() && b<paths->at(j)->end()){
            ++comparisons;
            if( a->upperBound.x>=b->lowerBound.x &&
                a->lowerBound.x<=b->upperBound.x &&
                a->lowerBound.y<=b->upperBound.y &&
                a->upperBound.y>=b->lowerBound.y){
              pairs.emplace_back(&*a,&*b);
            }
            unsigned diff(std::max((a->end.x>b->end.x ? a->end.x-b->end.x : b->end.x-a->end.x),(a->end.y>b->end.y ? a->end.y-b->end.y : b->end.y-a->end.y))/(2*movelen));
            if(diff){
              a+=diff;
              b+=diff;
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
    mutable unsigned comparisons;
    unsigned movelen;
};

#endif
