// Sweep and Prune algorithm

#ifndef SAP_H_
#define SAP_H_
#include <vector>
#include "BroadPhase.h"


template<typename BB>
class SAP : public BroadPhase<BB>{
  struct Compare{
    inline bool operator()(BB const* a, BB const* b){return *a<*b;}
  };
  public:
    SAP(unsigned n):BroadPhase<BB>(n){}

    SAP(std::vector<std::vector<BB>*> const* paths):BroadPhase<BB>(paths){
      for(auto const& p:*paths){
        for(auto const& v:*p){
          sorted.push_back(&v);
        }
      }
      std::sort(sorted.begin(),sorted.end(),Compare());
    }

    virtual void insert(BB const* v){
      // Use interpolation search to insert...
      // Making the dangerous assumption that the range starts at zero!
      unsigned maxval(sorted.back()->start.t);
      if(maxval){
        unsigned i(sorted.size()*(v->start.t/maxval));
        auto loc(sorted.begin()+i);
        if((*v)<(**loc)){
          auto upper(loc);
          do{upper+=this->k;}while(upper<sorted.end()&&(*v)<(**upper));
          if(upper>sorted.end())upper=sorted.end();
          loc=std::lower_bound(loc,upper,v,Compare());
        }else if((**loc)<(*v)){
          auto lower(loc);
          do{lower-=this->k;}while(lower>sorted.begin()&&(**lower)<(*v));
          if(lower<sorted.begin())lower=sorted.begin();
          loc=std::lower_bound(lower,loc,v,Compare());
        }
        sorted.insert(loc,v);
      }else{
        // Binary search..
        auto loc(std::lower_bound(sorted.begin(),sorted.end(),v,Compare()));
        sorted.insert(loc,v);
      }
    }

    virtual void getConflicts(BB const* v, std::vector<BB const*>& conflicting)const{
      // Use interpolation to find range of BBs to compare with
      // Making the dangerous assumption that the range starts at zero!
      unsigned maxval(sorted.back()->start.t);
      if(maxval){
        unsigned i(sorted.size()*(v->start.t/maxval));
        auto loc(sorted.begin()+i);
        if((*v)<(**loc)){
          auto upper(loc);
          do{upper+=this->k;}while(upper<sorted.end()&&(*v)<(**upper));
          if(upper>sorted.end())upper=sorted.end();
          loc=std::lower_bound(loc,upper,v,Compare());
        }else if((**loc)<(*v)){
          auto lower(loc);
          do{lower-=this->k;}while(lower>sorted.begin()&&(**lower)<(*v));
          if(lower<sorted.begin())lower=sorted.begin();
          loc=std::lower_bound(lower,loc,v,Compare());
        }
        while(loc!=sorted.end() && (*loc)->upperBound.t>=v->upperBound.t){
          if(v->agent!=(*loc)->agent &&
              v->upperBound.x>=(*loc)->lowerBound.x &&
              v->lowerBound.x<=(*loc)->upperBound.x &&
              v->lowerBound.y<=(*loc)->upperBound.y &&
              v->upperBound.y>=(*loc)->lowerBound.y){
            conflicting.push_back(*loc);
          }
          ++loc;
        }
      }else{
        // Binary search..
        auto loc(std::lower_bound(sorted.begin(),sorted.end(),v,Compare()));
        while(loc!=sorted.end() && (*loc)->upperBound.t>=v->upperBound.t){
          if(v->agent!=(*loc)->agent &&
              v->upperBound.x>=(*loc)->lowerBound.x &&
              v->lowerBound.x<=(*loc)->upperBound.x &&
              v->lowerBound.y<=(*loc)->upperBound.y &&
              v->upperBound.y>=(*loc)->lowerBound.y){
            conflicting.push_back(*loc);
          }
          ++loc;
        }
      }
    }

    virtual void replace(std::vector<BB>* o, std::vector<BB>* n){
      auto oi(o->begin()); // New
      auto ni(n->begin()); // New
      auto beforen(sorted.begin());
      auto ato(std::lower_bound(sorted.begin(),sorted.end(),&*oi,Compare())); // Old
      while(oi!=o->end()&&ni!=n->end()){
        if(*oi<*ni){
          beforen=std::lower_bound(ato,sorted.end(),&*ni,Compare());
        }else if(*ni<*oi){
          beforen=std::lower_bound(beforen,ato,&*ni,Compare());
        }else{
          beforen=ato;
        }
        if(ato<beforen){
          std::rotate(ato,ato+1,beforen); // Move element o to the new index
          --beforen;
        }
        else if(ato>beforen)
          std::rotate(beforen,ato,ato+1); // Move element o to the new index
        *beforen=&*ni; // replace value of o with n
        ++ni;
        if(++oi==o->end()){break;}
        ato=std::lower_bound(ato,sorted.end(),&*oi,Compare()); // Old
      }
      while(oi!=o->end()){
        std::remove(ato++,sorted.end(),&*oi);
        ++oi;
      }
      while(ni!=n->end()){
        sorted.insert(std::lower_bound(ato++,sorted.end(),&*ni,Compare()),&*ni);
        ++ni;
      }
    }
  
    virtual void getAllPairs(std::vector<std::pair<BB const*,BB const*>>& pairs)const{
      auto a(sorted.cbegin());
      auto b(sorted.cbegin()+1);
      while(a!=sorted.end()){
        while(b!=sorted.end() && (*a)->upperBound.t>=(*b)->lowerBound.t){
          if((*a)->agent!=(*b)->agent &&
              (*a)->upperBound.x>=(*b)->lowerBound.x &&
              (*a)->lowerBound.x<=(*b)->upperBound.x &&
              (*a)->lowerBound.y<=(*b)->upperBound.y &&
              (*a)->upperBound.y>=(*b)->lowerBound.y){
            pairs.emplace_back(*a,*b);
          }
          ++b;
        }
        ++a;
        b=a+1;
      }
    }

    virtual void clear(){sorted.resize(0);}

    std::vector<BB const*> sorted;
};

#endif
