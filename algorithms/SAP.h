// Sweep and Prune algorithm

#ifndef SAP_H_
#define SAP_H_
#include <vector>
#include "BroadPhase.h"

template<typename BB>
class SAP : public BroadPhase<BB>{
  public:
    SAP(){}

    SAP(std::vector<std::vector<BB>*> const& paths){
      for(auto const& p:paths){
        for(auto const& v:*p){
          sorted.push_back(&v);
        }
      }
      std::sort(sorted.begin(),sorted.end());
    }

    virtual void insert(BB const* v){
    }

    virtual double hasConflict(BB const& v)const{
    }

    virtual double hasConflict(std::vector<BB> const& path)const{
      for(auto const& v:path){
        if(hasConflict(&v)){return true;}
      }
      return false;
    }

    virtual void replace(std::vector<BB> const& o, std::vector<BB> const& n){
    }
  
    virtual void getAllPairs(std::vector<std::pair<BB*,BB*>>& pairs)const{
    }

    virtual void getPaths(std::vector<std::vector<BB>>& paths)const{
    }

    virtual void clear(){sorted.resize(0);}

    std::vector<BB*> sorted;
};

#endif
