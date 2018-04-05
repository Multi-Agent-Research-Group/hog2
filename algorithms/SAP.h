// Sweep and Prune algorithm

#ifndef SAP_H_
#define SAP_H_
#include <vector>
#include <unordered_set>
#include "BroadPhase.h"


template<typename BB>
class SAP : public BroadPhase<BB>{
  struct Node{
    Node():value(nullptr),key(nullptr),end(false){}
    Node(BB const& v, typename BB::State const& k, bool e=true):value(&v),key(&k),end(e){}
    BB const* value;
    typename BB::State const* key;
    bool end;
    bool operator<(Node const& other)const{return *key<*other.key;}
    bool operator==(Node const& other)const{return value==other.value;}
  };
  public:
    SAP(unsigned n):BroadPhase<BB>(n){}

    SAP(std::vector<std::vector<BB>*>* paths,bool presorted=true):BroadPhase<BB>(paths),comparisons(0){
      if(!presorted){
        for(auto const& p:*paths){
          for(auto const& v:*p){
            this->sorted.emplace_back(v,v.start,false);
            this->sorted.emplace_back(v,v.end);
          }
        }
        std::sort(this->sorted.begin(),this->sorted.end());
      }
    }

    virtual void insert(BB const* v){
      ins(*v,v->start,false);
      ins(*v,v->end,true);
    }

    virtual void ins(BB const& n, typename BB::State const& q, bool end){
      // Use interpolation search to insert...
      // Making the dangerous assumption that the range starts at zero!
      if(!this->sorted.size()){
        this->sorted.emplace_back(n,q,end);
        return;
      }
      unsigned maxend(this->sorted.back().key->t);
      if(maxend<q.t){
        this->sorted.emplace_back(n,q,end);
        return;
      }
      if(maxend){
        Node v(n,q,end);
        unsigned i(this->sorted.size()*(q.t/(float)maxend));
        if(i>=this->sorted.size())i=this->sorted.size()-1;
        auto loc(this->sorted.begin()+i);
        if(v<(*loc)){
          auto upper(loc);
          do{upper+=this->k;}while(upper<this->sorted.end()&&v<(*upper));
          if(upper>this->sorted.end())upper=this->sorted.end();
          loc=std::lower_bound(loc,upper,v);
        }else if((*loc)<v){
          auto lower(loc);
          do{lower-=this->k;}while(lower>this->sorted.begin()&&(*lower)<v);
          if(lower<this->sorted.begin())lower=this->sorted.begin();
          loc=std::lower_bound(lower,loc,v);
        }
        this->sorted.insert(loc,v);
      }else{
        Node v(n,q,end);
        // Binary search..
        auto loc(std::lower_bound(this->sorted.begin(),this->sorted.end(),v));
        this->sorted.insert(loc,v);
      }
    }

    virtual void getConflicts(BB const* v, std::vector<BB const*>& conflicting)const{
      // Use interpolation to find range of BBs to compare with
      // Making the dangerous assumption that the range starts at zero!
      if(!this->sorted.size())return;
      unsigned maxval(this->sorted.back().key->t);
      if(maxval<v->start.t)return; // past the end
      if(maxval){
        unsigned i(this->sorted.size()*(v->start.t/(float)maxval));
        if(i>=this->sorted.size())i=this->sorted.size()-1;
        auto loc(this->sorted.begin()+i);
        if(v->start.t<loc->key->t){
          auto upper(loc);
          do{upper+=this->k/2;}while(upper<this->sorted.end()&&v->start.t<=upper->key->t);
          if(upper>this->sorted.end())upper=this->sorted.end();
          loc=std::lower_bound(loc,upper,Node(*v,v->start));
        }else if(loc->key->t<v->start.t){
          auto lower(loc);
          do{lower-=this->k/2;}while(lower>this->sorted.begin()&&lower->key->t<=v->start.t);
          if(lower<this->sorted.begin())lower=this->sorted.begin();
          loc=std::lower_bound(lower,loc,Node(*v,v->start));
        }else{
          auto lower(loc);
          while(lower>this->sorted.begin() && loc->key->t==v->start.t){loc--;}
        }
        sweep(loc,v,conflicting);
      }else{
        // Binary search..
        auto loc(std::lower_bound(this->sorted.begin(),this->sorted.end(),Node(*v,v->start)));
        sweep(loc,v,conflicting);
      }
    }

    void sweep(typename std::vector<Node>::const_iterator loc, BB const* v, std::vector<BB const*>& conflicting)const{
      std::unordered_set<BB const*> active;
      while(loc!=this->sorted.end() && v->end.t<=loc->key->t){
        if(loc->end){
          if(active.erase(loc->value) &&
              v->upperBound.x>=loc->value->lowerBound.x &&
              v->lowerBound.x<=loc->value->upperBound.x &&
              v->lowerBound.y<=loc->value->upperBound.y &&
              v->upperBound.y>=loc->value->lowerBound.y){
            // Add to conflicts
            conflicting.push_back(loc->value);
          }
        }else{
          active.emplace(loc->value);
        }
        ++loc;
      }
      // Add any hanging active states to conflicts
      for(auto const& value:active){
        if( v->upperBound.x>=value->lowerBound.x &&
            v->lowerBound.x<=value->upperBound.x &&
            v->lowerBound.y<=value->upperBound.y &&
            v->upperBound.y>=value->lowerBound.y){
          conflicting.push_back(value);
        }
      }
    }

    virtual void replace(std::vector<BB>* o, std::vector<BB>* n){
      auto oi(o->begin()); // New
      auto ni(n->begin()); // New
      auto beforen(this->sorted.begin());
      auto beforen2(this->sorted.begin());
      auto ato(std::lower_bound(this->sorted.begin(),this->sorted.end(),Node(*oi,oi->start,false))); // Old
      auto ato2(std::lower_bound(this->sorted.begin(),this->sorted.end(),Node(*oi,oi->end))); // Old
      while(oi!=o->end()&&ni!=n->end()){
        if(*oi==*ni){++ni;++oi;continue;} // no change
        if(oi->start<ni->start){
          beforen=std::lower_bound(ato,this->sorted.end(),Node(*ni,ni->start,false));
        }else if(ni->start<oi->start){
          beforen=std::lower_bound(beforen,ato,Node(*ni,ni->start,false));
        }else{
          beforen=ato;
        }
        if(oi->end<ni->end){
          beforen2=std::lower_bound(ato,this->sorted.end(),Node(*ni,ni->end));
        }else if(ni->end<oi->end){
          beforen2=std::lower_bound(beforen,ato,Node(*ni,ni->end));
        }else{
          beforen2=ato2;
        }
        if(ato<beforen){
          std::rotate(ato,ato+1,beforen); // Move element o to the new index
          --beforen;
        }
        else if(ato>beforen)
          std::rotate(beforen,ato,ato+1); // Move element o to the new index
        beforen->value=&*ni; // replace value of o with n
        beforen->key=&(ni->start); // replace value of o with n

        if(ato2<beforen2){
          std::rotate(ato2,ato2+1,beforen2); // Move element o to the new index
          --beforen2;
        }
        else if(ato2>beforen2)
          std::rotate(beforen2,ato2,ato2+1); // Move element o to the new index
        beforen2->value=&*ni; // replace value of o with n
        beforen2->key=&(ni->end); // replace value of o with n
        ++ni;
        if(++oi==o->end()){break;}
        ato=std::lower_bound(ato,this->sorted.end(),Node(*oi,oi->start,false)); // Old
        ato2=std::lower_bound(ato2,this->sorted.end(),Node(*oi,oi->end)); // Old
      }
      while(oi!=o->end()){
        std::remove(ato++,this->sorted.end(),Node(*oi,oi->start,false));
        std::remove(ato++,this->sorted.end(),Node(*oi,oi->end));
        ++oi;
      }
      while(ni!=n->end()){
        Node s(*ni,ni->start,false);
        Node e(*ni,ni->start);
        this->sorted.insert(std::lower_bound(ato++,this->sorted.end(),s),s);
        this->sorted.insert(std::lower_bound(ato2++,this->sorted.end(),e),e);
        ++ni;
      }
    }
  
    virtual void getAllPairs(std::vector<std::pair<BB const*,BB const*>>& pairs)const{
      std::unordered_set<BB const*> active;
      this->comparisons=0;
      for(const auto& bb: this->sorted){
        if(bb.end){
          active.erase(bb.value);
        }else{
          this->comparisons++;
          for (auto val: active){
            if(bb.value->agent!=val->agent &&
                bb.value->upperBound.x>=val->lowerBound.x &&
                bb.value->lowerBound.x<=val->upperBound.x &&
                bb.value->lowerBound.y<=val->upperBound.y &&
                bb.value->upperBound.y>=val->lowerBound.y){
              if(bb.value->agent<(val->agent)){
                pairs.emplace_back(bb.value,val);
              }else{
                pairs.emplace_back(val,bb.value);
              }
            }
          }
          active.emplace(bb.value);
        }
      }
    }

    virtual void clear(){this->sorted.resize(0);}

    std::vector<Node> sorted;
    mutable unsigned comparisons;
};

#endif
