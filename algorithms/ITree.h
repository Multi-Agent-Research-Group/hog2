// Sweep and Prune algorithm

#ifndef ITree_H_
#define ITree_H_
#include <vector>
#include <unordered_set>
#include "BroadPhase.h"
#include "options.hpp"
#include "intervaltree.hpp"

using namespace ygg;

template<class Node, typename Key>
class AABBTraits : public ITreeNodeTraits<Node> {
  public:
    using key_type = Key;

    static Key get_lower(const Node & node) {
      return node.value->lowerBound;
    }

    static Key get_upper(const Node & node) {
      return node.value->upperBound;
    }
};

template<class BB>
class Node : public ITreeNodeBase<Node<BB>, AABBTraits<Node<BB>,typename BB::State>> {
  public:
    Node(BB const* bb):value(bb){}
    BB const* value;
};

template<typename BB>
class ITree : public BroadPhase<BB>{
  public:
    IntervalTree<Node<BB>, AABBTraits<Node<BB>,typename BB::State>> tree;

    ITree(unsigned n):BroadPhase<BB>(n){}

    ITree(std::vector<std::vector<BB>*>* paths,bool presorted=true):BroadPhase<BB>(paths){
      if(!presorted){
        for(auto const& p:*paths){
          for(auto const& v:*p){
            nodes.push_back(new Node<BB>(v));
            this->tree.insert(*nodes.back());
          }
        }
      }
    }

    virtual void insert(BB const* v){
      nodes.push_back(new Node<BB>(v));
      this->tree.insert(*nodes.back());
    }

    virtual void getConflicts(BB const* v, std::vector<BB const*>& conflicting)const{
      Node<BB> n(v);
      auto result(this->tree.query(n));
      for(auto const r:result){
        if(r.value->agent!=v->agent)
          conflicting.push_back(r.value);
      }
    }

    virtual void replace(std::vector<BB>* o, std::vector<BB>* n){
      auto oi(o->begin()); // New
      auto ni(n->begin()); // New
      while(oi!=o->end()&&ni!=n->end()){
        if(*oi==*ni){++ni;++oi;continue;} // no change
        Node<BB> old(&*oi);
        this->tree.remove(old);
        nodes.push_back(new Node<BB>(&*ni));
        this->tree.insert(*nodes.back());
      }
      while(oi!=o->end()){
        Node<BB> old(&*oi);
        this->tree.remove(old);
        ++oi;
      }
      while(ni!=n->end()){
        nodes.push_back(new Node<BB>(&*ni));
        this->tree.insert(*nodes.back());
        ++ni;
      }
    }
  
    virtual void getAllPairs(std::vector<std::pair<BB const*,BB const*>>& pairs)const{
      assert(!"All pairs not implemented for interval tree");
    }

    virtual void clear(){this->tree.clear();
      while(!nodes.empty()) delete nodes.front(), nodes.pop_front();
    }
    std::list<Node<BB>*> nodes;
};

#endif
