// Broadphase collision detector

#ifndef BP_H_
#define BP_H_

template<typename BB>
class BroadPhase{
  public:
    BroadPhase(){}

    BroadPhase(std::vector<std::vector<BB>*> const& paths){}

    virtual double hasConflict(BB const& v)const=0;

    virtual double hasConflict(std::vector<BB> const& path)const{
      for(auto const& v:path){
        if(hasConflict(&v)){return true;}
      }
      return false;
    }

    virtual void insert(BB const& v)=0;

    virtual void replace(std::vector<BB> const& o, std::vector<BB> const& n)=0;
  
    virtual void getAllPairs(std::vector<std::pair<BB,BB>>& pairs)const=0;

    virtual void getPaths(std::vector<std::vector<BB>>& paths)const=0;

    virtual void clear()=0;
};

#endif

