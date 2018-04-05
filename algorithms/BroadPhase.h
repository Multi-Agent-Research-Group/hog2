// Broadphase collision detector

#ifndef BP_H_
#define BP_H_

template<typename BB>
class BroadPhase{
  public:
    BroadPhase(unsigned n):k(n){}

    BroadPhase(std::vector<std::vector<BB>*>* paths):k(paths->size()){}

    virtual void getConflicts(BB const* v, std::vector<BB const*>& conflicting)const=0;

    virtual void insert(BB const* v)=0;

    virtual void update(std::vector<std::vector<BB>*>* paths){}

    virtual void replace(std::vector<BB>* o, std::vector<BB>* n)=0;
  
    virtual void getAllPairs(std::vector<std::pair<BB const*,BB const*>>& pairs)const=0;

    virtual void clear()=0;

    unsigned k;
};

#endif

