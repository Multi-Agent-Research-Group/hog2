
#ifndef IntervalTree_h
#define IntervalTree_h

#include <iostream>
#include <vector>


namespace ITree {
    struct Interval
    {
            virtual float GetLowPoint() = 0;
            virtual float GetHighPoint() = 0;
    };

    struct BasicInterval : public Interval{
        BasicInterval(float one, float two) : a(one), b(two) {}
        float a;
        float b;
        float GetLowPoint() {return a;}
        float GetHighPoint() {return b;}
    };
     
    struct ITNode
    {
            ITNode():i(0),max(0),left(0),right(0){}
            Interval *i;
            float max;
            ITNode *left, *right;
            ~ITNode() {
                if(left)
                  delete left;
                if(right)
                  delete right;
                if(i)
                  delete i;
            }
    };

    ITNode * newNode(Interval* ipt);
    ITNode *insert(ITNode *root, Interval* i);
    bool doOverlap(Interval* i1, Interval* i2);
    std::vector<Interval*> intervalSearch(ITNode *root, Interval* i, unsigned d=0);
}

class IntervalTree {
    public:
        IntervalTree() :count(0) {}
        ~IntervalTree() { delete root; }

        ITree::ITNode* Insert(ITree::Interval* it) {++count; return root=ITree::insert(root, it);}
        inline unsigned size()const{return count;}
        std::vector<ITree::Interval*> GetOverlappingIntervals(ITree::Interval* i1) {return ITree::intervalSearch(root, i1);}

    private:
        ITree::ITNode* root = nullptr;
        unsigned count;
};

#endif
