
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
            Interval *i;
            float max;
            ITNode *left, *right;
            ~ITNode() {
                delete left;
                delete right;
                delete i;
            }
    };

    ITNode * newNode(Interval* ipt);
    ITNode *insert(ITNode *root, Interval* i);
    bool doOverlap(Interval* i1, Interval* i2);
    std::vector<Interval*> intervalSearch(ITNode *root, Interval* i);
}

class IntervalTree {
    public:
        IntervalTree() {}
        ~IntervalTree() { delete root; }

        ITree::ITNode* Insert(ITree::Interval* it) {return ITree::insert(root, it);}
        std::vector<ITree::Interval*> GetOverlappingIntervals(ITree::Interval* i1) {return ITree::intervalSearch(root, i1);}

    private:
        ITree::ITNode* root = nullptr;
};

#endif
