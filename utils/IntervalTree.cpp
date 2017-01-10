//
//  AirplaneTicketAuthority.cpp
//  hog2 glut
//
//  Created by David Chan on 7/24/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//


#include "IntervalTree.h"

namespace ITree {
     
    // A utility function to create a new Interval Search Tree Node
    ITNode * newNode(Interval* ipt)
    {
        ITNode *temp = new ITNode();
        temp->i = ipt;
        temp->max = temp->i->GetHighPoint();
        temp->left = temp->right = NULL;
        return temp;
    };
     
    // A utility function to insert a new Interval Search Tree Node
    // This is similar to BST Insert.  Here the low value of interval
    // is used tomaintain BST property
    ITNode *insert(ITNode *root, Interval* i)
    {
        std::cout << "insert\n";
        // Base case: Tree is empty, new node becomes root
        if (root == NULL)
            return newNode(i);
     
        // Get low value of interval at root
        float l = root->i->GetLowPoint();
     
        // If root's low value is smaller, then new interval goes to
        // left subtree
        if (i->GetLowPoint() < l)
            root->left = insert(root->left, i);
     
        // Else, new node goes to right subtree.
        else
            root->right = insert(root->right, i);
     
        // Update the max value of this ancestor if needed
        if (root->max < i->GetHighPoint())
            root->max = i->GetHighPoint();
     
        return root;
    }
     
    // A utility function to check if given two intervals overlap
    bool doOverlap(Interval* i1, Interval* i2)
    {
        std::cout << i1->GetLowPoint() << "," << i1->GetHighPoint() << " : " << i2->GetLowPoint() << "," << i2->GetHighPoint() << "\n";
        if (i1->GetLowPoint() <= i2->GetHighPoint() && i2->GetLowPoint() <= i1->GetHighPoint())
            return true;
        return false;
    }
     
    // The main function that searches a given interval i in a given
    // Interval Tree.
    std::vector<Interval*> intervalSearch(ITNode *root, Interval* i, unsigned d)
    {
        std::cout << "IntervalSearch depth " << d << "\n";
        // Base Case, (sub)tree is empty
        if (root == NULL){
            std::vector<Interval*> x;
            std::cout << "Root is null\n";
            return x;
        }
            
     
        // If given interval overlaps with root
        if (doOverlap(root->i, i)) {
            std::cout << "O root\n";
            std::vector<Interval*> r;
            r.push_back(root->i);

            std::cout << " left\n";
            std::vector<Interval*> a = intervalSearch(root->left, i,d+1);
            r.insert(r.end(), a.begin(), a.end());
           
            std::cout << " right\n";
            std::vector<Interval*> b = intervalSearch(root->right, i,d+1);
            r.insert(r.end(), b.begin(), b.end());
            return r;
        }
     
        // If left child of root is present and max of left child is
        // greater than or equal to given interval, then i may
        // overlap with an interval is left subtree
        if (root->left != NULL && root->left->max >= i->GetLowPoint()){
            std::cout << "O left\n";
            return intervalSearch(root->left, i,d+1);
        }
     
        // Else interval can only overlap with right subtree
        std::cout << "O right\n";
        return intervalSearch(root->right, i,d+1);
    }
}
