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
        ITNode *temp = new ITNode;
        temp->i = ipt;
        temp->max = temp->i->GetHighPoint();
        temp->left = temp->right = NULL;
    };
     
    // A utility function to insert a new Interval Search Tree Node
    // This is similar to BST Insert.  Here the low value of interval
    // is used tomaintain BST property
    ITNode *insert(ITNode *root, Interval* i)
    {
        // Base case: Tree is empty, new node becomes root
        if (root == NULL)
            return newNode(i);
     
        // Get low value of interval at root
        int l = root->i->GetLowPoint();
     
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
        if (i1->GetLowPoint() <= i2->GetHighPoint() && i2->GetLowPoint() <= i1->GetHighPoint())
            return true;
        return false;
    }
     
    // The main function that searches a given interval i in a given
    // Interval Tree.
    std::vector<Interval*> intervalSearch(ITNode *root, Interval* i)
    {
        // Base Case, tree is empty
        if (root == NULL){
            std::vector<Interval*> x;
            return x;
        }
            
     
        // If given interval overlaps with root
        if (doOverlap(root->i, i)) {
            std::vector<Interval*> a = intervalSearch(root->left, i);
            std::vector<Interval*> b = intervalSearch(root->right, i);
            a.insert(a.end(), b.begin(), b.end());
            return a;
        }
     
        // If left child of root is present and max of left child is
        // greater than or equal to given interval, then i may
        // overlap with an interval is left subtree
        if (root->left != NULL && root->left->max >= i->GetLowPoint())
            return intervalSearch(root->left, i);
     
        // Else interval can only overlap with right subtree
        return intervalSearch(root->right, i);
    }
}