/*
  Copyright (c) 2009 Erin Catto http://www.box2d.org
  Copyright (c) 2016-2017 Lester Hedges <lester.hedges+aabbcc@gmail.com>
  Copyright (c) 2017 Thayne Walker

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.

  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.

  3. This notice may not be removed or altered from any source distribution.

  This code was adapted from parts of the Box2D Physics Engine,
  http://www.box2d.org
*/

#ifndef _AABB_H
#define _AABB_H

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

/// Null node flag.
const unsigned NULL_NODE = 0xffffffff;

namespace aabb
{
    /*! \brief The axis-aligned bounding box object.

        Axis-aligned bounding boxes (AABBs) store information for the minimum
        orthorhombic bounding-box for an object in two- or three-dimensional
        space (the bounding box is either a rectangle, or rectangular prism).

        Class member functions provide functionality for merging AABB objects
        and testing overlap with other AABBs.
     */
    template<typename state, unsigned dim>
    class AABB
    {
    public:
        AABB(){}
        //! Constructor.
        /*! \param lowerBound_
                The lower bound in each dimension.

            \param upperBound_
                The upper bound in each dimension.
         */
        AABB(state const& start, state const& end);

        // Create AABB by merging two AABBs
        AABB(AABB const& one, AABB const& two);

        /// Compute the surface area of the box.
        unsigned computeSurfaceArea() const;

        /// Get the surface area of the box.
        inline unsigned getSurfaceArea() const { return surfaceArea; }

        //! Merge two AABBs into this one.
        /*! \param aabb1
                A reference to the first AABB.

            \param aabb2
                A reference to the second AABB.
         */
        void merge(const AABB&, const AABB&);

        //! Test whether the AABB is contained within this one.
        /*! \param aabb
                A reference to the AABB.

            \return
                Whether the AABB is fully contained.
         */
        bool contains(const AABB&) const;

        //! Test whether the AABB overlaps this one.
        /*! \param aabb
                A reference to the AABB.

            \return
                Whether the AABB overlaps.
         */
        inline bool overlaps(const AABB<state,dim>& aabb) const{
          for(unsigned i(0); i<dim; ++i){
            if(aabb.upperBound[i] < lowerBound[i] || aabb.lowerBound[i] > upperBound[i])
              return false;
          }
          return true;
        }

        //! Compute the centre of the AABB.
        /*! \returns
                The position vector of the AABB centre.
         */

        /*inline void computeCentre(unsigned* position){
          position[0]=0.5 * (lowerBound[0] + upperBound[0]);
          position[1]=0.5 * (lowerBound[1] + upperBound[1]);
          position[2]=0.5 * (lowerBound[2] + upperBound[2]);
        }*/

        /// Lower bound of AABB in each dimension.
        unsigned lowerBound[dim];

        /// Upper bound of AABB in each dimension.
        unsigned upperBound[dim];

        /// The position of the AABB centre.
        //unsigned centre[dim];

        /// The AABB's surface area.
        unsigned surfaceArea;
    };

    /*! \brief A node of the AABB tree.

        Each node of the tree contains an AABB object which corresponds to a
        particle, or a group of particles, in the simulation box.

        Nodes are aware of their position within in the tree. The isLeaf member
        function allows the tree to query whether the node is a leaf, i.e. to
        determine whether it holds a single particle.
     */
    template<typename state, unsigned dim>
    struct Node
    {
        /// The fattened axis-aligned bounding box.
        AABB<state,dim> aabb;

        /// Index of the parent node.
        unsigned parent;

        /// Index of the next node.
        //unsigned next;

        /// Index of the left-hand child.
        unsigned left;

        /// Index of the right-hand child.
        unsigned right;

        /// Height of the node. This is 0 for a leaf and -1 for a free node.
        int height;

        /// The index of the particle that the node contains (leaf nodes only).
        unsigned particle;

        //! Test whether the node is a leaf.
        /*! \return
                Whether the node is a leaf node.
         */
        inline bool isLeaf() const{
          return (left == NULL_NODE);
        }
    };

    /*! \brief The dynamic AABB tree.

        The dynamic AABB tree is a hierarchical data structure that can be used
        to efficiently query overlaps between objects of arbitrary shape and
        size that lie inside of a simulation box.
     */
    template<typename state, unsigned dim>
    class Tree
    {
    public:
        //! Constructor
        /*!
            \param nParticles
                The number of particles (for fixed particle number systems).
         */
        Tree(unsigned nParticles = 16);

        //! Set the size of the simulation box.
        /*! \param boxSize_
                The size of the simulation box in each dimension.
         */
        void setBoxSize(const std::vector<unsigned>&);

        //! Insert a particle into the tree (arbitrary shape with bounding box).
        /*! \param index
                The index of the particle.

            \param lowerBound
                The lower bound in each dimension.

            \param upperBound
                The upper bound in each dimension.
         */
        void insertParticle(unsigned, state const& lower, state const& upper);

        //! Remove a particle from the tree.
        /*! \param particle
                The particle index (particleMap will be used to map the node).
         */
        void removeParticle(unsigned);

        //! Update the tree if a particle moves outside its fattened AABB.
        /*! \param particle
                The particle index (particleMap will be used to map the node).

            \param position
                The position vector of the particle.

            \return
                Whether the particle was reinserted.
         */
        bool updateParticle(unsigned, state const& lower, state const& upper);

        //! Query the tree to find candidate interactions for a particle.
        /*! \param particle
                The particle index.

            \return particles
                A vector of particle indices.
         */
        void query(unsigned, std::vector<unsigned>& ids);

        //! Query the tree to find candidate interactions for an AABB.
        /*! \param particle
                The particle index.

            \param aabb
                The AABB.

            \return particles
                A vector of particle indices.
         */
        void query(unsigned, const AABB<state,dim>&, std::vector<unsigned>& ids);

        //! Query the tree to find candidate interactions for an AABB.
        /*! \param aabb
                The AABB.

            \return particles
                A vector of particle indices.
         */
        void query(const AABB<state,dim>&,std::vector<unsigned>& ids);

        //! Get a particle AABB.
        /*! \param particle
                The particle index.
         */
        const AABB<state,dim>& getAABB(unsigned);

        //! Get the height of the tree.
        /*! \return
                The height of the binary tree.
         */
        inline unsigned getHeight() const{
          if (root == NULL_NODE) return 0;
          return nodes[root].height;
        }



        //! Get the number of nodes in the tree.
        /*! \return
          The number of nodes in the tree.
         */
        inline unsigned getNodeCount() const{
          return nodeCount;
        }

        //! Compute the maximum balancance of the tree.
        /*! \return
                The maximum difference between the height of two
                children of a node.
         */
        unsigned computeMaximumBalance() const;

        //! Compute the surface area ratio of the tree.
        /*! \return
                The ratio of the sum of the node surface area to the surface
                area of the root node.
         */
        //float computeSurfaceAreaRatio() const;

        /// Validate the tree.
        void validate() const;

        /// Rebuild an optimal tree.
        void rebuild();

    private:
        /// The index of the root node.
        unsigned root;

        /// The dynamic tree.
        std::vector<Node<state,dim>> nodes;

        /// The current number of nodes in the tree.
        unsigned nodeCount;

        /// The current node capacity.
        unsigned nodeCapacity;

        /// The position of node at the top of the free list.
        unsigned freeList;

        /// A map between particle and node indices.
        std::map<unsigned, unsigned> particleMap;

        //! Allocate a new node.
        /*! \return
                The index of the allocated node.
         */
        unsigned allocateNode();

        //! Free an existing node.
        /*! \param node
                The index of the node to be freed.
         */
        void freeNode(unsigned);

        //! Insert a leaf into the tree.
        /*! \param leaf
                The index of the leaf node.
         */
        void insertLeaf(unsigned);

        //! Remove a leaf from the tree.
        /*! \param leaf
                The index of the leaf node.
         */
        void removeLeaf(unsigned);

        //! Balance the tree.
        /*! \param node
                The index of the node.
         */
        unsigned balance(unsigned);

        //! Compute the height of the tree.
        /*! \return
                The height of the entire tree.
         */
        unsigned computeHeight() const;

        //! Compute the height of a sub-tree.
        /*! \param node
                The index of the root node.

            \return
                The height of the sub-tree.
         */
        unsigned computeHeight(unsigned) const;

        //! Assert that the sub-tree has a valid structure.
        /*! \param node
                The index of the root node.
         */
        void validateStructure(unsigned) const;

        //! Assert that the sub-tree has valid metrics.
        /*! \param node
                The index of the root node.
         */
        void validateMetrics(unsigned) const;

    };

    template<typename state, unsigned dim>
    AABB<state,dim>::AABB(state const& start, state const& end){
      for(unsigned i(0); i<dim; ++i){
        lowerBound[i]=std::min(start[i],end[i]);
        upperBound[i]=std::max(start[i],end[i]);
      }
      surfaceArea = computeSurfaceArea();
      //computeCentre(centre);
    }

    // Create merged node from two nodes
    template<typename state, unsigned dim>
    AABB<state,dim>::AABB(const AABB<state,dim>& aabb1, const AABB<state,dim>& aabb2){
        merge(aabb1,aabb2);
    }

    template<typename state, unsigned dim>
    unsigned AABB<state,dim>::computeSurfaceArea() const{
      unsigned wx(upperBound[0] - lowerBound[0]);
      unsigned wy(upperBound[1] - lowerBound[1]);
      switch(dim){
        case 2:
          return 2 * (wx*wy);
        case 3:{
                 unsigned wz(upperBound[2] - lowerBound[2]);
                 return 2 * (wx*wy + wx*wz + wy*wz);
               }
        case 4:{
                 unsigned ww(upperBound[2] - lowerBound[2]);
                 unsigned wt(upperBound[3] - lowerBound[3]);
                 return 2 * (wx*wy + wx*ww + wy*ww + wx*wt + wy*wt + ww*wt);
               }
      }
      assert(!"surface area for higher dimensions is not yet implemented");
    }

    template<typename state, unsigned dim>
    void AABB<state,dim>::merge(const AABB<state,dim>& aabb1, const AABB<state,dim>& aabb2){
        for (unsigned i=0;i<dim;i++){
            lowerBound[i] = std::min(aabb1.lowerBound[i], aabb2.lowerBound[i]);
            upperBound[i] = std::max(aabb1.upperBound[i], aabb2.upperBound[i]);
        }

        surfaceArea = computeSurfaceArea();
        //computeCentre(centre);
    }

    template<typename state, unsigned dim>
    bool AABB<state,dim>::contains(AABB<state,dim> const& aabb) const{
        for (unsigned i=0;i<dim;i++){
            if (lowerBound[i] < aabb.lowerBound[i]) return false;
            if (upperBound[i] > aabb.upperBound[i]) return false;
        }
        return true;
    }



    template<typename state, unsigned dim>
    Tree<state,dim>::Tree(unsigned nParticles) {
        // Initialise the tree.
        root = NULL_NODE;
        nodeCount = 0;
        nodeCapacity = nParticles;
        nodes.resize(nodeCapacity);

        // Build a linked list for the list of free nodes.
        for (unsigned i=0;i<nodeCapacity-1;i++){
            //nodes[i].next = i + 1;
            nodes[i].height = -1;
        }
        //nodes[nodeCapacity-1].next = NULL_NODE;
        nodes[nodeCapacity-1].height = -1;

        // Assign the index of the first free node.
        freeList = 0;
    }

    template<typename state, unsigned dim>
    unsigned Tree<state,dim>::allocateNode()
    {
        // Expand the node pool as needed.
        if (freeList == NULL_NODE)
        {
            assert(nodeCount == nodeCapacity);

            // The free list is empty. Rebuild a bigger pool.
            nodeCapacity *= 2;
            nodes.resize(nodeCapacity);

            // Build a linked list for the list of free nodes.
            for (unsigned i=nodeCount;i<nodeCapacity-1;i++)
            {
                //nodes[i].next = i + 1;
                nodes[i].height = -1;
            }
            //nodes[nodeCapacity-1].next = NULL_NODE;
            nodes[nodeCapacity-1].height = -1;

            // Assign the index of the first free node.
            freeList = nodeCount;
        }

        // Peel a node off the free list.
        unsigned node = freeList;
        freeList = node+1;//nodes[node].next;
        nodes[node].parent = nodes[node].left = nodes[node].right = NULL_NODE;
        nodes[node].height = 0;
        nodeCount++;

        return node;
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::freeNode(unsigned node)
    {
        assert(0 <= node && node < nodeCapacity);
        assert(0 < nodeCount);

        //nodes[node].next = freeList;
        nodes[node].height = -1;
        freeList = node;
        nodeCount--;
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::insertParticle(unsigned particle, state const& start, state const& end)
    {
        // Allocate a new node for the particle.
        unsigned node = allocateNode();
        nodes[node].aabb=AABB<state,dim>(start,end);

        // Zero the height.
        nodes[node].height = 0;

        // Insert a new leaf into the tree.
        insertLeaf(node);

        // Add the new particle to the map.
        particleMap.emplace(particle, node);

        // Store the particle index.
        nodes[node].particle = particle;
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::removeParticle(unsigned particle)
    {
        // Map iterator.
        std::map<unsigned, unsigned>::iterator it;

        // Find the particle.
        it = particleMap.find(particle);

        // Extract the node index.
        unsigned node = it->second;

        // Erase the particle from the map.
        particleMap.erase(it);

        assert(0 <= node && node < nodeCapacity);
        assert(nodes[node].isLeaf());

        removeLeaf(node);
        freeNode(node);
    }

    template<typename state, unsigned dim>
    bool Tree<state,dim>::updateParticle(unsigned particle, state const& start, state const& end)
    {
        // Extract the node index.
        unsigned node(particleMap.find(particle)->second);

        assert(0 <= node && node < nodeCapacity);
        assert(nodes[node].isLeaf());

        // Create the new AABB.
        AABB<state,dim> aabb(start,end);

        // Remove the current leaf.
        removeLeaf(node);

        // Assign the new AABB.
        nodes[node].aabb = aabb;

        // Insert a new leaf node.
        insertLeaf(node);

        return true;
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::query(unsigned particle, std::vector<unsigned>& result)
    {
        assert(particleMap.count(particle));

        // Test overlap of particle AABB against all other particles.
        query(particle, nodes[particleMap.find(particle)->second].aabb,result);
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::query(unsigned particle, const AABB<state,dim>& aabb,std::vector<unsigned>& particles)
    {
        std::vector<unsigned> stack;
        stack.reserve(256);
        stack.push_back(root);

        while (stack.size() > 0)
        {
            unsigned node = stack.back();
            stack.pop_back();

            // Copy the AABB.
            AABB<state,dim> nodeAABB = nodes[node].aabb;

            if (node == NULL_NODE) continue;

            // Test for overlap between the AABBs.
            if (aabb.overlaps(nodeAABB))
            {
                // Check that we're at a leaf node.
                if (nodes[node].isLeaf())
                {
                    // Can't interact with itself.
                    if (nodes[node].particle != particle)
                        particles.push_back(nodes[node].particle);
                }
                else
                {
                    stack.push_back(nodes[node].left);
                    stack.push_back(nodes[node].right);
                }
            }
        }
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::query(const AABB<state,dim>& aabb,std::vector<unsigned>& result)
    {
        // Test overlap of AABB against all particles.
        query(std::numeric_limits<unsigned>::max(), aabb, result);
    }

    template<typename state, unsigned dim>
    const AABB<state,dim>& Tree<state,dim>::getAABB(unsigned particle)
    {
        return nodes[particleMap[particle]].aabb;
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::insertLeaf(unsigned leaf)
    {
        if (root == NULL_NODE)
        {
            root = leaf;
            nodes[root].parent = NULL_NODE;
            return;
        }

        // Find the best sibling for the node.

        AABB<state,dim>& leafAABB = nodes[leaf].aabb;
        unsigned index(root);

        while (!nodes[index].isLeaf())
        {
            // Extract the children of the node.
            unsigned left(nodes[index].left);
            unsigned right(nodes[index].right);

            AABB<state,dim> combinedAABB(nodes[index].aabb, leafAABB);
            unsigned combinedSurfaceArea = combinedAABB.getSurfaceArea();

            // Cost of creating a new parent for this node and the new leaf.
            unsigned cost = 2.0 * combinedSurfaceArea;

            // Minimum cost of pushing the leaf further down the tree.
            signed inheritanceCost = 2.0 * (combinedSurfaceArea - nodes[index].aabb.getSurfaceArea());

            // Cost of descending to the left.
            unsigned costLeft;
            if (nodes[left].isLeaf())
            {
                AABB<state,dim> aabb(leafAABB, nodes[left].aabb);
                costLeft = aabb.getSurfaceArea() + inheritanceCost;
            }
            else
            {
                AABB<state,dim> aabb(leafAABB, nodes[left].aabb);
                costLeft = (aabb.getSurfaceArea() - nodes[left].aabb.getSurfaceArea()) + inheritanceCost;
            }

            // Cost of descending to the right.
            unsigned costRight;
            if (nodes[right].isLeaf())
            {
                AABB<state,dim> aabb(leafAABB, nodes[right].aabb);
                costRight = aabb.getSurfaceArea() + inheritanceCost;
            }
            else
            {
                AABB<state,dim> aabb(leafAABB, nodes[right].aabb);
                costRight = (aabb.getSurfaceArea() - nodes[right].aabb.getSurfaceArea()) + inheritanceCost;
            }

            // Descend according to the minimum cost.
            if ((cost < costLeft) && (cost < costRight)) break;

            // Descend.
            if (costLeft < costRight) index = left;
            else                      index = right;
        }

        unsigned sibling(index);

        // Create a new parent.
        unsigned oldParent = nodes[sibling].parent;
        unsigned newParent = allocateNode();
        nodes[newParent].parent = oldParent;
        nodes[newParent].aabb.merge(leafAABB, nodes[sibling].aabb);
        nodes[newParent].height = nodes[sibling].height + 1;

        // The sibling was not the root.
        if (oldParent != NULL_NODE)
        {
            if (nodes[oldParent].left == sibling) nodes[oldParent].left = newParent;
            else                                  nodes[oldParent].right = newParent;

            nodes[newParent].left = sibling;
            nodes[newParent].right = leaf;
            nodes[sibling].parent = newParent;
            nodes[leaf].parent = newParent;
        }
        // The sibling was the root.
        else
        {
            nodes[newParent].left = sibling;
            nodes[newParent].right = leaf;
            nodes[sibling].parent = newParent;
            nodes[leaf].parent = newParent;
            root = newParent;
        }

        // Walk back up the tree fixing heights and AABBs.
        index = nodes[leaf].parent;
        while (index != NULL_NODE)
        {
            index = balance(index);

            unsigned left = nodes[index].left;
            unsigned right = nodes[index].right;

            assert(left != NULL_NODE);
            assert(right != NULL_NODE);

            nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);
            nodes[index].aabb.merge(nodes[left].aabb, nodes[right].aabb);

            index = nodes[index].parent;
        }
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::removeLeaf(unsigned leaf)
    {
        if (leaf == root)
        {
            root = NULL_NODE;
            return;
        }

        unsigned parent = nodes[leaf].parent;
        unsigned grandParent = nodes[parent].parent;
        unsigned sibling;

        if (nodes[parent].left == leaf) sibling = nodes[parent].right;
        else                            sibling = nodes[parent].left;

        // Destroy the parent and connect the sibling to the grandparent.
        if (grandParent != NULL_NODE)
        {
            if (nodes[grandParent].left == parent) nodes[grandParent].left = sibling;
            else                                   nodes[grandParent].right = sibling;

            nodes[sibling].parent = grandParent;
            freeNode(parent);

            // Adjust ancestor bounds.
            unsigned index = grandParent;
            while (index != NULL_NODE)
            {
                index = balance(index);

                unsigned left = nodes[index].left;
                unsigned right = nodes[index].right;

                nodes[index].aabb.merge(nodes[left].aabb, nodes[right].aabb);
                nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);

                index = nodes[index].parent;
            }
        }
        else
        {
            root = sibling;
            nodes[sibling].parent = NULL_NODE;
            freeNode(parent);
        }
    }

    template<typename state, unsigned dim>
    unsigned Tree<state,dim>::balance(unsigned node)
    {
        assert(node != NULL_NODE);

        if (nodes[node].isLeaf() || (nodes[node].height < 2))
            return node;

        unsigned left = nodes[node].left;
        unsigned right = nodes[node].right;

        assert((0 <= left) && (left < nodeCapacity));
        assert((0 <= right) && (right < nodeCapacity));

        int currentBalance = nodes[right].height - nodes[left].height;

        // Rotate right branch up.
        if (currentBalance > 1)
        {
            unsigned rightLeft = nodes[right].left;
            unsigned rightRight = nodes[right].right;

            assert((0 <= rightLeft) && (rightLeft < nodeCapacity));
            assert((0 <= rightRight) && (rightRight < nodeCapacity));

            // Swap node and its right-hand child.
            nodes[right].left = node;
            nodes[right].parent = nodes[node].parent;
            nodes[node].parent = right;

            // The node's old parent should now point to its right-hand child.
            if (nodes[right].parent != NULL_NODE)
            {
                if (nodes[nodes[right].parent].left == node) nodes[nodes[right].parent].left = right;
                else
                {
                    assert(nodes[nodes[right].parent].right == node);
                    nodes[nodes[right].parent].right = right;
                }
            }
            else root = right;

            // Rotate.
            if (nodes[rightLeft].height > nodes[rightRight].height)
            {
                nodes[right].right = rightLeft;
                nodes[node].right = rightRight;
                nodes[rightRight].parent = node;
                nodes[node].aabb.merge(nodes[left].aabb, nodes[rightRight].aabb);
                nodes[right].aabb.merge(nodes[node].aabb, nodes[rightLeft].aabb);

                nodes[node].height = 1 + std::max(nodes[left].height, nodes[rightRight].height);
                nodes[right].height = 1 + std::max(nodes[node].height, nodes[rightLeft].height);
            }
            else
            {
                nodes[right].right = rightRight;
                nodes[node].right = rightLeft;
                nodes[rightLeft].parent = node;
                nodes[node].aabb.merge(nodes[left].aabb, nodes[rightLeft].aabb);
                nodes[right].aabb.merge(nodes[node].aabb, nodes[rightRight].aabb);

                nodes[node].height = 1 + std::max(nodes[left].height, nodes[rightLeft].height);
                nodes[right].height = 1 + std::max(nodes[node].height, nodes[rightRight].height);
            }

            return right;
        }

        // Rotate left branch up.
        if (currentBalance < -1)
        {
            unsigned leftLeft = nodes[left].left;
            unsigned leftRight = nodes[left].right;

            assert((0 <= leftLeft) && (leftLeft < nodeCapacity));
            assert((0 <= leftRight) && (leftRight < nodeCapacity));

            // Swap node and its left-hand child.
            nodes[left].left = node;
            nodes[left].parent = nodes[node].parent;
            nodes[node].parent = left;

            // The node's old parent should now point to its left-hand child.
            if (nodes[left].parent != NULL_NODE)
            {
                if (nodes[nodes[left].parent].left == node) nodes[nodes[left].parent].left = left;
                else
                {
                    assert(nodes[nodes[left].parent].right == node);
                    nodes[nodes[left].parent].right = left;
                }
            }
            else root = left;

            // Rotate.
            if (nodes[leftLeft].height > nodes[leftRight].height)
            {
                nodes[left].right = leftLeft;
                nodes[node].left = leftRight;
                nodes[leftRight].parent = node;
                nodes[node].aabb.merge(nodes[right].aabb, nodes[leftRight].aabb);
                nodes[left].aabb.merge(nodes[node].aabb, nodes[leftLeft].aabb);

                nodes[node].height = 1 + std::max(nodes[right].height, nodes[leftRight].height);
                nodes[left].height = 1 + std::max(nodes[node].height, nodes[leftLeft].height);
            }
            else
            {
                nodes[left].right = leftRight;
                nodes[node].left = leftLeft;
                nodes[leftLeft].parent = node;
                nodes[node].aabb.merge(nodes[right].aabb, nodes[leftLeft].aabb);
                nodes[left].aabb.merge(nodes[node].aabb, nodes[leftRight].aabb);

                nodes[node].height = 1 + std::max(nodes[right].height, nodes[leftLeft].height);
                nodes[left].height = 1 + std::max(nodes[node].height, nodes[leftRight].height);
            }

            return left;
        }

        return node;
    }

    template<typename state, unsigned dim>
    unsigned Tree<state,dim>::computeHeight() const
    {
        return computeHeight(root);
    }

    template<typename state, unsigned dim>
    unsigned Tree<state,dim>::computeHeight(unsigned node) const
    {
        assert((0 <= node) && (node < nodeCapacity));

        if (nodes[node].isLeaf()) return 0;

        unsigned height1 = computeHeight(nodes[node].left);
        unsigned height2 = computeHeight(nodes[node].right);

        return 1 + std::max(height1, height2);
    }

    template<typename state, unsigned dim>
    unsigned Tree<state,dim>::computeMaximumBalance() const
    {
        unsigned maxBalance = 0;
        for (unsigned i=0; i<nodeCapacity; i++)
        {
            if (nodes[i].height <= 1)
                continue;

            assert(nodes[i].isLeaf() == false);

            unsigned balance = std::abs(nodes[nodes[i].left].height - nodes[nodes[i].right].height);
            maxBalance = std::max(maxBalance, balance);
        }

        return maxBalance;
    }

    /*template<typename state, unsigned dim>
    float Tree<state,dim>::computeSurfaceAreaRatio() const
    {
        if (root == NULL_NODE) return 0.0;

        float rootArea = nodes[root].aabb.computeSurfaceArea();
        float totalArea = 0.0;

        for (unsigned i=0; i<nodeCapacity;i++)
        {
            if (nodes[i].height < 0) continue;

            totalArea += nodes[i].aabb.computeSurfaceArea();
        }

        return totalArea / rootArea;
    }*/

    template<typename state, unsigned dim>
    void Tree<state,dim>::validate() const
    {
#ifndef NDEBUG
        validateStructure(root);
        validateMetrics(root);

        unsigned freeCount = 0;
        unsigned freeIndex = freeList;

        while (freeIndex != NULL_NODE)
        {
            assert((0 <= freeIndex) && (freeIndex < nodeCapacity));
            //freeIndex = nodes[freeIndex].next;
            freeCount++;
        }

        assert(getHeight() == computeHeight());
        assert((nodeCount + freeCount) == nodeCapacity);
#endif
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::rebuild()
    {
        unsigned nodeIndices[nodeCount];
        unsigned count = 0;

        for (unsigned i=0;i<nodeCapacity;i++)
        {
            // Free node.
            if (nodes[i].height < 0) continue;

            if (nodes[i].isLeaf())
            {
                nodes[i].parent = NULL_NODE;
                nodeIndices[count] = i;
                count++;
            }
            else freeNode(i);
        }

        while (count > 1)
        {
            unsigned minCost = std::numeric_limits<unsigned>::max();
            int iMin = -1, jMin = -1;

            for (unsigned i=0;i<count;i++)
            {
                AABB<state,dim> aabbi = nodes[nodeIndices[i]].aabb;

                for (unsigned j=i+1;j<count;j++)
                {
                    AABB<state,dim> aabbj = nodes[nodeIndices[j]].aabb;
                    AABB<state,dim> aabb(aabbi, aabbj);
                    unsigned cost = aabb.getSurfaceArea();

                    if (cost < minCost)
                    {
                        iMin = i;
                        jMin = j;
                        minCost = cost;
                    }
                }
            }

            unsigned index1 = nodeIndices[iMin];
            unsigned index2 = nodeIndices[jMin];

            unsigned parent = allocateNode();
            nodes[parent].left = index1;
            nodes[parent].right = index2;
            nodes[parent].height = 1 + std::max(nodes[index1].height, nodes[index2].height);
            nodes[parent].aabb.merge(nodes[index1].aabb, nodes[index2].aabb);
            nodes[parent].parent = NULL_NODE;

            nodes[index1].parent = parent;
            nodes[index2].parent = parent;

            nodeIndices[jMin] = nodeIndices[count-1];
            nodeIndices[iMin] = parent;
            count--;
        }

        root = nodeIndices[0];

        //validate();
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::validateStructure(unsigned node) const
    {
        if (node == NULL_NODE) return;

        if (node == root) assert(nodes[node].parent == NULL_NODE);

        unsigned left = nodes[node].left;
        unsigned right = nodes[node].right;

        if (nodes[node].isLeaf())
        {
            assert(left == NULL_NODE);
            assert(right == NULL_NODE);
            assert(nodes[node].height == 0);
            return;
        }

        assert((0 <= left) && (left < nodeCapacity));
        assert((0 <= right) && (right < nodeCapacity));

        assert(nodes[left].parent == node);
        assert(nodes[right].parent == node);

        validateStructure(left);
        validateStructure(right);
    }

    template<typename state, unsigned dim>
    void Tree<state,dim>::validateMetrics(unsigned node) const
    {
        if (node == NULL_NODE) return;

        unsigned left = nodes[node].left;
        unsigned right = nodes[node].right;

        if (nodes[node].isLeaf())
        {
            assert(left == NULL_NODE);
            assert(right == NULL_NODE);
            assert(nodes[node].height == 0);
            return;
        }

        assert((0 <= left) && (left < nodeCapacity));
        assert((0 <= right) && (right < nodeCapacity));

        int height1 = nodes[left].height;
        int height2 = nodes[right].height;
        int height = 1 + std::max(height1, height2);
        assert(nodes[node].height == height);

        AABB<state,dim> aabb(nodes[left].aabb, nodes[right].aabb);

        for(unsigned i=0;i<dim;i++){
            assert(aabb.lowerBound[i] == nodes[node].aabb.lowerBound[i]);
            assert(aabb.upperBound[i] == nodes[node].aabb.upperBound[i]);
        }

        validateMetrics(left);
        validateMetrics(right);
    }
}

#endif /* _AABB_H */
