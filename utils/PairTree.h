/******************************************************************************
**
** Copyright (C) 2017 Ivan Pinezhaninov <ivan.pinezhaninov@gmail.com>
**
** This file is part of the PairTree - Red-Black balanced interval tree
** which can be found at https://github.com/IvanPinezhaninov/PairTree/.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
** OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
** THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
*  Modified by Thayne Walker 2019
******************************************************************************/

#ifndef PAIRTREE_H
#define PAIRTREE_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <ostream>
#include <utility>
#include <vector>

template <typename ValueType>
struct PairInterval
{
    PairInterval(ValueType const &v, uint16_t minimum, uint16_t maximum) : value(v), minInterval(minimum), maxInterval(maximum) {}
    PairInterval(ValueType const &v) : value(v), minInterval(SHRT_MAX), maxInterval(0) {}
    PairInterval() : minInterval(SHRT_MAX), maxInterval(0) {}
    ValueType value;
    uint16_t minInterval;
    uint16_t maxInterval;
    inline ValueType const *operator->() const { return &value; }
    inline bool operator==(PairInterval<ValueType> const &rhs) const
    {
        return value == rhs.value;
    }

    inline bool operator<(PairInterval<ValueType> const &rhs) const
    {
        return value < rhs.value;
    }
};

template <typename state> class PairIntervalCIter;

template <typename ValueType>
class PairTree
{
public:
    using Intervals = std::vector<PairInterval<ValueType>>;
    using Values = std::vector<ValueType*>;
    friend class PairIntervalCIter<ValueType>;
    typedef PairIntervalCIter<ValueType> iterator;

    iterator begin(){return iterator(findMin(),*this,0);}
    iterator end(){return iterator(m_nill,*this,0);}
    iterator find(ValueType const &v)
    {
        if (m_root == m_nill)
        {
            return iterator(m_nill, *this, 0);
        }
        PairInterval<ValueType> interval(v);
        auto node(findNode(m_root, interval));
        if (node == m_nill)
        {
            return iterator(m_nill, *this, 0);
        }
        auto it(std::find(node->intervals.cbegin(), node->intervals.cend(), interval));
        if(it==node->intervals.cend()){
            return iterator(m_nill, *this, 0);
        }

        return iterator(node, *this, node->intervals.cbegin()-it);
    }

    PairTree() :
        m_nill(new Node()),
        m_root(m_nill),
        m_size(0)
    {
    }


    template <typename Container>
    PairTree(const Container &intervals) :
        PairTree()
    {
        for (const PairInterval<ValueType> &interval : intervals) {
            insert(interval);
        }
    }


    template <typename ForwardIterator>
    PairTree(ForwardIterator begin, ForwardIterator end) :
        PairTree()
    {
        while (begin != end) {
            insert(*begin);
            ++begin;
        }
    }


    virtual ~PairTree()
    {
        if (nullptr != m_root) {
            destroySubtree(m_root);
        }

        delete m_nill;
    }


    PairTree(const PairTree &other) :
        m_nill(new Node()),
        m_root(copySubtree(other.m_root, other.m_nill, m_nill)),
        m_size(other.m_size)
    {
    }


    PairTree(PairTree &&other) noexcept :
        m_nill(other.m_nill),
        m_root(other.m_root),
        m_size(other.m_size)
    {
        other.m_nill = nullptr;
        other.m_root = nullptr;
    }


    PairTree &operator=(const PairTree &other)
    {
        if (this != &other) {
            PairTree(other).swap(*this);
        }

        return *this;
    }


    PairTree &operator=(PairTree &&other) noexcept
    {
        other.swap(*this);
        return *this;
    }


    void swap(PairTree &other) noexcept
    {
        std::swap(m_nill, other.m_nill);
        std::swap(m_root, other.m_root);
        std::swap(m_size, other.m_size);
    }

    void insert(const std::vector<std::unique_ptr<const ValueType>> &values){
      for(auto const& v:values){
        insert(*v.get());
      }
    }


    bool insert(const ValueType &value,uint16_t minimum,uint16_t maximum){
      return insert(PairInterval<ValueType>(value,minimum,maximum));
    }

    bool insert(const PairInterval<ValueType> &interval)
    {
        assert(nullptr != m_root && nullptr != m_nill);

        if (m_root == m_nill) {
            // Tree is empty
            assert(0 == m_size);
            m_root = new Node(interval, Color::Black, m_nill);
            m_size = 1;
            return true;
        }

        Node *node = findNode(m_root, interval);
        assert(node != m_nill);

        if (interval->first.t < node->intervals.front()->first.t) {
            createChildNode(node, interval, Position::Left);
            return true;
        } else if (node->intervals.front()->first.t < interval->first.t) {
            createChildNode(node, interval, Position::Right);
            return true;
        }

        if (!isNodeHasInterval(node, interval)) {
            auto it = std::lower_bound(node->intervals.begin(), node->intervals.end(), interval,
                                       [] (const PairInterval<ValueType> &lhs, const PairInterval<ValueType> &rhs) -> bool { return (lhs->second.t < rhs->second.t); });

            node->intervals.insert(it, interval);

            if (node->high < interval->second.t) {
                node->high = interval->second.t;
            }

            if (node->highest < node->high) {
                node->highest = node->high;
                insertionFixNodeLimits(node);
            }

            ++m_size;
            return true;
        }

        // Value already exists
        return false;
    }

    bool remove(const ValueType &value){
      return remove(PairInterval<ValueType>(value));
    }


    bool remove(const PairInterval<ValueType> &interval)
    {
        assert(nullptr != m_root && nullptr != m_nill);

        if (m_root == m_nill) {
            // Tree is empty
            assert(0 == m_size);
            return false;
        }

        Node *node = findNode(m_root, interval);
        assert(node != m_nill);

        auto it = std::find(node->intervals.begin(), node->intervals.end(), interval);
        if (it != node->intervals.cend()) {
            node->intervals.erase(it);
            if (isNodeAboutToBeDestroyed(node)) {
                Node *child = m_nill;
                if (node->right == m_nill) {
                    child = node->left;
                } else if (node->left == m_nill) {
                    child = node->right;
                } else {
                    Node *nextValueNode = node->right;
                    while (nextValueNode->left != m_nill) {
                        nextValueNode = nextValueNode->left;
                    }
                    node->intervals = std::move(nextValueNode->intervals);
                    node->high = std::move(nextValueNode->high);
                    removeFixNodeLimits(node);
                    node = nextValueNode;
                    child = nextValueNode->right;
                }

                if (child == m_nill && node->parent == m_nill) {
                    // Node is root without children
                    swapRelations(node, child);
                    destroyNode(node);
                    return true;
                }

                if (Color::Red == node->color || Color::Red == child->color) {
                    swapRelations(node, child);
                    if (Color::Red == child->color) {
                        child->color = Color::Black;
                    }
                } else {
                    assert(Color::Black == node->color);

                    if (child == m_nill) {
                        child = node;
                    } else {
                        swapRelations(node, child);
                    }

                    removeFix(child);

                    if (node->parent != m_nill) {
                        setChildNode(node->parent, m_nill, nodePosition(node));
                    }
                }

                if (node->parent != m_nill) {
                    removeFixNodeLimits(node->parent, 1);
                }
                destroyNode(node);
            } else {
                if(interval->second.t == node->high){
                    node->high = findHighest(node->intervals);
                }

                if(interval->second.t == node->highest){
                    removeFixNodeLimits(node);
                }

                --m_size;
            }

            return true;
        }

        // Value not found
        return false;
    }


    bool contains(const PairInterval<ValueType> &interval) const
    {
        assert(nullptr != m_root && nullptr != m_nill);

        if (m_root == m_nill) {
            // Tree is empty
            assert(0 == m_size);
            return false;
        }

        Node *node = findNode(m_root, interval);
        assert(node != m_nill);

        return isNodeHasInterval(node, interval);
    }


    Intervals intervals() const
    {
        Intervals out;
        out.reserve(m_size);

        if (m_root != m_nill) {
            subtreeIntervals(m_root, out);
        }

        return out;
    }


    void findOverlapping(ValueType const* value, Intervals &out, bool boundary = true) const
    {
      PairInterval<ValueType> interval(value);
      if (!out.empty()) {
        out.clear();
      }

      if (m_root != m_nill) {
        subtreeOverlappingIntervals(m_root, interval, boundary, [&out] (const PairInterval<ValueType> &in) -> void { out.push_back(in); });
      }

      out.shrink_to_fit();
    }


    Intervals findOverlapping(ValueType const* value, bool boundary = true) const
    {
      PairInterval<ValueType> interval(value);
      Intervals out;
      out.reserve(size_t(m_size * VECTOR_RESERVE_RATE));
      findOverlappingIntervals(interval, out, boundary);
      return out;
    }


    void findInnerIntervals(const PairInterval<ValueType> &interval, Intervals &out, bool boundary = true) const
    {
        if (!out.empty()) {
            out.clear();
        }

        if (m_root != m_nill) {
            subtreeInnerIntervals(m_root, interval, boundary, [&out] (const PairInterval<ValueType> &in) -> void { out.push_back(in); });
        }

        out.shrink_to_fit();
    }


    Intervals findInnerIntervals(const PairInterval<ValueType> &interval, bool boundary = true) const
    {
        Intervals out;
        out.reserve(size_t(m_size * VECTOR_RESERVE_RATE));
        findInnerIntervals(interval, out, boundary);
        return out;
    }


    void findOuterIntervals(const PairInterval<ValueType> &interval, Intervals &out, bool boundary = true) const
    {
        if (!out.empty()) {
            out.clear();
        }

        if (m_root != m_nill) {
            subtreeOuterIntervals(m_root, interval, boundary, [&out] (const PairInterval<ValueType> &in) -> void { out.push_back(in); });
        }

        out.shrink_to_fit();
    }


    Intervals findOuterIntervals(const PairInterval<ValueType> &interval, bool boundary = true) const
    {
        Intervals out;
        out.reserve(size_t(m_size * VECTOR_RESERVE_RATE));
        findOuterIntervals(interval, out, boundary);
        return out;
    }


    void findIntervalsContainPoint(const PairInterval<ValueType> &point, Intervals &out, bool boundary = true) const
    {
        if (!out.empty()) {
            out.clear();
        }

        if (m_root != m_nill) {
            subtreeIntervalsContainPoint(m_root, point, boundary, [&out] (const PairInterval<ValueType> &in) -> void { out.push_back(in); });
        }

        out.shrink_to_fit();
    }


    Intervals findIntervalsContainPoint(const PairInterval<ValueType> &point, bool boundary = true) const
    {
        Intervals out;
        out.reserve(size_t(m_size * VECTOR_RESERVE_RATE));
        findIntervalsContainPoint(point, out, boundary);
        return out;
    }


    unsigned countOverlappingIntervals(const PairInterval<ValueType> &interval, bool boundary = true) const
    {
        unsigned count = 0;

        if (m_root != m_nill) {
            subtreeOverlappingIntervals(m_root, interval, boundary, [&count] (const PairInterval<ValueType> &) -> void { ++count; });
        }

        return count;
    }


    unsigned countInnerIntervals(const PairInterval<ValueType> &interval, bool boundary = true) const
    {
        unsigned count = 0;

        if (m_root != m_nill) {
            subtreeInnerIntervals(m_root, interval, boundary, [&count] (const PairInterval<ValueType> &) -> void { ++count; });
        }

        return count;
    }


    unsigned countOuterIntervals(const PairInterval<ValueType> &interval, bool boundary = true) const
    {
        unsigned count = 0;

        if (m_root != m_nill) {
            subtreeOuterIntervals(m_root, interval, boundary, [&count] (const PairInterval<ValueType> &) -> void { ++count; });
        }

        return count;
    }


    unsigned countIntervalsContainPoint(const PairInterval<ValueType> &point, bool boundary = true) const
    {
        unsigned count = 0;

        if (m_root != m_nill) {
            subtreeIntervalsContainPoint(m_root, point, boundary, [&count] (const PairInterval<ValueType> &) -> void { ++count; });
        }

        return count;
    }


    bool empty() const
    {
        return (0 == m_size);
    }


    size_t size() const
    {
        return m_size;
    }


    void clear()
    {
        assert(nullptr != m_root && nullptr != m_nill);

        destroySubtree(m_root);
        m_root = m_nill;
        m_size = 0;
    }

    enum class Color : char {
        Black,
        Red
    };

    struct Node{
        Node() = default;

        Node(const PairInterval<ValueType> &interval, Color col, Node *nill) :
            color(col),
            parent(nill),
            left(nill),
            right(nill),
            high(interval->second.t),
            lowest(interval->first.t),
            highest(interval->second.t)
        {
            intervals.push_back(interval);
        }

        Color color = Color::Black;
        Node *parent = nullptr;
        Node *left = nullptr;
        Node *right = nullptr;

        unsigned high;
        unsigned lowest;
        unsigned highest;
        Intervals intervals;
    };

    Node *m_nill;
    Node *m_root;

private:
    constexpr static const double VECTOR_RESERVE_RATE = 0.25;

    enum class Position : char {
        Left,
        Right
    };



    Node *copySubtree(Node *otherNode, Node *otherNill, Node *parent) const
    {
        assert(nullptr != otherNode && nullptr != otherNill && nullptr != parent);

        if (otherNode == otherNill) {
            return m_nill;
        }

        Node *node = new Node();
        node->intervals = otherNode->intervals;
        node->high = otherNode->high;
        node->lowest = otherNode->lowest;
        node->highest = otherNode->highest;
        node->color = otherNode->color;
        node->parent = parent;
        node->left = copySubtree(otherNode->left, otherNill, node);
        node->right = copySubtree(otherNode->right, otherNill, node);

        return node;
    }


    void destroySubtree(Node *node) const
    {
        assert(nullptr != node);

        if (node == m_nill) {
            return;
        }

        destroySubtree(node->left);
        destroySubtree(node->right);

        delete node;
    }


    Node *findNode(Node *node, const PairInterval<ValueType> &interval) const
    {
        assert(nullptr != node);
        assert(node != m_nill);

        Node *child = m_nill;
        if (interval->first.t < node->intervals.front()->first.t) {
            child = childNode(node, Position::Left);
        } else if (node->intervals.front()->first.t < interval->first.t) {
            child = childNode(node, Position::Right);
        } else {
            return node;
        }

        return (child == m_nill) ? node : findNode(child, interval);
    }


    Node *siblingNode(Node *node) const
    {
        assert(nullptr != node);

        return (Position::Left == nodePosition(node))
                ? childNode(node->parent, Position::Right)
                : childNode(node->parent, Position::Left);
    }


    Node *childNode(Node *node, Position position) const
    {
        assert(nullptr != node);

        switch (position) {
        case Position::Left:
            return node->left;
        case Position::Right:
            return node->right;
        default:
            assert(false);
            return nullptr;
        }
    }


    void setChildNode(Node *node, Node *child, Position position) const
    {
        assert(nullptr != node && nullptr != child);
        assert(node != m_nill);

        switch (position) {
        case Position::Left:
            node->left = child;
            break;
        case Position::Right:
            node->right = child;
            break;
        default:
            assert(false);
            break;
        }

        if (child != m_nill) {
            child->parent = node;
        }
    }


    Position nodePosition(Node *node) const
    {
        assert(nullptr != node && nullptr != node->parent);

        return (node->parent->left == node) ? Position::Left : Position::Right;
    }


    void createChildNode(Node *parent, const PairInterval<ValueType> &interval, Position position)
    {
        assert(nullptr != parent);
        assert(childNode(parent, position) == m_nill);

        Node *child = new Node(interval, Color::Red, m_nill);
        setChildNode(parent, child, position);
        insertionFixNodeLimits(child);
        insertionFix(child);
        ++m_size;
    }


    void destroyNode(Node *node)
    {
        --m_size;
        delete node;
    }


    void updateNodeLimits(Node *node) const
    {
        assert(nullptr != node);

        Node *left = isNodeAboutToBeDestroyed(node->left) ? m_nill : node->left;
        Node *right = isNodeAboutToBeDestroyed(node->right) ? m_nill : node->right;

        auto lowest = (left != m_nill) ? left->lowest : node->intervals.front()->first.t;

        if(node->lowest != lowest){
            node->lowest = lowest;
        }

        auto highest = std::max({ left->highest, right->highest, node->high });

        if(node->highest != highest) {
            node->highest = highest;
        }
    }


    bool isNodeAboutToBeDestroyed(Node *node) const
    {
        assert(nullptr != node);

        return (node != m_nill && node->intervals.empty());
    }


    void subtreeIntervals(Node *node, Intervals &out) const
    {
        assert(nullptr != node);

        if (node == m_nill) {
            return;
        }

        subtreeIntervals(node->left, out);

        out.insert(out.end(), node->intervals.begin(), node->intervals.end());

        subtreeIntervals(node->right, out);
    }


    template <typename Callback>
    void subtreeOverlappingIntervals(Node *node, const PairInterval<ValueType> &interval, bool boundary, Callback &&callback) const
    {
        assert(nullptr != node);

        if (node == m_nill) {
            return;
        }

        if (node->left != m_nill
                && (boundary ? !(node->left->highest < interval->first.t) : interval->first.t < node->left->highest)) {
            subtreeOverlappingIntervals(node->left, interval, boundary, std::forward<Callback>(callback));
        }

        if (boundary ? !(interval->second.t < node->intervals.front()->first.t) : node->intervals.front()->first.t < interval->second.t) {
            for (auto it = node->intervals.rbegin(); it != node->intervals.rend(); ++it) {
                if (boundary ? !((*it)->second.t < interval->first.t) : interval->first.t < (*it)->second.t) {
                    callback(*it);
                } else {
                    break;
                }
            }

            subtreeOverlappingIntervals(node->right, interval, boundary, std::forward<Callback>(callback));
        }
    }


    template <typename Callback>
    void subtreeInnerIntervals(Node *node, const PairInterval<ValueType> &interval, bool boundary, Callback &&callback) const
    {
        assert(nullptr != node);

        if (node == m_nill) {
            return;
        }

        if (boundary ? !(node->intervals.front()->first.t < interval->first.t) : interval->first.t < node->intervals.front()->first.t) {
            subtreeInnerIntervals(node->left, interval, boundary, std::forward<Callback>(callback));
            for (auto it = node->intervals.begin(); it != node->intervals.end(); ++it) {
                if (boundary ? !(interval->second.t < (*it)->second.t) : (*it)->second.t < interval->second.t) {
                    callback(*it);
                } else {
                    break;
                }
            }
        }

        if (node->right != m_nill
                && (boundary ? !(interval->second.t < node->right->lowest) : node->right->lowest < interval->second.t)) {
            subtreeInnerIntervals(node->right, interval, boundary, std::forward<Callback>(callback));
        }
    }


    template <typename Callback>
    void subtreeOuterIntervals(Node *node, const PairInterval<ValueType> &interval, bool boundary, Callback &&callback) const
    {
        assert(nullptr != node);

        if (node == m_nill) {
            return;
        }

        if (node->left != m_nill
                && (boundary ? !(node->left->highest < interval->second.t) : interval->second.t < node->left->highest)) {
            subtreeOuterIntervals(node->left, interval, boundary, std::forward<Callback>(callback));
        }

        if (boundary ? !(interval->first.t < node->intervals.front()->first.t) : node->intervals.front()->first.t < interval->first.t) {
            for (auto it = node->intervals.rbegin(); it != node->intervals.rend(); ++it) {
                if (boundary ? !((*it)->second.t < interval->second.t) : interval->second.t < (*it)->second.t) {
                    callback(*it);
                } else {
                    break;
                }
            }

            subtreeOuterIntervals(node->right, interval, boundary, std::forward<Callback>(callback));
        }
    }


    template <typename Callback>
    void subtreeIntervalsContainPoint(Node *node, const PairInterval<ValueType> &point, bool boundary, Callback &&callback) const
    {
        assert(nullptr != node);

        if (node == m_nill) {
            return;
        }

        if (node->left != m_nill
                && (boundary ? !(node->left->highest < point) : point < node->left->highest)) {
            subtreeIntervalsContainPoint(node->left, point, boundary, std::forward<Callback>(callback));
        }

        if (boundary ? !(point < node->intervals.front()->first.t) : node->intervals.front()->first.t < point) {
            for (auto it = node->intervals.rbegin(); it != node->intervals.rend(); ++it) {
                if (boundary ? !((*it)->second.t < point) : point < (*it)->second.t) {
                    callback(*it);
                } else {
                    break;
                }
            }

            subtreeIntervalsContainPoint(node->right, point, boundary, std::forward<Callback>(callback));
        }
    }


    bool isNodeHasInterval(Node *node, const PairInterval<ValueType> &interval) const
    {
        assert(nullptr != node);

        return (node->intervals.cend() != std::find(node->intervals.cbegin(), node->intervals.cend(), interval));
    }


    unsigned findHighest(const Intervals &intervals) const
    {
        assert(!intervals.empty());

        auto it = std::max_element(intervals.cbegin(), intervals.cend(),
                                   [] (const PairInterval<ValueType> &lhs, const PairInterval<ValueType> &rhs) -> bool { return lhs->second.t < rhs->second.t; });

        assert(it != intervals.cend());

        return (*it)->second.t;
    }


    //bool isNotEqual(const ValueType &lhs, const ValueType &rhs) const
    //{
        //return (lhs < rhs || rhs < lhs);
    //}


    //bool isEqual(const ValueType &lhs, const ValueType &rhs) const
    //{
        //return !isNotEqual(lhs, rhs);
    //}


    void swapRelations(Node *node, Node *child)
    {
        assert(nullptr != node && nullptr != child);

        if (node->parent == m_nill) {
            if (child != m_nill) {
                child->parent = m_nill;
            }
            m_root = child;
        } else {
            setChildNode(node->parent, child, nodePosition(node));
        }
    }


    void rotateCommon(Node *node, Node *child) const
    {
        assert(nullptr != node && nullptr != child);
        assert(node != m_nill && child != m_nill);

        std::swap(node->color, child->color);

        updateNodeLimits(node);

        if (child->highest < node->highest) {
            child->highest = node->highest;
        }

        if (node->lowest < child->lowest) {
            child->lowest = node->lowest;
        }
    }


    Node *rotateLeft(Node *node)
    {
        assert(nullptr != node && nullptr != node->right);

        Node *child = node->right;
        swapRelations(node, child);
        setChildNode(node, child->left, Position::Right);
        setChildNode(child, node, Position::Left);
        rotateCommon(node, child);

        return child;
    }


    Node *rotateRight(Node *node)
    {
        assert(nullptr != node && nullptr != node->left);

        Node *child = node->left;
        swapRelations(node, child);
        setChildNode(node, child->right, Position::Left);
        setChildNode(child, node, Position::Right);
        rotateCommon(node, child);

        return child;
    }


    Node *rotate(Node *node)
    {
        assert(nullptr != node);

        switch (nodePosition(node)) {
        case Position::Left:
            return rotateRight(node->parent);
        case Position::Right:
            return rotateLeft(node->parent);
        default:
            assert(false);
            return nullptr;
        }
    }


    void insertionFixNodeLimits(Node *node)
    {
        assert(nullptr != node && nullptr != node->parent);

        while (node->parent != m_nill) {
            bool finish = true;

            if (node->parent->highest < node->highest) {
                node->parent->highest = node->highest;
                finish = false;
            }

            if (node->lowest < node->parent->lowest) {
                node->parent->lowest = node->lowest;
                finish = false;
            }

            if (finish) {
                break;
            }

            node = node->parent;
        }
    }


    void removeFixNodeLimits(Node *node, unsigned minRange = 0)
    {
        assert(nullptr != node && nullptr != node->parent);

        unsigned range = 0;
        while (node != m_nill) {
            bool finish = (minRange < range);

            updateNodeLimits(node);

            //if (isNotEqual(node->highest, node->parent->highest)) {
            if(node->highest != node->parent->highest){
                finish = false;
            }

            if(node->lowest != node->parent->lowest) {
                finish = false;
            }

            if (finish) {
                break;
            }

            node = node->parent;
            ++range;
        }
    }


    void insertionFix(Node *node)
    {
        assert(nullptr != node && nullptr != node->parent);

        while (Color::Red == node->color && Color::Red == node->parent->color) {
            Node *parent = node->parent;
            Node *uncle = siblingNode(parent);
            switch (uncle->color) {
            case Color::Red:
                uncle->color = Color::Black;
                parent->color = Color::Black;
                parent->parent->color = Color::Red;
                node = parent->parent;
                break;
            case Color::Black:
                if (nodePosition(node) != nodePosition(parent)) {
                    parent = rotate(node);
                }
                node = rotate(parent);
                break;
            default:
                assert(false);
                break;
            }
        }

        if (node->parent == m_nill && Color::Black != node->color) {
            node->color = Color::Black;
        }
    }


    void removeFix(Node *node)
    {
        assert(nullptr != node && nullptr != node->parent);

        while (Color::Black == node->color && node->parent != m_nill) {
            Node *sibling = siblingNode(node);
            if (Color::Red == sibling->color) {
                rotate(sibling);
                sibling = siblingNode(node);
            }

            assert(nullptr != sibling && nullptr != sibling->left && nullptr != sibling->right);
            assert(Color::Black == sibling->color);

            if (Color::Black == sibling->left->color && Color::Black == sibling->right->color) {
                sibling->color = Color::Red;
                node = node->parent;
            } else {
                if (Position::Left == nodePosition(sibling) && Color::Black == sibling->left->color) {
                    sibling = rotateLeft(sibling);
                } else if (Position::Right == nodePosition(sibling) && Color::Black == sibling->right->color) {
                    sibling = rotateRight(sibling);
                }
                rotate(sibling);
                node = siblingNode(node->parent);
            }
        }

        if (Color::Black != node->color) {
            node->color = Color::Black;
        }
    }

    Node* findMin()const{
        auto current(m_root);
        auto parent(m_root);
        while(current != m_nill){
            parent=current;
            current=current->left;
        }
        return parent;
    }


    unsigned m_size;
};

template <typename ValueType>
class PairIntervalCIter
{
private:
    typename PairTree<ValueType>::Node *node;
    PairTree<ValueType> &t;
    unsigned offset;

public:
    PairIntervalCIter(typename PairTree<ValueType>::Node *n, PairTree<ValueType> &tree, unsigned o) : node(n), t(tree), offset(o) {}
    bool operator==(PairIntervalCIter const &itr)
    {
        return node == itr.node && offset == itr.offset;
    }
    bool operator!=(PairIntervalCIter const &itr)
    {
        return node != itr.node || offset != itr.offset;
    }
    PairIntervalCIter &operator++()
    {
        if (offset == node->intervals.size() - 1)
        {
            typename PairTree<ValueType>::Node *p;
            if (node == t.m_nill)
            {
                // ++ from end(). get the root of the tree
                node = t.m_root;

                // error! ++ requested for an empty tree
                //if (node == t.m_nill)
                    //throw UnderflowException{};

                // move to the smallest value in the tree,
                // which is the first node inorder
                while (node->left != t.m_nill)
                {
                    node = node->left;
                }
            }
            else if (node->right != t.m_nill)
            {
                // successor is the farthest left node of
                // right subtree
                node = node->right;

                while (node->left != t.m_nill)
                {
                    node = node->left;
                }
            }
            else
            {
                // have already processed the left subtree, and
                // there is no right subtree. move up the tree,
                // looking for a parent for which node is a left child,
                // stopping if the parent becomes NULL. a non-NULL parent
                // is the successor. if parent is NULL, the original node
                // was the last node inorder, and its successor
                // is the end of the list
                p = node->parent;
                while (p != t.m_nill && node == p->right)
                {
                    node = p;
                    p = p->parent;
                }

                // if we were previously at the right-most node in
                // the tree, node = t.m_nill, and the iterator specifies
                // the end of the list
                node = p;
            }
            offset=0;
        }
        else
        {
            ++offset;
        }
        return *this;
    }
    PairInterval<ValueType> & operator*(){ return node->intervals[offset]; }
    PairInterval<ValueType> & operator->(){ return node->intervals[offset]; }
};

template <typename ValueType>
void swap(PairTree<ValueType> &lhs, PairTree<ValueType> &rhs)
{
    lhs.swap(rhs);
}

#endif // INTERVALTREE_H
