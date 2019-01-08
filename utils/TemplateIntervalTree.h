#ifndef __INTERVAL_TREE_H
#define __INTERVAL_TREE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>

#include "FPUtil.h"

template <class T, typename K = std::size_t>
class TemplateInterval {
public:
    K start;
    K stop;
    T value;
    TemplateInterval(K s, K e, const T& v)
        : start(s)
        , stop(e)
        , value(v)
    { }
    bool operator==(TemplateInterval<T,K> other)const{return fequal(other.start,start) && fequal(other.stop,stop) && other.value == value;}
};

template <class T, typename K>
K intervalStart(const TemplateInterval<T,K>& i) {
    return i.start;
}

template <class T, typename K>
K intervalStop(const TemplateInterval<T,K>& i) {
    return i.stop;
}

template <class T, typename K>
  std::ostream& operator<<(std::ostream& out, TemplateInterval<T,K>& i) {
    out << "Interval(" << i.start << ", " << i.stop << "): " << i.value;
    return out;
}

template <class T, typename K = std::size_t>
class TemplateIntervalStartSorter {
public:
    bool operator() (const TemplateInterval<T,K>& a, const TemplateInterval<T,K>& b) {
        return a.start < b.start;
    }
};

template <class T, typename K = std::size_t>
class TemplateIntervalTree {

  public:
    typedef TemplateInterval<T,K> interval;
    typedef std::vector<interval> intervalVector;
    typedef TemplateIntervalTree<T,K> intervalTree;

  protected:
    TemplateIntervalTree<T,K>( interval const& val)
      : left(nullptr)
        , right(nullptr)
        , center(val.start)
        , dp(1)
    {
      intervals.push_back(val);
    }

    // Perform insertion from an ordered vector in preorder precedence
    unsigned insert_preorder(intervalVector a, unsigned d=0){
      if(a.size()==1){
        insert(a[0]);
        return 1;
      }
      size_t const half_size(a.size() / 2);
      intervalVector split_lo(a.begin(), a.begin() + half_size);
      intervalVector split_hi(a.begin() + half_size, a.end());
      insert(a[half_size]);
      dp = std::max(insert_preorder(split_lo,d+1),
                    insert_preorder(split_hi,d+1));
      return dp+1;
    }

  private:
    std::unique_ptr<intervalTree> copyTree(const intervalTree& orig){
      return std::unique_ptr<intervalTree>(new intervalTree(orig));
    }
    intervalVector intervals;
    std::unique_ptr<intervalTree> left;
    std::unique_ptr<intervalTree> right;
    K center;
    unsigned dp; // depth

  public:
    inline unsigned empty()const{return intervals.empty();}
    inline unsigned depth()const{return dp;}

    TemplateIntervalTree<T,K>(void)
      : left(nullptr)
        , right(nullptr)
        , center(0)
        , dp(0)
    { }



    TemplateIntervalTree<T,K>(const intervalTree& other)
      :intervals(other.intervals),
      left(other.left ? copyTree(*other.left) : nullptr),
      right(other.right ? copyTree(*other.right) : nullptr),
      center(other.center),
      dp(other.dp)
    { }

    TemplateIntervalTree<T,K>& operator=(const intervalTree& other) {
      center = other.center;
      intervals = other.intervals;
      left = other.left ? copyTree(*other.left) : nullptr;
      right = other.right ? copyTree(*other.right) : nullptr;
      dp = other.dp;
      return *this;
    }

    // Note: Assumes inputs are in sorted order... will work
    // if intervals are not in order, but the tree will not be balanced.
    TemplateIntervalTree<T,K>( intervalVector const& ivals)
      : left(nullptr)
      , right(nullptr)
      , dp(insert_preorder(ivals))
    { }

    void insert(K const& s, K const& e, T const& val)
    {
      insert(interval(s,e,val));
    }

    void insert(interval const& interval){
      if(intervals.empty()) {
        center=interval.start;
        intervals.push_back(interval);
      } else if (interval.stop < center) {
        if(left){
          left->insert(interval);
        }else{
          left = std::unique_ptr<intervalTree>(new intervalTree(interval));
        }
      } else if (interval.start > center) {
        if(right){
          right->insert(interval);
        }else{
          right = std::unique_ptr<intervalTree>(new intervalTree(interval));
        }
      } else {
        intervals.push_back(interval);
      }
    }

    bool remove(K const& s, K const& e, T const& val) {
      interval interval(s,e,val);
      if (interval.stop < center) {
        if(left){return left->remove(s,e,val);}
      } else if (interval.start > center) {
        if(right){return right->remove(s,e,val);}
      } else {
        auto it = std::find(intervals.begin(), intervals.end(), interval);

        if (it != intervals.end()) {
          using std::swap;

          // swap the one to be removed with the last element
          // and remove the item at the end of the container
          // to prevent moving all items after the value by one
          swap(*it, intervals.back());
          intervals.pop_back();
          // Can't update depth easily so we won't
          return true;
        }
      }
      return false;
    }

    std::vector<T> findOverlapping(K start, K stop) const {
      std::vector<T> ov;
      this->findOverlapping(start, stop, ov);
      return ov;
    }

    void findOverlapping(K start, K stop, std::vector<T>& overlapping) const {
      if (!intervals.empty() && ! (stop < intervals.front().start)) {
        for (typename intervalVector::const_iterator i = intervals.begin(); i != intervals.end(); ++i) {
          const interval& interval = *i;
          if (interval.stop >= start && interval.start <= stop) {
            overlapping.push_back(interval.value);
          }
        }
      }

      if (left && start <= center) {
        left->findOverlapping(start, stop, overlapping);
      }

      if (right && stop >= center) {
        right->findOverlapping(start, stop, overlapping);
      }

    }

    intervalVector findContained(K start, K stop) const {
      intervalVector contained;
      this->findContained(start, stop, contained);
      return contained;
    }

    void findContained(K start, K stop, intervalVector& contained) const {
      if (!intervals.empty() && ! (stop < intervals.front().start)) {
        for (typename intervalVector::const_iterator i = intervals.begin(); i != intervals.end(); ++i) {
          const interval& interval = *i;
          if (interval.start >= start && interval.stop <= stop) {
            contained.push_back(interval);
          }
        }
      }

      if (left && start <= center) {
        left->findContained(start, stop, contained);
      }

      if (right && stop >= center) {
        right->findContained(start, stop, contained);
      }

    }

    void clear(){
      left.reset(); // Should cascade
      right.reset(); // Should cascade
      intervals.clear();
      dp=0;
    }

    ~TemplateIntervalTree(void) = default;

};

#endif

