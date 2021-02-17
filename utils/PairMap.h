// A datastructure for storing n choose 2 pairs
// Copyright thayne Walker 11/27/2020
#ifndef PAIRMAP_H
#define PAIRMAP_H

template <typename T>
struct PairMap{
  PairMap():n(0){}
  PairMap(unsigned num):n(num),vals(n*(n-1)/2){
  }
  void resize(unsigned num){
    n=num;
    vals.resize(n*(n-1)/2);
  }
  T const& get(unsigned i, unsigned j)const{
    if(i>j)
      return vals[(n*(n-1)/2) - (n-j)*((n-j)-1)/2 + i - j - 1];
    return vals[(n*(n-1)/2) - (n-i)*((n-i)-1)/2 + j - i - 1];
  }
  template <typename I>
  T const& get(std::vector<I> const& v)const{
    return get(v[0],v[1]);
  }
  void set(unsigned i, unsigned j, T const& val){
    if(i>j)
      vals[(n*(n-1)/2) - (n-j)*((n-j)-1)/2 + i - j - 1] = val;
    else
      vals[(n*(n-1)/2) - (n-i)*((n-i)-1)/2 + j - i - 1] = val;
  }
  template <typename I>
  void set(std::vector<I> const& v, T const& val){
    set(v[0],v[1],val);
  }
  void setAll(T const& v){
    vals.assign(v);
  }
  void clear(){
    vals.resize(0);
    vals.resize(n);
  }
  // Reverse operation (in case needed later.)
  // i = n - 2 - floor(sqrt(-8*k + 4*n*(n-1)-7)/2.0 - 0.5)
  // j = k + i + 1 - n*(n-1)/2 + (n-i)*((n-i)-1)/2
  unsigned n;
  std::vector<T> vals;
};

#endif