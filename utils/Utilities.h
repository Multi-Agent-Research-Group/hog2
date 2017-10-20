#ifndef UTILITIES_H
#define UTILITIES_H

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <sys/resource.h>

namespace Util{
  // Set upper limit on memory consumption
  void setmemlimit(unsigned megabytes)
  {
    struct rlimit memlimit;

    megabytes*=(1024*1024);
    memlimit.rlim_cur = memlimit.rlim_max = megabytes;
    setrlimit(RLIMIT_AS, &memlimit);
  }

  // swaps two values
  template <typename T>
    void swap(T* a , T*b)
    {
      T temp(*a);
      *a = *b;
      *b = temp;
    }

  //returns integer part of a floating point number
  int iPartOfNumber(float x)
  {
    return (int)x;
  }

  //returns fractional part of a number
  float fPartOfNumber(float x)
  {
    if (x>0) return x - iPartOfNumber(x);
    else return x - (iPartOfNumber(x)+1);

  }

  //returns 1 - fractional part of number
  float rfPartOfNumber(float x)
  {
    return 1 - fPartOfNumber(x);
  }

  template<typename Out>
  void split(const std::string &s, char delim, Out result);
  std::vector<std::string> split(const std::string &s, char delim);
};

template<typename Out>
void Util::split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> Util::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

#endif
