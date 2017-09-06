#ifndef UTILITIES_H
#define UTILITIES_H

namespace Util{

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
};

#endif
