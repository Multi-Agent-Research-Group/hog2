#include "Vector2D.h"

std::ostream& operator <<(std::ostream & out, Vector2D const& v)
{
  out << "(" << v.x << ", " << v.y << ")";
  return out;
}

