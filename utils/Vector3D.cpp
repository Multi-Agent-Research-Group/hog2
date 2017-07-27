#include "Vector3D.h"

std::ostream& operator <<(std::ostream & out, Vector3D const& v)
{
  out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return out;
}

