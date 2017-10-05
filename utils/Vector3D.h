/**
* This file is part of HOG2.
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <math.h>
#include "FPUtil.h"
#include <ostream>

class Vector3D {
  public:
    Vector3D(float _x,float _y,float _z):x(_x),y(_y),z(_z){}
    Vector3D():x(0),y(0),z(0){}
    inline void Set(float _x, float _y, float _z) { x=_x; y=_y; z=_z; }

    inline bool operator==(const Vector3D &rhs)const{return (fequal(x,rhs.x)&&fequal(y,rhs.y)&&fequal(z,rhs.z));}
    inline bool operator<(const Vector3D &rhs)const{return fequal(x,rhs.x)?(fequal(y,rhs.y)?fless(z,rhs.z):fless(y,rhs.y)):fless(x,rhs.x);}

    // Dot product
    inline double operator*(Vector3D const& other)const{return x * other.x + y * other.y + z * other.z;}
    // Negation
    inline Vector3D operator-(Vector3D const& other)const{return Vector3D(x-other.x,y-other.y,z-other.z);}
    inline void operator-=(Vector3D const& other){x-=other.x;y-=other.y;z-=other.z;}
    inline Vector3D operator-()const{return Vector3D(-x,-y,-z);}
    // Slope angle of this vector
    //inline float atan(){ return atan2(y,x); }
    // Square
    inline float sq(){ return (*this) * (*this); }
    inline float len(){ return sqrt(sq()); }

    inline Vector3D operator/(const double num)const{return Vector3D(x/num, y/num, z/num);}

    inline Vector3D operator*(const double num)const{return Vector3D(x*num, y*num, z*num);}

    inline Vector3D operator+(const Vector3D& v2)const{return Vector3D(x+v2.x, y+v2.y, z+v2.z);}

    inline void operator+=(const Vector3D& v2){x+=v2.x; y+=v2.y;z+=v2.z;}
    inline void operator *=(double s) { x*=s; y*=s; z*=s; }
    inline void operator /=(double s) { x/=s; y/=s; z/=s; }

    inline void Normalize(){
      if(x==0&&y==0&&z==0) return;
      float magnitude(len());
      x /= magnitude;
      y /= magnitude;
      z /= magnitude;
    }

    float x, y, z;
};

// Calculate cross product of vectors
inline Vector3D cross(Vector3D const& v1, Vector3D const& v2){ return Vector3D((v1.y*v2.z)-(v1.z*v2.y),(v1.z*v2.x)-(v1.x*v2.z),(v1.x*v2.y)-(v1.y*v2.x));}

std::ostream& operator <<(std::ostream & out, Vector3D const& v);

#endif
