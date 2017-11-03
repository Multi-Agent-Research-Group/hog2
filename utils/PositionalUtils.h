/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
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
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef PositionalUtils_h_
#define PositionalUtils_h_

#include <cmath>
#include <stdlib.h>
#include "Vector2D.h"
#include "Vector3D.h"
#include <limits.h>

namespace Util {
// Get the straight line distance from a to b
inline double distance(double dx, double dy){ // x and y differences
  return sqrt(dx*dx+dy*dy);
}

// Get the straight line distance from a to b
inline double distance(double x1, double y1, double x2, double y2){
  return distance(y1-y2,x1-x2);
}

// Get the straight line distance from a to b
inline double distance(double dx, double dy, double dz){ // x, y and z differences
  return sqrt(dx*dx+dy*dy+dz*dz);
}

// Get the straight line distance from a to b
inline double distance(double x1, double y1, double z1, double x2, double y2, double z2){
  return distance(y1-y2,x1-x2,z1-z2);
}

inline double distance(Vector2D const& A, Vector2D const& B){
  return distance(A.x,A.y,B.x,B.y);
}

inline double distance(Vector3D const& A, Vector3D const& B){
  return distance(A.x,A.y,A.z,B.x,B.y,B.z);
}

// Get the heading from a to b (clockwise from north)
template<unsigned steps360>
inline unsigned heading(double x1, double y1, double x2, double y2){
  return unsigned(round((M_PI/2.0-atan2(y2-y1,x2-x1))*(steps360/2/M_PI)+steps360))%steps360;
}

template<unsigned steps360>
inline signed angle(double x1, double y1, double x2, double y2){
  return signed(round(atan2(y2-y1,x2-x1)*(steps360/2/M_PI)));
}

// Get the non-directional difference in angles
template<unsigned steps360>
inline unsigned angleDiff(unsigned a, unsigned b){
  unsigned d(abs(a-b)%steps360);
  return d>(steps360/2)?steps360-d:d;
}

// Get the difference from angle a to angle b
template<unsigned steps360>
inline signed relativeAngleDiff(unsigned a, unsigned b){
  signed d(b-a);
  return d>(steps360/2)?-(steps360-d):d;
}

// Angular difference between heading from a to b and "heading"
template<unsigned steps360>
inline unsigned headingDiff(double x1, double y1, double x2, double y2, double hdg){
  return angleDiff<steps360>(heading<steps360>(x1, y1, x2, y2), hdg);
}

// Difference between heading from a to b and "heading"
template<unsigned steps360>
inline signed relativeHeadingDiff(double x1, double y1, double x2, double y2, double hdg){
  return relativeAngleDiff<steps360>(heading<steps360>(x1, y1, x2, y2), hdg);
}

template<unsigned steps360>
inline double steps2Rad(signed steps){return steps/double(steps360)*360.*M_PI/180.;}

template<unsigned steps360>
inline double steps2deg(signed steps){return steps/double(steps360)*360.;}

inline double distanceOfPointToLine(Vector2D v, Vector2D w, Vector2D const& p){
  const double l2((v-w).sq());  // i.e. |w-v|^2 -  avoid a sqrt
  if (fequal(l2,0.0)) return distance(p, v);   // v == w case
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  // We clamp t from [0,1] to handle points outside the segment vw.
  const double t(max(0.0, min(1.0, ((p - v)*(w - v)) / l2)));
  //const double T(((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2);
  //const Vector2D projection((v + t) * (w - v));  // Projection falls on the segment
  w-=v;
  w*=t;
  v+=w; // piecewise multiply
  return distance(p, v);
  //double dy(a.y-b.y);
  //double dx(a.x-b.x);
  //return fabs(dy*x.x-dx-x.y+a.x*b.y-a.y*b.x)/sqrt(dy*dy+dx*dx);
}

// Get the coordinates of a secant line cut by a circle at center c with radius r
inline std::pair<Vector2D,Vector2D> secantLine(Vector2D a, Vector2D b, Vector2D const& c, double r){
  // Translate a and b as if c were (0,0)
  a-=c;
  b-=c;
  double dy(a.y-b.y);
  double dx(a.x-b.x);
  double dr(sqrt(dx*dx+dy*dy));
  double drsq(dr*dr);
  double det(b.x*a.y-a.x*b.y);
  double signOfY(dy<0?-1:1);
  double rsq(r*r);
  double dscr(rsq*drsq-det*det);
  if(fgreater(dscr,0.0)){
    double sqrtdscr(sqrt(dscr));
    double w1(det*dy);
    double w2(signOfY*dx*sqrtdscr);
    double z1(-det*dx);
    double z2(abs(dy)*sqrtdscr);
    double x1((w1+w2)/drsq);
    double x2((w1-w2)/drsq);
    if(fless(x2,x1)){double tmp(x1);x1=x2;x2=tmp;} // Swap
    double y1((z1+z2)/drsq);
    double y2((z1-z2)/drsq);
    if(fless(y2,y1)){double tmp(y1);y1=y2;y2=tmp;} // Swap
    Vector2D p1(fless(a.x,b.x)?x1:x2,fless(a.y,b.y)?y1:y2);
    Vector2D p2(fless(a.x,b.x)?x2:x1,fless(a.y,b.y)?y2:y1);
    
    bool aIn(false);
    bool bIn(false);
    if(fleq(a.x*a.x+a.y*a.y,rsq)){
      p1=a;
      aIn=true;
    }
    if(fleq(b.x*b.x+b.y*b.y,rsq)){
      p2=b;
      bIn=true;
    }
      // Check whether the line segment actually goes across the circle or not
    if(!aIn && !bIn && fgreater(distance(p1,p2),distance(a,b))) return std::make_pair(Vector2D(0,0),Vector2D(0,0));
    return std::make_pair(p1+c,p2+c);
  }
  return std::make_pair(Vector2D(0,0),Vector2D(0,0)); // Indicates no secant line, or tangent point only...
}

inline double meanDistanceOfPointToLine(Vector2D const& a, Vector2D const& b, Vector2D const& x){
  // From wolfram alpha: Integrate[Sqrt[(x - a)^2 + b^2], x] =>
  // ((-a + x)*(a^2 + b^2 - 2*a*x + x^2) - b^2*Sqrt[a^2 + b^2 - 2*a*x + x^2]* Log[2*(a + Sqrt[b^2 + (a - x)^2] - x)])/ (2*Sqrt[a^2 + b^2 - 2*a*x + x^2]) 
  //double asq(a.sq());
  //double bsq(b.sq());
  //double xsq(x.sq());
  //double dax(a - x);
  //return (dax*(asq + bsq - 2*a*x + xsq) - bsq*sqrt(asq + bsq - 2*a*x + xsq)* log(2*(a + sqrt(bsq + dax.sq()) - x)))/ (2*sqrt(asq + bsq - 2*a*x + xsq)) 
  // Need to detect perfect alignment...
  //
  // X       A--B
  //
  // This case will cause div/0 error
  if(fequal(0,headingDiff<USHRT_MAX>(x.x,x.y,b.x,b.y,heading<USHRT_MAX>(x.x,x.y,a.x,a.y))))
    return distance(x,(a+b)/2.);
  Vector2D dax(a-x);
  Vector2D dba(b-a);
  double K1(dax.sq());
  double K2(2*(dba*dax));
  double K3(dba.sq());
  double L1(sqrt(K3*(K1+K2+K3)));
  double L2(sqrt(K3*K1));
  return (4.*K3*L1 + 2.*K2*(L1-L2) + (K2*K2-4.*K1*K3)*log((K2+2.*L2)/(2.*K3+K2+2.*L1)))/(8.*pow(K3,1.5));
}

// From http://www.cplusplus.com/forum/beginner/49408/

double linesIntersect(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, double* out=nullptr);
bool intersectionPoint(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, Vector2D& out);
// Assume "rounded" line with radius (thus the line width is 2*r)
bool fatLinesIntersect(Vector2D const& A1, Vector2D const& A2, double r1, Vector2D const& B1, Vector2D const& B2, double r2);
float closestDistanceBetweenLineSegments(Vector3D const& s1, Vector3D const& d1, Vector3D const& s2, Vector3D const& d2);
bool fatLinesIntersect(Vector3D const& A1, Vector3D const& A2, double r1, Vector3D const& B1, Vector3D const& B2, double r2);

};

#endif
