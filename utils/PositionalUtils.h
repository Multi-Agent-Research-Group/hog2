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
#include <deque>
#include <algorithm>
#include <memory>

namespace Util {

inline double distanceSquared(double dx, double dy){ // x and y differences
  return dx*dx+dy*dy;
}

// Get the straight line distance from a to b
inline double distance(double dx, double dy){ // x and y differences
  return sqrt(dx*dx+dy*dy);
}

inline double distanceSquared(double x1, double y1, double x2, double y2){
  return distanceSquared(y1-y2,x1-x2);
}

// Get the straight line distance from a to b
inline double distance(double x1, double y1, double x2, double y2){
  return distance(y1-y2,x1-x2);
}

inline double distanceSquared(double dx, double dy, double dz){ // x, y and z differences
  return dx*dx+dy*dy+dz*dz;
}

// Get the straight line distance from a to b
inline double distance(double dx, double dy, double dz){ // x, y and z differences
  return sqrt(dx*dx+dy*dy+dz*dz);
}

inline double distanceSquared(double x1, double y1, double z1, double x2, double y2, double z2){
  return distanceSquared(y1-y2,x1-x2,z1-z2);
}

// Get the straight line distance from a to b
inline double distance(double x1, double y1, double z1, double x2, double y2, double z2){
  return distance(y1-y2,x1-x2,z1-z2);
}

template <typename state>
inline double distanceSquared(state const& A, state const& B){
  return distanceSquared(A.x,A.y,B.x,B.y);
}
template <typename state>
inline double distance(state const& A, state const& B){
  return sqrt(distanceSquared(A.x,A.y,B.x,B.y));
}

template <typename state>
inline double distance3dSquared(state const& A, state const& B){
  return distanceSquared(A.x,A.y,A.z,B.x,B.y,B.z);
}

template <typename state>
inline double distance3d(state const& A, state const& B){
  return sqrt(distanceSquared(A.x,A.y,A.z,B.x,B.y,B.z));
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
  unsigned d((a>b?a-b:b-a)%steps360);
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

inline double sqDistanceOfPointToLine(Vector2D v, Vector2D w, Vector2D const& p){
  const double l2((v-w).sq());  // i.e. |w-v|^2 -  avoid a sqrt
  if (fequal(l2,0.0)) return distanceSquared(p, v);   // v == w case
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
  return distanceSquared(p, v);
  //double dy(a.y-b.y);
  //double dx(a.x-b.x);
  //return fabs(dy*x.x-dx-x.y+a.x*b.y-a.y*b.x)/sqrt(dy*dy+dx*dx);
}

inline double distanceOfPointToLine(Vector2D v, Vector2D w, Vector2D const& p){
  return sqrt(sqDistanceOfPointToLine(v,w,p));
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

template <typename state>
bool linesIntersect(state const& A1, state const& A2, state const& B1, state const& B2, double* out=nullptr);
template <typename state>
bool intersectionPoint(state const& A1, state const& A2, state const& B1, state const& B2, state& out);
// Assume "rounded" line with radius (thus the line width is 2*r)
float closestDistanceBetweenLineSegments(Vector3D const& s1, Vector3D const& d1, Vector3D const& s2, Vector3D const& d2);
float closestDistanceBetweenLineSegments(Vector2D const& s1, Vector2D const& d1, Vector2D const& s2, Vector2D const& d2);
template <typename state>
bool fatLinesIntersect(state const& A1, state const& A2, double r1, state const& B1, state const& B2, double r2);
template <typename state>
bool pointInPoly(std::vector<state> const& poly, state const& p);
template <typename state>
bool lineIntersectsPoly(std::vector<state> const& poly, state const& p1, state const& p2);
template <typename state>
void convexHull(std::vector<state> points, std::vector<Vector2D>& hull);
template <typename state>
bool sat(std::vector<state>const& a, std::vector<state>const& b, double radius);
};

// From http://www.cplusplus.com/forum/beginner/49408/

template <typename state>
bool Util::linesIntersect(state const& A1, state const& A2, state const& B1, state const& B2, double* out){
  state a(A2-A1);
  state b(B2-B1);

  double f(det(a,b));
  if(!f)      // lines are parallel
    return A1==B1||A1==B2||A2==B1||A2==B2?true:false;

  state c(B2-A2);
  double aa(det(a,c));
  double bb(det(b,c));

  if(f < 0)
  {
    if(aa > 0)     return false;
    if(bb > 0)     return false;
    if(aa < f)     return false;
    if(bb < f)     return false;
  }
  else
  {
    if(aa < 0)     return false;
    if(bb < 0)     return false;
    if(aa > f)     return false;
    if(bb > f)     return false;
  }

  if(out)
    *out = 1.0 - (aa / f);
  return true;
}

template <typename state>
bool Util::intersectionPoint(state const& A1, state const& A2, state const& B1, state const& B2, state& out){
  double pct(0);
  if(linesIntersect(A1,A2,B1,B2,&pct)){
    out=((B2 - B1) * pct) + B1;
    return true;
  }
  return false;
}

// Assume "rounded" line with radius (thus the line width is 2*r)
/*template <typename state>
bool Util::fatLinesIntersect(state const& A1, state const& A2, double r1, state const& B1, state const& B2, double r2){
  // If A and B intersect, we're done
  if(linesIntersect(A1,A2,B1,B2)){return true;}

  state NA(normal(A1,A2)*r1); // Normal of line from A1 to A2 
  state NB(normal(B1,B2)*r2); // Normal of line from B1 to B2 
  NA.Normalize();
  NA*=r1;
  NB.Normalize();
  NB*=r2;

  // If A and B are parallel, then we can just check the distance
  double r(r1+r2);
  if((NA==NB || NA==-NB) && fless(distanceOfPointToLine(A1,A2,B1),r)){return true;}

  // Project along normal in both directions
  state A11(A1+NA);
  state A21(A2+NA);
  state A12(A1-NA);
  state A22(A2-NA);

  state B11(B1+NB);
  state B21(B2+NB);
  state B12(B1-NB);
  state B22(B2-NB);

  if(linesIntersect(A11,A21,B11,B21)){return true;}
  if(linesIntersect(A11,A21,B12,B22)){return true;}
  if(linesIntersect(A12,A22,B11,B21)){return true;}
  if(linesIntersect(A12,A22,B12,B22)){return true;}

  // Finally, check endpoints
  if(fless(distanceOfPointToLine(A11,A21,B1),r)){return true;}
  if(fless(distanceOfPointToLine(A12,A22,B1),r)){return true;}
  if(fless(distanceOfPointToLine(A11,A21,B2),r)){return true;}
  if(fless(distanceOfPointToLine(A12,A22,B2),r)){return true;}
  if(fless(distanceOfPointToLine(B11,B21,A1),r)){return true;}
  if(fless(distanceOfPointToLine(B12,B22,A1),r)){return true;}
  if(fless(distanceOfPointToLine(B11,B21,A2),r)){return true;}
  if(fless(distanceOfPointToLine(B12,B22,A2),r)){return true;}

  return false;
}*/

/*float Util::closestDistanceBetweenLineSegments(Vector2D const& s1, Vector2D const& d1, Vector2D const& s2, Vector2D const& d2)
{
  Vector2D u(d1-s1);
  Vector2D v(d2-s2);
  Vector2D w(s1-s2);
  float a(u*u);         // always >= 0
  float b(u*v);
  float c(v*v);         // always >= 0
  float d(u*w);
  float e(v*w);
  float D(a*c - b*b);        // always >= 0
  float sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
  float tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

  // Compute the line parameters of the two closest points
  if (D < TOLERANCE) { // the lines are almost parallel
    sN = 0.0;         // force using point s1
    sD = 1.0;         // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  } else {                 // get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    } else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a){
      sN = sD;
    } else {
      sN = -d;
      sD = a;
    }
  } else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0) {
      sN = 0;
    } else if ((-d + b) > a) {
      sN = sD;
    } else {
      sN = (-d +  b);
      sD = a;
    }
  }
  // finally do the division to get sc and tc
  sc = (abs(sN) < TOLERANCE ? 0.0 : sN / sD);
  tc = (abs(tN) < TOLERANCE ? 0.0 : tN / tD);

  // get the difference of the two closest points
  Vector2D dP(w + (u*sc) - (v*tc));  // =  S1(sc) - S2(tc)

  return dP.len();   // return the closest distance
}*/

template <typename state>
bool Util::fatLinesIntersect(state const& A1, state const& A2, double r1, state const& B1, state const& B2, double r2){
  return fleq(closestDistanceBetweenLineSegments(A1,A2,B1,B2),r1+r2);
}

// For convex polygons only...
// Also assumes that no polygon extends beyond 2^12 (4096) on the x axis
template <typename state>
bool Util::pointInPoly(std::vector<state> const& poly, state const& p1){
  unsigned crossings(0);
  state endpoint(state(p1.x+0xfff,p1.y));
  crossings+=linesIntersect(poly.front(),poly.back(),p1,endpoint);
  for(unsigned i(1); i<poly.size()&&crossings<2; ++i){
    crossings+=linesIntersect(poly[i-1],poly[i],p1,endpoint);
  }
  return crossings%2;
}

template <typename state>
bool Util::lineIntersectsPoly(std::vector<state> const& poly, state const& p1, state const& p2){
  if(linesIntersect(poly.front(),poly.back(),p1,p2))return true;
  for(unsigned i(1); i<poly.size(); ++i){
    if(linesIntersect(poly[i-1],poly[i],p1,p2))return true;
  }
  return false;
}

template <typename state>
struct privateUtils{
  static std::unique_ptr<state> p0;

  // A utility function to find next to top in a stack
  static state nextToTop(std::deque<state> &S)
  {
    return S[S.size()-2];
  }

  // A utility function to swap two points
  static void swap(state &p1, state &p2)
  {
    state temp = p1;
    p1 = p2;
    p2 = temp;
  }

  // A utility function to return square of distance
  // between p1 and p2
  static double distSq(state const& p1, state const& p2)
  {
    return (p1.x - p2.x)*(p1.x - p2.x) +
      (p1.y - p2.y)*(p1.y - p2.y);
  }

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  static int orientation(state const& p, state const& q, state const& r)
  {
    double val = (q.y - p.y) * (r.x - q.x) -
      (q.x - p.x) * (r.y - q.y);

    //if (val == 0) return 0;  // collinear
    return val?((val > 0)? 1: 2):0; // clock or counterclock wise
  }

  // A function used by library function qsort() to sort an array of
  // points with respect to the first point
  static bool compare(state const& p1, state const& p2){
    // Find orientation
    int o = orientation(*p0, p1, p2);
    if (o == 0)
      return (distSq(*p0, p2) > distSq(*p0, p1))? true : false;

    return (o == 2)? true: false;
  }

  static bool contains(double n, state const& range, double doubleRadius){
    return (fleq(range.min()-doubleRadius, n) && fleq(n-doubleRadius, range.max()));
  }

  static bool overlap(state const& a, state const& b, double doubleRadius){
    if (contains(a.x,b,doubleRadius)) return true;
    if (contains(a.y,b,doubleRadius)) return true;
    if (contains(b.x,a,doubleRadius)) return true;
    if (contains(b.y,a,doubleRadius)) return true;
    return false;
  }

  static bool sat(std::vector<state>const& pa, std::vector<state>const& pb, state const& a, state const& b, double doubleRadius){
    state axis((b-a).perp());
    axis.Normalize();
    state ppa(axis.projectPolyOntoSelf(pa));
    state ppb(axis.projectPolyOntoSelf(pb));
    return overlap(ppa,ppb,doubleRadius);
  }
};

template <typename state>
std::unique_ptr<state> privateUtils<state>::p0(new state);

// Graham Scan
// Prints convex hull of a set of n points.
template <typename state>
void Util::convexHull(std::vector<state> points, std::vector<Vector2D>& hull){
  if (points.size() < 3){
    hull.reserve(points.size());
    for(auto const& p: points){
      hull.push_back(Vector2D(p));
    }
    return;
  }
   // Find the bottommost point
   int ymin = points[0].y, min = 0;
   for (int i = 1; i < points.size(); i++)
   {
     //std::cout << points[i] << "\n";
     int y = points[i].y;
 
     // Pick the bottom-most or chose the left
     // most point in case of tie
     if ((y < ymin) || (ymin == y &&
         points[i].x < points[min].x))
        ymin = points[i].y, min = i;
   }
 
   // Place the bottom-most point at first position
   privateUtils<state>::swap(points[0], points[min]);
 
   // Sort n-1 points with respect to the first point.
   // A point p1 comes before p2 in sorted ouput if p2
   // has larger polar angle (in counterclockwise
   // direction) than p1
   privateUtils<state>::p0.reset(new state(points[0]));
   std::sort(points.begin()+1,points.end(),
       [](state const& a, state const& b) -> bool {
       return privateUtils<state>::compare(a,b);
       });
 
   // If two or more points make same angle with p0,
   // Remove all but the one that is farthest from p0
   // Remember that, in above sorting, our criteria was
   // to keep the farthest point at the end when more than
   // one points have same angle.
   int m = 1; // Initialize size of modified array
   for (int i=1; i<points.size(); i++)
   {
       // Keep removing i while angle of i and i+1 is same
       // with respect to p0
       while (i < points.size()-1 && privateUtils<state>::orientation(*privateUtils<state>::p0, points[i],
                                    points[i+1]) == 0)
          i++;
 
 
       points[m] = points[i];
       m++;  // Update size of modified array
   }
 
   // If modified array of points has less than 3 points,
   // convex hull is not possible, but we may have a line
   // or point... return whatever we have.
   if (m < 3){
     for(int i(0);i<m; ++i)
       hull.push_back(Vector2D(points[i]));
     return;
   }
 
   // Create an empty stack and push first three points
   // to it.
   std::deque<state> S;
   S.push_back(points[0]);
   S.push_back(points[1]);
   S.push_back(points[2]);
 
   // Process remaining n-3 points
   for (int i = 3; i < m; i++)
   {
      // Keep removing top while the angle formed by
      // points next-to-top, top, and points[i] makes
      // a non-left turn
      while (privateUtils<state>::orientation(privateUtils<state>::nextToTop(S), S.back(), points[i]) != 2)
         S.pop_back();
      S.push_back(points[i]);
   }
 
   // Now stack has the output points, print contents of stack
   hull.reserve(S.size());
   while (!S.empty()){
       hull.push_back(Vector2D(S.back()));
       S.pop_back();
   }
}

// Separating axis theorem for polygonal intersection test
template <typename state>
bool Util::sat(std::vector<state>const& a, std::vector<state>const& b, double radius){
  double doubleRadius(radius*2);
  if(a.size()==1){
    if(b.size()==1) return (a[0].x==b[0].x&&a[0].y==b[0].y);
    if(b.size()==2) return fequal(Util::sqDistanceOfPointToLine(b[0],b[1],a[0]),0);
    if(b.size()>=3) return Util::pointInPoly(b,a[0]);
  }else if(a.size()==2){
    if(b.size()==1) return fequal(Util::sqDistanceOfPointToLine(a[0],a[1],b[0]),0);
    if(b.size()==2) return Util::linesIntersect(a[0],a[1],b[0],b[1]);
    if(b.size()>=3) return Util::lineIntersectsPoly(b,a[0],a[1]);
  }
  if(b.size()==1) return Util::pointInPoly(a,b[0]);
  if(b.size()==2) return Util::lineIntersectsPoly(a,b[0],b[1]);

  unsigned i;
  for (i=1;i<a.size();i++){
    if(!privateUtils<state>::sat(a,b,a[i-1],a[i],doubleRadius)) return false;
  }
  if(!privateUtils<state>::sat(a,b,a[a.size()-1],a[0],doubleRadius)) return false;
  for (i=1;i<b.size();i++){
    if(!privateUtils<state>::sat(a,b,b[i-1],b[i],doubleRadius)) return false;
  }
  if(!privateUtils<state>::sat(a,b,b[b.size()-1],b[0],doubleRadius)) return false;
  return true;
}
#endif
