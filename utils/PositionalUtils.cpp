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
#include "PositionalUtils.h"
#include <deque>

// From http://www.cplusplus.com/forum/beginner/49408/

double Util::linesIntersect(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, double* out){
  Vector2D a(A2-A1);
  Vector2D b(B2-B1);

  double f(det(a,b));
  if(!f)      // lines are parallel
    return A1==B1||A1==B2||A2==B1||A2==B2?true:false;

  Vector2D c(B2-A2);
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

bool Util::intersectionPoint(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, Vector2D& out){
  double pct(0);
  if(linesIntersect(A1,A2,B1,B2,&pct)){
    out=((B2 - B1) * pct) + B1;
    return true;
  }
  return false;
}

// Assume "rounded" line with radius (thus the line width is 2*r)
bool Util::fatLinesIntersect(Vector2D const& A1, Vector2D const& A2, double r1, Vector2D const& B1, Vector2D const& B2, double r2){
  // If A and B intersect, we're done
  if(linesIntersect(A1,A2,B1,B2)){return true;}

  Vector2D NA(normal(A1,A2)*r1); // Normal of line from A1 to A2 
  Vector2D NB(normal(B1,B2)*r2); // Normal of line from B1 to B2 
  NA.Normalize();
  NA*=r1;
  NB.Normalize();
  NB*=r2;

  // If A and B are parallel, then we can just check the distance
  double r(r1+r2);
  if((NA==NB || NA==-NB) && fless(distanceOfPointToLine(A1,A2,B1),r)){return true;}

  // Project along normal in both directions
  Vector2D A11(A1+NA);
  Vector2D A21(A2+NA);
  Vector2D A12(A1-NA);
  Vector2D A22(A2-NA);

  Vector2D B11(B1+NB);
  Vector2D B21(B2+NB);
  Vector2D B12(B1-NB);
  Vector2D B22(B2-NB);

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
}

float Util::closestDistanceBetweenLineSegments(Vector3D const& s1, Vector3D const& d1, Vector3D const& s2, Vector3D const& d2)
{
  Vector3D u(d1-s1);
  Vector3D v(d2-s2);
  Vector3D w(s1-s2);
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
  Vector3D dP(w + (u*sc) - (v*tc));  // =  S1(sc) - S2(tc)

  return dP.len();   // return the closest distance
}

bool Util::fatLinesIntersect(Vector3D const& A1, Vector3D const& A2, double r1, Vector3D const& B1, Vector3D const& B2, double r2){
  return fleq(closestDistanceBetweenLineSegments(A1,A2,B1,B2),r1+r2);
}

// For convex polygons only...
// Also assumes that no polygon extends beyond 2^12 (4096) on the x axis
bool Util::pointInPoly(std::vector<Vector2D> const& poly, Vector2D const& p1){
  unsigned crossings(0);
  Vector2D endpoint(Vector2D(p1.x+0xfff,p1.y));
  crossings+=linesIntersect(poly.front(),poly.back(),p1,endpoint);
  for(unsigned i(1); i<poly.size()&&crossings<2; ++i){
    crossings+=linesIntersect(poly[i-1],poly[i],p1,endpoint);
  }
  return crossings%2;
}
bool Util::lineIntersectsPoly(std::vector<Vector2D> const& poly, Vector2D const& p1, Vector2D const& p2){
  if(linesIntersect(poly.front(),poly.back(),p1,p2))return true;
  for(unsigned i(1); i<poly.size(); ++i){
    if(linesIntersect(poly[i-1],poly[i],p1,p2))return true;
  }
  return false;
}
namespace privateUtils{
  Vector2D p0;
  // A utility function to find next to top in a stack
  Vector2D nextToTop(std::deque<Vector2D> &S)
  {
    return S[S.size()-2];
  }

  // A utility function to swap two points
  void swap(Vector2D &p1, Vector2D &p2)
  {
    Vector2D temp = p1;
    p1 = p2;
    p2 = temp;
  }

  // A utility function to return square of distance
  // between p1 and p2
  double distSq(Vector2D const& p1, Vector2D const& p2)
  {
    return (p1.x - p2.x)*(p1.x - p2.x) +
      (p1.y - p2.y)*(p1.y - p2.y);
  }

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  int orientation(Vector2D const& p, Vector2D const& q, Vector2D const& r)
  {
    double val = (q.y - p.y) * (r.x - q.x) -
      (q.x - p.x) * (r.y - q.y);

    //if (val == 0) return 0;  // collinear
    return val?((val > 0)? 1: 2):0; // clock or counterclock wise
  }

  // A function used by library function qsort() to sort an array of
  // points with respect to the first point
  bool compare(Vector2D const& p1, Vector2D const& p2){
    // Find orientation
    int o = orientation(p0, p1, p2);
    if (o == 0)
      return (distSq(p0, p2) >= distSq(p0, p1))? true : false;

    return (o == 2)? true: false;
  }

};

// Graham Scan
// Prints convex hull of a set of n points.
void Util::convexHull(std::vector<Vector2D> points, std::vector<Vector2D>& hull){
  if (points.size() < 3){
    hull.reserve(points.size());
    hull.insert(hull.begin(),points.begin(),points.end());
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
   privateUtils::swap(points[0], points[min]);
 
   // Sort n-1 points with respect to the first point.
   // A point p1 comes before p2 in sorted ouput if p2
   // has larger polar angle (in counterclockwise
   // direction) than p1
   privateUtils::p0 = points[0];
   std::sort(points.begin()+1,points.end(),
       [](Vector2D const& a, Vector2D const& b) -> bool {
       return privateUtils::compare(a,b);
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
       while (i < points.size()-1 && privateUtils::orientation(privateUtils::p0, points[i],
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
       hull.push_back(points[i]);
     return;
   }
 
   // Create an empty stack and push first three points
   // to it.
   std::deque<Vector2D> S;
   S.push_back(points[0]);
   S.push_back(points[1]);
   S.push_back(points[2]);
 
   // Process remaining n-3 points
   for (int i = 3; i < m; i++)
   {
      // Keep removing top while the angle formed by
      // points next-to-top, top, and points[i] makes
      // a non-left turn
      while (privateUtils::orientation(privateUtils::nextToTop(S), S.back(), points[i]) != 2)
         S.pop_back();
      S.push_back(points[i]);
   }
 
   // Now stack has the output points, print contents of stack
   hull.reserve(S.size());
   while (!S.empty()){
       hull.push_back(S.back());
       S.pop_back();
   }
}
