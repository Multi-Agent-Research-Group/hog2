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
#ifndef UnitTests_h_
#define UnitTests_h_

//#include <CGAL/convex_hull_2.h>
//#include <CGAL/ch_graham_andrew.h>
//#include <CGAL/intersections.h>
//#include <CGAL/Boolean_set_operations_2.h>
#include "VelocityObstacle.h"
#include "CollisionDetection.h"
#include "Timer.h"
#include <gtest/gtest.h>
#include <map>
#include "Map2DConstrainedEnvironment.h"
#include "Grid3DConstrainedEnvironment.h"
//#include "AnyAngleSipp.h"
#include "ThetaStar.h"
#include "PositionalUtils.h"
#include "AABB.h"
#include <unordered_set>
#include <sstream>
#include <algorithm>
#include <deque>

//#include "EPEThetaStar.h"
//#include "PEThetaStar.h"

//typedef std::vector<Point_2> Points;

/*void convexHullCGAL2(std::vector<xytLoc> points, std::vector<Vector2D>& hull){
  if (points.size() < 3){
    hull.reserve(points.size());
    hull.insert(hull.begin(),points.begin(),points.end());
    return;
  }
  Points pts;
  pts.reserve(points.size());
  pts.insert(pts.begin(),points.begin(),points.end());
  CGAL::ch_graham_andrew(pts.begin(),pts.end(), std::back_inserter(hull));
}
void convexHullCGAL(std::vector<xytLoc> points, std::vector<Vector2D>& hull){
  if (points.size() < 3){
    hull.reserve(points.size());
    hull.insert(hull.begin(),points.begin(),points.end());
    return;
  }
  Points pts;
  pts.reserve(points.size());
  pts.insert(pts.begin(),points.begin(),points.end());
  CGAL::convex_hull_2(pts.begin(),pts.end(), std::back_inserter(hull));
}*/
// A globle point needed for  sorting points with reference
// to  the first point Used in compare function of qsort()
xytLoc p0;
 
// A utility function to find next to top in a stack
xytLoc nextToTop(std::deque<xytLoc> &S)
{
    return S[S.size()-2];
    //xytLoc p = S.top();
    //S.pop();
    //xytLoc res = S.top();
    //S.push(p);
    //return res;
}
 
// A utility function to swap two points
int swap(xytLoc &p1, xytLoc &p2)
{
    xytLoc temp = p1;
    p1 = p2;
    p2 = temp;
}
 
// A utility function to return square of distance
// between p1 and p2
int distSq(xytLoc const& p1, xytLoc const& p2)
{
    return (p1.x - p2.x)*(p1.x - p2.x) +
          (p1.y - p2.y)*(p1.y - p2.y);
}
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(xytLoc const& p, xytLoc const& q, xytLoc const& r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    //if (val == 0) return 0;  // colinear
    return val?((val > 0)? 1: 2):0; // clock or counterclock wise
}
int orientation(Vector2D const& p, Vector2D const& q, xytLoc const& r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    //if (val == 0) return 0;  // colinear
    return val?((val > 0)? 1: 2):0; // clock or counterclock wise
}
 
// A function used by library function qsort() to sort an array of
// points with respect to the first point
bool compare(xytLoc const& p1, xytLoc const& p2){
   // Find orientation
   int o = orientation(p0, p1, p2);
   if (o == 0)
     return (distSq(p0, p2) >= distSq(p0, p1))? true : false;
 
   return (o == 2)? true: false;
}
 
// Graham Scan
// Prints convex hull of a set of n points.
void convexHull(std::vector<xytLoc> points, std::vector<Vector2D>& hull){
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
   swap(points[0], points[min]);
 
   // Sort n-1 points with respect to the first point.
   // A point p1 comes before p2 in sorted ouput if p2
   // has larger polar angle (in counterclockwise
   // direction) than p1
   p0 = points[0];
   //qsort(&points[1], n-1, sizeof(xytLoc), compare);
   std::sort(points.begin()+1,points.end(),
       [](xytLoc const& a, xytLoc const& b) -> bool {
       return compare(a,b);
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
       while (i < points.size()-1 && orientation(p0, points[i],
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
   std::deque<xytLoc> S;
   S.push_back(points[0]);
   S.push_back(points[1]);
   S.push_back(points[2]);
 
   // Process remaining n-3 points
   for (int i = 3; i < m; i++)
   {
      // Keep removing top while the angle formed by
      // points next-to-top, top, and points[i] makes
      // a non-left turn
      while (orientation(nextToTop(S), S.back(), points[i]) != 2)
         S.pop_back();
      S.push_back(points[i]);
   }
 
   //std::cout << "-----------\n";
   // Now stack has the output points, print contents of stack
   hull.reserve(S.size());
   while (!S.empty()){
       //xytLoc p = S.back();
       hull.push_back(S.back());
       //std::cout << "(" << p.x << ", " << p.y <<")" << std::endl;
       S.pop_back();
   }
}

// Jarvis' march
void convexHull2(std::vector<xytLoc> const& points, std::vector<Vector2D>& hull){
    // There must be at least 3 points
    if (points.size() < 3) return;
 
    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < points.size(); i++)
        if (points[i].x < points[l].x)
            l = i;
 
    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    do
    {
        // Add current point to result
        if(hull.size()<2){
          hull.push_back(points[p]);
        }else if(orientation(hull[hull.size()-2],hull.back(),points[p])){
          hull.push_back(points[p]);
        }else{ // Collinear - replace
          hull.back()=points[p];
        }
 
        // Search for a point 'q' such that orientation(p, x,
        // q) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p+1)%points.size();
        for (int i = 0; i < points.size(); i++)
        {
           // If i is more counterclockwise than current q, then
           // update q
           if (orientation(points[p], points[i], points[q]) == 2)
               q = i;
        }
 
        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;
 
    } while (p != l);  // While we don't come to first point
 
    // Print Result
    //for (int i = 0; i < hull.size(); i++)
        //std::cout << "(" << hull[i].x << ", "
              //<< hull[i].y << ")\n";
}

typedef struct {float x, y;} vec;
typedef struct {vec p0, dir;} ray;
typedef struct {vec p0, p1, dir;} seg;
typedef struct {int n; vec *vertices; seg *edges;} polygon; // Assumption: Simply connected => chain vertices together

polygon new_polygon(int nvertices, vec *vertices){
	seg *edges = (seg*)malloc(sizeof(seg)*(nvertices));
	int i;
	for (i = 0; i < nvertices-1; i++){
		vec dir = {vertices[i+1].x-vertices[i].x, vertices[i+1].y-vertices[i].y};seg cur = {vertices[i], vertices[i+1], dir};
		edges[i] = cur;
	}
	vec dir = {vertices[0].x-vertices[nvertices-1].x, vertices[0].y-vertices[nvertices-1].y};seg cur = {vertices[nvertices-1], vertices[0], dir};
	edges[nvertices-1] = cur;
	polygon shape = {nvertices, vertices, edges};
	return shape;
}

vec v(float x, float y){
	vec a = {x, y};
	return a;
}

vec perp(vec v){
	vec b = {v.y, -v.x};
	return b;
}

seg segment(vec p0, vec p1){
	vec dir = {p1.x-p0.x, p1.y-p0.y};
	seg s = {p0, p1, dir};
	return s;
}

polygon Polygon(int nvertices, ...){
	va_list args;
	va_start(args, nvertices);
	vec *vertices = (vec*)malloc(sizeof(vec)*nvertices);
	int i;
	for (i = 0; i < nvertices; i++){
		vertices[i] = va_arg(args, vec);
	}
	va_end(args);
	return new_polygon(nvertices, vertices);
}

bool contains(double n, Vector2D const& range){
  return (range.min() <= n && n <= range.max());
}

bool overlap(Vector2D const& a, Vector2D const& b){
  if (contains(a.x,b)) return true;
  if (contains(a.y,b)) return true;
  if (contains(b.x,a)) return true;
  if (contains(b.y,a)) return true;
  return false;
}

bool sat(std::vector<Vector2D>const& pa, std::vector<Vector2D>const& pb, Vector2D const& a, Vector2D const& b){
  Vector2D axis((b-a).perp());
  axis.Normalize();
  Vector2D ppa(axis.projectPolyOntoSelf(pa));
  Vector2D ppb(axis.projectPolyOntoSelf(pb));
  return overlap(ppa,ppb);
}

/*
// Separating axis theorem for polygonal intersection test
bool sat(std::vector<Vector2D>const& a, std::vector<Vector2D>const& b){
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
    if(!sat(a,b,a[i-1],a[i])) return false;
  }
  if(!sat(a,b,a[a.size()-1],a[0])) return false;
  for (i=1;i<b.size();i++){
    if(!sat(a,b,b[i-1],b[i])) return false;
  }
  if(!sat(a,b,b[b.size()-1],b[0])) return false;
  return true;
}
*/
/*bool sat2(std::vector<Vector2D>const& a, std::vector<Vector2D>const& b){
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

  Points A;
  A.reserve(a.size());
  A.insert(A.begin(),a.begin(),a.end());
  Points B;
  B.reserve(b.size());
  B.insert(B.begin(),b.begin(),b.end());
  return CGAL::do_intersect(VelocityObstacle::Polygon_2(A.begin(),A.end()),VelocityObstacle::Polygon_2(B.begin(),B.end()));
}*/


/*
// Separating axis theorem for polygonal intersection test
bool sat(std::vector<xytLoc>const& a, std::vector<xytLoc>const& b){
  std::vector<Vector2D> aa;
  std::vector<Vector2D> bb;
  aa.reserve(a.size());
  bb.reserve(b.size());
  for(auto const& aaa:a)aa.push_back(aaa);
  for(auto const& bbb:b)bb.push_back(bbb);
  return sat(aa,bb);
}*/

TEST(VelocityObstacle, IsInsidePass){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  VelocityObstacle VO(A,VA,B,VB,radius);
  ASSERT_TRUE(VO.IsInside(A+VA));
  ASSERT_TRUE(VO.IsInside(A+Vector2D(-1,1.4)));
}

TEST(VelocityObstacle, IsInsideFail){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  VelocityObstacle VO(A,VA,B,VB,radius);
  ASSERT_FALSE(VO.IsInside(A+VB)); // This is right on the apex
  ASSERT_FALSE(VO.IsInside(A+A)); // This is way outside on the right
  ASSERT_FALSE(VO.IsInside(A-A)); // This is way outside on the left
}

TEST(VelocityObstacle, DetectCollisionWhenExists){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //    ^ ^
  //    |/
  //    X
  //   /|
  //  B |
  //    A
  //
  //==========

  // Because diagonals take sqrt(2) time, a collision occurs
  // The collision occurs at about time 2.3 if agents start at the same time

  // Suppose edges cross in the middle at some point.
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose edges end before collision actually occurs (at time step 2.2)
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,2.2,B,VB,radius,0.0,2.2));
  // Suppose agents start at a different time
  ASSERT_FALSE(collisionImminent(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,1.0,4.0,B,VB,radius,0.0,6.0));
}

TEST(VelocityObstacle, DetectCollisionWhenDiagonalParallel){
  Vector2D A(1,0);
  Vector2D VA(1,1);
  VA.Normalize();
  double radius(.4); // Larger radius...
  Vector2D B(0,0);
  Vector2D VB(1,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //    
  //     ^^
  //    //
  //   //
  //  //
  // BA  
  //
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose agents start at a different time
  // This is a crash since agent B cuts through the corner while A is there
  // and both of them take up 80% of the space
  ASSERT_TRUE(collisionImminent(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Crash does not occur with smaller radius size.
  ASSERT_FALSE(collisionImminent(A,VA,.25,1.0,3.2,B,VB,.25,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Crash does not occur with smaller radius size.
  ASSERT_FALSE(collisionImminent(A,VA,.25,0.0,6.0,B,VB*2,.25,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,1.0,6.0,B,VB,radius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,1.0,6.0));
}

TEST(VelocityObstacle, DetectCollisionWhenNoneExists){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(2,1);
  Vector2D VB(0,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //   ^ ^
  //   | |
  //   | |
  //   | |
  //   | |
  //   B A
  //
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose agents start at a different time
  ASSERT_FALSE(collisionImminent(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,1.0,4.0,B,VB,radius,0.0,6.0));
}

TEST(VelocityObstacle, DetectCollisionWhenParallel){
  Vector2D A(3,2);
  Vector2D VA(0,1);
  VA.Normalize();
  double aradius(0.9);
  Vector2D B(2,0);
  Vector2D VB(0,1);
  VB.Normalize();
  double bradius(.25);

  // SCENARIO A is "fatter" than B or vice versa
  // radius of one is too fat to pass the other in parallel
  //==========
  //   ^ ^
  //   | |
  //   | |
  //   | |
  //   | A:r=0.9
  //   |
  //   B:r=.25
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB,bradius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,2.3,B,VB,bradius,0.0,2.3));
  // Suppose agents start at a different time
  ASSERT_TRUE(collisionImminent(A,VA,aradius,2.0,3.2,B,VB,bradius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB,bradius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB*2.2,bradius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB*2,bradius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA,aradius,1.0,8.0,B,VB*2.2,bradius,0.0,6.0));
}

TEST(VelocityObstacle, DetectCollisionWhenHeadOn){
  Vector2D A(3,5);
  Vector2D VA(0,-1);
  VA.Normalize();
  double aradius(0.25);
  Vector2D B(3,1);
  Vector2D VB(0,1);
  VB.Normalize();
  double bradius(.25);

  // SCENARIO A is "fatter" than B or vice versa
  // radius of one is too fat to pass the other in parallel
  //==========
  //    A
  //    |
  //    v
  //    ^
  //    |
  //    B
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB,bradius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,2.0,B,VB,bradius,0.0,2.0));
  // Suppose edges end before collision
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,1.0,B,VB,bradius,0.0,1.0));
  // Suppose agents start at a different time
  ASSERT_TRUE(collisionImminent(A,VA,aradius,1,4,B,VB,bradius,0.0,3));
  ASSERT_FALSE(collisionImminent(A,VA,aradius,1.01,2.0,B,VB,bradius,0.0,2.0));
  // Agents stop in a collision state
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,1.8,B,VB,bradius,0.0,1.8));
  // Suppose one agent is moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB,bradius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB*2,bradius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB*2,bradius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA,aradius,1.0,8.0,B,VB*2.2,bradius,0.0,6.0));
}

TEST(Quadratic, NoDuration){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,0,0,B,VB,radius,0,5));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0,5,B,VB,radius,5,10));
  ASSERT_FALSE(collisionImminent(A,VA,radius,5,5,B,VB,radius,5,10));
  ASSERT_TRUE(collisionImminent(A,VA,radius,5,10,B,VB,radius,5,10));
}

TEST(Quadratic, IsInsidePass){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  ASSERT_TRUE(collisionImminent(A,VA,radius,0,5,B,VB,radius,0,5));
}

TEST(Quadratic, IsInsideFail){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A+VB,VA,radius,0,5,B,VB,radius,0,5));
  ASSERT_FALSE(collisionImminent(A+A,VA,radius,0,5,B,VB,radius,0,5));
  ASSERT_FALSE(collisionImminent(A-A,VA,radius,0,5,B,VB,radius,0,5));
}

TEST(Quadratic, CloseCall){
  Vector2D A(26,2);
  Vector2D VA(1,-1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(26,1);
  Vector2D VB(1,0);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,12.4,13.8,B,VB,radius,12,13));
}

TEST(Quadratic3D, DetectCollisionWhenExists){
  Vector3D A(5,1,1);
  Vector3D VA(0,1,1);
  VA.Normalize();
  double radius(.25);
  Vector3D B(1,2,2);
  Vector3D VB(1,1,1);
  VB.Normalize();

  // SCENARIO (side view) (x,y-axis)
  //============
  //
  //      ^ ^
  //      |/
  //      X
  //     /|
  //    / |
  //   /  |
  //  B   |
  //      A
  //
  //============

  // SCENARIO (side view) (y,z-axis)
  //============
  //
  //        ^
  //       /
  //      /
  //     /
  //    /
  //   B 
  //  A  
  //
  //============

  // SCENARIO (top view) (z,x-axis)
  //============
  //
  //      ^ ^
  //      |/
  //      X
  //     /|
  //    / |
  //   /  |
  //  B   |
  //      A
  //
  //============

  // Suppose edges cross in the middle at some point.
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,8.0,B,VB,radius,0.0,8.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.3,B,VB,radius,0.0,6.3));
  // Suppose edges end before collision actually occurs (at time step 2.2)
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.2,B,VB,radius,0.0,6.2));
  // Suppose agents start at a different time
  ASSERT_FALSE(collisionImminent(A,VA,radius,1.0,9.0,B,VB,radius,0.0,9.0));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,8.0,B,VB,radius,0.0,8.0));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,8.0,B,VB*2,radius,0.0,8.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,0.0,4.0,B,VB*2,radius,0.0,4.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,3.5,8.0,B,VB,radius,0.0,8.0));
}


TEST(Quadratic, DetectCollisionWhenExists){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //    ^ ^
  //    |/
  //    X
  //   /|
  //  B |
  //    A
  //
  //==========

  // Because diagonals take sqrt(2) time, a collision occurs
  // The collision occurs at about time 2.3 if agents start at the same time

  // Suppose edges cross in the middle at some point.
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose edges end before collision actually occurs (at time step 2.2)
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,2.2,B,VB,radius,0.0,2.2));
  // Suppose agents start at a different time
  ASSERT_FALSE(collisionImminent(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,1.0,4.0,B,VB,radius,0.0,6.0));
}

TEST(Quadratic, DetectCollisionWhenDiagonalParallel){
  Vector2D A(1,0);
  Vector2D VA(1,1);
  VA.Normalize();
  double radius(.4); // Larger radius...
  Vector2D B(0,0);
  Vector2D VB(1,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //    
  //     ^^
  //    //
  //   //
  //  //
  // BA  
  //
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose agents start at a different time
  // This is a crash since agent B cuts through the corner while A is there
  // and both of them take up 80% of the space
  ASSERT_TRUE(collisionImminent(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Crash does not occur with smaller radius size.
  ASSERT_FALSE(collisionImminent(A,VA,.25,1.0,3.2,B,VB,.25,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Crash does not occur with smaller radius size.
  ASSERT_FALSE(collisionImminent(A,VA,.25,0.0,6.0,B,VB*2,.25,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA*2,radius,1.0,6.0,B,VB,radius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,1.0,6.0));
}

TEST(Quadratic, DetectCollisionWhenNoneExists){
  Vector2D A(3,1);
  Vector2D VA(0,1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(2,1);
  Vector2D VB(0,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //   ^ ^
  //   | |
  //   | |
  //   | |
  //   | |
  //   B A
  //
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose agents start at a different time
  ASSERT_FALSE(collisionImminent(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_FALSE(collisionImminent(A,VA*2,radius,1.0,4.0,B,VB,radius,0.0,6.0));
}

TEST(Quadratic, DetectCollisionWhenParallel){
  Vector2D A(3,2);
  Vector2D VA(0,1);
  VA.Normalize();
  double aradius(0.9);
  Vector2D B(2,0);
  Vector2D VB(0,1);
  VB.Normalize();
  double bradius(.25);

  // SCENARIO A is "fatter" than B or vice versa
  // radius of one is too fat to pass the other in parallel
  //==========
  //   ^ ^
  //   | |
  //   | |
  //   | |
  //   | A:r=0.9
  //   |
  //   B:r=.25
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB,bradius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,2.3,B,VB,bradius,0.0,2.3));
  // Suppose agents start at a different time
  ASSERT_TRUE(collisionImminent(A,VA,aradius,2.0,3.2,B,VB,bradius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB,bradius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB*2.2,bradius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB*2,bradius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA,aradius,1.0,8.0,B,VB*2.2,bradius,0.0,6.0));
}

TEST(Quadratic, DetectCollisionWhenHeadOn){
  Vector2D A(3,5);
  Vector2D VA(0,-1);
  VA.Normalize();
  double aradius(0.25);
  Vector2D B(3,1);
  Vector2D VB(0,1);
  VB.Normalize();
  double bradius(.25);

  // SCENARIO A is "fatter" than B or vice versa
  // radius of one is too fat to pass the other in parallel
  //==========
  //    A
  //    |
  //    v
  //    ^
  //    |
  //    B
  //==========

  // No collision occurs here

  // Suppose edges cross in the middle at some point.
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB,bradius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,2.0,B,VB,bradius,0.0,2.0));
  // Suppose edges end before collision
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,1.0,B,VB,bradius,0.0,1.0));
  // Suppose agents start at a different time
  ASSERT_TRUE(collisionImminent(A,VA,aradius,1,4,B,VB,bradius,0.0,3));
  ASSERT_FALSE(collisionImminent(A,VA,aradius,1.01,2.0,B,VB,bradius,0.0,2.0));
  // Agents stop in a collision state
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,1.8,B,VB,bradius,0.0,1.8));
  // Suppose one agent is moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB,bradius,0.0,6.0));
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,6.0,B,VB*2,bradius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(collisionImminent(A,VA*2,aradius,0.0,3.0,B,VB*2,bradius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(collisionImminent(A,VA,aradius,1.0,8.0,B,VB*2.2,bradius,0.0,6.0));
}

TEST(Quadratic, DetectCollisionWhenOneEndsEarlier){
  Vector2D A(7,7);
  Vector2D VA(-1,-1);
  VA.Normalize();
  double aradius(0.25);
  Vector2D B(0,0);
  Vector2D VB(1,1);
  VB.Normalize();
  double bradius(.25);

  // Suppose edges cross in the middle at some point.
  ASSERT_FALSE(collisionImminent(A,VA,aradius,0.0,4.24264,B,VB,bradius,0.0,9.8995));
  ASSERT_TRUE(collisionImminent(A,VA,aradius,0.0,4.84264,B,VB,bradius,0.0,9.8995));
}

TEST(Quadratic, Contrived){
  Vector2D A(0,0);
  Vector2D VA(2,1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(2,0);
  Vector2D VB(-1,0);
  VB.Normalize();

  // SCENARIO
  //==========
  //  .>
  // B<A
  //
  //==========

  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,sqrt(5.),B,VB,radius,0.0,1.0));
  ASSERT_TRUE(collisionImminent(B,VB,radius,0.0,1.0,A,VA,radius,0.0,sqrt(5.)));
}

TEST(Quadratic, Contrived2){
  Vector2D A(0,0);
  Vector2D VA(7,7);
  VA.Normalize();
  double radius(.25);
  Vector2D B(4,4);
  Vector2D VB(-1,1);
  VB.Normalize();

  double sB(Util::distance(7,0,4,4));
  ASSERT_TRUE(collisionImminent(A,VA,radius,0.0,sqrt(2.)*7.,B,VB,radius,sB,sB+sqrt(2.)));
  //ASSERT_TRUE(collisionImminent(B,VB,radius,0.0,1.0,A,VA,radius,0.0,sqrt(5.)));
}

TEST(Quadratic, Contrived3){
  Vector2D A(0,0);
  Vector2D VA(7,7);
  VA.Normalize();
  double radius(.25);
  Vector2D B(7,0);
  Vector2D VB(-3,4);
  VB.Normalize();

  double sB(Util::distance(7,0,4,4));
  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,sqrt(2.)*7.,B,VB,radius,0,5));
  //ASSERT_TRUE(collisionImminent(B,VB,radius,0.0,1.0,A,VA,radius,0.0,sqrt(5.)));
}

TEST(Quadratic, Contrived4){
  Vector2D A(1,2);
  Vector2D VA(1,1);
  VA.Normalize();
  double radius(.5);
  Vector2D B(1,2);
  Vector2D VB(1,0);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,1.41421,B,VB,radius,1.41421,7.41421));
  //ASSERT_TRUE(collisionImminent(B,VB,radius,0.0,1.0,A,VA,radius,0.0,sqrt(5.)));
}

TEST(Quadratic, Contrived5){
  {
    Vector2D A(2,4);
    Vector2D VA(0,0);
    VA.Normalize();
    double radius(.5);
    Vector2D B(2,4);
    Vector2D VB(-2,-4);
    VB.Normalize();

    ASSERT_TRUE(collisionImminent(A,VA,radius,6.38516,11.6569,B,VB,radius,6.12311,10.5952));
  }
  {
    Vector2D A(2,4);
    Vector2D VA(0,0);
    VA.Normalize();
    double radius(.5);
    Vector2D B(1.8828, 3.76561);
    Vector2D VB(1.55279, 3.10557);
    VB-=B;
    VB.Normalize();

    ASSERT_TRUE(collisionImminent(A,VA,radius,6.38516,7.1231,B,VB,radius,6.38516,7.1231));
  }
}

TEST(Quadratic, Contrived6){
  Vector2D A(7,6);
  Vector2D VA(-2,4);
  VA.Normalize();
  double radius(.5);
  Vector2D B(2,5);
  Vector2D VB(4,-3);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,0.0,4.47214,B,VB,radius,2.23607,5));
}

TEST(Quadratic, Contrived7){
  Vector2D A(3,7);
  Vector2D VA(1,0);
  VA.Normalize();
  double radius(.5);
  Vector2D B(3,7);
  Vector2D VB(1,0);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,5,6,B,VB,radius,2.82843,5.82843));
}

TEST(Quadratic, Contrived8){
  Vector2D A(3,7);
  Vector2D VA(0,0);
  VA.Normalize();
  double radius(.5);
  Vector2D B(3,7);
  Vector2D VB(0,0);
  VB.Normalize();

  ASSERT_TRUE(collisionImminent(A,VA,radius,5,6,B,VB,radius,2.82843,5.82843));
}

TEST(Quadratic, Contrived9){
  Vector2D A(3,7);
  Vector2D VA(0,0);
  VA.Normalize();
  double radius(.5);
  Vector2D B(3,7);
  Vector2D VB(0,0);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,5,6,B,VB,radius,2.82843,4.82843));
}

TEST(Quadratic, Contrived10){
  Vector2D A(3,7);
  Vector2D VA(0,0);
  VA.Normalize();
  double radius(.5);
  Vector2D B(3,7);
  Vector2D VB(1,-1);
  VB.Normalize();

  ASSERT_FALSE(collisionImminent(A,VA,radius,5,6,B,VB,radius,2.82843,5.82843));
}

TEST(Quadratic, Contrived11){
  Vector2D A(4,6);
  Vector2D VA(1,0);
  VA.Normalize();
  double radius(.25);
  Vector2D B(5,6);
  Vector2D VB(0,1);
  VB.Normalize();

  ASSERT_TRUE(collisionImminent(A,VA,radius,1,2,B,VB,radius,1.41421,2.41421));
}

TEST(collisionCheck3D, Test1){
  ASSERT_TRUE(collisionCheck3D(xyztLoc(5,5,0,1.41421f),xyztLoc(6,6,0,2.82843f),xyztLoc(6,6,0,2.0f),xyztLoc(5,6,0,3.0f),.25));
}

void drawcircle(int x0, int y0, int r, std::map<int,int>& coords){
    int x = r;
    int y = 0;
    int err = 0;

    while (x >= y){
        // Full right side of circle, with coords swapped (y first so we sort on y primarily)
        auto r(coords.emplace(y0 + y, x0 + x));
        if(!r.second && r.first->second < x0+x){
          r.first->second=x0+x;
        }
        
        r=coords.emplace(y0 + x, x0 + y);
        if(!r.second && r.first->second < x0+y){
          r.first->second=x0+y;
        }
        r=coords.emplace(y0 - x, x0 + y);
        if(!r.second && r.first->second < x0+y){
          r.first->second=x0+y;
        }
        r=coords.emplace(y0 - y, x0 + x);
        if(!r.second && r.first->second < x0+x){
          r.first->second=x0+y;
        }

        //coords.emplace(x0 + x, y0 + y);
        //coords.emplace(x0 + y, y0 + x);
        //coords.emplace(x0 - y, y0 + x);
        //coords.emplace(x0 - x, y0 + y);
        //coords.emplace(x0 - x, y0 - y);
        //coords.emplace(x0 - y, y0 - x);
        //coords.emplace(x0 + y, y0 - x);
        //coords.emplace(x0 + x, y0 - y);

        y++;
        if(err <= 0){
            err += 2*y + 1;
        }else{
            x--;
            err -= 2*x + 1;
        }
    }
}

int rint(int low=0, int high=10){
    int width(high-low);
    return rand()%width + low;
}
float rfloat(float low=0, float high=10){
    float width(high-low);
    return float(rand()%int(width*1000.0))/1000.0 + low;
}

/*TEST(PerfTest, VOSquare){
  std::vector<Vector2D> square={{-.3,.3},{-.3,-.3},{.3,-.3},{.3,.3}};
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    if(collisionImminentPolygonalAgents(Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),square,rfloat(0,5),rfloat(5,10),Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),square,rfloat(0,5),rfloat(5,10))){
std::cout << "f";}
else{std::cout << "p";}
  }
  std::cout << "\nTotal time (VelocityObstacleSquare)" << t.EndTimer() << "\n";
}

TEST(PerfTest, VO){
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    if(collisionImminent(Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,5),rfloat(5,10),Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,5),rfloat(5,10))){
std::cout << "f";}
else{std::cout << "p";}
  }
  std::cout << "\nTotal time (VelocityObstacleCircle)" << t.EndTimer() << "\n";
}*/

TEST(PerfTest, Quadratic){
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    if(collisionImminent(Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,5),rfloat(5,10),Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,5),rfloat(5,10))){
    }
  }
  std::cout << "\nTotal time (Quadratic)" << t.EndTimer() << "\n";
}

TEST(PerfTest, GatedQuadratic){
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    TemporalVector3D a1(rint(),rint(),0,int(rfloat(0,5)*10)/10.0);
    xyztLoc A1(a1.x,a1.y,0,a1.t);
    TemporalVector3D a2(rint(),rint(),0,0);
    a2.t=a1.t+int(Util::distance3d(a1,a2)*10)/10.0;
    if(a2.t==a1.t)a2.t=1.0;
    xyztLoc A2(a2.x,a2.y,0,a2.t);
    TemporalVector3D b1(rint(),rint(),0,int(rfloat(0,5)*10)/10.0);
    xyztLoc B1(b1.x,b1.y,0,b1.t);
    TemporalVector3D b2(rint(),rint(),0,0);
    b2.t=b1.t+int(Util::distance3d(b1,b2)*10)/10.0;
    if(b2.t==b1.t)b2.t=1.0;
    xyztLoc B2(b2.x,b2.y,0,b2.t);
    bool v1=collisionCheck3D(a1,a2,b1,b2,.25);
    bool v2=collisionCheck2D(A1,A2,B1,B2,.25);
    if (v1 != v2)
    {
      std::cout << a1<<a2<<b1<<b2<<"\n";
      std::cout << A1<<A2<<B1<<B2<<"\n";
      v1 = collisionCheck3D(a1, a2, b1, b2, .25);
      v2 = collisionCheck2D(A1, A2, B1, B2, .25);
    }
    auto VA(a2-a1); VA.Normalize();
    auto VB(b2-b1); VB.Normalize();
    bool v3(collisionImminent(A1, VA, .25, a1.t, a2.t, B1, VB, .25, b1.t, b2.t));
    if (v1 != v3)
    {
      std::cout << a1<<a2<<b1<<b2<<"\n";
      std::cout << A1<<A2<<B1<<B2<<"\n";
      v1 = collisionCheck3D(a1, a2, b1, b2, .25);
      v2 = collisionCheck2D(A1, A2, B1, B2, .25);
      v3 = collisionImminent(A1, VA, .25, a1.t, a2.t, B1, VB, .25, b1.t, b2.t);
    }
    ASSERT_EQ(v1, v2);
    ASSERT_EQ(v3, v2);
  }
    std::cout << "\nTotal time (Gated Quadratic)" << t.EndTimer() << "\n";
}

/*TEST(PerfTest, AABB){
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    float a1x(rfloat());
    float a1y(rfloat());
    float a1t(rfloat(0,5));
    float a2x(rfloat());
    float a2y(rfloat());
    float a2t(rfloat(5,10));
    float b1x(rfloat());
    float b1y(rfloat());
    float b1t(rfloat(0,5));
    float b2x(rfloat());
    float b2y(rfloat());
    float b2t(rfloat(5,10));
    float mnax(std::min(a1x,a2x));
    float mxax(std::max(a1x,a2x));
    float mnay(std::min(a1y,a2y));
    float mxay(std::max(a1y,a2y));
    float mnat(std::min(a1t,a2t));
    float mxat(std::max(a1t,a2t));
    float mnbx(std::min(b1x,b2x));
    float mxbx(std::max(b1x,b2x));
    float mnby(std::min(b1y,b2y));
    float mxby(std::max(b1y,b2y));
    float mnbt(std::min(b1t,b2t));
    float mxbt(std::max(b1t,b2t));
    if(mnax<=mxbx && mxax>=mnbx &&
    mnay<=mxby && mxay>=mnby &&
    mnat<=mxbt && mxat>=mnbt){
std::cout << "f";}
else{std::cout << "p";}
  }
  std::cout << "\nTotal time (AABB)" << t.EndTimer() << "\n";
}

TEST(PerfTest, StaticPoly){
  std::vector<Vector2D> square={{-.3,0},{0,-.3},{.3,0},{0,.3}};
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    std::cout << i << "\n";
    Vector2D a1(rfloat(), rfloat());
    Vector2D a2(rfloat(),rfloat());
    Vector2D b1(rfloat(),rfloat());
    Vector2D b2(rfloat(),rfloat());
    Points pointsa;
    pointsa.reserve(square.size()*2);
    for(auto const& pa:square){
      pointsa.push_back(a1+pa);
      pointsa.push_back(a2+pa);
    }
    Points pointsb;
    pointsb.reserve(square.size()*2);
    for(auto const& pb:square){
      pointsb.push_back(b1+pb);
      pointsb.push_back(b2+pb);
    }
    Points hulla;
    Points hullb;
    CGAL::convex_hull_2(pointsa.begin(),pointsa.end(), std::back_inserter(hulla));
    CGAL::convex_hull_2(pointsb.begin(),pointsb.end(), std::back_inserter(hullb));
    if(CGAL::do_intersect(VelocityObstacle::Polygon_2(hulla.begin(),hulla.end()),VelocityObstacle::Polygon_2(hullb.begin(),hullb.end()))){
      std::cout << "f";
    }else{
      std::cout << "p";
    }
  }
  std::cout << "\nTotal time (Static Square)" << t.EndTimer() << "\n";
}*/

TEST(DISABLED_PerfTest, IncrementalCircular){
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    Vector2D a1(rfloat(), rfloat());
    float a1t(rfloat(0,5));
    Vector2D a2(rfloat(),rfloat());
    float a2t(rfloat(5,10));
    Vector2D b1(rfloat(),rfloat());
    float b1t(rfloat(0,5));
    Vector2D b2(rfloat(),rfloat());
    float b2t(rfloat(5,10));
    float mnat(std::min(a1t,a2t));
    float mxat(std::max(a1t,a2t));
    float mnbt(std::min(b1t,b2t));
    float mxbt(std::max(b1t,b2t));
    if(mnat<=mxbt && mxat>=mnbt){
      float start(std::max(mnbt,mnat));
      float stop(std::min(mxbt,mxat));
      float step((stop-start)/10.);
      float transa((a1t-start));
      float transb((b1t-start));
      Vector2D da(a2-a1);
      float dat((a1t-a2t));
      Vector2D db(b2-b1);
      float dbt((b1t-b2t));
      a1+=da*(transa/dat); // Translate to start time
      b1+=db*(transb/dbt); // Translate to start time
      da*=.2;
      db*=.2;
      bool pass(true);
      for(int j(0); j<5; ++j){
        Vector2D d(a1-b1);
        if(d.sq() < .25){
          pass=false;
          std::cout << "f";
          break;
        }
        a1+=da;
        b1+=db;
      }
      if(pass){std::cout << "p";}
    }else{std::cout << "p";}
  }
  std::cout << "\nTotal time (Incremental Circular)" << t.EndTimer() << "\n";
}

/*TEST(PerfTest, IncrementalPoly){
  std::vector<Vector2D> square={{-.3,.3},{-.3,-.3},{.3,-.3},{.3,.3}};
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    //std::cout << i << "\n";
    Vector2D a1(rfloat(), rfloat());
    float a1t(rfloat(0,5));
    Vector2D a2(rfloat(),rfloat());
    float a2t(rfloat(5,10));
    Vector2D b1(rfloat(),rfloat());
    float b1t(rfloat(0,5));
    Vector2D b2(rfloat(),rfloat());
    float b2t(rfloat(5,10));
    float mnat(std::min(a1t,a2t));
    float mxat(std::max(a1t,a2t));
    float mnbt(std::min(b1t,b2t));
    float mxbt(std::max(b1t,b2t));
    if(mnat<=mxbt && mxat>=mnbt){
      float start(std::max(mnbt,mnat));
      float stop(std::min(mxbt,mxat));
      float step((stop-start)/10.);
      float transa((a1t-start));
      float transb((b1t-start));
      Vector2D da(a2-a1);
      float dat((a1t-a2t));
      Vector2D db(b2-b1);
      float dbt((b1t-b2t));
      a1+=da*(transa/dat); // Translate to start time
      b1+=db*(transb/dbt); // Translate to start time
      da*=.2;
      db*=.2;
      bool pass(true);
      for(int j(0); j<5; ++j){
        if(VelocityObstacle::AgentOverlap(a1,b1,square,square)){
          pass=false;
          std::cout << "f";
          break;
        }
        a1+=da;
        b1+=db;
      }
      if(pass){std::cout << "p";}
    }else{std::cout << "p";}
  }
  std::cout << "\nTotal time (Incremental Square)" << t.EndTimer() << "\n";
}*/

TEST(PerfTest, Gen){
  Timer t;
  t.StartTimer();
  for(int i(0); i<100000; ++i){
    float a1x(rfloat());
    float a1y(rfloat());
    float a1t(rfloat(0,5));
    float a2x(rfloat());
    float a2y(rfloat());
    float a2t(rfloat(5,10));
    float b1x(rfloat());
    float b1y(rfloat());
    float b1t(rfloat(0,5));
    float b2x(rfloat());
    float b2y(rfloat());
    float b2t(rfloat(5,10));
  }
  std::cout << "Total time (Gen)" << t.EndTimer() << "\n";
}

TEST(RadialVisibility, ComputeVisibilityGrid){
  std::map<int,int> coords;
  Vector2D p(5,5);
  int r=5;
  drawcircle(p.x,p.y,r,coords);
  
  int d(r*2+1);
  std::vector<char> bits(d*d);
  for(auto const& c:coords){
    if(c.second-p.x)
      memset(&bits[d-c.second+c.first*d],0x01,(c.second-p.x)*2);
  }
  for(int x(0); x<d; ++x){
    for(int y(0); y<d; ++y){
      if(bits[x+d*y])
      std::cout << "1";
      else
      std::cout << " ";
    }
    std::cout << ".\n";
  }
}
  
TEST(CollisionInterval, GetCollisionIntervalHalfSize){
  Vector2D A(3,2);
  Vector2D VA(1,2);
  VA.Normalize();
  double radius(.5);
  Vector2D B(4,2);
  Vector2D VB(1,3);
  VB.Normalize();

  auto intvl(getCollisionInterval(A,VA,radius,0.0,sqrt(5),B,VB,radius,0.0,sqrt(10)));
  auto intvl2(getCollisionInterval(A,VA,radius,0.0,6.,B,VB,radius,0.0,6.));
  ASSERT_DOUBLE_EQ(intvl.first,intvl2.first);
}
  
TEST(CollisionInterval, GetCollisionIntervalTouching){
  Vector2D A(4,2);
  Vector2D VA(1,3);
  VA.Normalize();
  double radius(.5);
  Vector2D B(5,7);
  Vector2D VB(2,-6);
  VB.Normalize();

  auto intvl(getCollisionInterval(A,VA,radius,0.0,3.16228,B,VB,radius,0.0,6.32456));
  auto intvl2(getCollisionInterval(A,VA,radius,0.0,6.,B,VB,radius,0.0,6.));
  ASSERT_TRUE(fequal(intvl.first,intvl2.first));
  ASSERT_TRUE(fequal(intvl.second,intvl2.second));
}

TEST(CollisionInterval, GetCollisionSameStartDifferentTime){
  Vector2D A(4,4);
  Vector2D VA(-1,-1);
  VA.Normalize();
  double radius(.5);
  Vector2D B(4,4);
  Vector2D VB(-1,0);
  VB.Normalize();

  auto intvl(getCollisionInterval(A,VA,radius,3.23607,8.89292,B,VB,radius,4.05028,6.05028));
  auto intvl2(getCollisionInterval(B,VB,radius,4.05028,6.05028,A,VA,radius,3.23607,8.89292));
  ASSERT_TRUE(fequal(intvl.first,intvl2.first));
  ASSERT_TRUE(fequal(intvl.second,intvl2.second));
  ASSERT_DOUBLE_EQ(4.050279,intvl2.first);
}

TEST(CollisionInterval, GetCollisionSameLocationNotMovingDifferentTime){
  Vector2D A(7,2);
  Vector2D VA(0,0);
  VA.Normalize();
  double radius(.5);
  Vector2D B(7,2);
  Vector2D VB(0,0);
  VB.Normalize();

  auto collision(collisionImminent(A,VA,radius,9,10,B,VB,radius,7.5231056213378906,8.5231056213378906));
  ASSERT_FALSE(collision);
  auto intvl(getCollisionInterval(A,VA,radius,9,10,B,VB,radius,7.5231056213378906,8.5231056213378906));
  ASSERT_DOUBLE_EQ(-1,intvl.first);
  ASSERT_DOUBLE_EQ(-1,intvl.second);
}

TEST(CollisionInterval, GetCollisionIntervalWhenExists){
  Vector2D A(4,1);
  Vector2D VA(0,1);
  VA.Normalize();
  double radius(.25);
  Vector2D B(1,2);
  Vector2D VB(1,1);
  VB.Normalize();

  // SCENARIO
  //==========
  //    ^ ^
  //    |/
  //    X
  //   /|
  //  B |
  //    A
  //
  //==========

  auto intvl(getCollisionInterval(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  std::cout << "Collision interval is: " << intvl.first << "," << intvl.second << "\n";
  ASSERT_TRUE(intvl.first>0);
}

/*TEST(AnyAngle, GetPath){
  Map map(8,8);
  AnyAngleSipp<Map,AANode> AA(0,false);
  AANode s(1,1);
  AANode g(7,3);
  std::vector<AANode> succ;
  AA.GetPath(succ,s,g,map);
  for(auto const& ss: succ)
    std::cout << ss.x << "," << ss.y << "@" << ss.g << "\n";
}

TEST(AnyAngle, GetPathWithTimeConstraint){
  Map map(8,8);
  AnyAngleSipp<Map,AANode> AA(0,false);
  Intervals intervals;
  intervals.push_back({0,.5});
  intervals.push_back({4,99999999});
  AA.setConstraint(2,1,1.1);
  AA.setSafeIntervals(2,1,intervals);
  AANode s(1,1);
  AANode g(7,3);
  std::vector<AANode> succ;
  AA.GetPath(succ,s,g,map);
  for(auto const& ss: succ)
    std::cout << ss.x << "," << ss.y << "@" << ss.g << "\n";
}*/

// Heuristics
class StraightLineHeuristic : public Heuristic<xytLoc> {
  public:
  double HCost(const xytLoc &a,const xytLoc &b) const {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
  }
};

// Heuristics
class StraightLineHeuristic3D : public Heuristic<xyztLoc> {
  public:
  double HCost(const xyztLoc &a,const xyztLoc &b) const {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
  }
};

/*
TEST(Theta, GetPath){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1,0},{7,3,0},solution);
  for(auto const& ss: solution)
    std::cout << ss << "\n";
}

TEST(EPETheta, GetPath){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  menv.SetFiveConnected();
  EPEThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1,0},{7,3,0},solution);
  for(auto const& ss: solution)
    std::cout << ss << "\n";
}

TEST(PETheta, GetPath){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  menv.SetFiveConnected();
  PEThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1,0},{7,3,0},solution);
  for(auto const& ss: solution)
    std::cout << ss.x << "," << ss.y << "\n";
}
*/

/*TEST(Theta, GetObstructedPath){
  Map map(8,8);
  MapEnvironment env(&map);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,1,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  env.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,MapEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1},{7,3},solution);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  std::cout << "\n";
}

TEST(Theta, GetObstructedPath){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  //env.SetIgnoreTime(true);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,1,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1,0},{7,3,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  std::cout << "\n";
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
}

TEST(Theta1, GetObstructedPath1){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(4,6,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{0,5,0},{6,6,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "\n";
}

TEST(Theta1, TestAnomaly){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  // No moving obstacle...
  tstar.GetPath(&env,{5,0,0},{7,3,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "\n";
}
*/
/*TEST(Theta1, TestStepAside){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  //map.SetTerrainType(2,0,kOutOfBounds);
  // Make This map
  // ##########
  // #        #
  // #        #
  // #        #
  // #  #     #
  // ### ######
  // #s      g#
  // ##########
  //  Add a moving obstacle
  // ##########
  // #        #
  // #        #
  // #        #
  // #  #     #
  // ### ######
  // #s   <-og#
  // ##########
  map.SetTerrainType(0,0,kOutOfBounds);
  map.SetTerrainType(0,1,kOutOfBounds);
  map.SetTerrainType(0,2,kOutOfBounds);
  map.SetTerrainType(1,0,kOutOfBounds);
  map.SetTerrainType(1,2,kOutOfBounds);
  map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,2,kOutOfBounds);
  map.SetTerrainType(3,0,kOutOfBounds);
  map.SetTerrainType(3,3,kOutOfBounds);
  map.SetTerrainType(4,0,kOutOfBounds);
  map.SetTerrainType(4,2,kOutOfBounds);
  map.SetTerrainType(5,0,kOutOfBounds);
  map.SetTerrainType(5,2,kOutOfBounds);
  map.SetTerrainType(6,0,kOutOfBounds);
  map.SetTerrainType(6,2,kOutOfBounds);
  map.SetTerrainType(7,0,kOutOfBounds);
  map.SetTerrainType(7,1,kOutOfBounds);
  map.SetTerrainType(7,2,kOutOfBounds);
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  // No moving obstacle...
  tstar.GetPath(&env,{1,1,0},{6,1,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "with obstacle\n";
  env.AddConstraint((Constraint<TemporalVector3D>*)new Collision<TemporalVector3D>({6,1,0,0},{1,1,0,.6}));
  agentRadius=.25;
  tstar.GetPath(&env,{1,1,0},{6,1,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    if(!env.LineOfSight(solution[i-1],solution[i])){
      std::cout << "No LOS: " << solution[i-1] << "-->" << solution[i] << "\n";
    }
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  agentRadius=.5;
  std::cout << "\n";
}

TEST(Theta1, TestDodging){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  //map.SetTerrainType(2,0,kOutOfBounds);
  // Make This map
  // #########
  // #       #
  // #       #
  // #       #
  // #       #
  // #########
  // #    <-o#
  // #s <-o g#
  // #########
  map.SetTerrainType(0,0,kOutOfBounds);
  map.SetTerrainType(0,1,kOutOfBounds);
  map.SetTerrainType(0,2,kOutOfBounds);
  map.SetTerrainType(0,3,kOutOfBounds);
  map.SetTerrainType(1,0,kOutOfBounds);
  map.SetTerrainType(1,3,kOutOfBounds);
  map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,3,kOutOfBounds);
  map.SetTerrainType(3,0,kOutOfBounds);
  map.SetTerrainType(3,3,kOutOfBounds);
  map.SetTerrainType(4,0,kOutOfBounds);
  map.SetTerrainType(4,3,kOutOfBounds);
  map.SetTerrainType(5,0,kOutOfBounds);
  map.SetTerrainType(5,3,kOutOfBounds);
  map.SetTerrainType(6,0,kOutOfBounds);
  map.SetTerrainType(6,3,kOutOfBounds);
  map.SetTerrainType(7,0,kOutOfBounds);
  map.SetTerrainType(7,1,kOutOfBounds);
  map.SetTerrainType(7,2,kOutOfBounds);
  map.SetTerrainType(7,3,kOutOfBounds);
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  //tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  env.AddConstraint((Constraint<TemporalVector3D>*)new Collision<TemporalVector3D>({4,1,0,0},{1,1,0,6}));
  env.AddConstraint((Constraint<TemporalVector3D>*)new Collision<TemporalVector3D>({7,2,0,0},{1,2,0,6}));
  tstar.GetPath(&env,{1,1,0},{6,1,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "\n";
}

TEST(DISABLED_Theta, TestMotionConstrained3D){
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  Grid3DConstrainedEnvironment env(&menv);
  ThetaStar<xyztLoc,t3DDirection,Grid3DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic3D());
  //tstar.SetVerbose(true);
  std::vector<xyztLoc> solution;
  //env.SetMaxTurnAzimuth(45.0); //(only 45 deg turns allowed)
  //env.SetMaxPitch(30.0); //(only 30 deg pitch change allowed)
  tstar.GetPath(&env,{4,4,0,0.0f},{4,4,2,0.0f},solution); // Turn around
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "," << ss.z << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "\n";
}

TEST(DISABLED_Theta, Test3D){
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  Grid3DConstrainedEnvironment env(&menv);
  ThetaStar<xyztLoc,t3DDirection,Grid3DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic3D());
  //tstar.SetVerbose(true);
  std::vector<xyztLoc> solution;
  env.AddConstraint((Constraint<xyztLoc>*)new Collision<xyztLoc>(xyztLoc(4,1,0,0.0f),xyztLoc(1,1,0,6.0f)));
  env.AddConstraint((Constraint<xyztLoc>*)new Collision<xyztLoc>(xyztLoc(7,2,0,0.0f),xyztLoc(1,2,0,6.0f)));
  tstar.GetPath(&env,{1,1,0,0u},{6,1,0,0u},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "," << ss.z << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << Util::heading<360>(0,0,0,-1) << "\n";
  std::cout << Util::angle<360>(0,0,0,-1) << "\n";
  std::cout << "\n";
}
*/
#define WORD_BITS (8 * sizeof(unsigned))

// w=1 for 5,9 connected; 2 for 25 and 3 for 49
static inline unsigned index(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2, unsigned w){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-w;
  d1.y-=s1.y-w;
  // Translate d2 relative to s2
  d2.x-=s2.x-w;
  d2.y-=s2.y-w;
  s2.x-=s1.x-w;
  s2.y-=s1.y-w;
  unsigned W(w*2+1);
  unsigned Wsq(W*W);
  unsigned W4(Wsq*Wsq);
  return d1.y*W+d1.x + Wsq*(s2.y*W+s2.x) + W4*(d2.y*W+d2.x);
}

static inline unsigned index(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2, unsigned w){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-w;
  d1.y-=s1.y-w;
  // Translate d2 relative to s2
  d2.x-=s2.x-w;
  d2.y-=s2.y-w;
  s2.x-=s1.x-w;
  s2.y-=s1.y-w;
  unsigned W(w*2+1);
  unsigned W2(W*W);
  unsigned W3(W2*W);
  unsigned W6(W3*W3);
  return d1.z*W2+d1.y*W+d1.x + W3*(s2.z*W2+s2.y*W+s2.x) + W6*(d2.z*W2+d2.y*W+d2.x);
}

static inline unsigned index9(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2){return index(s1,d1,s2,d2,1);}
static inline unsigned index27(xyztLoc const& s1, xyztLoc d1, xyztLoc s2, xyztLoc d2){return index(s1,d1,s2,d2,1);}
static inline unsigned index25(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2){return index(s1,d1,s2,d2,2);}
static inline unsigned index49(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2){return index(s1,d1,s2,d2,3);}
static inline unsigned index125(xyztLoc const& s1, xyztLoc d1, xyztLoc s2, xyztLoc d2){return index(s1,d1,s2,d2,2);}

/*static inline bool get(unsigned* bitarray, size_t idx) {
    return bitarray[idx / WORD_BITS] & (1 << (idx % WORD_BITS));
}

static inline void set(unsigned* bitarray, size_t idx) {
    bitarray[idx / WORD_BITS] |= (1 << (idx % WORD_BITS));
}*/

// Polygonal intersection from:
// https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf
TEST(PolygonalIntersection, test1){
  ASSERT_TRUE(Util::fatLinesIntersect<Vector2D>({2,2},{2,3},.25,{2,3},{1,3},.25));
}


TEST(DISABLED_PreCollision, generate9Conn_25Rad){
  //9*(9*9)=729
  unsigned bitarray[9*9*9/WORD_BITS+1];
  unsigned bitarray1[9*9*9/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  memset(bitarray1,0,sizeof(bitarray));
  Map map(5,5);
  MapEnvironment env(&map);
  env.SetNineConnected();
  std::vector<xyLoc> n;
  xyLoc center(2,2);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2) &&
         !Util::fatLinesIntersect(center,m,.25,o,q,.25)){
          std::cout << "Collision between " << center<<"-->"<<m<<" and " << o << "-->"<<q << "detected, but lines do not intersect\n";
          int x=1;
          collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2);
          Util::fatLinesIntersect(center,m,.25,o,q,.25);
        }

        if(get(bitarray1,index9(center,m,o,q))){
          ASSERT_FALSE(true);
        }
        set(bitarray1,index9(center,m,o,q));
        if(Util::fatLinesIntersect(center,m,.25,o,q,.25)){
          set(bitarray,index9(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index9(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index9(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index9(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index9(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index9(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index9(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
  // Use the same for 5-connected
  missedOpportunities=0;
  wasHelpful=0;
  total=0;
  env.SetFiveConnected();
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index9(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index9(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index9(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index9(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index9(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index9(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate9Conn_5Rad){
  //9*(9*9)=729
  unsigned bitarray[9*9*9/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map map(5,5);
  MapEnvironment env(&map);
  env.SetNineConnected();
  std::vector<xyLoc> n;
  xyLoc center(2,2);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.5,o,q,.5)){
          set(bitarray,index(center,m,o,q,1));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.5);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index(center,m,o,q,1)));
          wasHelpful++;
        }else if(!get(bitarray,index(center,m,o,q,1)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          if(!get(bitarray,index(center,m,o,q,1))){
            std::cout << "Collision between " << center<<"-->"<<m<<" and " << o << "-->"<<q << "detected, but lines do not intersect\n";
            Util::fatLinesIntersect(center,m,.5,o,q,.5);
          }
          ASSERT_TRUE(get(bitarray,index(center,m,o,q,1)));
          wasHelpful++;
        }else if(!get(bitarray,index(center,m,o,q,1)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          if(!get(bitarray,index(center,m,o,q,1))){
            std::cout << "Collision between " << center<<"-->"<<m<<" and " << o << "-->"<<q << "detected, but lines do not intersect\n";
            Util::fatLinesIntersect(center,m,.5,o,q,.5);
          }
            
          ASSERT_TRUE(get(bitarray,index(center,m,o,q,1)));
          wasHelpful++;
        }else if(!get(bitarray,index(center,m,o,q,1)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
  // Use the same for 5-connected
  missedOpportunities=0;
  wasHelpful=0;
  total=0;
  env.SetFiveConnected();
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.5);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index(center,m,o,q,1)));
          wasHelpful++;
        }else if(!get(bitarray,index(center,m,o,q,1)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index(center,m,o,q,1)));
          wasHelpful++;
        }else if(!get(bitarray,index(center,m,o,q,1)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index(center,m,o,q,1)));
          wasHelpful++;
        }else if(!get(bitarray,index(center,m,o,q,1)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "size " << sizeof(bitarray) << " helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate25Conn_5Rad){
  //9*(9*9)=729
  unsigned bitarray[25*25*25/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map map(9,9);
  MapEnvironment env(&map);
  env.SetTwentyFiveConnected();
  env.SetFullBranching(true); // Include extra long branches
  std::vector<xyLoc> n;
  xyLoc center(4,4);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.5,o,q,.5)){
          set(bitarray,index25(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.5);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          if(!get(bitarray,index25(center,m,o,q))){
            std::cout << "Collision between " << center<<"-->"<<m<<" and " << o << "-->"<<q << "detected, but lines do not intersect\n";
            Util::fatLinesIntersect(center,m,.5,o,q,.5);
}
          ASSERT_TRUE(get(bitarray,index25(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index25(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index25(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index25(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index25(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index25(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "size " << sizeof(bitarray) << " helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate25Conn_25Rad){
  //9*(9*9)=729
  unsigned bitarray[25*25*25/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map map(9,9);
  MapEnvironment env(&map);
  env.SetTwentyFiveConnected();
  env.SetFullBranching(true); // Include extra long branches
  std::vector<xyLoc> n;
  xyLoc center(4,4);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.25,o,q,.25)){
          set(bitarray,index25(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index25(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index25(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index25(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index25(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index25(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index25(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "size " << sizeof(bitarray) << " helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate49Conn_5Rad){
  //9*(9*9)=729
  unsigned bitarray[49*49*49/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map map(13,13);
  MapEnvironment env(&map);
  env.SetTwentyFiveConnected();
  env.SetFullBranching(true); // Include extra long branches
  std::vector<xyLoc> n;
  xyLoc center(6,6);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.5,o,q,.5)){
          set(bitarray,index49(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.5);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index49(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index49(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index49(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index49(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index49(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index49(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "size " << sizeof(bitarray) << " helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate49Conn_25Rad){
  //9*(9*9)=729
  unsigned bitarray[49*49*49/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map map(13,13);
  MapEnvironment env(&map);
  env.SetTwentyFiveConnected();
  env.SetFullBranching(true); // Include extra long branches
  std::vector<xyLoc> n;
  xyLoc center(6,6);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.25,o,q,.25)){
          set(bitarray,index49(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector2D A(center);
        Vector2D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector2D B(o);
        Vector2D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index49(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index49(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index49(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index49(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index49(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index49(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "size " << sizeof(bitarray) << " helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate27Conn_25Rad){
  //27*(27*27)=19683
  unsigned bitarray[27*27*27/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map3D map(5,5,5);
  Grid3DEnvironment env(&map);
  env.SetOneConnected();
  std::vector<xyztLoc> n;
  xyztLoc center(2,2,2);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.25,o,q,.25)){
          set(bitarray,index27(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector3D A(center);
        Vector3D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector3D B(o);
        Vector3D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
  // Use the same for 0-connected
  missedOpportunities=0;
  wasHelpful=0;
  total=0;
  env.SetZeroConnected();
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector3D A(center);
        Vector3D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector3D B(o);
        Vector3D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate27Conn_5Rad){
  //27*(27*27)=19683
  unsigned bitarray[27*27*27/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map3D map(5,5,5);
  Grid3DEnvironment env(&map);
  env.SetOneConnected();
  std::vector<xyztLoc> n;
  xyztLoc center(2,2,2);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.5,o,q,.5)){
          set(bitarray,index27(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector3D A(center);
        Vector3D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.5);
        Vector3D B(o);
        Vector3D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
  // Use the same for 0-connected
  missedOpportunities=0;
  wasHelpful=0;
  total=0;
  env.SetZeroConnected();
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector3D A(center);
        Vector3D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector3D B(o);
        Vector3D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index27(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index27(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate125Conn_25Rad){
  //125*(125*125)=19683
  unsigned bitarray[125*125*125/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map3D map(9,9,9);
  Grid3DEnvironment env(&map);
  env.SetTwoConnected();
  std::vector<xyztLoc> n;
  xyztLoc center(4,4,4);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.25,o,q,.25)){
          set(bitarray,index125(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector3D A(center);
        Vector3D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.25);
        Vector3D B(o);
        Vector3D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index125(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index125(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index125(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index125(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index125(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index125(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

TEST(DISABLED_PreCollision, generate125Conn_5Rad){
  //125*(125*125)=19683
  unsigned bitarray[125*125*125/WORD_BITS+1];
  memset(bitarray,0,sizeof(bitarray));
  Map3D map(9,9,9);
  Grid3DEnvironment env(&map);
  env.SetTwoConnected();
  std::vector<xyztLoc> n;
  xyztLoc center(4,4,4);
  env.GetSuccessors(center,n);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,.5,o,q,.5)){
          set(bitarray,index125(center,m,o,q));
        }
      }
    }
  }

  unsigned missedOpportunities(0);
  unsigned wasHelpful(0);
  unsigned total(0);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<xyztLoc> p;
      env.GetSuccessors(o,p);
      for(auto const& q:p){
        Vector3D A(center);
        Vector3D VA(m);
        VA-=A;
        VA.Normalize();
        double radius(.5);
        Vector3D B(o);
        Vector3D VB(q);
        VB-=B;
        VB.Normalize();
        double d1(Util::distance(center,m));
        double d2(Util::distance(o,q));

        // This represents a broad-phase collision test -
        // It's ok if the bitarray contains true when the real check is false, but if it contains
        // false when the real check is true, that's bad
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index125(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index125(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,.1,.1+d1,B,VB,radius,0,d2)){
          ASSERT_TRUE(get(bitarray,index125(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index125(center,m,o,q)))missedOpportunities++;
        if(collisionImminent(A,VA,radius,0,d1,B,VB,radius,.2,.2+d2)){
          ASSERT_TRUE(get(bitarray,index125(center,m,o,q)));
          wasHelpful++;
        }else if(!get(bitarray,index125(center,m,o,q)))missedOpportunities++;
        total++;
      }
    }
  }
  std::cout << "helped " << wasHelpful << " missed " << missedOpportunities << " total " << total << "\n";
}

bool checkForCollision(xytLoc const& s1, xytLoc const& d1, xytLoc const& s2, xytLoc const& d2,float radius, MapEnvironment* env=nullptr,bool simple=false){
  if(s1==s2||d1==d2||s1.sameLoc(d2)&&s2.sameLoc(d1)){return false;}
  if(env && !env->collisionPreCheck(s1,d1,radius,s2,d2,radius,simple)) return false;
  Vector2D A(s1);
  Vector2D B(s2);
  Vector2D VA(d1);
  VA-=A;
  VA.Normalize();
  Vector2D VB(d2);
  VB-=B;
  VB.Normalize();
  if(collisionImminent(A,VA,radius,s1.t,d1.t,B,VB,radius,s2.t,d2.t)){
    //std::cout << "Collision at: " << s1 << "-->" << d1 << " " << s2 << "-->" << d2 << "\n";
    return true;
  }
  return false;
}

void countCollisions(std::vector<xytAABB> const& p1, std::vector<xytAABB> const& p2, float radius, unsigned& collisions, unsigned& total, MapEnvironment* env=nullptr,bool simple=false){
  auto a(p1.begin());
  auto b(p2.begin());
  while(a!=p1.end() && b!=p2.end()){
    ++total;
    if(a->overlaps(*b)){
      if(checkForCollision(*a->start,*a->end,*b->start,*b->end,radius,env,simple)){collisions++;}
    }
    if(fless(a->end->t,b->end->t+radius)){
      ++a;
    }else if(fgreater(a->end->t+radius,b->end->t)){
      ++b;
    }else{
      ++a;++b;
    }
  }
}

void countCollisionsItr(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2, float radius, unsigned& collisions, unsigned& total, MapEnvironment* env=nullptr,bool simple=false){
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    ++total;
    /*if(checkForCollision(*ap,*a,*bp,*b,radius,nullptr) && !checkForCollision(*ap,*a,*bp,*b,radius,env)){
      std::cout << "Missed collision for " << *ap << "-->" << *a << " "  << *bp << "-->" << *b << "\n";
      checkForCollision(*ap,*a,*bp,*b,radius,env);
    }*/
    if(checkForCollision(*ap,*a,*bp,*b,radius,env,simple)){collisions++;}
    if(fless(a->t,b->t+radius)){
      ++a;
      ++ap;
    }else if(fgreater(a->t+radius,b->t)){
      ++b;
      ++bp;
    }else{
      ++a;++b;
      ++ap;++bp;
    }
  }
}

void skipAndCountCollisions(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2, float radius, unsigned& collisions, unsigned& total, unsigned skip, MapEnvironment* env=nullptr,bool simple=false){
  unsigned a=1,b=1;
  while(a<p1.size() && b<p2.size()){
    ++total;
    unsigned sdiff(std::min(abs(p1[a].x-p2[b].x),abs(p1[a].y-p2[b].y))/(2*skip));
    if(sdiff>1){
      a+=sdiff-1;
      b+=sdiff-1;

      //std::cout << "Skip ahead " << (sdiff-2) << "\n";
      if(a>=p1.size() || b>=p2.size()){continue;}

      while(fgreater(p1[a-1].t,p2[b].t)){
        --a;
      }
      while(fgreater(p2[b-1].t,p1[a].t)){
        --b;
      }
    }
    if(std::min(p1[a-1].x,p1[a].x)>std::max(p2[b-1].x,p2[b].x)||
      std::max(p1[a-1].x,p1[a].x)<std::min(p2[b-1].x,p2[b].x)||
      std::min(p1[a-1].y,p1[a].y)>std::max(p2[b-1].y,p2[b].y)||
      std::max(p1[a-1].y,p1[a].y)<std::min(p2[b-1].y,p2[b].y)){;}else if(
    checkForCollision(p1[a-1],p1[a],p2[b-1],p2[b],radius,env,simple)){
      //std::cout << " " << a << "," << b << ";";
      collisions++;
    }
    if(fless(p1[a].t,p2[b].t+radius)){
      ++a;
    }else if(fgreater(p1[a].t+radius,p2[b].t)){
      ++b;
    }else{
      ++a;++b;
    }
  }
}


void countCollisions(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2, float radius, unsigned& collisions, unsigned& total, MapEnvironment* env=nullptr,bool simple=false){
  unsigned a=1,b=1;
  while(a<p1.size() && b<p2.size()){
    ++total;
    if(std::min(p1[a-1].x,p1[a].x)>std::max(p2[b-1].x,p2[b].x)||
      std::max(p1[a-1].x,p1[a].x)<std::min(p2[b-1].x,p2[b].x)||
      std::min(p1[a-1].y,p1[a].y)>std::max(p2[b-1].y,p2[b].y)||
      std::max(p1[a-1].y,p1[a].y)<std::min(p2[b-1].y,p2[b].y)){;}else if(
    checkForCollision(p1[a-1],p1[a],p2[b-1],p2[b],radius,env,simple)){collisions++;}
    if(fless(p1[a].t,p2[b].t+radius)){
      ++a;
    }else if(fgreater(p1[a].t+radius,p2[b].t)){
      ++b;
    }else{
      ++a;++b;
    }
  }
}

bool checkForCollisionWithBBox(xytLoc const& A1, xytLoc const& A2, xytLoc const& B1, xytLoc const& B2, float radius){
  unsigned dim(std::max(std::max(fabs(A1.x-A2.x),fabs(B1.x-B2.x)),std::max(fabs(A1.y-A2.y),fabs(B1.y-B2.y))));
  unsigned ssx(fabs(A1.x-B1.x));
  unsigned sdx(fabs(A1.x-B2.x));
  unsigned ssy(fabs(A1.y-B1.y));
  unsigned sdy(fabs(A1.y-B2.y));

  switch(dim){
    case 0:
    case 1:
        if(ssx<2 && ssy<2 && sdx <3 && sdy<3){
        }else if(sdx<2 && sdy<2 && ssx <3 && ssy<3){
        }else{return false;}
      break;
    case 2:
        if(ssx<3 && ssy<3 && sdx <5 && sdy<5){
        }else if(sdx<3 && sdy<3 && ssx <5 && ssy<5){
        }else{return false;}
      break;
    case 3:
        if(ssx<4 && ssy<4 && sdx <7 && sdy<7){
        }else if(sdx<4 && sdy<4 && ssx <7 && ssy<7){
        }else{return false;}
      break;
    default:
      break;
  };
  Vector2D A(A1);
  Vector2D B(B1);
  Vector2D VA(A2);
  VA-=A;
  VA.Normalize();
  Vector2D VB(B2);
  VB-=B;
  VB.Normalize();
  return collisionImminent(A,VA,radius,A1.t,A2.t,B,VB,radius,B1.t,B2.t);
}


unsigned countCollisionsWithBBox(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2, float radius){
  unsigned count(0);
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    if(checkForCollisionWithBBox(*ap,*a,*bp,*b,radius)){count++;}
    if(fless(a->t,b->t)){
      ++a;
      ++ap;
    }else if(fgreater(a->t,b->t)){
      ++b;
      ++bp;
    }else{
      ++a;++b;
      ++ap;++bp;
    }
  }
  return count;
}

unsigned getKey(int i, int j, int depth){
  return i*depth+j;
}
std::pair<unsigned, unsigned> unravelKey(unsigned key, int depth){
  return {key/depth,key%depth};
}

struct pathpointid{
  pathpointid(uint32_t k):agentid(k>>22),pointid(k&0x3fffff){}
  pathpointid(unsigned agent, unsigned point):agentid(agent),pointid(point){}
  unsigned pointid : 22; // Up to 262K points per agent
  unsigned agentid : 10; // Up to 1024 agents (high-order bits)
};

struct endpoint1{
  endpoint1():id(0,0),cvalue(0){}
  endpoint1(unsigned agent, unsigned point, unsigned dimension, xytLoc const& val):id(agent,point){
    if(dimension==0)
      value=val.t;
    else if(dimension==1)
      cvalue=val.x;
    else
      cvalue=val.y;
  }
  inline bool operator>(endpoint1 const& rhs)const{return cvalue==rhs.cvalue?id.agentid>rhs.id.agentid:cvalue>rhs.cvalue;}
  union{
    pathpointid id;
    uint32_t key;
  };
  union{
    float value;
    uint32_t cvalue; // comparison value (As long as value>0, this is safe to use as a comparison for floats
  };
};
static inline std::ostream& operator<<(std::ostream& os, endpoint1 const& v){os << v.cvalue; return os;}
/*struct cmp{ inline bool operator()(std::vector<endpoint1>::const_iterator& lhs, std::vector<endpoint1>::const_iterator& rhs)const{return lhs->cvalue==rhs->cvalue?lhs->key>rhs->key:lhs->cvalue>rhs->cvalue;} };

void mergeKSortedArrays(std::vector<std::vector<endpoint1>::const_iterator>& paths, std::vector<std::vector<endpoint1>::const_iterator> const& ends, std::vector<endpoint1>& sorted){
  std::make_heap(paths.begin(),paths.end(),cmp());
  size_t ix(0);
  while(paths.size()){
    std::pop_heap(paths.begin(),paths.end(),cmp());
    sorted[ix]=*paths.back()++;
    if(paths.back()==ends[sorted[ix++].id.agentid]){
      paths.pop_back();
    }
    std::push_heap(paths.begin(),paths.end(),cmp());
  }
}*/

void insertionSort(std::vector<endpoint1>& A){
  endpoint1 key(A[0]);
  int j(0);
  for (int i(1); i < A.size(); ++i){
    key = A[i];
    j = i-1;

    /* Move elements of A[0..i-1], that are greater than key, to one 
     *    position ahead of their current position.
     *        This loop will run at most k times */
    while (j >= 0 && A[j] > key){
      A[j+1] = A[j];
      j = j-1;
    }
    A[j+1] = key;
  }
}

struct ids{
  ids(pathpointid fir, pathpointid sec):first(fir),second(sec){
    if(fir.agentid>sec.agentid){
      second=fir;
      first=sec;
    }
  }
  pathpointid first;
  pathpointid second;
};

struct ID{
  ID(pathpointid fir, pathpointid sec):id(fir,sec){}
  union{
    ids id;
    uint64_t hash;
  };
  operator uint64_t(){return hash;}
};

void FindCollisions(std::vector<std::vector<endpoint1>> const& temp, std::vector<std::vector<xytLoc>> const& paths, std::vector<endpoint1>& axis, std::unordered_set<uint64_t>&pairs, std::vector<bool> const& reversed, MapEnvironment* env,float radius){
  std::unordered_set<uint32_t> active;
  active.emplace(axis[0].key);
  for(int i(1); i<axis.size(); ++i){
    for(auto const& val:active){
      pathpointid cand(val);
      if(cand.pointid-1 < temp[cand.agentid].size()){
        //std::cout << "Looking at " << axis[i].id.agentid << " " << axis[i].id.pointid << ":" << axis[i].cvalue << " vs " << cand.agentid << " " << cand.pointid << ":"<<temp[cand.agentid][cand.pointid+1].cvalue<<"\n";
        if(axis[i].cvalue>temp[cand.agentid][cand.pointid+1].cvalue){
          //std::cout << "removing " << cand.agentid << " " << cand.pointid << "\n";
          active.erase(val);
        }else{
          //std::cout << "inserting " << axis[i].id.agentid << " " << axis[i].id.pointid << "\n";
          pairs.insert(ID(val,axis[i].key));
        }
      }
      active.insert(axis[i].key);
    }
  }
}

void Update(std::vector<std::vector<endpoint1>> const& temp, std::vector<std::vector<xytLoc>> const& paths, std::vector<endpoint1>& axis, std::unordered_set<uint64_t>&pairs, std::vector<bool> const& reversed, MapEnvironment* env,float radius){
  for (int j = 1; j < axis.size(); j++){
    endpoint1 keyelement(axis[j]);
    int i(j - 1);
    while(i >= 0 && axis[i].cvalue > keyelement.cvalue){
      if(keyelement.id.pointid+2>temp[keyelement.id.agentid].size()){continue;} // This is an endpoint1
      endpoint1 swapper = axis[i];
      if(swapper.id.pointid+2>temp[keyelement.id.agentid].size()){continue;} // This is an endpoint1

      if(temp[swapper.id.agentid].size()<swapper.id.pointid+1 && keyelement.cvalue<=temp[swapper.id.agentid][swapper.id.pointid+1].cvalue){
        if(env->collisionPreCheck(
              reversed[swapper.id.agentid]?paths[swapper.id.agentid][paths[swapper.id.agentid].size()-swapper.id.pointid-1]:paths[swapper.id.agentid][swapper.id.pointid],
              reversed[swapper.id.agentid]?paths[swapper.id.agentid][paths[swapper.id.agentid].size()-swapper.id.pointid]:paths[swapper.id.agentid][swapper.id.pointid+1],
              radius,
              reversed[keyelement.id.agentid]?paths[keyelement.id.agentid][paths[keyelement.id.agentid].size()-keyelement.id.pointid-1]:paths[keyelement.id.agentid][keyelement.id.pointid],
              reversed[keyelement.id.agentid]?paths[keyelement.id.agentid][paths[keyelement.id.agentid].size()-keyelement.id.pointid]:paths[keyelement.id.agentid][keyelement.id.pointid+1],
              radius)){
          pairs.insert(ID(swapper.id,keyelement.id));
        }
      }else{
        pairs.erase(ID(swapper.id,keyelement.id));
      }

      axis[i + 1] = swapper;
      i = i - 1;
    }
    axis[i + 1] = keyelement;
  }
}

template<typename state, typename aabb>
void makeAABBs(std::vector<state> const& v,
    std::vector<aabb>& d, uint32_t agent, uint32_t dim)
{
    d.reserve(v.size()-1);
    auto first(v.cbegin());
    while (first+1 != v.end()) {
        d.emplace_back(&*first,&*first+1,agent,dim);
        ++first;
    }
}


// Take k semi-sorted arrays of paths in sorted time-order
// output d arrays, of sorted points, one for each dimension
template<typename aabb>
void createSortedLists(std::vector<std::vector<xytLoc>>const& paths, std::vector<std::vector<aabb>>& sorted, MapEnvironment* env, float radius=.25){
  //Assume 'sorted' has been initialized for the correct number of dimensions

  // Create D 1D AABBs
  // Append them to D vectors
  size_t total(0);
  std::vector<std::vector<std::vector<aabb>>> temp(sorted.size());
  for(int d(0); d<temp.size(); ++d){
    temp[d].resize(paths.size());
    for(int i(0); i<paths.size(); ++i){
      temp[d][i].reserve(paths[i].size());
      makeAABBs(paths[i],temp[d][i],i,d);
      if(d==0)
        total+=temp[i].size();
      sorted[d].insert(sorted[d].end(), temp[d][i].begin(), temp[d][i].end());
    }
    std::sort(sorted[d].begin(),sorted[d].end());
  }
}

// To copy pointers of an object into the destination array...
template<typename state, typename aabb>
void makeAABBs(std::vector<state> const& v,
    std::vector<aabb>& d, uint32_t agent)
{
  if(v.size()>1){
    d.reserve(v.size()-1);
    auto first(v.cbegin());
    while (first+1 != v.end()) {
      d.emplace_back(&*first,&*first+1,agent);
      ++first;
    }
  }
}

TEST(UTIL, CopyToPairs){
  std::vector<xytLoc> values={{1,1,1},{2,2,2},{3,3,3},{4,4,4},{5,5,5},{6,6,6},{7,7,7},{8,8,8},{9,9,9},{0,0,0}};
  std::vector<xytAABB> aabbs;
  aabbs.reserve(values.size()-1);
  makeAABBs(values,aabbs,0);
  ASSERT_EQ(9,aabbs.size());
  ASSERT_EQ(0,aabbs.back().end->x);
  ASSERT_EQ(1,aabbs.front().start->x);
}

TEST(UTIL, ConvexHullPoly){
  std::vector<xytLoc> values={{1,1,1},{2,3,2},{5,5,5},{6,6,6},{7,7,7},{8,8,8},{9,9,9}};
  std::vector<Vector2D> result;
  convexHull(values,result);
  ASSERT_EQ(3,result.size());
  ASSERT_EQ(Vector2D(2,3),result.front());
  ASSERT_EQ(Vector2D(9,9),result[1]);
  ASSERT_EQ(Vector2D(1,1),result[2]);
}

TEST(UTIL, convexHullLine){
  std::vector<xytLoc> values={{1,1,1},{2,2,2},{3,3,3},{4,4,4},{5,5,5},{6,6,6},{7,7,7},{8,8,8},{9,9,9},{0,0,0}};
  std::vector<Vector2D> result;
  convexHull(values,result);
  ASSERT_EQ(2,result.size());
  ASSERT_EQ(Vector2D(0,0),result.front());
  ASSERT_EQ(Vector2D(9,9),result.back());
}

TEST(UTIL, ConvexHullPoint){
  std::vector<xytLoc> values={{1,1,1}};
  std::vector<Vector2D> result;
  convexHull(values,result);
  ASSERT_EQ(1,result.size());
  ASSERT_EQ(Vector2D(1,1),result.front());
}

/*
TEST(UTIL, OverlapPolyPoly){
  std::vector<xytLoc> values1={{1,1,1},{1,9,1},{4,4,1}}; // Triangle
  std::vector<xytLoc> values2={{3,1,1},{3,9,1},{5,9,1},{5,1,1}}; // Rectangle
  std::vector<Vector2D> result1;
  std::vector<Vector2D> result2;
  convexHull(values1,result1);
  convexHull(values2,result2);
  ASSERT_EQ(true,sat(result1,result2));
  ASSERT_EQ(true,sat(result2,result1));
}

TEST(UTIL, OverlapPolyLine){
  std::vector<xytLoc> values1={{1,1,1},{2,2,1},{8,8,1}}; // Line
  std::vector<xytLoc> values2={{3,1,1},{3,9,1},{5,9,1},{5,1,1}}; // Rectangle
  std::vector<Vector2D> result1;
  std::vector<Vector2D> result2;
  convexHull(values1,result1);
  convexHull(values2,result2);
  ASSERT_EQ(true,sat(result1,result2));
  ASSERT_EQ(true,sat(result2,result1));
}

TEST(UTIL, OverlapPolyPoint){
  std::vector<xytLoc> values1={{4,8,1}}; // Point
  std::vector<xytLoc> values2={{3,1,1},{3,9,1},{5,9,1},{5,1,1}}; // Rectangle
  std::vector<Vector2D> result1;
  std::vector<Vector2D> result2;
  convexHull(values1,result1);
  convexHull(values2,result2);
  ASSERT_EQ(true,sat(result1,result2));
  ASSERT_EQ(true,sat(result2,result1));
}
*/

/*
void minmax_element(std::vector<xytLoc>::const_iterator& first, std::vector<xytLoc>::const_iterator& last, xytLoc& mn, xytLoc mx)
{
    mn=mx=*first;
 
    if (first == last) return;
    if (++first == last) return;
 
    if (comp(first.x<mn, *result.first)) {
        result.first = first;
    } else {
        result.second = first;
    }
    while (++first != last) {
        ForwardIt i = first;
        if (++first == last) {
            if (comp(*i, *result.first)) result.first = i;
            else if (!(comp(*i, *result.second))) result.second = i;
            break;
        } else {
            if (comp(*first, *i)) {
                if (comp(*first, *result.first)) result.first = first;
                if (!(comp(*i, *result.second))) result.second = i;
            } else {
                if (comp(*i, *result.first)) result.first = i;
                if (!(comp(*first, *result.second))) result.second = first;
            }
        }
    }
    return result;
}*/

// Merge two sorted arrays of pointers
template<class InputIt1, class InputIt2, class OutputIt>
OutputIt mergePtrs(InputIt1 first1, InputIt1 last1,
               InputIt2 first2, InputIt2 last2,
               OutputIt d_first)
{
    for (; first1 != last1; ++d_first) {
        if (first2 == last2) {
            return std::copy(first1, last1, d_first);
        }
        if (first2->lowerBound[0].cvalue < first1->lowerBound[0].cvalue) {
            *d_first = *first2;
            ++first2;
        } else {
            *d_first = *first1;
            ++first1;
        }
    }
    return std::copy(first2, last2, d_first);
}

template<typename aabb>
void mergeVecs(std::vector<std::vector<aabb>>& tosort, std::vector<aabb>& sorted){
  if(tosort.size()==2){
    mergePtrs(tosort.front().begin(),tosort.front().end(),tosort.back().begin(),tosort.back().end(),sorted.begin());
  }else{
    // Get merged sets of 2
    std::vector<std::vector<aabb>> temp(tosort.size()/2);
    for(int i(0); i<temp.size(); ++i){
      temp[i].resize(tosort[i*2].size()+tosort[i*2+1].size());
      mergePtrs(tosort[i*2].begin(),tosort[i*2].end(),tosort[i*2+1].begin(),tosort[i*2+1].end(),temp[i].begin());
    }
    // Even out the number of sets
    if(temp.size()%2){
      temp.resize(temp.size()+1);
    }
    mergeVecs(temp,sorted);
  }
}

template<typename state, typename aabb>
void merge(std::vector<std::vector<state>> const& solution, std::vector<aabb>& sorted){
  unsigned total(0);
  std::vector<std::vector<aabb>> temp(solution.size());
  std::vector<std::vector<aabb*>> ptrs(solution.size());
  for(int i(0); i<solution.size(); ++i){
    temp[i].reserve(solution[i].size());
    //ptrs[i].reserve(solution[i].size());
    makeAABBs(solution[i],temp[i],i);
    total+=temp[i].size();
  }
  sorted.resize(total);

  // Even out the number of sets
  if(solution.size()%2){
    temp.resize(temp.size()+1);
  }
  mergeVecs(temp,sorted);
}

template<typename aabb>
void getAllPairsSAP(std::vector<aabb> const& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
  auto a(sorted.begin()+1);
  unsigned touched(0);
  unsigned compared(0);
  unsigned npairs(0);
  std::vector<aabb const*> active;
  active.push_back(&*sorted.begin());
  while(a!=sorted.end()){
    touched++;
    //std::cout << "<"<<a->lowerBound[0].value<<"~"<<a->upperBound[0].value<<","<<a->lowerBound[1].cvalue<<"~"<<a->upperBound[1].cvalue<<","<<a->lowerBound[2].cvalue<<"~"<<a->upperBound[2].cvalue<<">\n";
    for(auto b(active.begin()); b!=active.end(); /*++b*/){
      compared++;
        //std::cout<<"  <"<<b->lowerBound[0].value<<"~"<<b->upperBound[0].value<<","<<b->lowerBound[1].cvalue<<"~"<<b->upperBound[1].cvalue<<","<<b->lowerBound[2].cvalue<<"~"<<b->upperBound[2].cvalue<<">\n";
      if((*b)->upperBound[0].cvalue<a->lowerBound[0].cvalue){
        active.erase(b);
        continue;
      }
      if(a->agent!=(*b)->agent &&
            a->upperBound[1].cvalue>=(*b)->lowerBound[1].cvalue &&
            a->lowerBound[1].cvalue<=(*b)->upperBound[1].cvalue &&
            a->lowerBound[2].cvalue<=(*b)->upperBound[2].cvalue &&
            a->upperBound[2].cvalue>=(*b)->lowerBound[2].cvalue){
          pairs.emplace_back(*a,**b);
          //std::cout << "***\n";
      }
      ++b;
    }
    active.push_back(&*a);
    //std::cout << "-------\n";
    ++a;
  }
  //std::cout << "touched " << touched << "\n";
  //std::cout << "compared " << compared << "\n";
  //std::cout << "pairs " << pairs.size() << "\n";
}

template<typename aabb>
void getAllPairsX(std::vector<aabb> const& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
  auto a(sorted.cbegin());
  auto b(sorted.cbegin()+1);
  unsigned touched(0);
  unsigned compared(0);
  unsigned npairs(0);
  while(a!=sorted.end()){
    touched++;
    //std::cout << "<"<<a->lowerBound[0].value<<"~"<<a->upperBound[0].value<<","<<a->lowerBound[1].cvalue<<"~"<<a->upperBound[1].cvalue<<","<<a->lowerBound[2].cvalue<<"~"<<a->upperBound[2].cvalue<<">\n";
    while(b!=sorted.end() && a->upperBound[1].cvalue>=b->lowerBound[1].cvalue){
      compared++;
        //std::cout<<"  <"<<b->lowerBound[0].value<<"~"<<b->upperBound[0].value<<","<<b->lowerBound[1].cvalue<<"~"<<b->upperBound[1].cvalue<<","<<b->lowerBound[2].cvalue<<"~"<<b->upperBound[2].cvalue<<">\n";
        if(a->agent!=b->agent &&
            a->upperBound[0].cvalue>=b->lowerBound[0].cvalue &&
            a->lowerBound[0].cvalue<=b->upperBound[0].cvalue &&
            a->lowerBound[2].cvalue<=b->upperBound[2].cvalue &&
            a->upperBound[2].cvalue>=b->lowerBound[2].cvalue){
          pairs.emplace_back(*a,*b);
          //std::cout << "***\n";
        }
      ++b;
    }
    //std::cout << "-------\n";
    b=++a;
    ++b;
  }
  //std::cout << "touched " << touched << "\n";
  //std::cout << "compared " << compared << "\n";
  //std::cout << "pairs " << pairs.size() << "\n";
}
template<typename aabb>
void getPathCollisionsAndReplaceSAPT(std::vector<aabb> const& path, std::vector<aabb>& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
  unsigned a(0);
  unsigned b(0);
  /*for(auto const& vv:path){
    std::cout << vv.lowerBound[0].cvalue << " ";
  }
  std::cout << "\n";
  for(auto const& vv:sorted){
    if(vv.agent==path[b].agent) std::cout << "*";
    std::cout << vv.lowerBound[0].cvalue << " ";
  }
  std::cout << "\n";*/
  while(a<sorted.size() && b<path.size() &&
        sorted[a].lowerBound[0].cvalue<path[b].lowerBound[0].cvalue){
    if(sorted[a].agent==path[b].agent){
      sorted.erase(sorted.begin()+a); // erase and skip
    }else{
    ++a;
    }
  }
  bool add(true);
  while(a<sorted.size() && b<path.size()){
    if(sorted[a].agent==path[b].agent){
      sorted.erase(sorted.begin()+a); // erase and skip
      continue;
    }else if(path[b].upperBound[0].cvalue<sorted[a].lowerBound[0].cvalue){
      ++b;
      add=true;
      continue;
    }else if(add && path[b].lowerBound[0].cvalue<sorted[a].lowerBound[0].cvalue){
      sorted.insert(sorted.begin()+a,path[b]);
      add=false;
    }else if(
        sorted[a].upperBound[1].cvalue>=path[b].lowerBound[1].cvalue &&
        sorted[a].lowerBound[1].cvalue<=path[b].upperBound[1].cvalue &&
        sorted[a].lowerBound[2].cvalue<=path[b].upperBound[2].cvalue &&
        sorted[a].upperBound[2].cvalue>=path[b].lowerBound[2].cvalue){
      pairs.emplace_back(sorted[a],path[b]);
    }
    ++a;
  }
  while(a==sorted.size() && b<path.size()){
    sorted.push_back(path[b]);
  }
  /*for(auto const& vv:sorted){
    if(vv.agent==path[0].agent) std::cout << "*";
    std::cout << vv.lowerBound[0].cvalue << " ";
  }*/
}

template<typename aabb>
void getPathCollisionsAndReplaceSAPX(std::vector<aabb> const& path, std::vector<aabb>& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
  unsigned a(0);
  unsigned b(0);
  /*for(auto const& vv:path){
    std::cout << vv.lowerBound[1].cvalue << " ";
  }
  std::cout << "\n";
  for(auto const& vv:sorted){
    if(vv.agent==path[b].agent) std::cout << "*";
    std::cout << vv.lowerBound[1].cvalue << " ";
  }
  std::cout << "\n";*/
  while(a<sorted.size() && b<path.size() &&
        sorted[a].lowerBound[1].cvalue<path[b].lowerBound[1].cvalue){
    if(sorted[a].agent==path[b].agent){
      sorted.erase(sorted.begin()+a); // erase and skip
    }else{
    ++a;
    }
  }
  bool add(true);
  while(a<sorted.size() && b<path.size()){
    if(sorted[a].agent==path[b].agent){
      sorted.erase(sorted.begin()+a); // erase and skip
      continue;
    }else if(path[b].upperBound[1].cvalue<sorted[a].lowerBound[1].cvalue){
      ++b;
      add=true;
      continue;
    }else if(add && path[b].lowerBound[1].cvalue<sorted[a].lowerBound[1].cvalue){
      sorted.insert(sorted.begin()+a,path[b]);
      add=false;
    }else if(
        sorted[a].upperBound[0].cvalue>=path[b].lowerBound[0].cvalue &&
        sorted[a].lowerBound[0].cvalue<=path[b].upperBound[0].cvalue &&
        sorted[a].lowerBound[2].cvalue<=path[b].upperBound[2].cvalue &&
        sorted[a].upperBound[2].cvalue>=path[b].lowerBound[2].cvalue){
      pairs.emplace_back(sorted[a],path[b]);
    }
    ++a;
  }
  while(a==sorted.size() && b<path.size()){
    sorted.push_back(path[b]);
  }
  /*for(auto const& vv:sorted){
    if(vv.agent==path[0].agent) std::cout << "*";
    std::cout << vv.lowerBound[1].cvalue << " ";
  }*/
}


template<typename aabb>
void getAllPairsSAPX(std::vector<aabb> const& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
  auto a(sorted.begin()+1);
  unsigned touched(0);
  unsigned compared(0);
  unsigned npairs(0);
  std::vector<aabb const*> active;
  active.push_back(&*sorted.begin());
  while(a!=sorted.end()){
    touched++;
    //std::cout << "<"<<a->lowerBound[0].value<<"~"<<a->upperBound[0].value<<","<<a->lowerBound[1].cvalue<<"~"<<a->upperBound[1].cvalue<<","<<a->lowerBound[2].cvalue<<"~"<<a->upperBound[2].cvalue<<">\n";
    for(auto b(active.begin()); b!=active.end(); /*++b*/){
      compared++;
        //std::cout<<"  <"<<b->lowerBound[0].value<<"~"<<b->upperBound[0].value<<","<<b->lowerBound[1].cvalue<<"~"<<b->upperBound[1].cvalue<<","<<b->lowerBound[2].cvalue<<"~"<<b->upperBound[2].cvalue<<">\n";
      if((*b)->upperBound[1].cvalue<a->lowerBound[1].cvalue){
        active.erase(b);
        continue;
      }
      if(a->agent!=(*b)->agent &&
            a->upperBound[0].cvalue>=(*b)->lowerBound[0].cvalue &&
            a->lowerBound[0].cvalue<=(*b)->upperBound[0].cvalue &&
            a->lowerBound[2].cvalue<=(*b)->upperBound[2].cvalue &&
            a->upperBound[2].cvalue>=(*b)->lowerBound[2].cvalue){
          pairs.emplace_back(*a,**b);
          //std::cout << "***\n";
      }
      ++b;
    }
    active.push_back(&*a);
    //std::cout << "-------\n";
    ++a;
  }
  //std::cout << "touched " << touched << "\n";
  //std::cout << "compared " << compared << "\n";
  //std::cout << "pairs " << pairs.size() << "\n";
}


template<typename aabb>
void getAllPairs(std::vector<aabb> const& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
  auto a(sorted.cbegin());
  auto b(sorted.cbegin()+1);
  unsigned touched(0);
  unsigned compared(0);
  unsigned npairs(0);
  while(a!=sorted.end()){
    touched++;
    //std::cout << "<"<<a->lowerBound[0].value<<"~"<<a->upperBound[0].value<<","<<a->lowerBound[1].cvalue<<"~"<<a->upperBound[1].cvalue<<","<<a->lowerBound[2].cvalue<<"~"<<a->upperBound[2].cvalue<<">\n";
    while(b!=sorted.end() && a->upperBound[0].cvalue>=b->lowerBound[0].cvalue){
      compared++;
        //std::cout<<"  <"<<b->lowerBound[0].value<<"~"<<b->upperBound[0].value<<","<<b->lowerBound[1].cvalue<<"~"<<b->upperBound[1].cvalue<<","<<b->lowerBound[2].cvalue<<"~"<<b->upperBound[2].cvalue<<">\n";
        if(a->agent!=b->agent &&
            a->upperBound[1].cvalue>=b->lowerBound[1].cvalue &&
            a->lowerBound[1].cvalue<=b->upperBound[1].cvalue &&
            a->lowerBound[2].cvalue<=b->upperBound[2].cvalue &&
            a->upperBound[2].cvalue>=b->lowerBound[2].cvalue){
          pairs.emplace_back(*a,*b);
          //std::cout << "***\n";
        }
      ++b;
    }
    //std::cout << "-------\n";
    b=++a;
    ++b;
  }
  //std::cout << "touched " << touched << "\n";
  //std::cout << "compared " << compared << "\n";
  //std::cout << "pairs " << pairs.size() << "\n";
}

void replaceAABBs(std::vector<xytAABB> const& n, std::vector<xytAABB>& sorted){
  //{
    //std::cout << "WITH: \n";
    //for(auto nn(n.begin()); nn!= n.end(); ++nn){
      //std::cout << "<" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << "><"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    //}
    // Verify that sorting property is still good
    //auto mm(sorted.begin());
    //for(auto nn(mm+1); nn!=sorted.end(); ++nn){
      //if(!(*mm<*nn || *mm==*nn)){
        //std::cout << "BAD BEFORE!<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "><"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      //}else{
        //std::cout << "BEFORE:<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << ">,<"<<mm->upperBound[0].value << "," << mm->upperBound[1].cvalue << "," << mm->upperBound[2].cvalue << ">\n";
      //}
      //ASSERT_LE(*mm,*nn);
      //ASSERT_TRUE(*mm<*nn || *mm==*nn);
      //++mm;
    //}
  //}
  auto ni(n.begin()); // New
  unsigned agent(ni->agent);
  auto oi(sorted.begin());
  while(oi!=sorted.end()){
    if(ni!=n.end()&&*ni<*oi){
      sorted.insert(oi,*ni);
      ++ni;
      ++oi;
    }else if(oi->agent==agent){
      sorted.erase(oi);
    }else{
     ++oi;
    }
  }
  while(ni!=n.end()){
    sorted.push_back(*ni);
    ++ni;
  }
  if(false){
    // Verify that sorting property is still good
    auto mm(sorted.begin());
    for(auto nn(mm+1); nn!=sorted.end(); ++nn){
      if(!(*mm<*nn || *mm==*nn)){
        //std::cout << "BAD AFTER! <" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "> >= <"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }else{
        //std::cout << "AFTER:<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << ">,<"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }
      ++mm;
      //ASSERT_LE(*mm,*nn);
      ASSERT_TRUE(*mm<*nn || *mm==*nn);
    }
  }
}

void replaceAABBs(std::vector<xytAABB> const& o, std::vector<xytAABB> const& n, std::vector<xytAABB>& sorted){

  if(false){
    //std::cout << "REPLACE: \n";
    for(auto nn(o.begin()); nn!= o.end(); ++nn){
      //std::cout << "<" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << "><"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    }
    //std::cout << "WITH: \n";
    for(auto nn(n.begin()); nn!= n.end(); ++nn){
      //std::cout << "<" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << "><"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    }
    // Verify that sorting property is still good
    auto mm(sorted.begin());
    for(auto nn(mm+1); nn!=sorted.end(); ++nn){
      if(!(*mm<*nn || *mm==*nn)){
        //std::cout << "BAD BEFORE!<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "><"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }else{
        //std::cout << "BEFORE:<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << ">,<"<<mm->upperBound[0].value << "," << mm->upperBound[1].cvalue << "," << mm->upperBound[2].cvalue << ">\n";
      }
      //ASSERT_LE(*mm,*nn);
      ASSERT_TRUE(*mm<*nn || *mm==*nn);
      ++mm;
    }
  }
  auto oi(o.begin()); // New
  auto ni(n.begin()); // New
  auto beforen(sorted.begin());
  auto ato(std::lower_bound(sorted.begin(),sorted.end(),*oi)); // Old
  while(oi!=o.end()&&ni!=n.end()){
    //std::cout << "old ["<<(ato-sorted.begin())<<"] <" << oi->lowerBound[0].value << "," << oi->lowerBound[1].cvalue << "," << oi->lowerBound[2].cvalue << "><" << oi->upperBound[0].value << "," << oi->upperBound[1].cvalue << "," << oi->upperBound[2].cvalue << ">\n";
    if(*oi<*ni){
      beforen=std::lower_bound(ato,sorted.end(),*ni);
    }else if(*ni<*oi){
      beforen=std::lower_bound(beforen,ato,*ni);
    }else{
      beforen=ato;
    }
    //std::cout << "new ["<<(beforen-sorted.begin())<<"] <" << ni->lowerBound[0].value << "," << ni->lowerBound[1].cvalue << "," << ni->lowerBound[2].cvalue << "><" << ni->upperBound[0].value << "," << ni->upperBound[1].cvalue << "," << ni->upperBound[2].cvalue << ">\n";
    if(ato<beforen){
      std::rotate(ato,ato+1,beforen); // Move element o to the new index
      --beforen;
    }
    else if(ato>beforen)
      std::rotate(beforen,ato,ato+1); // Move element o to the new index
    *beforen=*ni; // replace value of o with n
    ++ni;
    if(++oi==o.end()){break;}
    ato=std::lower_bound(ato,sorted.end(),*oi); // Old
  }
  while(oi!=o.end()){
    std::remove(ato++,sorted.end(),*oi++);
  }
  while(ni!=n.end()){
    sorted.insert(std::lower_bound(ato++,sorted.end(),*ni),*ni);
    ++ni;
  }
  if(false){
    // Verify that sorting property is still good
    auto mm(sorted.begin());
    for(auto nn(mm+1); nn!=sorted.end(); ++nn){
      if(!(*mm<*nn || *mm==*nn)){
        //std::cout << "BAD AFTER! <" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "> >= <"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }else{
        //std::cout << "AFTER:<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << ">,<"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }
      ++mm;
      //ASSERT_LE(*mm,*nn);
      //ASSERT_TRUE(*mm<*nn || *mm==*nn);
    }
  }
}

/*TEST(Theta1, GetObstructedPath2){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(4,6,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  menv.SetFiveConnected();
  EPEThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{0,5,0},{6,6,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "\n";
}

TEST(EPETheta, GetObstructedPath){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,1,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  menv.SetFiveConnected();
  EPEThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1,0},{7,3,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << "\n";
}

TEST(EPETheta, GetObstructedPathComp){
  Map map(8,8);
  MapEnvironment menv(&map);
  menv.SetFiveConnected();
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  // Set some obstacles...
  for(int i(0); i<10; ++i){
    map.SetTerrainType(rand()%8,rand()%8,kOutOfBounds);
  }

  for(int i(0); i<10; ++i){

    xytLoc start(rand()%8,rand()%8,0);
    while(!map.IsTraversable(start.x,start.y)){
      start.x=rand()%8;
      start.y=rand()%8;
    }
    xytLoc goal(rand()%8,rand()%8,0);
    while(!map.IsTraversable(goal.x,goal.y)){
      goal.x=rand()%8;
      goal.y=rand()%8;
    }
    std::cout << "search: " << start << "-->" << goal << "\n";
    for(int y(7);y>=0; --y){
      std::cout << y << ": ";
      for(int x(0);x<8; ++x){
        if(map.IsTraversable(x,y)){
          if(start.x==x && start.y==y)std::cout<<"s";
          else if(goal.x==x && goal.y==y)std::cout<<"g";
          else std::cout<<".";
        } else std::cout<<"#";
      }
      std::cout << ":\n";
    }

    std::vector<xytLoc> solution1;
    {
      ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
      tstar.SetHeuristic(new StraightLineHeuristic());
      tstar.SetVerbose(true);
      tstar.GetPath(&env,start,goal,solution1);
      std::cout << "Theta expanded" << tstar.GetNodesExpanded() << "\n";
      for(int i(1);i<solution1.size(); ++i){
        ASSERT_TRUE(env.LineOfSight(solution1[i-1],solution1[i]));
      }
      for(auto const& ss: solution1){
        std::cout << ss.x << "," << ss.y << "\n";
      }
    }

    std::vector<xytLoc> solution;
    {
      EPEThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
      tstar.SetHeuristic(new StraightLineHeuristic());
      tstar.SetVerbose(true);
      tstar.GetPath(&env,start,goal,solution);
      std::cout << "EPETheta expanded" << tstar.GetNodesExpanded() << "\n";
    }
    for(auto const& ss: solution){
      std::cout << ss.x << "," << ss.y << "\n";
    }
    ASSERT_EQ(solution1.size(),solution.size());
    for(int i(1);i<solution.size(); ++i){
      ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
      std::cout << solution1[i] << " ==? " << solution[i] << "\n";
      ASSERT_TRUE(solution1[i]==solution[i]);
    }
  }
  std::cout << "\n";
}*/
#endif
