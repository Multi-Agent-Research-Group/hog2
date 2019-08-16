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
#include "VelocityObstacle.h"
#include "Timer.h"
#include <assert.h>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include "string.h"
//#include <CGAL/Boolean_set_operations_2.h>
//#include <CGAL/Polygon_2.h>

VelocityObstacle::VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, double r1, double r2)
  : VO(a+vb), VL(0,0), VR(0,0)
{
  if(r2==DBL_MAX){r2=r1;}
  // Compute VL and VR
  std::vector<Vector2D> tangents;
  if(getTangentOfCircle(b+vb,r1+r2,VO,tangents)){
    VL=tangents[1];
    VR=tangents[0];
  }else{ // Punt. This should *never* happen
    VL=normal(VO,b+vb);
    VR=-VL;
  }
}

// Polygon points are relative to the center fo the body (a, b respectively)
/*VelocityObstacle::VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, std::vector<Vector2D>const& polyA, std::vector<Vector2D>const& polyB)
  : VO(a+vb), VL(0,0), VR(0,0)
{
  // Compute points in minkowski sum
  std::vector<Vector2D> points;
  points.reserve(polyA.size()*polyB.size()+1);

  Vector2D center(b+vb);
  for(auto const& pa:polyA){
    for(auto const& pb:polyB){
      points.push_back(center+pa+pb);
    }
  }
  
  std::vector<Vector2D> result;
  points.push_back(VO); // Add apex
  Util::convexHull(points,result);

  // Compute VL and VR
  int i=0;
  // Find the apex in the convex hull
  for(; i<result.size(); ++i){
    if(result[i] == VO){
      break;
    }
  }
  // VL and VR are on either side of the apex
  if(i==0){
    VL=result.back();
    VR=result[1];
  }
  else if(i==result.size()-1){
    VL=result[result.size()-2];
    VR=result.front();
  }else{
    VL=result[i-1];
    VR=result[i+1];
  }

  // If the center point is not between the tangents, swap them
  if(!IsInside(center)){
    Vector2D tmp(VL);
    VR=tmp;
    VL=VR;
  }
}*/

bool getTangentOfCircle(Vector2D const& center, double radius, Vector2D const& point, std::vector<Vector2D>& tangents){
  assert(fless(0,radius)); //If the agent has no size, there's no VOB to compute
 
  Vector2D n((point-center)/radius);
  double xy=n.sq();

  if(fleq(xy,1.0)){return false;} // point is in or on circle

  tangents.resize(2);
  double d(n.y*sqrt(xy-1.0));
  double tx0((n.x-d)/xy);
  double tx1((n.x+d)/xy);
  if(!fequal(0.0,n.y)){
    tangents[0].y=center.y+radius*(1.0-tx0*n.x)/n.y;
    tangents[1].y=center.y+radius*(1.0-tx1*n.x)/n.y;
  }else{
    d=radius*sqrt(1.0-tx0*tx0);
    tangents[0].y=center.y+d;
    tangents[1].y=center.y-d;
  }
  tangents[0].x=center.x+radius*tx0;
  tangents[1].x=center.x+radius*tx1;
  return true;
}

// Input is the two center points and their radiuses
/*bool VelocityObstacle::AgentOverlap(Vector2D const& A,Vector2D const& B,std::vector<Vector2D>const& polyA,std::vector<Vector2D>const& polyB){
  Points a;
  a.reserve(polyA.size());
  for(auto const& aa:polyA){
    a.push_back(A+aa);
  }
  Points b;
  b.reserve(polyB.size());
  for(auto const& bb:polyB){
    b.push_back(B+bb);
  }
  return CGAL::do_intersect(Polygon_2(a.begin(),a.end()),Polygon_2(b.begin(),b.end()));
}*/

// Input is the two center points and their radiuses
bool VelocityObstacle::AgentOverlap(Vector2D const& A,Vector2D const& B,double ar,double br){
  return fless((B-A).sq(),(ar+br)*(ar+br));
}

// Input should be a point relative to A after a normalized time step.
// e.g. VOB velocities are normalized, so the input point should be normalized
// from: http://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
bool VelocityObstacle::IsInside(Vector2D const& point) const{
     Vector2D apexDiff(point-VO);
     //Check if it is strictly right of the left vector
     return ((VL.x - VO.x)*(apexDiff.y) < (VL.y - VO.y)*(apexDiff.x)) &&
     // Check if it is strictly left of the right vector
       ((VR.x - VO.x)*(apexDiff.y) > (VR.y - VO.y)*(apexDiff.x));
}

