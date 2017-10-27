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

// From http://www.cplusplus.com/forum/beginner/49408/

double Util::linesIntersect(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, double* out){
  Vector2D a(A2-A1);
  Vector2D b(B2-B1);

  double f(det(a,b));
  if(!f)      // lines are parallel
    return false;

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

  // If A and B are parallel, then we can just check the distance
  double r(r1+r2);
  if((NA==NB || NA==-NB) && fless(distanceOfPointToLine(A1,A2,B1),r)){return true;}

  // Project along normal in both directions
  Vector2D A11(A1+NB);
  Vector2D A21(A2+NB);
  Vector2D A12(A1-NB);
  Vector2D A22(A2-NB);

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
