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

float Util::closestDistanceBetweenLineSegments(Vector2D const& s1, Vector2D const& d1, Vector2D const& s2, Vector2D const& d2)
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
}
