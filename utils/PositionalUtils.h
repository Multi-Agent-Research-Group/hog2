//
//  PositionalUtils.h
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//
#ifndef PositionalUtils_h_
#define PositionalUtils_h_

#include <cmath>
#include <stdlib.h>
#include "Vector2D.h"

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

// Get the heading from a to b
template<unsigned steps360>
inline unsigned heading(double x1, double y1, double x2, double y2){
  return unsigned(round((M_PI/2.0-atan2(y2-y1,x2-x1))*(steps360/2)+steps360))%steps360;
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

// Detect whether collision is occurring or will occur between 2 agents
// placed at pi and pj with velocity and radius.
bool collisionImminent(Vector2D const& pi, Vector2D const& vi, Vector2D const& pj, Vector2D const& vj, double r1, double r2){
  double r=r1+r2; // Combined radius
  Vector2D w(pi+pj);
  double c(w.sq()-r*r);
  if(c<0){return true;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector2D v(vi-vj);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0) || fless(b-sqrt(dscr)/a,0)){
    return false;
  }
  return true;
};

#endif
