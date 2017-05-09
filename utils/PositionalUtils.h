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

};

#endif
