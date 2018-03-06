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

#include "VelocityObstacle.h"
#include "Timer.h"
#include <gtest/gtest.h>
#include <map>
#include "Map2DConstrainedEnvironment.h"
#include "Grid3DConstrainedEnvironment.h"
#include "AnyAngleSipp.h"
#include "ThetaStar.h"
#include "PositionalUtils.h"
#include "AABB.h"
#include <unordered_set>
#include <sstream>
#include <algorithm>
//#include "EPEThetaStar.h"
//#include "PEThetaStar.h"

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
  ASSERT_TRUE(detectCollision(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(detectCollision(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose edges end before collision actually occurs (at time step 2.2)
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,2.2,B,VB,radius,0.0,2.2));
  // Suppose agents start at a different time
  ASSERT_FALSE(detectCollision(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(detectCollision(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(detectCollision(A,VA*2,radius,1.0,4.0,B,VB,radius,0.0,6.0));
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
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose agents start at a different time
  // This is a crash since agent B cuts through the corner while A is there
  // and both of them take up 80% of the space
  ASSERT_TRUE(detectCollision(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Crash does not occur with smaller radius size.
  ASSERT_FALSE(detectCollision(A,VA,.25,1.0,3.2,B,VB,.25,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_TRUE(detectCollision(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Crash does not occur with smaller radius size.
  ASSERT_FALSE(detectCollision(A,VA,.25,0.0,6.0,B,VB*2,.25,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(detectCollision(A,VA*2,radius,1.0,6.0,B,VB,radius,0.0,6.0));
  ASSERT_TRUE(detectCollision(A,VA,radius,0.0,6.0,B,VB*2,radius,1.0,6.0));
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
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,2.3,B,VB,radius,0.0,2.3));
  // Suppose agents start at a different time
  ASSERT_FALSE(detectCollision(A,VA,radius,1.0,3.2,B,VB,radius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,radius,0.0,3.0,B,VB,radius,0.0,6.0));
  ASSERT_FALSE(detectCollision(A,VA,radius,0.0,6.0,B,VB*2,radius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,radius,0.0,3.0,B,VB*2,radius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_FALSE(detectCollision(A,VA*2,radius,1.0,4.0,B,VB,radius,0.0,6.0));
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
  ASSERT_FALSE(detectCollision(A,VA,aradius,0.0,6.0,B,VB,bradius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_FALSE(detectCollision(A,VA,aradius,0.0,2.3,B,VB,bradius,0.0,2.3));
  // Suppose agents start at a different time
  ASSERT_TRUE(detectCollision(A,VA,aradius,2.0,3.2,B,VB,bradius,0.0,2.2));
  // Suppose one agent is moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,aradius,0.0,3.0,B,VB,bradius,0.0,6.0));
  ASSERT_TRUE(detectCollision(A,VA,aradius,0.0,6.0,B,VB*2.2,bradius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_FALSE(detectCollision(A,VA*2,aradius,0.0,3.0,B,VB*2,bradius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(detectCollision(A,VA,aradius,1.0,8.0,B,VB*2.2,bradius,0.0,6.0));
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
  ASSERT_TRUE(detectCollision(A,VA,aradius,0.0,6.0,B,VB,bradius,0.0,6.0));
  // Suppose edges end at the same point
  ASSERT_TRUE(detectCollision(A,VA,aradius,0.0,2.0,B,VB,bradius,0.0,2.0));
  // Suppose edges end before collision
  ASSERT_FALSE(detectCollision(A,VA,aradius,0.0,1.0,B,VB,bradius,0.0,1.0));
  // Suppose agents start at a different time
  ASSERT_TRUE(detectCollision(A,VA,aradius,1,4,B,VB,bradius,0.0,3));
  ASSERT_FALSE(detectCollision(A,VA,aradius,1.01,2.0,B,VB,bradius,0.0,2.0));
  // Agents stop in a collision state
  ASSERT_TRUE(detectCollision(A,VA,aradius,0.0,1.8,B,VB,bradius,0.0,1.8));
  // Suppose one agent is moving faster
  ASSERT_TRUE(detectCollision(A,VA*2,aradius,0.0,3.0,B,VB,bradius,0.0,6.0));
  ASSERT_TRUE(detectCollision(A,VA,aradius,0.0,6.0,B,VB*2,bradius,0.0,3.0));
  // Suppose both agents are moving faster
  ASSERT_TRUE(detectCollision(A,VA*2,aradius,0.0,3.0,B,VB*2,bradius,0.0,3.0));
  // Suppose one agent is moving faster, but starting later
  ASSERT_TRUE(detectCollision(A,VA,aradius,1.0,8.0,B,VB*2.2,bradius,0.0,6.0));
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

float rfloat(float low=-5, float high=5){
    float width(high-low);
    return float(rand()%int(width*1000))/(width*100.0) + low;
}

TEST(VelocityObstacle, PerfTest){
  //Timer t;
  //t.StartTimer();
  for(int i(0); i<10; ++i){
    detectCollision(Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,10),rfloat(0,10),Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,10),rfloat(0,10));
  }
  //std::cout << "Total time (VelocityObstacle)" << t.EndTimer() << "\n";
}

TEST(Quadratic, PerfTest){
  //Timer t;
  //t.StartTimer();
  for(int i(0); i<10; ++i){
    collisionImminent(Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,10),rfloat(0,10),Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,10),rfloat(0,10));
  }
  //std::cout << "Total time (Quadratic)" << t.EndTimer() << "\n";
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
  ASSERT_DOUBLE_EQ(4.05028,intvl2.first);
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

TEST(AnyAngle, GetPath){
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
}

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

/*
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
}*/

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

TEST(Theta1, TestStepAside){
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
  ASSERT_TRUE(Util::fatLinesIntersect({2,2},{2,3},.25,{2,3},{1,3},.25));
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

void countCollisions(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2, float radius, unsigned& collisions, unsigned& total, MapEnvironment* env=nullptr,bool simple=false){
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
struct cmp{ inline bool operator()(std::vector<endpoint1>::const_iterator& lhs, std::vector<endpoint1>::const_iterator& rhs)const{return lhs->cvalue==rhs->cvalue?lhs->key>rhs->key:lhs->cvalue>rhs->cvalue;} };

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
}

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
    d.reserve(v.size()-1);
    auto first(v.cbegin());
    while (first+1 != v.end()) {
        d.emplace_back(&*first,&*first+1,agent);
        ++first;
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
        if (*first2 < *first1) {
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
void getAllPairs2(std::vector<aabb> const& sorted, std::vector<std::pair<aabb,aabb>>& pairs){
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
  std::cout << "touched " << touched << "\n";
  std::cout << "compared " << compared << "\n";
  std::cout << "pairs " << pairs.size() << "\n";
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
  std::cout << "touched " << touched << "\n";
  std::cout << "compared " << compared << "\n";
  std::cout << "pairs " << pairs.size() << "\n";
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
  std::cout << "touched " << touched << "\n";
  std::cout << "compared " << compared << "\n";
  std::cout << "pairs " << pairs.size() << "\n";
}

void replaceAABBs(std::vector<xytAABB> const& o, std::vector<xytAABB> const& n, std::vector<xytAABB>& sorted){
  {
    std::cout << "REPLACE: \n";
    for(auto nn(o.begin()); nn!= o.end(); ++nn){
      std::cout << "<" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << "><"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    }
    std::cout << "WITH: \n";
    for(auto nn(n.begin()); nn!= n.end(); ++nn){
      std::cout << "<" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << "><"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    }
    // Verify that sorting property is still good
    auto mm(sorted.begin());
    for(auto nn(mm+1); nn!=sorted.end(); ++nn){
      if(!(*mm<*nn || *mm==*nn)){
        std::cout << "BAD BEFORE!<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "><"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }else{
        std::cout << "BEFORE:<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << ">,<"<<mm->upperBound[0].value << "," << mm->upperBound[1].cvalue << "," << mm->upperBound[2].cvalue << ">\n";
      }
      //ASSERT_LE(*mm,*nn);
      //ASSERT_TRUE(*mm<*nn || *mm==*nn);
      ++mm;
    }
  }
  auto oi(o.begin()); // New
  auto ni(n.begin()); // New
  auto beforen(sorted.begin());
  auto ato(std::lower_bound(sorted.begin(),sorted.end(),*oi)); // Old
  while(oi!=o.end()&&ni!=n.end()){
    std::cout << "old ["<<(ato-sorted.begin())<<"] <" << oi->lowerBound[0].value << "," << oi->lowerBound[1].cvalue << "," << oi->lowerBound[2].cvalue << "><" << oi->upperBound[0].value << "," << oi->upperBound[1].cvalue << "," << oi->upperBound[2].cvalue << ">\n";
    if(*oi<*ni){
      beforen=std::lower_bound(ato,sorted.end(),*ni);
    }else if(*ni<*oi){
      beforen=std::lower_bound(beforen,ato,*ni);
    }else{
      beforen=ato;
    }
    std::cout << "new ["<<(beforen-sorted.begin())<<"] <" << ni->lowerBound[0].value << "," << ni->lowerBound[1].cvalue << "," << ni->lowerBound[2].cvalue << "><" << ni->upperBound[0].value << "," << ni->upperBound[1].cvalue << "," << ni->upperBound[2].cvalue << ">\n";
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
  {
    // Verify that sorting property is still good
    auto mm(sorted.begin());
    for(auto nn(mm+1); nn!=sorted.end(); ++nn){
      if(!(*mm<*nn || *mm==*nn)){
        std::cout << "BAD AFTER! <" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "> >= <"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }else{
        std::cout << "AFTER:<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << ">,<"<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
      }
      ++mm;
      //ASSERT_LE(*mm,*nn);
      //ASSERT_TRUE(*mm<*nn || *mm==*nn);
    }
  }
}

TEST(AABB, replaceAABBTEST){

  std::vector<std::vector<xytLoc>> p = {{{1,2,0},{2,3,1.4},{3,4,2.8}},
    {{5,3,0},{5,4,1},{5,6,2},{5,7,3}},
    {{7,4,0},{7,3,1},{6,2,2.4}}};

  std::vector<std::vector<xytAABB>> temp(3);
  std::vector<xytAABB> sorted;
  for(int i(0); i<p.size(); ++i){
    makeAABBs(p[i],temp[i],i);
    sorted.insert(sorted.end(), temp[i].begin(), temp[i].end());
  }
  for(auto const& r:temp){
    for(auto const& s:r){
      std::cout << "<" << s.lowerBound[0].value << "," << s.lowerBound[1].cvalue << "," << s.lowerBound[2].cvalue << ">,<"<<s.upperBound[0].value << "," << s.upperBound[1].cvalue << "," << s.upperBound[2].cvalue << ">\n";
    }
    std::cout <<"\n";
  }
  std::sort(sorted.begin(),sorted.end());
  for(auto const& s:sorted){
    std::cout << "<" << s.lowerBound[0].value << "," << s.lowerBound[1].cvalue << "," << s.lowerBound[2].cvalue << ">,<"<<s.upperBound[0].value << "," << s.upperBound[1].cvalue << "," << s.upperBound[2].cvalue << ">\n";
  }
  ASSERT_EQ(7,sorted.size());
  ASSERT_EQ(0,sorted[0].lowerBound[0].cvalue);
  ASSERT_EQ(1,sorted[0].lowerBound[2].cvalue);
  ASSERT_EQ(7,sorted[2].lowerBound[2].cvalue);
  ASSERT_EQ(5,sorted[6].upperBound[2].cvalue);
  ASSERT_EQ(2,sorted[3].lowerBound[1].cvalue);
  ASSERT_EQ(6,sorted[3].lowerBound[2].cvalue);
  ASSERT_EQ(4,sorted[4].lowerBound[1].cvalue);
  ASSERT_EQ(5,sorted[4].lowerBound[2].cvalue);
  std::cout <<"\n";

  std::vector<xytAABB> newpath = temp[1];
  newpath[0].upperBound[1].cvalue=newpath[1].lowerBound[1].cvalue=3;
  newpath[0].upperBound[2].cvalue=newpath[1].lowerBound[2].cvalue=6;

  for(auto const& s:newpath){
    std::cout << "<" << s.lowerBound[0].value << "," << s.lowerBound[1].cvalue << "," << s.lowerBound[2].cvalue << ">,<"<<s.upperBound[0].value << "," << s.upperBound[1].cvalue << "," << s.upperBound[2].cvalue << ">\n";
  }

  std::cout <<"\n";
  replaceAABBs(temp[1],newpath,sorted);
  for(auto const& s:sorted){
    std::cout << "<" << s.lowerBound[0].value << "," << s.lowerBound[1].cvalue << "," << s.lowerBound[2].cvalue << ">,<"<<s.upperBound[0].value << "," << s.upperBound[1].cvalue << "," << s.upperBound[2].cvalue << ">\n";
  }
  ASSERT_EQ(3,sorted[4].lowerBound[1].cvalue);
  ASSERT_EQ(6,sorted[4].lowerBound[2].cvalue);
}

TEST(AABB, insertTest){
  std::vector<std::vector<xytLoc>> waypoints;
  unsigned nagents(10);
  unsigned tnum(22);
  unsigned type(9);

  // Load problems from file
  {
    std::stringstream filename;
    filename << "../../test/environments/instances/8x8/" << nagents << "/" << tnum << ".csv";
    std::ifstream ss(filename.str());
    int x,y;
    float t(0.0);
    std::string line;
    while(std::getline(ss, line)){
      std::vector<xytLoc> wpts;
      std::istringstream is(line);
      std::string field;
      while(is >> field){
        size_t a(std::count(field.begin(), field.end(), ','));
        if(a==1){
          sscanf(field.c_str(),"%d,%d", &x,&y);
        }else if(a==2){
          sscanf(field.c_str(),"%d,%d,%f", &x,&y,&t);
        }else{
          assert(!"Invalid value inside problem file");
        }
        wpts.emplace_back(x,y,t);
      }
      waypoints.push_back(wpts);
    }
  }

  // Find paths
  TemplateAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  Map map(8,8);
  MapEnvironment menv(&map);
  menv.SetConnectedness(type);
  Map2DConstrainedEnvironment env(&menv);
  std::vector<std::vector<xytLoc>> p;
  for(int i(0); i<nagents; ++i){
    std::vector<xytLoc> path;
    if(waypoints[i][0]==waypoints[i][1]){ // Already at goal
      path.push_back(waypoints[i][0]);
    }else{
      astar.GetPath(&env,waypoints[i][0],waypoints[i][1],path);
    }
    p.push_back(path);
    std::cout << "AGENT " << i <<":\n";
    for(auto const& v:path){
      std::cout << v << "\n";
    }
  }

  float radius(.25);
  std::vector<std::vector<xytAABB>> temp(p.size());
  std::vector<xytAABB> sorted;
  for(int i(0); i<p.size(); ++i){
    //temp[i].reserve(p[i].size());
    makeAABBs(p[i],temp[i],i);
    sorted.insert(sorted.end(), temp[i].begin(), temp[i].end());
  }
  std::sort(sorted.begin(),sorted.end());

  std::cout << "AABBs\n";
  for(auto foo:temp){
    for(auto nn(foo.begin()); nn!=foo.end(); ++nn){
      std::cout << "AABB:" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">,<"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    }
    std::cout << "\n";
  }
  {
    Timer tmr0;
    Timer tmrz;
    unsigned count0(0);
    tmr0.StartTimer();
    //for(auto const& s:sorted)
    //std::cout << "<"<<s.lowerBound[0].value<<"~"<<s.upperBound[0].value<<","<<s.lowerBound[1].cvalue<<"~"<<s.upperBound[1].cvalue<<","<<s.lowerBound[2].cvalue<<"~"<<s.upperBound[2].cvalue<<">\n";
    tmrz.StartTimer();
    std::vector<std::pair<xytAABB,xytAABB>> pairs;
    getAllPairs(sorted,pairs);
    {
      // Verify that sorting property is still good
      auto mm(sorted.begin());
      for(auto nn(mm+1); nn!=sorted.end(); ++nn){
        if(!(*mm<*nn || *mm==*nn)){
          std::cout << "!<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "> > "<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
        }else{
          std::cout << "sorted<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "> > "<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
        }
        //ASSERT_LE(*mm,*nn);
        //ASSERT_TRUE(*mm<*nn || *mm==*nn);
        ++mm;
      }
    }
    for(auto pp(pairs.begin()); pp!=pairs.end(); /*pp++*/){
      if(checkForCollision(*pp->first.start,*pp->first.end,*pp->second.start,*pp->second.end,radius,&menv)){
        ++pp;
      }
      pairs.erase(pp);
    }
    for(auto pp(pairs.begin()); pp!=pairs.end(); pp++){
      unsigned i(pp->first.agent);
      Constraint<xytLoc>* c = new Collision<xytLoc>(*pp->second.start,*pp->second.end);
      env.AddConstraint(c);
      std::vector<xytLoc> path;
      astar.GetPath(&env,waypoints[i][0],waypoints[i][1],path);
      std::vector<xytAABB> aabbs;
      makeAABBs(path,aabbs,i);
      
      replaceAABBs(temp[i],aabbs,sorted);
      temp[i]=aabbs;
      //std::cout << "Replaced some\n";
    }
    
    std::cout << "Sort test (t) on " << nagents << " with " << pairs.size() << " checks, " << count0 << " collisions took " << tmrz.EndTimer() << "(" << tmr0.EndTimer() << ")\n";
  }

}

void broadphaseTest(int type, unsigned nagents, unsigned tnum){
  const int depth(1000); // Limit on path length...
  std::vector<std::vector<xytLoc>> waypoints;

  // Load problems from file
  {
    std::stringstream filename;
    filename << "../../test/environments/instances/64x64/" << nagents << "/" << tnum << ".csv";
    std::ifstream ss(filename.str());
    int x,y;
    float t(0.0);
    std::string line;
    while(std::getline(ss, line)){
      std::vector<xytLoc> wpts;
      std::istringstream is(line);
      std::string field;
      while(is >> field){
        size_t a(std::count(field.begin(), field.end(), ','));
        if(a==1){
          sscanf(field.c_str(),"%d,%d", &x,&y);
        }else if(a==2){
          sscanf(field.c_str(),"%d,%d,%f", &x,&y,&t);
        }else{
          assert(!"Invalid value inside problem file");
        }
        wpts.emplace_back(x,y,t);
      }
      waypoints.push_back(wpts);
    }
  }

  // Find paths
  TemplateAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
  Map map(64,64);
  MapEnvironment menv(&map);
  menv.SetConnectedness(type);
  Map2DConstrainedEnvironment env(&menv);
  std::vector<std::vector<xytLoc>> p;
  for(int i(0); i<nagents; ++i){
    std::vector<xytLoc> path;
    if(waypoints[i][0]==waypoints[i][1]){ // Already at goal
      path.push_back(waypoints[i][0]);
    }else{
      astar.GetPath(&env,waypoints[i][0],waypoints[i][1],path);
    }
    p.push_back(path);
  }

  float radius(.25);

  //for(auto const& a:sorted)
    //std::cout << *(a.start) << "\n";
  std::vector<std::vector<AABB>> sweepPrune(3);
  Timer tmr;
  tmr.StartTimer();
  createSortedLists(p,sweepPrune,&menv,radius);
  std::cout << "Took " << tmr.EndTimer() << " to get sorted lists\n";

  // Create the AABB tree
  aabb::Tree<xytLoc> tree(nagents*132);

  // To load precheck bit-vector
  checkForCollision(p[0][0],p[0][0],p[0][0],p[0][0],radius,&menv);

  // Insert particles and count collisions using BVH
  unsigned count1(0);
  unsigned total1(0);
  Timer tmr1;
  tmr1.StartTimer();
  for(int i(0); i<nagents; ++i){
    for(int j(1); j<p[i].size(); ++j){
      tree.insertParticle(i*depth+j,p[i][j-1],p[i][j]);
    }
  }

  std::vector<std::pair<std::pair<unsigned,unsigned>,std::pair<unsigned,unsigned>>> collisions1;
  Timer tmr3;
  tmr3.StartTimer();
  std::unordered_set<unsigned> seen;
  for(int i(0); i<nagents; ++i){
    for(int j(1); j<p[i].size(); ++j){
      unsigned mykey(getKey(i,j,depth));
      std::vector<unsigned> result;
      tree.query(mykey,result);
      for(auto const& v:result){
        unsigned key(v<mykey?v+mykey*nagents*depth:mykey+v*nagents*depth);
        auto a(unravelKey(v,depth));
        if((a.first == i) || seen.find(key)!=seen.end()){continue;}
        total1++;
        seen.insert(key);
        count1 += checkForCollision(p[i][j-1],p[i][j],p[a.first][a.second-1],p[a.first][a.second],radius,&menv);
      }
    }
  }
  std::cout << "BVH test on " << nagents << " with " << total1 << " checks, " << count1 << " collisions took " << tmr1.EndTimer() << "(" << tmr3.EndTimer() << ")\n";


  {
    std::vector<std::vector<xytAABB>> temp(p.size());
    for(int i(0); i<p.size(); ++i){
      temp[i].reserve(p[i].size());
      makeAABBs(p[i],temp[i],i);
    }
    unsigned count2(0);
    unsigned total2(0);
    Timer tmr2;
    tmr2.StartTimer();
    // Count collisions brute force
    for(int i(0); i<nagents; ++i){
      for(int j(i+1); j<nagents; ++j){
        //std::cout << i << " VS " << j << "\n";
        countCollisions(temp[i],temp[j],radius,count2,total2,&menv);
      }
    }
    std::cout << "Brute force AABB test with precheck on " << nagents << " with " << total2 << " checks, " << count2 << " collisions took " << tmr2.EndTimer() << "\n";
  }

  {
    std::vector<std::vector<xytAABB>> temp(p.size());
    for(int i(0); i<p.size(); ++i){
      temp[i].reserve(p[i].size());
      makeAABBs(p[i],temp[i],i);
    }
    unsigned count2(0);
    unsigned total2(0);
    Timer tmr2;
    tmr2.StartTimer();
    // Count collisions brute force
    for(int i(0); i<nagents; ++i){
      for(int j(i+1); j<nagents; ++j){
        //std::cout << i << " VS " << j << "\n";
        countCollisions(temp[i],temp[j],radius,count2,total2);
      }
    }
    std::cout << "Brute force AABB test on " << nagents << " with " << total2 << " checks, " << count2 << " collisions took " << tmr2.EndTimer() << "\n";
  }

  /*unsigned count2(0);
  unsigned total2(0);
  Timer tmr2;
  tmr2.StartTimer();
  // Count collisions brute force
  for(int i(0); i<nagents; ++i){
    for(int j(i+1); j<nagents; ++j){
      //std::cout << i << " VS " << j << "\n";
      countCollisions(p[i],p[j],radius,count2,total2);
    }
  }
  std::cout << "Brute force test on " << nagents << " with " << total2 << " checks, " << count2 << " collisions took " << tmr2.EndTimer() << "\n";
*/
  unsigned count4(0);
  unsigned total4(0);
  Timer tmr4;
  tmr4.StartTimer();
  // Count collisions brute force
  for(int i(0); i<nagents; ++i){
    for(int j(i+1); j<nagents; ++j){
      //std::cout << i << " VS " << j << "\n";
      countCollisions(p[i],p[j],radius,count4,total4,&menv);
    }
  }
  std::cout << "Brute force with precheck on " << nagents << " with " << total4 << " checks, " << count4 << " collisions took " << tmr4.EndTimer() << "\n";

  unsigned count5(0);
  unsigned total5(0);
  Timer tmr5;
  tmr5.StartTimer();
  // Count collisions brute force
  for(int i(0); i<nagents; ++i){
    for(int j(i+1); j<nagents; ++j){
      //std::cout << i << " VS " << j << "\n";
      countCollisions(p[i],p[j],radius,count5,total5,&menv,true);
    }
  }
  std::cout << "Brute force with simple precheck on " << nagents << " with " << total4 << " checks, " << count5 << " collisions took " << tmr5.EndTimer() << "\n";

  {
    std::vector<xytAABB> sorted;
    Timer tmr0;
    Timer tmrz;
    unsigned count0(0);
    tmr0.StartTimer();
    merge(p,sorted);
    //for(auto const& s:sorted)
    //std::cout << "<"<<s.lowerBound[0].value<<"~"<<s.upperBound[0].value<<","<<s.lowerBound[1].cvalue<<"~"<<s.upperBound[1].cvalue<<","<<s.lowerBound[2].cvalue<<"~"<<s.upperBound[2].cvalue<<">\n";
    tmrz.StartTimer();
    std::vector<std::pair<xytAABB,xytAABB>> pairs;
    getAllPairs(sorted,pairs);
    for(auto const& pp:pairs){
      count0 += checkForCollision(*pp.first.start,*pp.first.end,*pp.second.start,*pp.second.end,radius,&menv);
    }
    std::cout << "Sort test (t) on " << nagents << " with " << pairs.size() << " checks, " << count0 << " collisions took " << tmrz.EndTimer() << "(" << tmr0.EndTimer() << ")\n";
  }
  {
    std::vector<xytAABB> sorted;
    Timer tmr0;
    Timer tmrz;
    unsigned count0(0);
    tmr0.StartTimer();
    std::vector<std::vector<xytAABB>> temp(p.size());
    for(int i(0); i<p.size(); ++i){
      temp[i].reserve(p[i].size());
      makeAABBs(p[i],temp[i],i);
      sorted.insert(sorted.end(), temp[i].begin(), temp[i].end());
    }
    // sort by "x" value
    std::sort(sorted.begin(),sorted.end(),
        [](xytAABB const& a, xytAABB const& b) -> bool {
        return a.lowerBound[1].cvalue < b.lowerBound[1].cvalue;
        });
    //for(auto const& s:sorted)
    //std::cout << "<"<<s.lowerBound[0].value<<"~"<<s.upperBound[0].value<<","<<s.lowerBound[1].cvalue<<"~"<<s.upperBound[1].cvalue<<","<<s.lowerBound[2].cvalue<<"~"<<s.upperBound[2].cvalue<<">\n";
    tmrz.StartTimer();
    std::vector<std::pair<xytAABB,xytAABB>> pairs;
    getAllPairsX(sorted,pairs);
    for(auto const& pp:pairs){
      count0 += checkForCollision(*pp.first.start,*pp.first.end,*pp.second.start,*pp.second.end,radius,&menv);
    }
    std::cout << "Sort test (x) on " << nagents << " with " << pairs.size() << " checks, " << count0 << " collisions took " << tmrz.EndTimer() << "(" << tmr0.EndTimer() << ")\n";
  }
  {
    std::vector<xytAABB> sorted;
    Timer tmr0;
    Timer tmrz;
    unsigned count0(0);
    tmr0.StartTimer();
    merge(p,sorted);
    //for(auto const& s:sorted)
    //std::cout << "<"<<s.lowerBound[0].value<<"~"<<s.upperBound[0].value<<","<<s.lowerBound[1].cvalue<<"~"<<s.upperBound[1].cvalue<<","<<s.lowerBound[2].cvalue<<"~"<<s.upperBound[2].cvalue<<">\n";
    tmrz.StartTimer();
    std::vector<std::pair<xytAABB,xytAABB>> pairs;
    getAllPairs2(sorted,pairs);
    for(auto const& pp:pairs){
      count0 += checkForCollision(*pp.first.start,*pp.first.end,*pp.second.start,*pp.second.end,radius,&menv);
    }
    std::cout << "Sort test 2 on " << nagents << " with " << pairs.size() << " checks, " << count0 << " collisions took " << tmrz.EndTimer() << "(" << tmr0.EndTimer() << ")\n";
  }
}

TEST(AABB, BVHTreeTest){
  //int types[]={5,9,25,49};
  int types[]={9};
  //int types[]={9,25,49};
  for(int type:types){
    //for(int nagents(5); nagents<201; nagents+=5){
    for(int nagents(202); nagents<201; nagents+=5){
      for(int i(0); i<100; ++i){
        std::cout << "===========================================================\n";
        std::cout << type << "Connected, " << nagents << " AGENTS, Test " << i << "\n";
        std::cout << "===========================================================\n";
        broadphaseTest(type,nagents,i);
      }
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
