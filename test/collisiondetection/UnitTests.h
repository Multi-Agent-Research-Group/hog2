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
    float(rand()%int(width*1000))/(width*100.0) + low;
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
  std::cout << std::endl;
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
  std::cout << std::endl;
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
  std::cout << std::endl;
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
  std::cout << std::endl;
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
  std::cout << "with obstacle" << std::endl;
  env.AddConstraint(Constraint<TemporalVector>({6,1,0},{1,1,6}));
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
  std::cout << std::endl;
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
  env.AddConstraint(Constraint<TemporalVector>({4,1,0},{1,1,6}));
  env.AddConstraint(Constraint<TemporalVector>({7,2,0},{1,2,6}));
  tstar.GetPath(&env,{1,1,0},{6,1,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << std::endl;
}

TEST(DISABLED_Theta, TestMotionConstrained3D){
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  Grid3DConstrainedEnvironment env(&menv);
  ThetaStar<xyztLoc,t3DDirection,Grid3DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic3D());
  //tstar.SetVerbose(true);
  std::vector<xyztLoc> solution;
  env.SetMaxTurnAzimuth(45.0); //(only 45 deg turns allowed)
  env.SetMaxPitch(30.0); //(only 30 deg pitch change allowed)
  tstar.GetPath(&env,{4,4,0,90.0,0.0,0},{4,4,2,270.0,0.0,0},solution); // Turn around
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "," << ss.z << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << std::endl;
}

TEST(DISABLED_Theta, Test3D){
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  Grid3DConstrainedEnvironment env(&menv);
  ThetaStar<xyztLoc,t3DDirection,Grid3DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic3D());
  //tstar.SetVerbose(true);
  std::vector<xyztLoc> solution;
  env.AddConstraint(Constraint<TemporalVector3D>({4,1,0,0},{1,1,0,6}));
  env.AddConstraint(Constraint<TemporalVector3D>({7,2,0,0},{1,2,0,6}));
  tstar.GetPath(&env,{1,1,0,0},{6,1,0,0},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "," << ss.z << "\n";
  }
  ASSERT_TRUE(solution.size()>1);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << Util::heading<360>(0,0,0,-1) << "\n";
  std::cout << Util::angle<360>(0,0,0,-1) << "\n";
  std::cout << std::endl;
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

static inline unsigned index9(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2){return index(s1,d1,s2,d2,1);}
static inline unsigned index25(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2){return index(s1,d1,s2,d2,2);}
static inline unsigned index49(xyLoc const& s1, xyLoc d1, xyLoc s2, xyLoc d2){return index(s1,d1,s2,d2,3);}

static inline bool get(unsigned* bitarray, size_t idx) {
    return bitarray[idx / WORD_BITS] | (1 << (idx % WORD_BITS));
}

static inline void set(unsigned* bitarray, size_t idx) {
    bitarray[idx / WORD_BITS] |= (1 << (idx % WORD_BITS));
}

// Polygonal intersection from:
// https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf


TEST(PreCollision, generate9Conn_25Rad){
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
        Vector2D VA(o);
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
        Vector2D VA(o);
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

TEST(PreCollision, generate9Conn_5Rad){
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
        Vector2D VA(o);
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
        Vector2D VA(o);
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

TEST(PreCollision, generate25Conn_5Rad){
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
        Vector2D VA(o);
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

TEST(PreCollision, generate25Conn_25Rad){
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
        Vector2D VA(o);
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

TEST(PreCollision, generate49Conn_5Rad){
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
        Vector2D VA(o);
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

TEST(PreCollision, generate49Conn_25Rad){
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
        Vector2D VA(o);
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

bool checkForCollision(xytLoc const& s1, xytLoc const& d1, xytLoc const& s2, xytLoc const& d2,float radius, MapEnvironment* env=nullptr){
  if(env && !env->collisionPreCheck(s1,d1,radius,s2,d2,radius)) return false;
  Vector2D A(s1);
  Vector2D B(s2);
  Vector2D VA(d1);
  VA-=A;
  VA.Normalize();
  Vector2D VB(d2);
  VB-=B;
  VB.Normalize();
  return collisionImminent(A,VA,radius,s1.t,d1.t,B,VB,radius,s2.t,d2.t);
}

unsigned countCollisions(std::vector<xytLoc> const& p1, std::vector<xytLoc> const& p2, float radius,MapEnvironment* env=nullptr){
  unsigned count(0);
  auto ap(p1.begin());
  auto a(ap+1);
  auto bp(p2.begin());
  auto b(bp+1);
  while(a!=p1.end() && b!=p2.end()){
    if(checkForCollision(*ap,*a,*bp,*b,radius,env)){count++;}
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
TEST(AABB, BVHTreeTest){
  const int nagents(300);
  const int depth(1024); // Limit on path length...
  std::vector<std::vector<xytLoc>> waypoints;

  // Load problems from file
  {
    std::ifstream ss("../../test/environments/instances/64x64/300/0.csv");
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
  menv.SetNineConnected();
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
  // Create the AABB tree
  aabb::Tree<xytLoc> tree(nagents*100);
  float radius(.25);
  // To load precheck bit-vector
  checkForCollision(p[0][0],p[0][1],p[0][0],p[0][1],radius,&menv);

  // Insert particles and count collisions using BVH
  unsigned count1(0);
  Timer tmr1;
  tmr1.StartTimer();
  for(int i(0); i<nagents; ++i){
    for(int j(1); j<p[i].size(); ++j){
      tree.insertParticle(i*depth+j,p[i][j-1],p[i][j],radius);
    }
  }

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
        seen.insert(key);
        count1 += checkForCollision(p[i][j-1],p[i][j],p[a.first][a.second],p[a.first][a.second+1],radius,&menv);
      }
    }
  }
  std::cout << "BVH test on " << nagents << " with " << count1 << " collisions took " << tmr1.EndTimer() << "(" << tmr3.EndTimer() << ")\n";


  unsigned count2(0);
  Timer tmr2;
  tmr2.StartTimer();
  // Count collisions brute force
  for(int i(0); i<nagents; ++i){
    for(int j(i+1); j<nagents; ++j){
      count2+=countCollisions(p[i],p[j],radius);
    }
  }
  std::cout << "Brute force test on " << nagents << " with " << count2 << " collisions took " << tmr2.EndTimer() << "\n";

  unsigned count4(0);
  Timer tmr4;
  tmr4.StartTimer();
  // Count collisions brute force
  for(int i(0); i<nagents; ++i){
    for(int j(i+1); j<nagents; ++j){
      count4 +=countCollisions(p[i],p[j],radius,&menv);
    }
  }
  std::cout << "Brute force with precheck on " << nagents << " with " << count4 << " collisions took " << tmr4.EndTimer() << "\n";
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
  std::cout << std::endl;
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
  std::cout << std::endl;
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
  std::cout << std::endl;
}*/
#endif
