#ifndef UnitTests_h_
#define UnitTests_h_

#include "VelocityObstacle.h"
#include "Timer.h"
#include <gtest/gtest.h>
#include <map>
#include "Map2DConstrainedEnvironment.h"
#include "AnyAngleSipp.h"
#include "ThetaStar.h"
#include "EPEThetaStar.h"
#include "PEThetaStar.h"

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

TEST(Quadratic, WaitingCollision){
  Vector2D A(3,6);
  Vector2D VA(-1,0);
  VA.Normalize();
  double aradius(0.25);
  Vector2D B(1,2);
  Vector2D VB(1,5);
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
  ASSERT_TRUE(collisionImminent(A,VA,aradius,5.0,6.0,B,VB,bradius,1.0,6.09902));
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
  for(int i(0); i<1000; ++i){
    detectCollision(Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,10),rfloat(0,10),Vector2D(rfloat(),rfloat()),Vector2D(rfloat(),rfloat()),.25,rfloat(0,10),rfloat(0,10));
  }
  //std::cout << "Total time (VelocityObstacle)" << t.EndTimer() << "\n";
}

TEST(Quadratic, PerfTest){
  //Timer t;
  //t.StartTimer();
  for(int i(0); i<1000; ++i){
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
  
TEST(CollisionInterval, GetCollisionIntervalWhenExists){
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

  auto intvl(getCollisionInterval(A,VA,radius,0.0,6.0,B,VB,radius,0.0,6.0));
  std::cout << "Collision interval is: " << intvl.first << "," << intvl.second << "\n";
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

TEST(Theta, GetPath){
  Map map(8,8);
  MapEnvironment menv(&map);
  Map2DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  menv.SetFiveConnected();
  ThetaStar<xytLoc,tDirection,Map2DConstrainedEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  tstar.SetVerbose(true);
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
  tstar.SetVerbose(true);
  std::vector<xytLoc> solution;
  tstar.GetPath(&env,{1,1,0},{7,3,0},solution);
  for(auto const& ss: solution)
    std::cout << ss << "\n";
}

/*
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
  tstar.SetVerbose(true);
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
  tstar.SetVerbose(true);
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
  tstar.SetVerbose(true);
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
