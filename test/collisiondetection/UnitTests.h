#ifndef UnitTests_h_
#define UnitTests_h_

#include "VelocityObstacle.h"
#include <gtest/gtest.h>

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
#endif
