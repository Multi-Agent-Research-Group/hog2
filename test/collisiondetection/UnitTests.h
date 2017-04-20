#ifndef UnitTests_h_
#define UnitTests_h_

#include "VelocityObstacle.h"
#include <gtest/gtest.h>

TEST(VelocityObstacle, IsInside){
Vector2D A(1,2);
Vector2D VA(1,1);
double radius(.25);
double astart(0.0);
double aend(6.0);
Vector2D B(3,1);
Vector2D VB(0,1);
double bstart(0.0);
double bend(6.0);

VelocityObstacle VO(A,VA,B,VB,radius);
ASSERT_TRUE(VO.IsInside(A+VB));
}

#endif
