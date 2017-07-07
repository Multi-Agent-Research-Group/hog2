#ifndef UnitTests_h_
#define UnitTests_h_

#include "VelocityObstacle.h"
#include "Timer.h"
#include <gtest/gtest.h>
#include <map>
#include "Map2DEnvironment.h"
#include "AnyAngleSipp.h"
#include "ThetaStar.h"
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
class StraightLineHeuristic : public Heuristic<xyLoc> {
  public:
  double HCost(const xyLoc &a,const xyLoc &b) const {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
  }
};

TEST(Theta, GetPath){
  Map map(8,8);
  MapEnvironment env(&map);
  env.SetFiveConnected();
  ThetaStar<xyLoc,tDirection,MapEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  std::vector<xyLoc> solution;
  tstar.GetPath(&env,{1,1},{7,3},solution);
  for(auto const& ss: solution)
    std::cout << ss.x << "," << ss.y << "\n";
}

TEST(PETheta, GetPath){
  Map map(8,8);
  MapEnvironment env(&map);
  env.SetFiveConnected();
  PEThetaStar<xyLoc,tDirection,MapEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  std::vector<xyLoc> solution;
  tstar.GetPath(&env,{1,1},{7,3},solution);
  for(auto const& ss: solution)
    std::cout << ss.x << "," << ss.y << "\n";
}

TEST(Theta, GetObstructedPath){
  Map map(8,8);
  MapEnvironment env(&map);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,1,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  env.SetFiveConnected();
  ThetaStar<xyLoc,tDirection,MapEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  std::vector<xyLoc> solution;
  tstar.GetPath(&env,{1,1},{7,3},solution);
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  std::cout << std::endl;
}

TEST(PETheta, GetObstructedPath){
  Map map(8,8);
  MapEnvironment env(&map);
  //map.SetTerrainType(2,0,kOutOfBounds);
  map.SetTerrainType(2,1,kOutOfBounds);
  //map.SetTerrainType(2,2,kOutOfBounds);
  std::cout << map.IsTraversable(2,1) << "traversable\n";
  env.SetFiveConnected();
  PEThetaStar<xyLoc,tDirection,MapEnvironment> tstar;
  tstar.SetHeuristic(new StraightLineHeuristic());
  std::vector<xyLoc> solution;
  tstar.GetPath(&env,{1,1},{7,3},solution);
  for(auto const& ss: solution){
    std::cout << ss.x << "," << ss.y << "\n";
  }
  for(int i(1);i<solution.size(); ++i){
    ASSERT_TRUE(env.LineOfSight(solution[i-1],solution[i]));
  }
  std::cout << std::endl;
}
#endif
