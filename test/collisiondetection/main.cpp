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
#include "UnitTests.h"

void renderScene(){}

int main(int argc, char** argv){
  TemporalVector3D a1(1,3,0,4.8);
    xyztLoc A1(a1.x,a1.y,a1.z,a1.t);
    TemporalVector3D a2(2,5,0,7);
    xyztLoc A2(a2.x,a2.y,a2.z,a2.t);
    TemporalVector3D b1(2,5,0,1.7);
    xyztLoc B1(b1.x,b1.y,b1.z,b1.t);
    TemporalVector3D b2(1,3,0,3.9);
    xyztLoc B2(b2.x,b2.y,b2.z,b2.t);
      std::cout << a1<<a2<<b1<<b2<<"\n";
      std::cout << A1<<A2<<B1<<B2<<"\n";
    bool v1=collisionCheck3D(a1,a2,b1,b2,.25);
    bool v2=collisionCheck2D(A1,A2,B1,B2,.25);
    auto VA(a2-a1); VA.Normalize();
    auto VB(b2-b1); VB.Normalize();
    bool v3 = collisionImminent(A1, VA, .25, a1.t, a2.t, B1, VB, .25, b1.t, b2.t);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
