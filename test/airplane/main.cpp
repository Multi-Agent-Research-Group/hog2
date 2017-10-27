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
#include "TestEnvironment.h"
#include "UnitTests.h"

int main(int argc, char **argv){
  //testAdmissibility();
  test2DPathUniqueness();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  //testCardinalEnvHCost();
  //testSimpleActions();
  //testNormalActions();
  //testCardinalActions();
  //testHighway8Actions();
  //testCardinalHighwayActions();
  //testGridCardinalActions();
  //testGrid3DOctileActions();
  //testCardinalHeadingTo();
  //testMultiAgent();
  //TestQuadcopterActions();
  //testGetAction();
  //testLoadPerimeterHeuristic();
  //testHCost();
  //testHeadingTo();
  //testConstraints();
  //compareHeuristicSpeed();
  //testHash();
  //testIntervalTree();
  //testPEAStar();

  return 0;

}
