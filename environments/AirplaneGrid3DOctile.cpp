//
//  AirplaneGrid3DOctile.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "AirplaneGrid3DOctile.h"

AirplaneGrid3DOctileEnvironment::AirplaneGrid3DOctileEnvironment(
  unsigned width,
  unsigned length,
  unsigned height,
  double climbRate,
  double minSpeed,
  double maxSpeed,
  uint8_t numSpeeds,
  double cruiseBurnRate,
  double speedBurnDelta,
  double climbCost,
  double descendCost,
  double gridSize,
  std::string const& perimeterFile
): AirplaneEnvironment(
  width,
  length,
  height,
  climbRate,
  minSpeed,
  maxSpeed,
  numSpeeds,
  cruiseBurnRate,
  speedBurnDelta,
  climbCost,
  descendCost,
  gridSize,
  perimeterFile){
}

void AirplaneGrid3DOctileEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);

  // Instead of "turning", just give the new orientation

  // Force a turn into a cardinal direction
  actions.push_back(airplaneAction(0,0,0));
  actions.push_back(airplaneAction(1,0,0));
  actions.push_back(airplaneAction(2,0,0));
  actions.push_back(airplaneAction(3,0,0));
  actions.push_back(airplaneAction(4,0,0));
  actions.push_back(airplaneAction(5,0,0));
  actions.push_back(airplaneAction(6,0,0));
  actions.push_back(airplaneAction(7,0,0));

  if(nodeID.height >0)
  {
    actions.push_back(airplaneAction(0,0,-1));
    actions.push_back(airplaneAction(1,0,-1));
    actions.push_back(airplaneAction(2,0,-1));
    actions.push_back(airplaneAction(3,0,-1));
    actions.push_back(airplaneAction(4,0,-1));
    actions.push_back(airplaneAction(5,0,-1));
    actions.push_back(airplaneAction(6,0,-1));
    actions.push_back(airplaneAction(7,0,-1));
    actions.push_back(airplaneAction(-1,0,-1));
  }

  if(nodeID.height <height)
  {
    actions.push_back(airplaneAction(0,0,1));
    actions.push_back(airplaneAction(1,0,1));
    actions.push_back(airplaneAction(2,0,1));
    actions.push_back(airplaneAction(3,0,1));
    actions.push_back(airplaneAction(4,0,1));
    actions.push_back(airplaneAction(5,0,1));
    actions.push_back(airplaneAction(6,0,1));
    actions.push_back(airplaneAction(7,0,1));
    actions.push_back(airplaneAction(-1,0,1));

  }
}

/** Gets the action required to go from node1 to node2 */
airplaneAction AirplaneGrid3DOctileEnvironment::GetAction(const airplaneState &node1, const airplaneState &node2) const
{
  airplaneAction a;
  if(node2.x>node1.x&&node2.y==node1.y){a.turn=2;}
  else if(node2.x>node1.x&&node2.y>node1.y){a.turn=1;}
  else if(node2.x>node1.x&&node2.y<node1.y){a.turn=3;}
  else if(node2.x<node1.x&&node2.y==node1.y){a.turn=6;}
  else if(node2.x<node1.x&&node2.y>node1.y){a.turn=7;}
  else if(node2.x<node1.x&&node2.y<node1.y){a.turn=5;}
  else if(node2.x==node1.x&&node2.y>node1.y){a.turn=0;}
  else if(node2.x==node1.x&&node2.y<node1.y){a.turn=4;}
  else{a.turn=-1;}
  if(node2.height>node1.height){a.height=1;}
  else if(node2.height<node1.height){a.height=-1;}
  return a;
}

void AirplaneGrid3DOctileEnvironment::ApplyAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0||action.turn==1||action.turn==7){nodeID.y++;}
if(action.turn==2||action.turn==1||action.turn==3){nodeID.x++;}
if(action.turn==4||action.turn==5||action.turn==3){nodeID.y--;}
if(action.turn==6||action.turn==5||action.turn==7){nodeID.x--;}
nodeID.speed=3;
nodeID.height+=action.height;
}

void AirplaneGrid3DOctileEnvironment::UndoAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0||action.turn==1||action.turn==7){nodeID.y--;}
if(action.turn==2||action.turn==1||action.turn==3){nodeID.x--;}
if(action.turn==4||action.turn==5||action.turn==3){nodeID.y++;}
if(action.turn==6||action.turn==5||action.turn==7){nodeID.x++;}
nodeID.speed=3;
nodeID.height-=action.height;
}

void AirplaneGrid3DOctileEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);
  actions.push_back(airplaneAction(0,0,0));
  actions.push_back(airplaneAction(1,0,0));
  actions.push_back(airplaneAction(2,0,0));
  actions.push_back(airplaneAction(3,0,0));
  actions.push_back(airplaneAction(4,0,0));
  actions.push_back(airplaneAction(5,0,0));
  actions.push_back(airplaneAction(6,0,0));
  actions.push_back(airplaneAction(7,0,0));

  if(nodeID.height > 0)
  {
    actions.push_back(airplaneAction(0,0,1));
    actions.push_back(airplaneAction(1,0,1));
    actions.push_back(airplaneAction(2,0,1));
    actions.push_back(airplaneAction(3,0,1));
    actions.push_back(airplaneAction(4,0,1));
    actions.push_back(airplaneAction(5,0,1));
    actions.push_back(airplaneAction(6,0,1));
    actions.push_back(airplaneAction(7,0,1));
    actions.push_back(airplaneAction(-1,0,1));
  }

  if(nodeID.height <height)
  {
    actions.push_back(airplaneAction(0,0,-1));
    actions.push_back(airplaneAction(1,0,-1));
    actions.push_back(airplaneAction(2,0,-1));
    actions.push_back(airplaneAction(3,0,-1));
    actions.push_back(airplaneAction(4,0,-1));
    actions.push_back(airplaneAction(5,0,-1));
    actions.push_back(airplaneAction(6,0,-1));
    actions.push_back(airplaneAction(7,0,-1));
    actions.push_back(airplaneAction(-1,0,-1));

  }
}

double AirplaneGrid3DOctileEnvironment::GCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Cost of doing nothing
  if(node1.landed && node2.landed) return 0.0;
  int total(abs(node1.x-node2.x) + abs(node1.y-node2.y) + abs(node1.height-node2.height));
  return cruiseBurnRate*sqrt(total);
}
double AirplaneGrid3DOctileEnvironment::HCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Estimate fuel cost...
  static const int cruiseSpeed(3);
  static const int speedcost[5][5] = {{4,2,1,2,4}, //1-->1,1-->2,1-->3,1-->4,1-->5
                                      {3,1,0,1,3},
                                      {2,1,0,1,2},
                                      {3,1,0,1,3},
                                      {4,2,1,2,4}};
  int diffx(abs(node1.x-node2.x));
  int diffy(abs(node1.y-node2.y));
  int diffz(abs(node1.height-node2.height));
  int& small(diffx);
  int& medium(diffy);
  int& large(diffz);
  if (small > medium) {
      std::swap(small, medium);
  }
  if (small > large) {
      std::swap(small, large);
  }
  if (medium > large) {
      std::swap(medium, large);
  }
  
  int diff1d(large-medium);
  int diff2d(medium-small);
  int diff3d(small);

   return cruiseBurnRate*(diff1d+diff2d*M_SQRT2+diff3d*sqrt(3.0));
}

bool AirplaneGrid3DOctileEnvironment::GoalTest(const airplaneState &node, const airplaneState &goal) const{
  // We only care about x,y position
  return node.x==goal.x && node.y==goal.y && node.height==goal.height;
}

void AirplaneGrid3DOctileEnvironment::loadPerimeterDB(){} // Do nothing
