//
//  AirplaneGrid3DCardinal.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "AirplaneGrid3DCardinal.h"

AirplaneGrid3DCardinalEnvironment::AirplaneGrid3DCardinalEnvironment(
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

void AirplaneGrid3DCardinalEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);

  actions.push_back(airplaneAction(0,0,0));
  actions.push_back(airplaneAction(2,0,0));
  actions.push_back(airplaneAction(4,0,0));
  actions.push_back(airplaneAction(6,0,0));
  actions.push_back(airplaneAction(-1,0,1)); // No horiz. movement
  actions.push_back(airplaneAction(-1,0,-1));
}

/** Gets the action required to go from node1 to node2 */
airplaneAction AirplaneGrid3DCardinalEnvironment::GetAction(const airplaneState &node1, const airplaneState &node2) const
{
  airplaneAction a;
  if(node2.x>node1.x){a.turn=2;}
  else if(node2.x<node1.x){a.turn=6;}
  else if(node2.y>node1.y){a.turn=0;}
  else if(node2.y<node1.y){a.turn=4;}
  else{
    if(node2.height<node1.height){a.height=-1;}
    else if(node2.height>node1.height){a.height=1;}
  }
  return a;
}

void AirplaneGrid3DCardinalEnvironment::ApplyAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0){nodeID.y++;}
if(action.turn==2){nodeID.x++;}
if(action.turn==4){nodeID.y--;}
if(action.turn==6){nodeID.x--;}
nodeID.height += action.height;
nodeID.speed=3;
}

void AirplaneGrid3DCardinalEnvironment::UndoAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0){nodeID.y--;}
if(action.turn==2){nodeID.x--;}
if(action.turn==4){nodeID.y++;}
if(action.turn==6){nodeID.x++;}
nodeID.height -= action.height;
nodeID.speed=3;
}

void AirplaneGrid3DCardinalEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  return GetActions(nodeID,actions);
}


double AirplaneGrid3DCardinalEnvironment::GCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Cost of doing nothing
  if(node1.landed && node2.landed) return 0.0;
  return cruiseBurnRate;
}
double AirplaneGrid3DCardinalEnvironment::HCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Estimate fuel cost...
  static const int cruise(3);
  int diffx(abs(node1.x-node2.x));
  int diffy(abs(node1.y-node2.y));
  int diffz(abs(node1.height-node2.height));
  int diff(diffx+diffy+diffz);
  /*int speedDiff1(std::max(0,abs(cruise-node1.speed)-1));
  int speedDiff2(abs(cruise-node2.speed));
  int speedDiff(abs(node2.speed-node1.speed));
  int speedChanges(speedDiff1+speedDiff2<=diff?(speedDiff1+speedDiff2):speedDiff);

  if(diff<speedChanges){ diff=speedChanges; }*/
  
  return diff*cruiseBurnRate;//+std::min(speedChanges,diff)*speedBurnDelta;
}

bool AirplaneGrid3DCardinalEnvironment::GoalTest(const airplaneState &node, const airplaneState &goal) const{
  return node.x==goal.x && node.y==goal.y && node.height==goal.height;
}

void AirplaneGrid3DCardinalEnvironment::loadPerimeterDB(){} // Do nothing
