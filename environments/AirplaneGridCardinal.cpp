//
//  AirplaneGridCardinal.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "AirplaneGridCardinal.h"

AirplaneGridCardinalEnvironment::AirplaneGridCardinalEnvironment(
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

void AirplaneGridCardinalEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);

  // Instead of "turning", just give the new orientation

  // Force a turn into a cardinal direction
  actions.push_back(airplaneAction(0,0,0));
  actions.push_back(airplaneAction(2,0,0));
  actions.push_back(airplaneAction(4,0,0));
  actions.push_back(airplaneAction(6,0,0));

  /*if(nodeID.speed > minSpeed)
  {
    actions.push_back(airplaneAction(0,-1,0));
    actions.push_back(airplaneAction(2,-1,0));
    actions.push_back(airplaneAction(4,-1,0));
    actions.push_back(airplaneAction(6,-1,0));
  }

  if(nodeID.speed < numSpeeds)
  {
    actions.push_back(airplaneAction(0,1,0));
    actions.push_back(airplaneAction(2,1,0));
    actions.push_back(airplaneAction(4,1,0));
    actions.push_back(airplaneAction(6,1,0));

  }*/
}

/** Gets the action required to go from node1 to node2 */
airplaneAction AirplaneGridCardinalEnvironment::GetAction(const airplaneState &node1, const airplaneState &node2) const
{
  airplaneAction a;
  if(node2.x>node1.x){a.turn=2;}
  else if(node2.x<node1.x){a.turn=6;}
  else if(node2.y>node1.y){a.turn=0;}
  else if(node2.y<node1.y){a.turn=4;}
  /*if(node2.speed>node1.speed){a.speed=1;}
  else if(node2.speed<node1.speed){a.speed=-1;}*/
  return a;
}

void AirplaneGridCardinalEnvironment::ApplyAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0){nodeID.y++;}
if(action.turn==2){nodeID.x++;}
if(action.turn==4){nodeID.y--;}
if(action.turn==6){nodeID.x--;}
//nodeID.speed+=action.speed;
nodeID.speed=3;
}

void AirplaneGridCardinalEnvironment::UndoAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0){nodeID.y--;}
if(action.turn==2){nodeID.x--;}
if(action.turn==4){nodeID.y++;}
if(action.turn==6){nodeID.x++;}
//nodeID.speed-=action.speed;
nodeID.speed=3;
}

void AirplaneGridCardinalEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);
  actions.push_back(airplaneAction(0,0,0));
  actions.push_back(airplaneAction(2,0,0));
  actions.push_back(airplaneAction(4,0,0));
  actions.push_back(airplaneAction(6,0,0));

  /*if(nodeID.speed > minSpeed)
  {
    actions.push_back(airplaneAction(0,1,0));
    actions.push_back(airplaneAction(2,1,0));
    actions.push_back(airplaneAction(4,1,0));
    actions.push_back(airplaneAction(6,1,0));
  }

  if(nodeID.speed < numSpeeds)
  {
    actions.push_back(airplaneAction(0,-1,0));
    actions.push_back(airplaneAction(2,-1,0));
    actions.push_back(airplaneAction(4,-1,0));
    actions.push_back(airplaneAction(6,-1,0));

  }*/
}


double AirplaneGridCardinalEnvironment::GCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Cost of doing nothing
  if(node1.landed && node2.landed) return 0.0;
  return cruiseBurnRate*(((node1.x-node2.x) && (node1.y-node2.y))?M_SQRT2:1.0);
}
double AirplaneGridCardinalEnvironment::HCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Estimate fuel cost...
  static const int cruise(3);
  int diffx(abs(node1.x-node2.x));
  int diffy(abs(node1.y-node2.y));
  int diff(diffx+diffy);
  /*int speedDiff1(std::max(0,abs(cruise-node1.speed)-1));
  int speedDiff2(abs(cruise-node2.speed));
  int speedDiff(abs(node2.speed-node1.speed));
  int speedChanges(speedDiff1+speedDiff2<=diff?(speedDiff1+speedDiff2):speedDiff);

  if(diff<speedChanges){ diff=speedChanges; }*/
  
  return diff*cruiseBurnRate;//+std::min(speedChanges,diff)*speedBurnDelta;
}

bool AirplaneGridCardinalEnvironment::GoalTest(const airplaneState &node, const airplaneState &goal) const{
  // We only care about x,y position
  return node.x==goal.x && node.y==goal.y;
}

void AirplaneGridCardinalEnvironment::loadPerimeterDB(){} // Do nothing
