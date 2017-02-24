//
//  AirplaneGridOctile.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "AirplaneGridOctile.h"

AirplaneGridOctileEnvironment::AirplaneGridOctileEnvironment(
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

void AirplaneGridOctileEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
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

  /*if(nodeID.speed > minSpeed)
  {
    actions.push_back(airplaneAction(0,-1,0));
    actions.push_back(airplaneAction(1,-1,0));
    actions.push_back(airplaneAction(2,-1,0));
    actions.push_back(airplaneAction(3,-1,0));
    actions.push_back(airplaneAction(4,-1,0));
    actions.push_back(airplaneAction(5,-1,0));
    actions.push_back(airplaneAction(6,-1,0));
    actions.push_back(airplaneAction(7,-1,0));
  }

  if(nodeID.speed < numSpeeds)
  {
    actions.push_back(airplaneAction(0,1,0));
    actions.push_back(airplaneAction(1,1,0));
    actions.push_back(airplaneAction(2,1,0));
    actions.push_back(airplaneAction(3,1,0));
    actions.push_back(airplaneAction(4,1,0));
    actions.push_back(airplaneAction(5,1,0));
    actions.push_back(airplaneAction(6,1,0));
    actions.push_back(airplaneAction(7,1,0));

  }*/
}

/** Gets the action required to go from node1 to node2 */
airplaneAction AirplaneGridOctileEnvironment::GetAction(const airplaneState &node1, const airplaneState &node2) const
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
  /*if(node2.speed>node1.speed){a.speed=1;}
  else if(node2.speed<node1.speed){a.speed=-1;}*/
  return a;
}

void AirplaneGridOctileEnvironment::ApplyAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0||action.turn==1||action.turn==7){nodeID.y++;}
if(action.turn==2||action.turn==1||action.turn==3){nodeID.x++;}
if(action.turn==4||action.turn==5||action.turn==3){nodeID.y--;}
if(action.turn==6||action.turn==5||action.turn==7){nodeID.x--;}
nodeID.speed=3;
//nodeID.speed+=action.speed;
}

void AirplaneGridOctileEnvironment::UndoAction(airplaneState &nodeID, airplaneAction action) const{

if(action.turn==0||action.turn==1||action.turn==7){nodeID.y--;}
if(action.turn==2||action.turn==1||action.turn==3){nodeID.x--;}
if(action.turn==4||action.turn==5||action.turn==3){nodeID.y++;}
if(action.turn==6||action.turn==5||action.turn==7){nodeID.x++;}
nodeID.speed=3;
//nodeID.speed-=action.speed;
}

void AirplaneGridOctileEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
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

  /*if(nodeID.speed > minSpeed)
  {
    actions.push_back(airplaneAction(0,1,0));
    actions.push_back(airplaneAction(1,1,0));
    actions.push_back(airplaneAction(2,1,0));
    actions.push_back(airplaneAction(3,1,0));
    actions.push_back(airplaneAction(4,1,0));
    actions.push_back(airplaneAction(5,1,0));
    actions.push_back(airplaneAction(6,1,0));
    actions.push_back(airplaneAction(7,1,0));
  }

  if(nodeID.speed < numSpeeds)
  {
    actions.push_back(airplaneAction(0,-1,0));
    actions.push_back(airplaneAction(1,-1,0));
    actions.push_back(airplaneAction(2,-1,0));
    actions.push_back(airplaneAction(3,-1,0));
    actions.push_back(airplaneAction(4,-1,0));
    actions.push_back(airplaneAction(5,-1,0));
    actions.push_back(airplaneAction(6,-1,0));
    actions.push_back(airplaneAction(7,-1,0));
  }*/
}

double AirplaneGridOctileEnvironment::GCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Cost of doing nothing
  if(node1.landed && node2.landed) return 0.0;
  return cruiseBurnRate*(((node1.x-node2.x) && (node1.y-node2.y))?M_SQRT2:1.0);
}
double AirplaneGridOctileEnvironment::HCost(const airplaneState &node1, const airplaneState &node2) const
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
  int diff(abs(diffy-diffx));
  int diag(abs((diffx+diffy)-diff)/2);

  /*
  // Make the assumption that the perimeter is at least n steps away (diff+diag >= speedDiff1+speedDiff2)
  int speedMult=speedcost[node1.speed-1][node2.speed-1];
  int speedDiff1(std::max(0,abs(cruiseSpeed-node1.speed)));

  // Moving toward cruise speed does not cost extra
  int speedDiff2(abs(cruiseSpeed-node2.speed));
  int speedDiff(abs(node2.speed-node1.speed));

  int speedChanges=std::max(0,(diff+diag>=speedDiff1+speedDiff2)?(speedMult):(speedDiff-1));
 
  int cspeedMult(std::min(diff,speedChanges));
  double dspeedDiff=std::max(0,speedChanges-cspeedMult)*speedBurnDelta;
*/
   return diff*cruiseBurnRate+
      (/*dspeedDiff*/+diag*cruiseBurnRate)*M_SQRT2+
      //cspeedMult*speedBurnDelta+
      ((node1.landed && node2.landed) ? cruiseBurnRate : 0.0);
}

bool AirplaneGridOctileEnvironment::GoalTest(const airplaneState &node, const airplaneState &goal) const{
  // We only care about x,y position
  return node.x==goal.x && node.y==goal.y;
}

void AirplaneGridOctileEnvironment::loadPerimeterDB(){} // Do nothing
