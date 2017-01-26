//
//  AirplaneHighway4Cardinal.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "AirplaneHighway4Cardinal.h"

AirplaneHighway4CardinalEnvironment::AirplaneHighway4CardinalEnvironment(
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

void AirplaneHighway4CardinalEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  if(abs(nodeID.x-getGoal().x) <=2 && abs(nodeID.y-getGoal().y) <=2 &&abs(nodeID.height-getGoal().height) <=2)
  {
    AirplaneEnvironment::GetActions(nodeID,actions);
    return;
  }

  actions.resize(0);

  // Allow 4 highways with heading (0; 2; 4; 6) etc.
  uint8_t hmod((nodeID.height%4));

  if(hmod != nodeID.heading/2 || nodeID.heading%2)
  {
    int travel(nodeID.cardinalHeadingTo(getGoal()));
    int turns(hdgDiff<8>(nodeID.heading,travel));
    int levels(hdgDiff<4>(hmod,travel/2));
    if((travel-nodeID.heading < 0 && abs(travel-nodeID.heading) <= turns) ||
    (travel-nodeID.heading > 0 && abs(travel-nodeID.heading) > 4)) turns *= -1;
    if((travel/2-hmod < 0 && abs(travel/2-hmod) <= levels) ||
    (travel/2-hmod > 0 && abs(travel/2-hmod) > 4)) levels *= -1;

    int turn(0);
    int ht(0);
    int turnb(0);

    if(nodeID.heading%2){
      if(turns<0)
      {
        turn=-k45;
        turnb=k45;
      }else{
        turn=k45;
        turnb=-k45;
      }
    }else{
      if(turns<0)
      {
        turn=-k90;
        turnb=k90;
      }
      else{
        turn=k90;
        turnb=-k90;
      }
    }

    if(levels<0)
    {
      ht=-1;
    }
    else if(levels>0)
    {
      ht=1;
    }
    actions.push_back(airplaneAction(turn,0,ht));
    actions.push_back(airplaneAction(turnb,0,ht*-1));
    return;
  }
  // 45, 90, 0, shift
  // speed:
  // faster, slower
  // height:
  // up / down

  // If the airplane is on the ground, the only option is to takeoff or stay landed
  /*if (nodeID.landed)
  {
    // Figure out which landing strip we're at
    for (landingStrip st : landingStrips)
    {
      //std::cout << "Comparing " << nodeID << " and " << st.goal_state << std::endl;
      if (nodeID == st.goal_state)
      {
        // Add the takeoff action
        actions.push_back(airplaneAction(0,0,0,1));
        // Add the landed no-op action
        actions.push_back(airplaneAction(0,0,0,3));
        return;
      }
    }
    // There should never be a situation where we get here
    std::cout << nodeID << std::endl;
    assert(false && "Airplane trying to takeoff, but not at a landing strip");
  } 
  else 
  {
    // Check to see if we can land
    for (landingStrip st : landingStrips)
    {
      if (nodeID == st.landing_state)
      {
        // Add the takeoff action
        actions.push_back(airplaneAction(0,0,0,2));
      }
    }
    // We don't have to land though, so we keep going.
  }*/

  // no change
  if(hmod==nodeID.heading/2) actions.push_back(airplaneAction(0, 0, 0));

  if (nodeID.height > 1)
  {
    uint8_t hmodd((nodeID.height+3)%4);
    uint8_t hdg((nodeID.heading/2+3)%4);
    // decrease height
    //actions.push_back(airplaneAction(0, 0, -1));
    if(hmodd==hdg)actions.push_back(airplaneAction(-k90, 0, -1));

    // decrease height, decrease speed
    if (nodeID.speed > minSpeed)
    {
      if(hmodd==hdg)actions.push_back(airplaneAction(-k90, -1, -1));
    }
    // decrease height, increase speed
    if (nodeID.speed < numSpeeds+minSpeed)
    {
      if(hmodd==hdg)actions.push_back(airplaneAction(-k90, +1, -1));
    }
  }

  if (nodeID.height < height)
  {
    uint8_t hmodd((nodeID.height+1)%4);
    uint8_t hdg((nodeID.heading/2+1)%4);
    // increase height
    if(hmodd==hdg)actions.push_back(airplaneAction(k90, 0, +1));

    if (nodeID.speed > minSpeed)
    {
      // increase height, decrease speed
      if(hmodd==hdg)actions.push_back(airplaneAction(k90, -1, +1));
    }

    if (nodeID.speed < numSpeeds+minSpeed)
    {
      // increase height, increase speed
      if(hmodd==hdg)actions.push_back(airplaneAction(k90, +1, +1));
    }
  }

  if (nodeID.speed > minSpeed)
  {
    // decrease speed
    if(hmod==nodeID.heading/2)actions.push_back(airplaneAction(0, -1, 0));
  }

  if (nodeID.speed < numSpeeds+minSpeed)
  {
    // increase speed
    if(hmod==nodeID.heading/2)actions.push_back(airplaneAction(0, +1, 0));
  }
}

void AirplaneHighway4CardinalEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  //assert(false && "This should never be used... It's not ready.");
  if(abs(nodeID.x-getGoal().x) <=2 && abs(nodeID.y-getGoal().y) <=2 &&abs(nodeID.height-getGoal().height) <=2)
  {
    AirplaneEnvironment::GetReverseActions(nodeID,actions);
    return;
  }

  uint8_t hmod(nodeID.height%4);
  // Note that this refers to the start in the forward direction
  // If the start is non-conformant, we have to allow only certain moves
  if(nodeID.heading!=hmod || (getStart().heading!=getStart().height%4 && abs(nodeID.x-getStart().x) <=2 && abs(nodeID.y-getStart().y) <=2 &&abs(nodeID.height-getStart().height) <=2)){
    AirplaneEnvironment::GetReverseActions(nodeID,actions);
    return;
  }
  actions.resize(0);

  // no change
  if(hmod==nodeID.heading) actions.push_back(airplaneAction(0, 0, 0));

  if (nodeID.height < height)
  {
    uint8_t hmodd((nodeID.height+1)%4);
    uint8_t hdg((nodeID.heading+1)%4);
    // decrease height
    //actions.push_back(airplaneAction(0, 0, -1));
    if(hmodd==hdg)actions.push_back(airplaneAction(-k90, 0, -1));

    // decrease height, decrease speed
    if (nodeID.speed > minSpeed)
    {
      if(hmodd==hdg)actions.push_back(airplaneAction(-k90, 1, -1));
    }

    // increase height, decrease speed
    if (nodeID.speed < numSpeeds+minSpeed)
    {
      if(hmodd==hdg)actions.push_back(airplaneAction(-k90, -1, -1));
    }
  }

  if (nodeID.height > 0)
  {
    uint8_t hmodd((nodeID.height+7)%4);
    uint8_t hdg((nodeID.heading+7)%4);
    // increase height
    if(hmodd==hdg)actions.push_back(airplaneAction(k90, 0, +1));

    if (nodeID.speed > minSpeed)
    {
      // increase height, decrease speed
      if(hmodd==hdg)actions.push_back(airplaneAction(k90, 1, +1));
    }

    if (nodeID.speed < numSpeeds+minSpeed)
    {
      // increase height, increase speed
      if(hmodd==hdg)actions.push_back(airplaneAction(k90, -1, +1));
    }
  }

  if (nodeID.speed > minSpeed)
  {
    // decrease speed
    if(hmod==nodeID.heading)actions.push_back(airplaneAction(0, 1, 0));
  }

  if (nodeID.speed < numSpeeds+minSpeed)
  {
    // increase speed
    if(hmod==nodeID.heading)actions.push_back(airplaneAction(0, -1, 0));
  }
}

double AirplaneHighway4CardinalEnvironment::myHCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Estimate fuel cost...
  static const int cruise(3);
  int diffx(abs(node1.x-node2.x));
  int diffy(abs(node1.y-node2.y));
  int diff(diffx+diffy);
  int vertDiff(node2.height-node1.height);
  int speedDiff1(std::max(0,abs(cruise-node1.speed)-1));
  int speedDiff2(abs(cruise-node2.speed));
  int speedDiff(abs(node2.speed-node1.speed));
  double vcost(vertDiff>0?climbCost:descendCost);
  vertDiff=std::max(hdgDiff<4>(node1.heading,node1.cardinalHeadingTo(node2))/2,(unsigned)abs(vertDiff));
  int maxMove(speedDiff1+speedDiff2+vertDiff<=diff?(speedDiff1+speedDiff2+vertDiff):speedDiff+vertDiff);

  int speedChanges(((diff-vertDiff)>=speedDiff1+speedDiff2)?(speedDiff1+speedDiff2):speedDiff);

  if(diff<vertDiff+speedChanges){ diff=(vertDiff+speedChanges); }
  
  return diff*cruiseBurnRate+std::min(speedChanges,diff)*speedBurnDelta+std::min(vertDiff,diff)*vcost;
}

