//
//  AirplaneSimple.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "AirplaneSimple.h"

AirplaneSimpleEnvironment::AirplaneSimpleEnvironment(
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
  perimeterFile){}

void AirplaneSimpleEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{


  actions.resize(0);
  switch(nodeID.type) {
    case AirplaneType::QUAD:
      if (AppendLandingActionsQuad(nodeID, actions))
        GetActionsQuad(nodeID, actions);
      break;
    case AirplaneType::PLANE:
      if (AppendLandingActionsPlane(nodeID, actions))
        GetActionsPlane(nodeID, actions);
      break;
    default: break;
  }
}


void AirplaneSimpleEnvironment::GetActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
  
  // No change
  actions.push_back(airplaneAction(0, 0, 0));

  // Turns
  actions.push_back(airplaneAction(k45, 0, 0));
  actions.push_back(airplaneAction(-k45, 0, 0));
  actions.push_back(airplaneAction(k90, 0, 0));
  actions.push_back(airplaneAction(-k90, 0, 0));

  // Do a 180 turn
  if (nodeID.speed == 1) {
    actions.push_back(airplaneAction(k135, 0, 0));
    actions.push_back(airplaneAction(-k135, 0, 0));
    actions.push_back(airplaneAction(k180, 0, 0));
  // Hover
    actions.push_back(airplaneAction(kWait,0,0));
  }

  // Height change
  if (nodeID.height > 1)
          actions.push_back(airplaneAction(0, 0, -1));
  if (nodeID.height < height)
          actions.push_back(airplaneAction(0, 0, 1));

  // Speed Change
  if (nodeID.speed > 1)
          actions.push_back(airplaneAction(0, -1, 0));
  if (nodeID.speed < numSpeeds)
          actions.push_back(airplaneAction(0, 1, 0));
}

void AirplaneSimpleEnvironment::GetActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
      // no change
  actions.push_back(airplaneAction(0, 0, 0));

  // each type of turn
  actions.push_back(airplaneAction(k45, 0, 0));
  actions.push_back(airplaneAction(-k45, 0, 0));
  actions.push_back(airplaneAction(k90, 0, 0));
  actions.push_back(airplaneAction(-k90, 0, 0));

        // decrease height
  if (nodeID.height > 1)
          actions.push_back(airplaneAction(0, 0, -1));
        // increase height
  if (nodeID.height < height)
          actions.push_back(airplaneAction(0, 0, 1));

        // decrease speed
  if (nodeID.speed > 1)
          actions.push_back(airplaneAction(0, -1, 0));
        // increase speed
  if (nodeID.speed < numSpeeds)
          actions.push_back(airplaneAction(0, 1, 0));
}

bool AirplaneSimpleEnvironment::AppendLandingActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
  if (nodeID.landed)
  {
// No-op action
        actions.push_back(airplaneAction(0,0,0,3));
    return false;
  }else{
    return true;
  }
  if (nodeID.landed)
  {
    // Figure out which landing strip we're at
    for (landingStrip st : landingStrips)
    {
      //std::cout << "Comparing " << nodeID << " and " << st.goal_state << std::endl;
      if (nodeID.x == st.goal_state.x && nodeID.y == st.goal_state.y && nodeID.height == st.goal_state.height &&
          nodeID.heading == st.goal_state.heading && nodeID.speed == st.goal_state.speed)
      {
        // Add the takeoff action
        actions.push_back(airplaneAction(0,0,0,1));
        // Add the landed no-op action
        actions.push_back(airplaneAction(0,0,0,3));
        return false;
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
      if (nodeID.x == st.landing_state.x && nodeID.y == st.landing_state.y && nodeID.height == st.landing_state.height &&
          nodeID.heading == st.landing_state.heading && nodeID.speed == st.landing_state.speed)
      {
        // Add the landing action
        actions.push_back(airplaneAction(0,0,0,2));
      }
    }
    return true;
  }
}

bool AirplaneSimpleEnvironment::AppendLandingActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
    // Do Nothing
    return true;
}


void AirplaneSimpleEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);
  switch(nodeID.type) {
    case AirplaneType::QUAD:
      GetReverseActionsQuad(nodeID, actions);
      break;
    case AirplaneType::PLANE:
      GetReverseActionsPlane(nodeID, actions);
      break;
    default: break;
  }
}


void AirplaneSimpleEnvironment::GetReverseActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const 
{
    actions.resize(0);

  actions.push_back(airplaneAction(0, 0, 0));

  // each type of turn
  actions.push_back(airplaneAction(k45, 0, 0));
  actions.push_back(airplaneAction(-k45, 0, 0));
  actions.push_back(airplaneAction(k90, 0, 0));
  actions.push_back(airplaneAction(-k90, 0, 0));
  
  // decrease height
  if (nodeID.height < height)
          actions.push_back(airplaneAction(0, 0, -1));
  
  // increase height
  if (nodeID.height > 0)
          actions.push_back(airplaneAction(0, 0, 1));

  // decrease speed
  if (nodeID.speed < numSpeeds)
          actions.push_back(airplaneAction(0, -1, 0));
  // increase speed
  if (nodeID.speed > minSpeed)
          actions.push_back(airplaneAction(0, 1, 0));
}


void AirplaneSimpleEnvironment::GetReverseActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const 
{
    // No change
  actions.push_back(airplaneAction(0, 0, 0));

  // Turns
  actions.push_back(airplaneAction(k45, 0, 0));
  actions.push_back(airplaneAction(-k45, 0, 0));
  actions.push_back(airplaneAction(k90, 0, 0));
  actions.push_back(airplaneAction(-k90, 0, 0));

  // Do a 180 turn
  if (nodeID.speed == 1) {
    actions.push_back(airplaneAction(k135, 0, 0));
    actions.push_back(airplaneAction(-k135, 0, 0));
    actions.push_back(airplaneAction(k180, 0, 0));
  }

  // Height change
  if (nodeID.height > 1)
          actions.push_back(airplaneAction(0, 0, 1));
  if (nodeID.height < height)
          actions.push_back(airplaneAction(0, 0, -1));

  // Speed Change
  if (nodeID.speed > 1)
          actions.push_back(airplaneAction(0, 1, 0));
  if (nodeID.speed < numSpeeds)
          actions.push_back(airplaneAction(0, -1, 0));

  // Hover
  if (nodeID.speed == 1) 
          actions.push_back(airplaneAction(kWait,0,0));
}

//1.257
double AirplaneSimpleEnvironment::myHCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Estimate fuel cost...
  static const int cruise(3);
  int diffx(abs(node1.x-node2.x));
  int diffy(abs(node1.y-node2.y));
  int diff(abs(diffx-diffy));
  int diag(abs((diffx+diffy)-diff)/2);
  int vertDiff(node2.height-node1.height);
  int speedDiff1(std::max(0,abs(cruise-node1.speed)-1));
  int speedDiff2(abs(cruise-node2.speed));
  int speedDiff(abs(node2.speed-node1.speed));
  double vcost(vertDiff>0?climbCost:descendCost);
  vertDiff=abs(vertDiff);
  //int travel(node1.headingTo(node2));
  //int numTurns(std::max(0,signed(hdgDiff<8>(node1.heading,travel)/2)-1));
  int maxMove(speedDiff1+speedDiff2+vertDiff<=diff?(speedDiff1+speedDiff2+vertDiff):speedDiff+vertDiff);

  // Change as many diagonal moves into horizontal as we can
  while(maxMove>diff+diag && diag>0){
    diag--;
    diff+=2;
  }

  int speedChanges(((diff-vertDiff)>=speedDiff1+speedDiff2)?(speedDiff1+speedDiff2):speedDiff);

  if(diff+diag<vertDiff+speedChanges){ diff+=(vertDiff+speedChanges)-(diff+diag); }
  
  //std::cout << "speed:"<<speedChanges<<"v:"<<vertDiff<<"\n";
  double total(diff*cruiseBurnRate+std::min(speedChanges,diff)*speedBurnDelta+vertDiff*vcost);
  speedChanges-=std::min(speedChanges,diff);

  //std::cout << node1<<"-->"<<node2<<"="<<total+(diag*cruiseBurnRate+speedChanges*speedBurnDelta)*M_SQRT2<<"\n";
  return total+(diag*cruiseBurnRate+speedChanges*speedBurnDelta)*M_SQRT2;
}

