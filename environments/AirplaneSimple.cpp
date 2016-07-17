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
	// 45, 90, 0, shift
	// speed:
	// faster, slower
	// height:
	// up / down
	actions.resize(0);


  // Handle landing states
  if (nodeID.landed)
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
        //std::cout << "Added a no-op action at state: " << nodeID <<std::endl;
        return;
      }
      std::cout << "Didn't match " << st.goal_state << " and " << nodeID << std::endl;
    }
    // There should never be a situation where we get here
    assert(false && "Airplane trying to takeoff, but not at a landing strip");
  } 
  else 
  {
    // Check to see if we can land
    for (landingStrip st : landingStrips)
    {
      if (nodeID == st.landing_state)
      {
        // Add the land action
        actions.push_back(airplaneAction(0,0,0,2));

      }
    }
    // We don't have to land though, so we keep going.
  }


  // no change
	actions.push_back(airplaneAction(0, 0, 0));

	// each type of turn
	actions.push_back(airplaneAction(k45, 0, 0));
	actions.push_back(airplaneAction(-k45, 0, 0));
	actions.push_back(airplaneAction(k90, 0, 0));
	actions.push_back(airplaneAction(-k90, 0, 0));
	//actions.push_back(airplaneAction(kShift, 0, 0));
	//actions.push_back(airplaneAction(-kShift, 0, 0));

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

void AirplaneSimpleEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
	// 45, 90, 0, shift
	// speed:
	// faster, slower
	// height:
	// up / down
	actions.resize(0);

	actions.push_back(airplaneAction(0, 0, 0));

	// each type of turn
	actions.push_back(airplaneAction(k45, 0, 0));
	actions.push_back(airplaneAction(-k45, 0, 0));
	actions.push_back(airplaneAction(k90, 0, 0));
	actions.push_back(airplaneAction(-k90, 0, 0));
	//actions.push_back(airplaneAction(kShift, 0, 0));
	//actions.push_back(airplaneAction(-kShift, 0, 0));

        // decrease height
	if (nodeID.height < height)
          actions.push_back(airplaneAction(0, 0, -1));
        // increase height
	if (nodeID.height > 0)
          actions.push_back(airplaneAction(0, 0, 1));

        // decrease speed
	if (nodeID.speed < numSpeeds+1)
          actions.push_back(airplaneAction(0, -1, 0));

        // increase speed
	if (nodeID.speed > 1)
          actions.push_back(airplaneAction(0, 1, 0));
}

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
  int travel(node1.headingTo(node2));
  int numTurns(std::max(0,signed(hdgDiff<8>(node1.heading,travel)/2)-1));
  int maxMove(speedDiff1+speedDiff2+vertDiff<=diff?(speedDiff1+speedDiff2+vertDiff):speedDiff+vertDiff);

  // Change as many diagonal moves into horizontal as we can
  while(maxMove>diff+diag && diag>0){
    diag--;
    diff+=2;
  }

  int speedChanges(((diff-vertDiff)>=speedDiff1+speedDiff2)?(speedDiff1+speedDiff2):speedDiff);

  if(diff+diag<vertDiff+speedChanges){ diff+=(vertDiff+speedChanges)-(diff+diag); }
  
  //std::cout << "speed:"<<speedChanges<<"v:"<<vertDiff<<"\n";
  double total(diff*cruiseBurnRate+std::min(speedChanges,diff)*speedBurnDelta+std::min(vertDiff,diff)*vcost);
  speedChanges-=std::min(speedChanges,diff);
  vertDiff-=std::min(vertDiff,diff);

  return total+(diag*cruiseBurnRate+speedChanges*speedBurnDelta+vertDiff*vcost)*M_SQRT2;
}

