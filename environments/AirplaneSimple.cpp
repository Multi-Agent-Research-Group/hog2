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
  double climbCostRatio,
  double descendCostRatio,
  double gridSize
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
  climbCostRatio,
  descendCostRatio,
  gridSize){}

void AirplaneSimpleEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
	// 45, 90, 0, shift
	// speed:
	// faster, slower
	// height:
	// up / down
	actions.resize(0);

  // If the airplane is on the ground, the only option is to takeoff
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
        return;
      }
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
	//actions.push_back(airplaneAction(k45, 0, 0));
	//actions.push_back(airplaneAction(-k45, 0, 0));
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

double AirplaneSimpleEnvironment::HCost(const airplaneState &node1, const airplaneState &node2) const
{
    // We want to estimate the heuristic to the landing state
    // Figure out which landing strip we're going to
   for (landingStrip st : landingStrips)
      {
        if (node2 == st.goal_state)
        {
         return HCost(node1, st.landing_state);
        } else if (node1 == st.goal_state)
        {
            return HCost(st.landing_state, node2);
        }
      }


  // Estimate fuel cost for cardinal direction movement only.
  int vertDiff = abs(node2.height - node1.height);
  double manhattan = abs(node1.x-node2.x)+ abs(node1.y-node2.y);

  double ratio =( vertDiff>0?climbCostRatio:descendCostRatio);
  unsigned hdgto = node1.headingTo(node2);
  unsigned headingChanges = (hdgDiff<8>(node1.heading,hdgto)+hdgDiff<8>(node2.heading,hdgto))/2;
  if(vertDiff <= manhattan)
  {
    return vertDiff*cruiseBurnRate*ratio+(manhattan+headingChanges)*cruiseBurnRate;
  }
  else
  {
    // We'll have to slow down in order to give enough time to climb/descend or turn
    double hvdiff(vertDiff-manhattan);
    return (hvdiff+headingChanges)*(cruiseBurnRate+speedBurnDelta)+vertDiff*cruiseBurnRate*ratio;
  }
}

