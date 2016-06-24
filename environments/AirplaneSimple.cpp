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
        // Estimate fuel cost for cardinal direction movement only.
        int vertDiff(node2.height-node1.height);
        double diffx(abs(node1.x-node2.x));
        double diffy(abs(node1.y-node2.y));
        double horizDiff(diffx+diffy);

        double ratio=(vertDiff>0?climbCostRatio:descendCostRatio);
        vertDiff=abs(vertDiff);
        unsigned headingChanges(0);//(abs(node1.heading-node1.headingTo(node2))%4-1);
        if(vertDiff <= horizDiff)
        {
          return vertDiff*cruiseBurnRate*ratio+(horizDiff-vertDiff+headingChanges)*cruiseBurnRate;
        }
        else
        {
          // We'll have to slow down in order to give enough time to climb/descend or turn
          double hvdiff(vertDiff-horizDiff);
          return (hvdiff+headingChanges)*(cruiseBurnRate+speedBurnDelta)+vertDiff*cruiseBurnRate*ratio;
        }
}

