//
//  AirplaneSimple.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirplaneSimple_h
#define AirplaneSimple_h

#include "Airplane.h"

class AirplaneSimpleEnvironment : public AirplaneEnvironment
{
public:
	AirplaneSimpleEnvironment(
          unsigned width=80,
          unsigned length=80,
          unsigned height=20,
          double climbRate=5, // in mps,
          double minSpeed=17, // in mps,
          double maxSpeed=32, // in mps
          uint8_t numSpeeds=5, // Number of discrete speeds
          double cruiseBurnRate=.0006, // Fuel burn rate in liters per unit distance
          double speedBurnDelta=0.0001, // Extra fuel cost for non-cruise speed
          double climbCostRatio=1.0475, // Fuel cost ratio for climbing
          double gridSize=3.0); // Horizontal grid width

	virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual double HCost(const airplaneState &node1, const airplaneState &node2) const;
};

#endif /* AirplaneSimple_h */
