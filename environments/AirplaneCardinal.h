//
//  AirplaneCardinal.h
//  hog2 glut
//
//  Created by Thayne Walker on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirplaneCardinal_h
#define AirplaneCardinal_h

#include "Airplane.h"

class AirplaneCardinalEnvironment : public AirplaneEnvironment
{
public:
	AirplaneCardinalEnvironment(
          unsigned width=80,
          unsigned length=80,
          unsigned height=20,
          double climbRate=5, // in mps,
          double minSpeed=1,
          double maxSpeed=5,
          uint8_t numSpeeds=5, // Number of discrete speeds
          double cruiseBurnRate=.0006, // Fuel burn rate in liters per unit distance
          double speedBurnDelta=0.0001, // Extra fuel cost for non-cruise speed
          double climbCost=0.0001, // Fuel cost ratio for climbing
          double descendCost=-0.00005, // Fuel cost for descending
          double gridSize=3.0, // Horizontal grid width
          std::string const& perimeterFile=std::string("airplanePerimeter.dat"));

        virtual char const*const name()const{return "AirplaneCardinalEnvironment";}

	virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual void GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
protected:
        virtual AirplaneEnvironment& getRef() {return *this;}
        virtual double myHCost(const airplaneState &node1, const airplaneState &node2) const;

};

#endif /* AirplaneCardinal_h */
