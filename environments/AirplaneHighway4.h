//
//  AirplaneHighway4.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirplaneHighway4_h
#define AirplaneHighway4_h

#include "Airplane.h"

class AirplaneHighway4Environment : public AirplaneEnvironment
{
public:
	AirplaneHighway4Environment(
          unsigned width=80,
          unsigned length=80,
          unsigned height=20,
          double climbRate=5, // in mps,
          double minSpeed=1,
          double maxSpeed=5,
          uint8_t numSpeeds=5, // Number of discrete speeds
          double cruiseBurnRate=.006, // Fuel burn rate in liters per unit distance
          double speedBurnDelta=0.001, // Extra fuel cost for non-cruise speed
          double climbCost=0.001, // Fuel cost ratio for climbing
          double descendCost=-0.0005, // Fuel cost for descending
          double gridSize=3.0, // Horizontal grid width
          std::string const& perimeterFile=std::string("airplanePerimeter.dat"));

        virtual char const*const name()const{return "AirplaneHighway4Environment";}

	virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual void GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
protected:
        virtual AirplaneEnvironment& getRef() {return *this;}
        virtual double myHCost(const airplaneState &node1, const airplaneState &node2) const;

};

#endif /* AirplaneHighway4_h */
