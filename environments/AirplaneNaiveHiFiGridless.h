//
//  AirplaneNaiveHiFiGridless.h
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/16.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef AirplaneNaiveHiFiGridless_h
#define AirplaneNaiveHiFiGridless_h

#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>
#include "AirplaneHiFiGridless.h"
#include "constants.h"
#include "AirStates.h"

bool operator==(PlatformAction const& a1, PlatformAction const& a2);


// Actual Environment
class AirplaneNaiveHiFiGridlessEnvironment : public AirplaneHiFiGridlessEnvironment
{
  public:
    // Constructor
    AirplaneNaiveHiFiGridlessEnvironment(
        unsigned width=80,
        unsigned length=80,
        unsigned height=20,
        unsigned minSpeed=1,
        unsigned maxSpeed=5,
        uint8_t numSpeeds=5, // Number of discrete speeds
        double goalRadius=PlatformState::SPEEDS[5],
        double maxTurn=7.5,
        double maxDive=7.5,
        double cruiseBurnRate=.06, // Fuel burn rate in liters per unit distance
        double climbCost=0.01, // Fuel cost for climbing
        double descendCost=-0.00005); // Fuel cost for descending
    //std::string const& perimeterFile=std::string("airplanePerimeter.dat"));

    virtual char const*const name()const{return "AirplaneNaiveHiFiGridlessEnvironment";}
    virtual void GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;
    virtual void GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;

};

#endif
