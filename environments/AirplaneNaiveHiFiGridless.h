/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

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
        double maxDive=15,
        double cruiseBurnRate=.06, // Fuel burn rate in liters per unit distance
        double climbCost=0.01, // Fuel cost for climbing
        double descendCost=-0.00005); // Fuel cost for descending
    //std::string const& perimeterFile=std::string("airplanePerimeter.dat"));

    virtual std::string name()const{return "AirplaneNaiveHiFiGridlessEnvironment";}
    virtual void GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;
    virtual void GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const;

};

#endif
