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

#ifndef AirplaneHighway4Cardinal_h
#define AirplaneHighway4Cardinal_h

#include "Airplane.h"

class AirplaneHighway4CardinalEnvironment : public AirplaneEnvironment
{
public:
	AirplaneHighway4CardinalEnvironment(
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

        virtual std::string name()const{return "AirplaneHighway4CardinalEnvironment";}

	virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual void GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
protected:
        virtual AirplaneEnvironment& getRef() {return *this;}
        virtual double myHCost(const airplaneState &node1, const airplaneState &node2) const;

};

#endif /* AirplaneHighway4Cardinal_h */
