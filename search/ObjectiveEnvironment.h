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

#ifndef OBJECTIVEENVIRONMENT_H
#define OBJECTIVEENVIRONMENT_H

#include <string>
#include "Heuristic.h"

class Map;

template <typename state>
class ObjectiveEnvironment : public Heuristic<state>{
public:
        virtual void buildHeuristic(){}
        virtual void loadCosts(char* filename){}
        virtual std::string name() const=0;
	virtual double GCost(state const& node1, state const& node2) const=0;
        virtual void OpenGLDraw(Map*)const{}
};

#endif
