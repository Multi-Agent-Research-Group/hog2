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

#ifndef RadialSafety2D_OBJECTIVEENVIRONMENT_H
#define RadialSafety2D_OBJECTIVEENVIRONMENT_H

#include "ObjectiveEnvironment.h"
#include "GridStates.h"
#include <sstream>

class RadialSafety2DObjectiveEnvironment : public ObjectiveEnvironment<xytLoc>{
  public:
    RadialSafety2DObjectiveEnvironment(std::vector<xytLoc> const& center, std::vector<double> radius, unsigned w, unsigned h, double scale=1.0);
    virtual void buildHeuristic();
    virtual std::string name()const{std::stringstream ss; ss<<"RadialSafety2DObjectiveEnvironment"; return ss.str();}
    virtual double HCost(xytLoc const& node1, xytLoc const& node2) const;
    virtual double GCost(xytLoc const& node1, xytLoc const& node2) const;
    void OpenGLDraw(Map* map)const;
  protected:
    std::vector<xytLoc> c;
    std::vector<double> r;
    std::vector<double> logr;
    unsigned width;
    unsigned height;
    double s;
    std::vector<double> rowMins;
    std::vector<double> colMins;
};

#endif
