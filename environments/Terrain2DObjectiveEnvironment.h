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

#ifndef TERRAIN2D_OBJECTIVEENVIRONMENT_H
#define TERRAIN2D_OBJECTIVEENVIRONMENT_H

#include "ObjectiveEnvironment.h"
#include "GridStates.h"

class Terrain2DObjectiveEnvironment : public ObjectiveEnvironment<xytLoc>{
  public:
    Terrain2DObjectiveEnvironment(unsigned w, unsigned h, char* filename, float s);
    ~Terrain2DObjectiveEnvironment();
    virtual void buildHeuristic();
    virtual void loadCosts(char* filename);
    virtual std::string name() const{return std::string("Terrain2DObjectiveEnvironment");}
    virtual double HCost(xytLoc const& node1, xytLoc const& node2) const;
    virtual double GCost(xytLoc const& node1, xytLoc const& node2) const;
    void SetConnectedness(unsigned c){connectivity=c;}
    void OpenGLDraw(Map* map)const;
  private:
    unsigned width; // x dimension
    unsigned height; // y dimension
    float scale;
    unsigned connectivity;
    float** elevation;
    std::vector<float> rowMins;
    std::vector<float> colMins;
    float cumulativeCost(int x0, int y0, int x1, int y1) const;
};


#endif
