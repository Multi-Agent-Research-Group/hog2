/*
 *  RadialSafety2DObjectiveEnvironment.cpp
 *  Represents "safety/danger" costs for proximity to a "center of threat"
 *  hog2
 *
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
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
