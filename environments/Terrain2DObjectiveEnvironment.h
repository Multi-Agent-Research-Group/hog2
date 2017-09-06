/*
 *  Terrain2DObjectiveEnvironment.cpp
 *  hog2
 *
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
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
