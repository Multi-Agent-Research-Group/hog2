/*
 *  ObjectiveEnvironment.cpp
 *  hog2
 *
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
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
