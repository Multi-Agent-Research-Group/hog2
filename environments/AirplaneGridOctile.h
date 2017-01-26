//
//  AirplaneGridOctile.h
//  hog2 glut
//
//  Created by Thayne Walker on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef AirplaneGridOctile_h
#define AirplaneGridOctile_h

#include "Airplane.h"

class AirplaneGridOctileEnvironment : public AirplaneEnvironment
{
  public:
    AirplaneGridOctileEnvironment(
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

    virtual char const*const name()const{return "AirplaneGridOctileEnvironment";}

    virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
    virtual airplaneAction GetAction(const airplaneState &node1, const airplaneState &node2) const;
    virtual void GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
    virtual void loadPerimeterDB();
    virtual double HCost(const airplaneState &node1, const airplaneState &node2) const;
    virtual double GCost(const airplaneState &node1, const airplaneState &node2) const;
    virtual bool GoalTest(const airplaneState &node, const airplaneState &goal) const;
    virtual void ApplyAction(airplaneState &s, airplaneAction dir) const;
    virtual void UndoAction(airplaneState &s, airplaneAction dir) const;
    virtual uint64_t GetStateHash(const airplaneState &node)const{airplaneState a(node);a.height=a.heading=a.landed=0;a.speed=3;return AirplaneEnvironment::GetStateHash(a);}
  protected:
    virtual AirplaneEnvironment& getRef() {return *this;}
};

#endif /* AirplaneGridOctile_h */
