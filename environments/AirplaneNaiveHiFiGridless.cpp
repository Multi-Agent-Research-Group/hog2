//
//  AirplaneNaiveHiFiGridless.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/17.
//

#include <stdio.h>
#include "AirplaneNaiveHiFiGridless.h"
#include <iostream>
#include <sstream>

#if defined(__APPLE__)
	#include <OpenGL/gl.h>
#else
	#include <gl.h>
#endif

#include "TemplateAStar.h"
#include "Heuristic.h"

/*bool operator==(const PlatformState &s1, const PlatformState &s2)
{
    return (fequal(s1.x,s2.x) && fequal(s1.y,s2.y) && fequal(s1.z,s2.z) && s1.headingHalfDegs==s2.headingHalfDegs && s1.speed == s2.speed && s1.pitchHalfDegs==s2.pitchHalfDegs);
}*/

/*bool operator==(const PlatformAction &a1, const PlatformAction &a2)
{
  return a1.turnHalfDegs == a2.turnHalfDegs && a1.speed==a2.speed && a1.pitchHalfDegs==a2.pitchHalfDegs;
}*/

AirplaneNaiveHiFiGridlessEnvironment::AirplaneNaiveHiFiGridlessEnvironment(
  unsigned width,
  unsigned length,
  unsigned height,
  unsigned minSpeed,
  unsigned maxSpeed,
  uint8_t numSpeeds,
  double radius,
  double turn,
  double dive,
  double cruiseBurnRate,
  double climbCost,
  double descendCost)
  : AirplaneHiFiGridlessEnvironment(width, length, height, minSpeed, maxSpeed, numSpeeds, radius, turn, dive, cruiseBurnRate, climbCost, descendCost){}

void AirplaneNaiveHiFiGridlessEnvironment::GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  actions.push_back(PlatformAction(0,0,0));

  actions.push_back(PlatformAction(-maxTurn,0,0));
  actions.push_back(PlatformAction(maxTurn,0,0));
  actions.push_back(PlatformAction(0,-maxDive,0));
  actions.push_back(PlatformAction(0,maxDive,0));

  actions.push_back(PlatformAction(-maxTurn,-maxDive,0));
  actions.push_back(PlatformAction(-maxTurn,maxDive,0));
  actions.push_back(PlatformAction(maxTurn,-maxDive,0));
  actions.push_back(PlatformAction(maxTurn,maxDive,0));
  if(nodeID.speed<maxSpeed)
  {
    actions.push_back(PlatformAction(0,0,1));

    actions.push_back(PlatformAction(-maxTurn,0,1));
    actions.push_back(PlatformAction(maxTurn,0,1));
    actions.push_back(PlatformAction(0,-maxDive,1));
    actions.push_back(PlatformAction(0,maxDive,1));

    actions.push_back(PlatformAction(-maxTurn,-maxDive,1));
    actions.push_back(PlatformAction(-maxTurn,maxDive,1));
    actions.push_back(PlatformAction(maxTurn,-maxDive,1));
    actions.push_back(PlatformAction(maxTurn,maxDive,1));
  }
  if(nodeID.speed>minSpeed)
  {
    actions.push_back(PlatformAction(0,0,-1));

    actions.push_back(PlatformAction(-maxTurn,0,-1));
    actions.push_back(PlatformAction(maxTurn,0,-1));
    actions.push_back(PlatformAction(0,-maxDive,-1));
    actions.push_back(PlatformAction(0,maxDive,-1));

    actions.push_back(PlatformAction(-maxTurn,-maxDive,-1));
    actions.push_back(PlatformAction(-maxTurn,maxDive,-1));
    actions.push_back(PlatformAction(maxTurn,-maxDive,-1));
    actions.push_back(PlatformAction(maxTurn,maxDive,-1));
  }
}

void AirplaneNaiveHiFiGridlessEnvironment::GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  GetActions(nodeID,actions);
}
