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
  actions.resize(0);
  static unsigned cruiseSpeed(3);
  double nnh((nodeID.headingTo(getGoal()))-nodeID.hdg());
  //std::cout << "Desired heading " << (nodeID.headingTo(getGoal())) <<"-"<<nodeID.hdg() <<"("<<nh<<")"<< "\n";
  if(fgreater(nnh,180.)){nnh=-(360.-nnh);} // Take complement
  double nh(fgreater(nnh,0)?std::min(maxTurn,nnh):std::max(-maxTurn,nnh)); // Never turn more than max
  //std::cout << "Turn amount " << nh << "\n";

  double nne((nodeID.elevationTo(getGoal()))-nodeID.pitch());
  //std::cout << "pitch " << (nodeID.elevationTo(getGoal())) << "-"<<nodeID.pitch()<<"=" << ne << "\n";
  double ne(fgreater(nne,0)?std::min(maxDive,nne):std::max(-maxDive,nne)); // Never pitch more than max

  if(fgreater(fabs(nnh),60.)){
    if(nodeID.speed>minSpeed){
      actions.push_back(PlatformAction(nh, ne, -1));
    }else{
      actions.push_back(PlatformAction(nh, ne, 0));
    }
  } else if(fgreater(fabs(nnh),30.)){
    if(nodeID.speed>minSpeed){
      actions.push_back(PlatformAction(nh, ne, -1));
    }else if(nodeID.speed==minSpeed){
      actions.push_back(PlatformAction(nh, ne, 1));
    }
    actions.push_back(PlatformAction(nh, ne, 0));
  }else{
    if(nodeID.speed>minSpeed){
      actions.push_back(PlatformAction(nh, ne, -1));
    }
    if(nodeID.speed<maxSpeed){
      actions.push_back(PlatformAction(nh, ne, 1));
    }
    actions.push_back(PlatformAction(nh, ne, 0));
    //actions.push_back(PlatformAction(nh+maxTurn, ne, 0));
    //actions.push_back(PlatformAction(nh-maxTurn, ne, 0));
    //actions.push_back(PlatformAction(nh, ne+maxDive, 0));
    //actions.push_back(PlatformAction(nh, ne-maxDive, 0));
  }
}

void AirplaneNaiveHiFiGridlessEnvironment::GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  GetActions(nodeID,actions);
}
