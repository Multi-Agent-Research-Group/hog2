//
//  AirplaneHiFiGridless.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/17.
//

#include <stdio.h>
#include "AirplaneHiFiGridless.h"
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

AirplaneHiFiGridlessEnvironment::AirplaneHiFiGridlessEnvironment(
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
    double descendCost
): width(width),
length(length),
height(height),
minSpeed(minSpeed),
maxSpeed(maxSpeed),
numSpeeds(numSpeeds),
goalRadius(radius),
maxTurn(turn),
maxDive(dive),
cruiseBurnRate(cruiseBurnRate),
climbCost(climbCost),
descendCost(descendCost),
nilGCost(false)
{
  DrawableConstraint::width=width;
  DrawableConstraint::length=length;
  DrawableConstraint::height=height;
  // Load the perimeter heuristic before we add any obstacles to the environment...
  //srandom(time(0));
  ground.resize((width+1)*(length+1));
  groundNormals.resize((width+1)*(length+1));
  int value = random()%255;
  int offset = 5;
  int steps = 5;

  // Generate the intial ground
  for (int x = 0; x <= width; x++)
  {
    SetGround(x, 0, std::max(std::min(255, value), 0));
    value += offset;
    steps--;
    if (steps == 0)
    {
      offset = (random()%70)-35;
      steps = random()%10;
    }
  }

  for (int y = 1; y <= length; y++)
  {
    value = GetGround(0, y-1);
    offset = (random()%70)-35;
    if (y > 1)
      offset = GetGround(0, y-2)-GetGround(0, y-1);
    steps = random()%10;

    for (int x = 0; x <= width; x++)
    {
      SetGround(x, y, std::max(std::min(255, value), 0));
      value += offset;
      steps--;
      if (steps == 0)
      {
        offset = (random()%70)-35;
        steps = random()%10;
      }
      if (abs(value-GetGround(x, y-1)) > 35)
        value = value/2 + GetGround(x, y-1)/2;
    }
  }

  // Smooth the ground
  std::vector<int> tmp((width+1)*(length+1));
  int maxVal = 0;
  for (int y = 0; y < length; y++)
  {
    for (int x = 0; x <= width; x++)
    {
      int sum = 0;
      int cnt = 0;
      for (int dx = -1; dx <= 1; dx++)
      {
        for (int dy = -1; dy <= 1; dy++)
        {
          if (Valid(x+dx, y+dy))
          {
            sum += GetGround(x+dx, y+dy);
            cnt++;
          }
        }
      }
      tmp[x+y*(length+1)] = sum/cnt;
      maxVal = std::max(sum/cnt, maxVal);
    }
  }

  // Extrude the ground
  for (int y = 0; y < length; y++)
  {
    for (int x = 0; x <= width; x++)
    {
      SetGround(x, y, (255*tmp[x+y*(length+1)])/maxVal);
    }
  }


  // Build normals for lighting
  for (int y = 0; y < length; y++)
  {
    for (int x = 0; x <= width; x++)
    {
      if (x < width)
      {
        recVec a = GetCoordinate(x, y, std::max((int)GetGround(x, y), 20));
        recVec b = GetCoordinate(x, y+1, std::max((int)GetGround(x, y+1), 20));
        recVec d = GetCoordinate(x+1, y, std::max((int)GetGround(x+1, y), 20));
        recVec n = (a-b).GetNormal(a-d);
        GetNormal(x, y) += n;
        GetNormal(x, y+1) += n;
        GetNormal(x+1, y) += n;
      }
      if (x > 0)
      {
        recVec a = GetCoordinate(x, y, std::max((int)GetGround(x, y), 20));
        recVec b = GetCoordinate(x-1, y+1, std::max((int)GetGround(x-1, y+1), 20));
        recVec d = GetCoordinate(x, y+1, std::max((int)GetGround(x, y+1), 20));
        recVec n = (a-b).GetNormal(a-d);
        GetNormal(x, y) += n;
        GetNormal(x-1, y+1) += n;
        GetNormal(x, y+1) += n;
      }
    }
  }
  for (int y = 0; y <= length; y++)
  {
    for (int x = 0; x <= width; x++)
    {
      GetNormal(x, y).normalise();
    }
  }

  // Create a landing strip
  PlatformState launchLoc(18, 29, 7, 180, 0, 1);
  PlatformState landingLoc(18, 29, 7, 0, 0, 1);
  PlatformState goalLoc(18, 23, 0, 0, 0, 1);
  gridlessLandingStrip l(15, 20, 17, 28, launchLoc, landingLoc, goalLoc);
  AddLandingStrip(l);
}

void AirplaneHiFiGridlessEnvironment::SetGround(int x, int y, uint8_t val)
{
  ground[x + y*(length+1)] = val;
}

uint8_t AirplaneHiFiGridlessEnvironment::GetGround(int x, int y) const
{
  return ground[x + y*(length+1)];
}

bool AirplaneHiFiGridlessEnvironment::Valid(int x, int y)
{
  return x >= 0 && x <= width && y >= 0 && y <= length;
}

recVec &AirplaneHiFiGridlessEnvironment::GetNormal(int x, int y)
{
  return groundNormals[x + y*(length+1)];
}

recVec AirplaneHiFiGridlessEnvironment::GetNormal(int x, int y) const
{
  return groundNormals[x + y*(length+1)];
}

void AirplaneHiFiGridlessEnvironment::RecurseGround(int x1, int y1, int x2, int y2)
{
  if (x1 >= x2-1 || y1 >= y2-1)
    return;
  int middlex = (x1+x2)/2;
  int middley = (y1+y2)/2;
  SetGround(middlex, y1, GetGround(x1, y1)/2+GetGround(x2, y1)/2+random()%(x2/2-x1/2)-(x2-x1)/4);
  SetGround(middlex, middley, GetGround(x1, y1)/2+GetGround(x2, y2)/2+random()%(x2/2-x1/2)-(x2-x1)/4);
  SetGround(middlex, y2, GetGround(x1, y2)/2+GetGround(x2, y2)/2+random()%(x2/2-x1/2)-(x2-x1)/4);
  SetGround(x1, middley, GetGround(x1, y1)/2+GetGround(x1, y2)/2+random()%(y2/2-y1/2)-(y2-y1)/4);
  SetGround(x2, middley, GetGround(x2, y1)/2+GetGround(x2, y2)/2+random()%(y2/2-y1/2)-(y2-y1)/4);
  RecurseGround(x1, y1, middlex, middley);
  RecurseGround(middlex, y1, x2, middley);
  RecurseGround(x1, middley, middlex, y2);
  RecurseGround(middlex, middley, x2, y2);
}


void AirplaneHiFiGridlessEnvironment::GetSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const
{
  std::cout << "Succ: " << nodeID << std::endl;
  GetActions(nodeID, internalActions);
  for (auto &act : internalActions)
  {
    PlatformState s;
    GetNextState(nodeID, act, s);
    //std::cout <<"  "<< act << "-->" << s << "\n";
    //std::cout << "  " << s <<":"<<GCost(nodeID,s)<<"+"<<HCost(s,getGoal())<<"="<<(GCost(nodeID,s)+HCost(nodeID,getGoal()))<< "\n";
    neighbors.push_back(s);
  }
}

void AirplaneHiFiGridlessEnvironment::GetReverseSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const
{
  GetReverseActions(nodeID, internalActions);
  for (auto &act : internalActions)
  {
    PlatformState s;
    s = nodeID;
    UndoAction(s, act);
    neighbors.push_back(s);
  }
  /*std::cout << "generated from "<<nodeID<<"\n";
    for(auto const& a: neighbors){
      std::cout << a << "\n";
    }*/
}

void AirplaneHiFiGridlessEnvironment::GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const{
  actions.resize(0);

  double nh((nodeID.headingTo(getGoal()))-nodeID.hdg());
  if(fgreater(nh,180.)){nh=-(360.-nh);} // Take complement
  double ne((nodeID.elevationTo(getGoal()))-nodeID.pitch());
  if(fless(fabs(nh),maxTurn)&&!fequal(nh,0)){
    if(fless(fabs(ne),maxDive)){
      actions.push_back(PlatformAction(nh,ne,0));
      if(nodeID.speed<maxSpeed)
        actions.push_back(PlatformAction(nh,ne,1));
      if(nodeID.speed>minSpeed)
        actions.push_back(PlatformAction(nh,ne,-1));
    }
  }else if(fless(fabs(ne),maxDive)&&!fequal(ne,0)){
    actions.push_back(PlatformAction(0,ne,0));
    if(nodeID.speed<maxSpeed)
      actions.push_back(PlatformAction(0,ne,1));
    if(nodeID.speed>minSpeed)
      actions.push_back(PlatformAction(0,ne,-1));
  }

  actions.push_back(PlatformAction(0,0,0));
  actions.push_back(PlatformAction(-maxTurn,0,0));
  actions.push_back(PlatformAction(maxTurn,0,0));

  if(nodeID.pitch()>-45){
    actions.push_back(PlatformAction(0,-maxDive,0));
    actions.push_back(PlatformAction(-maxTurn,-maxDive,0));
    actions.push_back(PlatformAction(maxTurn,-maxDive,0));
  }

  if(nodeID.pitch()<45){
    actions.push_back(PlatformAction(0,maxDive,0));
    actions.push_back(PlatformAction(-maxTurn,maxDive,0));
    actions.push_back(PlatformAction(maxTurn,maxDive,0));
  }

  /*if(nodeID.speed<maxSpeed)
  {
    actions.push_back(PlatformAction(0,0,1));

    actions.push_back(PlatformAction(-maxTurn,0,1));
    actions.push_back(PlatformAction(maxTurn,0,1));

    if(nodeID.pitch()>-45){
      actions.push_back(PlatformAction(0,-maxDive,1));
      actions.push_back(PlatformAction(-maxTurn,-maxDive,1));
      actions.push_back(PlatformAction(maxTurn,-maxDive,1));
    }

    if(nodeID.pitch()<45){
      actions.push_back(PlatformAction(0,maxDive,1));
      actions.push_back(PlatformAction(-maxTurn,maxDive,1));
      actions.push_back(PlatformAction(maxTurn,maxDive,1));
    }
  }
  if(nodeID.speed>minSpeed)
  {
    actions.push_back(PlatformAction(0,0,-1));

    actions.push_back(PlatformAction(-maxTurn,0,-1));
    actions.push_back(PlatformAction(maxTurn,0,-1));

    if(nodeID.pitch()>-45){
      actions.push_back(PlatformAction(0,-maxDive,-1));
      actions.push_back(PlatformAction(-maxTurn,-maxDive,-1));
      actions.push_back(PlatformAction(maxTurn,-maxDive,-1));
    }

    if(nodeID.pitch()<45){
      actions.push_back(PlatformAction(0,maxDive,-1));
      actions.push_back(PlatformAction(-maxTurn,maxDive,-1));
      actions.push_back(PlatformAction(maxTurn,maxDive,-1));
    }
  }*/
}

void AirplaneHiFiGridlessEnvironment::GetPerfectActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  //std::cout << "Actions from: " << nodeID << " to " << getGoal() << "\n";
  static unsigned cruiseSpeed(3);
  actions.resize(0);
  double nh((nodeID.headingTo(getGoal()))-nodeID.hdg());
  //std::cout << "Desired heading " << (nodeID.headingTo(getGoal())) <<"-"<<nodeID.hdg() <<"("<<nh<<")"<< "\n";
  if(fgreater(nh,180.)){nh=-(360.-nh);} // Take complement
  nh=fgreater(nh,0)?std::min(maxTurn,nh):std::max(-maxTurn,nh); // Never turn more than max
  //std::cout << "Turn amount " << nh << "\n";

  double ne((nodeID.elevationTo(getGoal()))-nodeID.pitch());
  //std::cout << "pitch " << (nodeID.elevationTo(getGoal())) << "-"<<nodeID.pitch()<<"=" << ne << "\n";
  ne=fgreater(ne,0)?std::min(maxDive,ne):std::max(-maxDive,ne); // Never pitch more than max
  //std::cout << "pitch " << ne << "\n";

  // Slow down if we're making a major turn (It keeps us closer to the goal)
  if(fequal(fabs(nh),maxTurn)){
    if(nodeID.speed>minSpeed){
      actions.push_back(PlatformAction(nh, ne, -1));
    }else{
      actions.push_back(PlatformAction(nh, ne, 0));
    }
  }else{
    // Try to do cruise speed
    if(nodeID.speed==cruiseSpeed){
      actions.push_back(PlatformAction(nh, ne, 0));
    }else if(nodeID.speed<cruiseSpeed){
      actions.push_back(PlatformAction(nh, ne, 1));
    }else{
      actions.push_back(PlatformAction(nh, ne, -1));
    }
  }
  return;

  for(auto a(actions.begin()); a!= actions.end();){
    PlatformState next(nodeID);
    ApplyAction(next,*a);
    if(ViolatesConstraint(nodeID,next)){
      //std::cout << "VIOLATES " << *a << "\n";
      actions.erase(a);
    }else{++a;}
  }

  if(actions.empty()){
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

  for(auto a(actions.begin()); a!= actions.end();){
    PlatformState next(nodeID);
    ApplyAction(next,*a);
    if(ViolatesConstraint(nodeID,next)){
      //std::cout << "VIOLATES " << *a << "\n";
      actions.erase(a);
    }else{++a;}
  }
  if(actions.empty()){
    actions.push_back(PlatformAction(0,0,0));

    actions.push_back(PlatformAction(-maxTurn*2,0,0));
    actions.push_back(PlatformAction(maxTurn*2,0,0));
    actions.push_back(PlatformAction(0,-maxDive*2,0));
    actions.push_back(PlatformAction(0,maxDive*2,0));

    actions.push_back(PlatformAction(-maxTurn*2,-maxDive*2,0));
    actions.push_back(PlatformAction(-maxTurn*2,maxDive*2,0));
    actions.push_back(PlatformAction(maxTurn*2,-maxDive*2,0));
    actions.push_back(PlatformAction(maxTurn*2,maxDive*2,0));
    if(nodeID.speed<maxSpeed)
    {
      actions.push_back(PlatformAction(0,0,1));

      actions.push_back(PlatformAction(-maxTurn*2,0,1));
      actions.push_back(PlatformAction(maxTurn*2,0,1));
      actions.push_back(PlatformAction(0,-maxDive*2,1));
      actions.push_back(PlatformAction(0,maxDive*2,1));

      actions.push_back(PlatformAction(-maxTurn*2,-maxDive*2,1));
      actions.push_back(PlatformAction(-maxTurn*2,maxDive*2,1));
      actions.push_back(PlatformAction(maxTurn*2,-maxDive*2,1));
      actions.push_back(PlatformAction(maxTurn*2,maxDive*2,1));
    }
    if(nodeID.speed>minSpeed)
    {
      actions.push_back(PlatformAction(0,0,-1));

      actions.push_back(PlatformAction(-maxTurn*2,0,-1));
      actions.push_back(PlatformAction(maxTurn*2,0,-1));
      actions.push_back(PlatformAction(0,-maxDive*2,-1));
      actions.push_back(PlatformAction(0,maxDive*2,-1));

      actions.push_back(PlatformAction(-maxTurn*2,-maxDive*2,-1));
      actions.push_back(PlatformAction(-maxTurn*2,maxDive*2,-1));
      actions.push_back(PlatformAction(maxTurn*2,-maxDive*2,-1));
      actions.push_back(PlatformAction(maxTurn*2,maxDive*2,-1));
    }
  }
  for(auto a(actions.begin()); a!= actions.end();){
    PlatformState next(nodeID);
    ApplyAction(next,*a);
    if(ViolatesConstraint(nodeID,next)){
      //std::cout << "VIOLATES " << *a << "\n";
      actions.erase(a);
    }else{++a;}
  }
  if(actions.empty()){
    std::cout << "Couldn't create any valid successors" << "\n";
    //assert(!"Couldn't create any valid successors");
  }
}

void AirplaneHiFiGridlessEnvironment::GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  GetActions(nodeID,actions);
}

/** Gets the action required to go from node1 to node2 */
PlatformAction AirplaneHiFiGridlessEnvironment::GetAction(const PlatformState &node1, const PlatformState &node2) const
{
  PlatformAction a;
  a.turn(node2.hdg()-node1.hdg());
  //std::cout << (node2.x-node1.x) << "x " << (node2.y-node1.y) << "y "<< a.turn() << "hdg\n";
  a.pitch(node2.pitch()-node1.pitch());
  //std::cout << a.pitch() << "att\n";
  a.speed=node2.speed-node1.speed;
  return a;
}

PlatformAction AirplaneHiFiGridlessEnvironment::GetReverseAction(const PlatformState &node1, const PlatformState &node2) const
{
  PlatformAction a;
  // TODO - add this if needed
  return a;
}

void AirplaneHiFiGridlessEnvironment::ApplyAction(PlatformState &s, PlatformAction dir) const
{
  s += dir;
}

void AirplaneHiFiGridlessEnvironment::UndoAction(PlatformState &s, PlatformAction dir) const
{
  s -= dir;
}

void AirplaneHiFiGridlessEnvironment::GetNextState(const PlatformState &currents, PlatformAction dir, PlatformState &news) const
{
  news = currents;
  ApplyAction(news, dir);
}

double AirplaneHiFiGridlessEnvironment::myHCost(const PlatformState &node1, const PlatformState &node2) const
{
  double vertDiff(node2.z-node1.z);
  double vcost(fequal(vertDiff,0)?0.0:(fgreater(vertDiff,0)?climbCost:descendCost));
  return sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      (node1.y-node2.y)*(node1.y-node2.y))*cruiseBurnRate
      +sqrt((node1.z-node2.z)*(node1.z-node2.z))*vcost;
}

double AirplaneHiFiGridlessEnvironment::ReverseHCost(const PlatformState &node1, const PlatformState &node2) const
{
  return HCost(node2,node1);
}

double AirplaneHiFiGridlessEnvironment::HCost(const PlatformState &node1, const PlatformState &node2) const
{
  static unsigned cruiseSpeed(3);
  //std::cout << "HCOST()\n";
  PlatformState temp(node1);
  double nh((temp.headingTo(node2))-node1.hdg());
  if(fgreater(nh,180.)){nh=-(360.-nh);} // Take complement
  //std::cout << "nh start" << nh << "\n";

  //if(fgreater(fabs(nh),PlatformState::angularPrecision)){
  std::vector<PlatformAction> actions;
  double total(0.0);
  do{
    AirplaneHiFiGridlessEnvironment::GetPerfectActions(temp,actions);
    PlatformState prev(temp);
    PlatformAction a(actions[0]);
    ApplyAction(temp,a);
    unsigned change(0);
    if(temp.speed<cruiseSpeed && fgreater(60.,fabs(nh))){change=temp.speed; temp.speed++;}
    //std::cout << "new" << temp << "\n";
    nh=(temp.headingTo(node2))-temp.hdg();
    if(fgreater(nh,180.)){nh=-(360.-nh);} // Take complement
    total += GCost(prev,temp);
    if(change){temp.speed=change;}
    //std::cout << "nh " << nh << "\n";
  }while(!GoalTest(temp,getGoal()));//fgreater(fabs(nh),maxTurn));
  return total;//+
  //(temp.distanceTo(node2)-goalRadius)*cruiseBurnRate*PlatformState::SPEED_COST[cruiseSpeed];

  //}else{
  //return (node1.distanceTo(node2)-goalRadius)*cruiseBurnRate*PlatformState::SPEED_COST[node1.speed];
  //}
}

double AirplaneHiFiGridlessEnvironment::ReverseGCost(const PlatformState &n1, const PlatformState &n2) const
{
  return GCost(n2,n1);
}

double AirplaneHiFiGridlessEnvironment::GCost(const PlatformState &node1, const PlatformState &node2) const
{
  float softCost(0);
  for(auto const& sc: sconstraints){
    softCost += sc.cost(node2,softConstraintEffectiveness);
  }
  return node1.distanceTo(node2)*PlatformState::SPEED_COST[node2.speed]+softCost;
}

double AirplaneHiFiGridlessEnvironment::GCost(const PlatformState &node1, const PlatformAction &act) const
{
  PlatformState node2(node1);
  ApplyAction(node2,act);
  return GCost(node1,node2);
}


bool AirplaneHiFiGridlessEnvironment::GoalTest(const PlatformState &node, const PlatformState &goal) const
{
  return fless(node.distanceTo(goal),goalRadius);
  //return fless(fabs(node.x-goal.x),1.0) &&
  //fless(fabs(node.y-goal.y),1.0) &&
  //fless(fabs(node.z-goal.z),1.0);
  //fmod(fabs(node.hdg()-goal.hdg())+360,360) < 6.1 &&
  //node.speed == goal.speed;
}

double AirplaneHiFiGridlessEnvironment::GetPathLength(const std::vector<PlatformState> &sol) const
{
  double gcost(0.0);
  if(sol.size()>1)
    for(auto n(sol.begin()+1); n!=sol.end(); ++n)
      gcost += GCost(*(n-1),*n);
  return gcost;
}


uint64_t AirplaneHiFiGridlessEnvironment::GetStateHash(const PlatformState &node) const
{
  return node.key();
}

void AirplaneHiFiGridlessEnvironment::GetStateFromHash(uint64_t hash, PlatformState& s) const
{
  assert(!"Not implemented yet.");
}

uint64_t AirplaneHiFiGridlessEnvironment::GetActionHash(PlatformAction act) const
{

  uint64_t h = 0;
  h |= act.turnHalfDegs;
  h = h << 8;
  h |= act.speed;
  h = h << 8;
  h |= act.pitchHalfDegs;
  return h;
}

recVec AirplaneHiFiGridlessEnvironment::GetCoordinate(int x, int y, int z) const
{
  return {(x-width/2.0)/(width/2.0), (y-width/2.0)/(width/2.0), -4.0*z/(255.0*width)};
}

void AirplaneHiFiGridlessEnvironment::OpenGLDraw() const
{
#ifndef NO_OPENGL
  glEnable(GL_LIGHTING);
  for (int y = 0; y < length; y++)
  {
    glBegin(GL_TRIANGLE_STRIP);
    for (int x = 0; x <= width; x++)
    {
      recColor c;

      recVec a = GetCoordinate(x, y, std::max((int)GetGround(x, y), 20));
      recVec b = GetCoordinate(x, y+1, std::max((int)GetGround(x, y+1), 20));

      //DoNormal(b-a, d-a);

      if (GetGround(x, y) <= 20)
      {
        glColor3f(0, 0, 1);
      }
      else {
        c = getColor(GetGround(x, y), 0, 255, 5);
        glColor3f(c.r, c.g, c.b);
      }

      for (gridlessLandingStrip st : landingStrips) 
      {
        if (x >= min(st.x1,st.x2) && x <= max(st.x1, st.x2))
        {
          if (y >= min(st.y1, st.y2) && y <= max(st.y1, st.y2))
          {
            glColor3f(0.5, 0.5, 0.5);
          }
        }
      }


      recVec tmp = GetNormal(x, y);
      glNormal3f(tmp.x, tmp.y, tmp.z);
      glVertex3f(a.x, a.y, a.z);

      if (GetGround(x, y+1) < 20)
      {
        glColor3f(0, 0, 1);
      }
      else {
        c = getColor(GetGround(x, y+1), 0, 255, 5);
        glColor3f(c.r, c.g, c.b);
      }


      for (gridlessLandingStrip st : landingStrips) 
      {
        if (x >= min(st.x1,st.x2) && x <= max(st.x1, st.x2))
        {
          if (y+1 >= min(st.y1, st.y2) && y+1 <= max(st.y1, st.y2))
          {
            glColor3f(0.5, 0.5, 0.5);
          }
        }
      }


      tmp = GetNormal(x, y+1);
      glNormal3f(tmp.x, tmp.y, tmp.z);
      glVertex3f(b.x, b.y, b.z);
    }
    glEnd(); // ground up to 5k feet (1 mile = 4 out of 20)
  }
  glDisable(GL_LIGHTING);
  glColor3f(1.0, 1.0, 1.0);
  DrawBoxFrame(0, 0, 0.75, 1.0);

  for (gridlessLandingStrip st : landingStrips)
  {
    //Draw the dashed line down the middle

    float xval = (max(st.x1, st.x2)-min(st.x1, st.x2))/2.0f + min(st.x1, st.x2);

    glLineStipple(1, 0x00FF);
    glLineWidth(100);
    glEnable(GL_LINE_STIPPLE);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((xval-width/2.0)/(width/2.0),(min(st.y1,st.y2)-width/2.0)/(width/2.0) ,-4.0*st.z/(255.0*width));
    glVertex3f((xval-width/2.0)/(width/2.0),(max(st.y1,st.y2)-width/2.0)/(width/2.0) ,-4.0*st.z/(255.0*width));
    glEnd();
    glLineStipple(0, 0xFFFF);
    glLineWidth(1);

    // Draw the launch location
    this->SetColor(0.5,0.5,0.5, 0.3);
    this->OpenGLDraw(st.launch_state);

  }

#endif

}

void AirplaneHiFiGridlessEnvironment::OpenGLDraw(const PlatformState &l) const
{
  {
    GLfloat r, g, b, t;
    GetColor(r, g, b, t);
    glColor3f(r, g, b);
  }
  // x & y range from 20*4 = 0 to 80 = -1 to +1
  // z ranges from 0 to 20 which is 0...
  GLfloat x = (l.x-width/2.0)/width/2.0;
  GLfloat y = (l.y-length/2.0)/length/2.0;
  GLfloat z = -l.z/width;
  glEnable(GL_LIGHTING);
  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(l.hdg()/360., 0, 0, 1);
  DrawAirplane();
  glPopMatrix();

  //DrawCylinder(l.x, l.y, l.height, 0, 0.001, 0.01);
}

void AirplaneHiFiGridlessEnvironment::OpenGLDraw(const PlatformState& o, const PlatformState &n, float perc) const
{
  {
    GLfloat r, g, b, t;
    GetColor(r, g, b, t);
    glColor3f(r, g, b);
  }

  GLfloat x1 = (o.x-width/2.0)/width/2.0;
  GLfloat y1 = (o.y-length/2.0)/length/2.0;
  GLfloat z1 = -o.z/width;
  GLfloat h1 = 360*o.hdg()/8.0;

  GLfloat x2 = (n.x-width/2.0)/width/2.0;
  GLfloat y2 = (n.y-length/2.0)/length/2.0;
  GLfloat z2 = -n.z/width;
  GLfloat h2 = 360*n.hdg()/8.0;
  if (o.hdg() < 2 && n.hdg() >= 6)
    h2 -= 360;
  if (o.hdg() >= 6 && n.hdg() < 2)
    h1 -= 360;

  glEnable(GL_LIGHTING);
  glPushMatrix();
  glTranslatef((1-perc)*x1+perc*x2, (1-perc)*y1+perc*y2, (1-perc)*z1+perc*z2);
  //std::cout << "plane " << x1 << " " << y1 << " " << z1 << "\n";


  float m[16];
  float el1(((1-perc)*o.pitch()+perc*n.pitch())*constants::degToRad);
  float ro1(((1-perc)*o.roll()+perc*n.roll())*constants::degToRad);
  float az1(((1-perc)*o.hdg()+perc*n.hdg()+180.)*-constants::degToRad);
  m[0] = cos(az1)*cos(el1); // CA*CE;
  m[1] = sin(az1)*cos(el1); // SA*CE
  m[2] = -sin(el1); // -SE
  m[3] = 0;

  m[4] = cos(az1)*sin(el1)*sin(ro1) - sin(az1)*cos(ro1); // CA*SE*SR - SA*CR
  m[5] = cos(az1)*cos(ro1) + sin(az1)*sin(el1)*sin(ro1); // CA*CR + SA*SE*SR
  m[6] = cos(el1)*sin(ro1); // CE*SR
  m[7] = 0;

  m[8] = cos(az1)*sin(el1)*cos(ro1) + sin(az1)*cos(ro1); // CA*SE*CR + SA*SR
  m[9] = sin(az1)*sin(el1)*cos(ro1) - cos(az1)*sin(ro1); // SA*SE*CR – CA*SR
  m[10] = cos(el1)*cos(ro1); // CE*CR
  m[11] = 0;

  m[12] = 0;
  m[13] = 0;
  m[14] = 0;
  m[15] = 1;

  glMultMatrixf(m);
  //glRotatef((1-perc)*o.pitch()+perc*n.pitch(), 0, 1, 0);
  //glRotatef((1-perc)*o.roll()+perc*n.roll(), 1, 0, 0);
  //glRotatef(-1.*((1-perc)*o.hdg()+perc*n.hdg()+180.), 0, 0, 1);

  DrawAirplane();
  glPopMatrix();
}

void AirplaneHiFiGridlessEnvironment::OpenGLDraw(const PlatformState &, const PlatformAction &) const
{
  //TODO: Implement this
}

void AirplaneHiFiGridlessEnvironment::DrawAirplane() const
{
  // Body
  DrawCylinder(0, 0, 0, 0, 0.01/5.0, 0.01);

  // Nose
  DrawHemisphere(0,-0.005,0,0.01/5.0);

  // Wings
  glBegin(GL_TRIANGLES);
  glVertex3f(0,-0.0047,0);
  glVertex3f(-0.01, 0, 0);
  glVertex3f(0.01,0,0);
  glEnd();

  // Tail
  glBegin(GL_TRIANGLES);
  glVertex3f(0,0.00,0);
  glVertex3f(0,0.005,0);
  glVertex3f(0,0.005,-0.004);
  glEnd();

}

void AirplaneHiFiGridlessEnvironment::GLDrawLine(const PlatformState &a, const PlatformState &b) const
{
  glColor4f(1.0, 1.0, 1.0, .5); // Make it partially opaque gray

  // Normalize coordinates between (-1, 1)
  GLfloat x_start((a.x-width/2.0)/width/2.0);
  GLfloat y_start((a.y-length/2.0)/length/2.0);
  GLfloat z_start(-a.z/width);

  GLfloat x_end((b.x-width/2.0)/width/2.0);
  GLfloat y_end((b.y-length/2.0)/length/2.0);
  GLfloat z_end(-b.z/width);

  glDisable(GL_LIGHTING);
  glPushMatrix();

  // Draw the edge line segments
  glBegin(GL_LINES);

  glVertex3f(x_start,y_start,z_start);
  glVertex3f(x_end,y_end,z_end);

  //std::cout << "Line " << x_start << " " << y_start << " " << z_start << "\n";
  glEnd();

  glPopMatrix();
}

void AirplaneHiFiGridlessEnvironment::GLDrawPath(const std::vector<PlatformState> &p, const std::vector<PlatformState> &waypoints) const
{
  if(p.size()<2) return;
  //TODO Draw waypoints as cubes.
  for(auto a(p.begin()+1); a!=p.end(); ++a){
    GLDrawLine(*(a-1),*a);
  }
}


std::vector<uint8_t> AirplaneHiFiGridlessEnvironment::getGround() 
{
  return std::vector<uint8_t>(ground);
}
std::vector<recVec> AirplaneHiFiGridlessEnvironment::getGroundNormals()
{
  return std::vector<recVec>(groundNormals);
}
std::vector<PlatformAction> AirplaneHiFiGridlessEnvironment::getInternalActions()
{
  return std::vector<PlatformAction>(internalActions);
}

void AirplaneHiFiGridlessEnvironment::AddLandingStrip(gridlessLandingStrip& strip)
{
  // Set the ground to average height on the strip
  float avgh = 0.0f;
  int n = 0;
  for (int i = min(strip.x1, strip.x2); i <= max(strip.x1, strip.x2); i++) 
  {
    for (int j = min(strip.y1, strip.y2); j <= max(strip.y1, strip.y2); j++) 
    {
      avgh += GetGround(i,j);
      n++;
    }
  }

  avgh = (avgh/n);

  for (int i = min(strip.x1, strip.x2); i <= max(strip.x1, strip.x2); i++) 
  {
    for (int j = min(strip.y1, strip.y2); j <= max(strip.y1, strip.y2); j++) 
    {
      SetGround(i, j, avgh);
    }
  }


  //Re-Calculate the normals
  for (int y = 0; y < length; y++)
  {
    for (int x = 0; x <= width; x++)
    {
      if (x < width)
      {
        recVec a = GetCoordinate(x, y, std::max((int)GetGround(x, y), 20));
        recVec b = GetCoordinate(x, y+1, std::max((int)GetGround(x, y+1), 20));
        recVec d = GetCoordinate(x+1, y, std::max((int)GetGround(x+1, y), 20));
        recVec n = (a-b).GetNormal(a-d);
        GetNormal(x, y) += n;
        GetNormal(x, y+1) += n;
        GetNormal(x+1, y) += n;
      }
      if (x > 0)
      {
        recVec a = GetCoordinate(x, y, std::max((int)GetGround(x, y), 20));
        recVec b = GetCoordinate(x-1, y+1, std::max((int)GetGround(x-1, y+1), 20));
        recVec d = GetCoordinate(x, y+1, std::max((int)GetGround(x, y+1), 20));
        recVec n = (a-b).GetNormal(a-d);
        GetNormal(x, y) += n;
        GetNormal(x-1, y+1) += n;
        GetNormal(x, y+1) += n;
      }
    }
  }
  for (int y = 0; y <= length; y++)
  {
    for (int x = 0; x <= width; x++)
    {
      GetNormal(x, y).normalise();
    }
  }

  // Add the landing strip
  strip.z =  avgh;
  this->landingStrips.push_back(strip);

}

void AirplaneHiFiGridlessEnvironment::AddConstraint(Constraint<PlatformState> c){constraints.push_back(c);}
void AirplaneHiFiGridlessEnvironment::ClearConstraints(){constraints.resize(0);}
void AirplaneHiFiGridlessEnvironment::ClearStaticConstraints(){static_constraints.resize(0);}

bool AirplaneHiFiGridlessEnvironment::ViolatesConstraint(const PlatformState &from, const PlatformState &to, int time) const
{
  return ViolatesConstraint(from, to);

}

// Basically the same code as above, but overloaded so the first section is not necessary
bool AirplaneHiFiGridlessEnvironment::ViolatesConstraint(const PlatformState &from, const PlatformState &to) const
{
  // If both are landed, then the plane's not going anywhere - and we're good
  if (from.landed && to.landed)
    return false;

  //Check if the action box violates any of the constraints that are in the constraints list
  for (Constraint<PlatformState> c : constraints)
  {
    if(c.ConflictsWith(from,to)){return true;}
  }


  // If landing, we don't want to check the static constraints - but we don't want planes taking off and landing
  // at the same time
  if (from.landed || to.landed)
  {
    return false;
  }

  // Generate a well formed set of boxes for the action box
  // which we will need later to compare
  float a_minx = std::min<int>(from.x, to.x);
  float a_maxx = std::max<int>(from.x, to.x);

  float a_miny = std::min<int>(from.y, to.y);
  float a_maxy = std::max<int>(from.y, to.y);

  float a_minz = std::min<int>(from.z, to.z);
  float a_maxz = std::max<int>(from.z, to.z);

  // Allocate some temp variables
  float c_minx, c_maxx, c_miny, c_maxy, c_minz, c_maxz;


  // Treat static constraints as a box
  for (Constraint<PlatformState> c : static_constraints)
  {
    // Check if the range of the constraint overlaps in time
    if (max(c.start_state.t, from.t) <= min(c.end_state.t, to.t) + 0.00001) 
    {
      // Generate a well formed set of boxes for the constraint box
      c_minx = c.start_state.x < c.end_state.x ? c.start_state.x : c.end_state.x;
      c_maxx = c.start_state.x > c.end_state.x ? c.start_state.x : c.end_state.x;

      c_miny = c.start_state.y < c.end_state.y ? c.start_state.y : c.end_state.y;
      c_maxy = c.start_state.y > c.end_state.y ? c.start_state.y : c.end_state.y;

      c_minz = c.start_state.z < c.end_state.z ? c.start_state.z : c.end_state.z;
      c_maxz = c.start_state.z > c.end_state.z ? c.start_state.z : c.end_state.z;


      if (std::max<int>(c_minx, a_minx) <= std::min<int>(c_maxx, a_maxx) && // Check if overlapping on the X axis
          std::max<int>(c_miny, a_miny) <= std::min<int>(c_maxy, a_maxy) && // Check if overlapping on the Y axis
          std::max<int>(c_minz, a_minz) <= std::min<int>(c_maxz, a_maxz)    // Check if overlapping on the Z axis
      )
      {
        // If we overlap on all three axis, then there must be a common point, and thus
        // we can return that the constraint was violated
        return true;
      }

    }
  }

  // If no constraint is violated, return false
  return false;
}

bool Constraint<PlatformState>::ConflictsWith(const PlatformState &state) const
{
  if(state.landed || end_state.landed && start_state.landed) return false;
  //std::cout << "VERTEX"<<*this << "ConflictsWith" << state << "...\n";
  // Unit time steps...
  if (state.t == start_state.t || state.t == end_state.t) {
    return fless(state.distanceTo(start_state)+state.distanceTo(end_state),start_state.distanceTo(end_state)*1.5);
  }
  return false;
}

// Check both vertex and edge constraints
bool Constraint<PlatformState>::ConflictsWith(const PlatformState &from, const PlatformState &to) const
{
  //std::cout << "EDGE"<<from << "ConflictsWith" << to << "...\n";
  if(from.landed && to.landed || end_state.landed && start_state.landed) return false;
  double dist(start_state.distanceTo(end_state));
  if (from.t == start_state.t) {
    PlatformState bisect((from.x+to.x)/2.,(from.y+to.y)/2.,(from.z+to.z)/2.,0,0,0);

    return fless(from.distanceTo(start_state)+from.distanceTo(end_state),dist*1.5) ||
      fless(bisect.distanceTo(start_state)+bisect.distanceTo(end_state),dist*1.5) ||
      fless(to.distanceTo(start_state)+to.distanceTo(end_state),dist*1.5);
  }else if(from.t==end_state.t){
    return from.distanceTo(end_state) < dist*.25;
  }else if(to.t==start_state.t){
    return to.distanceTo(start_state) < dist*.25;
  }

  return false;
}

bool Constraint<PlatformState>::ConflictsWith(const Constraint<PlatformState> &x) const
{
  return ConflictsWith(x.start_state, x.end_state);
}


void Constraint<PlatformState>::OpenGLDraw(MapInterface*) const 
{
  glLineWidth(2.0); // Make it wide
  static int halfWidth(DrawableConstraint::width/2.0);
  static int halfLength(DrawableConstraint::length/2.0);

  // Normalize coordinates between (-1, 1)
  GLfloat x_start = (start_state.x-halfWidth)/halfWidth;
  GLfloat y_start = (start_state.y-halfLength)/halfLength;
  GLfloat z_start = -start_state.z/DrawableConstraint::width;

  GLfloat x_end = (end_state.x-halfWidth)/halfWidth;
  GLfloat y_end = (end_state.y-halfLength)/halfLength;
  GLfloat z_end = -end_state.z/DrawableConstraint::width;


  GLfloat min_x, min_y, min_z, max_x, max_y, max_z;

  if (strip)
  {
    min_x = min(x_start-.5/halfWidth,x_end-.5/halfWidth);
    max_x = max(x_start+.5/halfWidth,x_end+.5/halfWidth);
    min_y = min(y_start-.5/halfLength,y_end-.5/halfLength);
    max_y = max(y_start+.5/halfLength,y_end+.5/halfLength);
    min_z = min(z_start,z_end);
    max_z = max(z_start+.5/halfWidth,z_end+.5/halfWidth);
  } else {
    min_x = min(x_start-.5/halfWidth,x_end-.5/halfWidth);
    max_x = max(x_start+.5/halfWidth,x_end+.5/halfWidth);
    min_y = min(y_start-.5/halfLength,y_end-.5/halfLength);
    max_y = max(y_start+.5/halfLength,y_end+.5/halfLength);
    min_z = min(z_start-.5/halfWidth,z_end-.5/halfWidth);
    max_z = max(z_start+.5/halfWidth,z_end+.5/halfWidth);
  }

  glDisable(GL_LIGHTING);
  glPushMatrix();

  // Draw the lower loop
  glBegin(GL_LINE_LOOP);
  glVertex3f(min_x, min_y, min_z);
  glVertex3f(max_x, min_y, min_z);
  glVertex3f(max_x, max_y, min_z);
  glVertex3f(min_x, max_y, min_z);
  glEnd();

  // Draw the upper loop
  glBegin(GL_LINE_LOOP);
  glVertex3f(min_x, min_y, max_z);
  glVertex3f(max_x, min_y, max_z);
  glVertex3f(max_x, max_y, max_z);
  glVertex3f(min_x, max_y, max_z);
  glEnd();

  // Draw the edge line segments
  glBegin(GL_LINES);

  glVertex3f(min_x, min_y, min_z);
  glVertex3f(min_x, min_y, max_z);

  glVertex3f(min_x, max_y, min_z);
  glVertex3f(min_x, max_y, max_z);

  glVertex3f(max_x, min_y, min_z);
  glVertex3f(max_x, min_y, max_z);

  glVertex3f(max_x, max_y, min_z);
  glVertex3f(max_x, max_y, max_z);

  glEnd();

  glPopMatrix();
}

template<>
TemplateAStar<PlatformState, PlatformAction, AirplaneHiFiGridlessEnvironment, AStarOpenClosed<PlatformState, NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment> > >* NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::currentAstar=0;
template<>
AirplaneHiFiGridlessEnvironment* NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::currentEnv=0;
template<>
uint8_t NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::currentAgent=0;
template<>
bool NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::randomalg=false;
template<>
bool NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::useCAT=false;
template<>
UnitTimeCAT<PlatformState,AirplaneHiFiGridlessEnvironment>* NonHolonomicComparator<PlatformState,PlatformAction,AirplaneHiFiGridlessEnvironment>::CAT=0;

template<>
double SoftConstraint<PlatformState>::cost(PlatformState const& other, double scale) const{
  double d(Util::distance(center.x,center.y,center.z,other.x,other.y,other.z)/scale);
  return std::max(0.0,logr/d-logr/radius);
}
