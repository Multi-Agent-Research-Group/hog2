//
//  AirplaneGridless.cpp
//  hog2 glut
//
//  Created by Thayne Walker on 2/6/17.
//  Copyright (c) Thayne Walker 2017.
//

#include <stdio.h>
#include "AirplaneGridless.h"
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

bool operator==(const PlatformAction &a1, const PlatformAction &a2)
{
  return a1.turnHalfDegs == a2.turnHalfDegs && a1.speed==a2.speed && a1.pitchHalfDegs==a2.pitchHalfDegs;
}

AirplaneGridlessEnvironment::AirplaneGridlessEnvironment(
  unsigned width,
  unsigned length,
  unsigned height,
  double minSpeed,
  double maxSpeed,
  uint8_t numSpeeds,
  double cruiseBurnRate,
  double climbCost,
  double descendCost
): width(width),
  length(length),
  height(height),
  minSpeed(minSpeed),
  maxSpeed(maxSpeed),
  numSpeeds(numSpeeds),
  cruiseBurnRate(cruiseBurnRate),
  climbCost(climbCost),
  descendCost(descendCost)
{
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

void AirplaneGridlessEnvironment::SetGround(int x, int y, uint8_t val)
{
    ground[x + y*(length+1)] = val;
}

uint8_t AirplaneGridlessEnvironment::GetGround(int x, int y) const
{
    return ground[x + y*(length+1)];
}

bool AirplaneGridlessEnvironment::Valid(int x, int y)
{
    return x >= 0 && x <= width && y >= 0 && y <= length;
}

recVec &AirplaneGridlessEnvironment::GetNormal(int x, int y)
{
    return groundNormals[x + y*(length+1)];
}

recVec AirplaneGridlessEnvironment::GetNormal(int x, int y) const
{
    return groundNormals[x + y*(length+1)];
}

void AirplaneGridlessEnvironment::RecurseGround(int x1, int y1, int x2, int y2)
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


void AirplaneGridlessEnvironment::GetSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const
{
    std::cout << "Succ: " << nodeID << "\n";
    GetActions(nodeID, internalActions);
    for (auto &act : internalActions)
    {
        PlatformState s;
        GetNextState(nodeID, act, s);
        //std::cout << "  " << s <<":"<<GCost(nodeID,s)<<"+"<<HCost(s,getGoal())<<"="<<(GCost(nodeID,s)+HCost(nodeID,getGoal()))<< "\n";
        neighbors.push_back(s);
    }
}

void AirplaneGridlessEnvironment::GetReverseSuccessors(const PlatformState &nodeID, std::vector<PlatformState> &neighbors) const
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

void AirplaneGridlessEnvironment::GetActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  actions.resize(0);
  actions.push_back(PlatformAction(0, 0, 0));  // No change
  actions.push_back(PlatformAction(-45, 0, 0)); // 45 degs left
  actions.push_back(PlatformAction(45, 0, 0));  // 45 degs right
  actions.push_back(PlatformAction(-90, 0, 0)); // 90 degs left
  actions.push_back(PlatformAction(90, 0, 0));  // 90 degs right

  // Increase speed at current height
  if (nodeID.speed < numSpeeds + minSpeed-1){
    actions.push_back(PlatformAction(0, 0, 1));
    actions.push_back(PlatformAction(-45, 0, 1));
    actions.push_back(PlatformAction(45, 0, 1));
    actions.push_back(PlatformAction(-90, 0, 1));
    actions.push_back(PlatformAction(90, 0, 1));
  }
  // Decrease speed at current height
  if (nodeID.speed > minSpeed){
    actions.push_back(PlatformAction(0, 0, -1));
    actions.push_back(PlatformAction(-45, 0, -1));
    actions.push_back(PlatformAction(45, 0, -1));
    actions.push_back(PlatformAction(-90, 0, -1));
    actions.push_back(PlatformAction(90, 0, -1));
  }

  // Check for decreasing height
  if (nodeID.z > 1) {
    // Decrease height
    actions.push_back(PlatformAction(0, -7.5, 0));
    actions.push_back(PlatformAction(-45, -7.5, 0));
    actions.push_back(PlatformAction(45, -7.5, 0));
    actions.push_back(PlatformAction(-90, -7.5, 0));
    actions.push_back(PlatformAction(90, -7.5, 0));
    // Decrease height and speed
    if (nodeID.speed > minSpeed){
      actions.push_back(PlatformAction(0, -7.5, -1));
      actions.push_back(PlatformAction(-45, -7.5, -1));
      actions.push_back(PlatformAction(45, -7.5, -1));
      actions.push_back(PlatformAction(-90, -7.5, -1));
      actions.push_back(PlatformAction(90, -7.5, -1));
    }
    // Decrease height increase speed
    if (nodeID.speed < numSpeeds+minSpeed-1) {
      actions.push_back(PlatformAction(0, -7.5, 1));
      actions.push_back(PlatformAction(-45, -7.5, 1));
      actions.push_back(PlatformAction(45, -7.5, 1));
      actions.push_back(PlatformAction(-90, -7.5, 1));
      actions.push_back(PlatformAction(90, -7.5, 1));
    }
  }

  // Check for increasing height
  if (nodeID.z < height){
    // Increase height
    actions.push_back(PlatformAction(0, 7.5, 0));
    actions.push_back(PlatformAction(-45, 7.5, 0));
    actions.push_back(PlatformAction(45, 7.5, 0));
    actions.push_back(PlatformAction(-90, 7.5, 0));
    actions.push_back(PlatformAction(90, 7.5, 0));
    // increase height decrease speed
    if (nodeID.speed > minSpeed){
      actions.push_back(PlatformAction(0, 7.5, -1));
      actions.push_back(PlatformAction(-45, 7.5, -1));
      actions.push_back(PlatformAction(45, 7.5, -1));
      actions.push_back(PlatformAction(-90, 7.5, -1));
      actions.push_back(PlatformAction(90, 7.5, -1));
    }
    // increase height increase speed
    if (nodeID.speed < numSpeeds+minSpeed-1) {
      actions.push_back(PlatformAction(0, 7.5, 1));
      actions.push_back(PlatformAction(-45, 7.5, 1));
      actions.push_back(PlatformAction(45, 7.5, 1));
      actions.push_back(PlatformAction(-90, 7.5, 1));
      actions.push_back(PlatformAction(90, 7.5, 1));
    }
  }
}

void AirplaneGridlessEnvironment::GetReverseActions(const PlatformState &nodeID, std::vector<PlatformAction> &actions) const
{
  actions.push_back(PlatformAction(0, 0, 0));  // No change
  actions.push_back(PlatformAction(-45, 0, 0)); // 45 degs left
  actions.push_back(PlatformAction(45, 0, 0));  // 45 degs right
  actions.push_back(PlatformAction(-90, 0, 0)); // 90 degs left
  actions.push_back(PlatformAction(90, 0, 0));  // 90 degs right

  // Increase speed at current height
  if (nodeID.speed < numSpeeds + minSpeed-1){
    actions.push_back(PlatformAction(0, 0, -1));
    actions.push_back(PlatformAction(-45, 0, -1));
    actions.push_back(PlatformAction(45, 0, -1));
    actions.push_back(PlatformAction(-90, 0, -1));
    actions.push_back(PlatformAction(90, 0, -1));
  }
  // Decrease speed at current height
  if (nodeID.speed > minSpeed){
    actions.push_back(PlatformAction(0, 0, 1));
    actions.push_back(PlatformAction(-45, 0, 1));
    actions.push_back(PlatformAction(45, 0, 1));
    actions.push_back(PlatformAction(-90, 0, 1));
    actions.push_back(PlatformAction(90, 0, 1));
  }

  // Check for decreasing height
  if (nodeID.z > 1) {
    // Increase height
    actions.push_back(PlatformAction(0, 7.5, 0));
    actions.push_back(PlatformAction(-45, 7.5, 0));
    actions.push_back(PlatformAction(45, 7.5, 0));
    actions.push_back(PlatformAction(-90, 7.5, 0));
    actions.push_back(PlatformAction(90, 7.5, 0));
    // increase height decrease speed
    if (nodeID.speed > minSpeed){
      actions.push_back(PlatformAction(0, 7.5, 1));
      actions.push_back(PlatformAction(-45, 7.5, 1));
      actions.push_back(PlatformAction(45, 7.5, 1));
      actions.push_back(PlatformAction(-90, 7.5, 1));
      actions.push_back(PlatformAction(90, 7.5, 1));
    }
    // increase height increase speed
    if (nodeID.speed < numSpeeds+minSpeed-1) {
      actions.push_back(PlatformAction(0, 7.5, -1));
      actions.push_back(PlatformAction(-45, 7.5, -1));
      actions.push_back(PlatformAction(45, 7.5, -1));
      actions.push_back(PlatformAction(-90, 7.5, -1));
      actions.push_back(PlatformAction(90, 7.5, -1));
    }
  }

  // Check for increasing height
  if (nodeID.z < height){
    // Decrease height
    actions.push_back(PlatformAction(0, -7.5, 0));
    actions.push_back(PlatformAction(-45, -7.5, 0));
    actions.push_back(PlatformAction(45, -7.5, 0));
    actions.push_back(PlatformAction(-90, -7.5, 0));
    actions.push_back(PlatformAction(90, -7.5, 0));
    // Decrease height and speed
    if (nodeID.speed > minSpeed){
      actions.push_back(PlatformAction(0, -7.5, 1));
      actions.push_back(PlatformAction(-45, -7.5, 1));
      actions.push_back(PlatformAction(45, -7.5, 1));
      actions.push_back(PlatformAction(-90, -7.5, 1));
      actions.push_back(PlatformAction(90, -7.5, 1));
    }
    // Decrease height increase speed
    if (nodeID.speed < numSpeeds+minSpeed-1) {
      actions.push_back(PlatformAction(0, -7.5, -1));
      actions.push_back(PlatformAction(-45, -7.5, -1));
      actions.push_back(PlatformAction(45, -7.5, -1));
      actions.push_back(PlatformAction(-90, -7.5, -1));
      actions.push_back(PlatformAction(90, -7.5, -1));
    }
  }
}

/** Gets the action required to go from node1 to node2 */
PlatformAction AirplaneGridlessEnvironment::GetAction(const PlatformState &node1, const PlatformState &node2) const
{
  PlatformAction a;
  a.turn(atan2(node2.x-node1.x,node2.y-node1.y)*constants::radToDeg);
  std::cout << (node2.x-node1.x) << "x " << (node2.y-node1.y) << "y "<< a.turn() << "hdg\n";
  a.pitch(atan2(node2.z-node1.z,PlatformState::SPEEDS[node2.speed])*constants::radToDeg);
  std::cout << a.pitch() << "att\n";
  a.speed=node2.speed-node1.speed;
  // TODO - add this if needed
  return a;
}

PlatformAction AirplaneGridlessEnvironment::GetReverseAction(const PlatformState &node1, const PlatformState &node2) const
{
  PlatformAction a;
  // TODO - add this if needed
  return a;
}

void AirplaneGridlessEnvironment::ApplyAction(PlatformState &s, PlatformAction dir) const
{
  s += dir;
}

void AirplaneGridlessEnvironment::UndoAction(PlatformState &s, PlatformAction dir) const
{
  s -= dir;
}

void AirplaneGridlessEnvironment::GetNextState(const PlatformState &currents, PlatformAction dir, PlatformState &news) const
{
    news = currents;
    ApplyAction(news, dir);
}

double AirplaneGridlessEnvironment::myHCost(const PlatformState &node1, const PlatformState &node2) const
{
  double vertDiff(node2.z-node1.z);
  double vcost(fequal(vertDiff,0)?0.0:(fgreater(vertDiff,0)?climbCost:descendCost));
  return sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      (node1.y-node2.y)*(node1.y-node2.y))*cruiseBurnRate
      +sqrt((node1.z-node2.z)*(node1.z-node2.z))*vcost;
}

double AirplaneGridlessEnvironment::ReverseHCost(const PlatformState &node1, const PlatformState &node2) const
{
  return HCost(node2,node1);
}

double AirplaneGridlessEnvironment::HCost(const PlatformState &node1, const PlatformState &node2) const
{
  //double vertDiff(node2.z-node1.z);
  //double vcost(fequal(vertDiff,0)?0.0:(fgreater(vertDiff,0)?climbCost:descendCost));
  //return sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      //(node1.y-node2.y)*(node1.y-node2.y))*cruiseBurnRate*PlatformState::SPEED_COST[3]
      //+fabs(vertDiff)*vcost;
      //std::cout << "H  d:"<<sqrt((node1.x-node2.x)*(node1.x-node2.x)+(node1.y-node2.y)*(node1.y-node2.y)+(node1.z-node2.z)*(node1.z-node2.z))<<" c:"<<PlatformState::SPEED_COST[node2.speed]<<"="<<sqrt((node1.x-node2.x)*(node1.x-node2.x)+(node1.y-node2.y)*(node1.y-node2.y)+(node1.z-node2.z)*(node1.z-node2.z))*PlatformState::SPEED_COST[node2.speed]<<"\n";
  return sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      (node1.y-node2.y)*(node1.y-node2.y)+
      (node1.z-node2.z)*(node1.z-node2.z))*
      cruiseBurnRate*PlatformState::SPEED_COST[node1.speed];
}

double AirplaneGridlessEnvironment::ReverseGCost(const PlatformState &n1, const PlatformState &n2) const
{
  return GCost(n2,n1);
}

double AirplaneGridlessEnvironment::GCost(const PlatformState &node1, const PlatformState &node2) const
{
  //double vertDiff(node2.z-node1.z);
  //double vcost(fequal(vertDiff,0)?0.0:(fgreater(vertDiff,0)?climbCost:descendCost));
  /*std::cout << "GCOST " << node1<<node2<<sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      (node1.y-node2.y)*(node1.y-node2.y))<<"*"<<cruiseBurnRate<<"*"<<PlatformState::SPEED_COST[node2.speed]<<"+"<<fabs(vertDiff)<<"*"<<vcost<<"="
      <<(sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      (node1.y-node2.y)*(node1.y-node2.y))*cruiseBurnRate*PlatformState::SPEED_COST[node2.speed]
      +fabs(vertDiff)*vcost)<<"\n";*/
  //return sqrt((node1.x-node2.x)*(node1.x-node2.x)+
      //(node1.y-node2.y)*(node1.y-node2.y))*cruiseBurnRate*PlatformState::SPEED_COST[node2.speed]
      //+fabs(vertDiff)*vcost;
      //std::cout << "  d:"<<sqrt((node1.x-node2.x)*(node1.x-node2.x)+(node1.y-node2.y)*(node1.y-node2.y)+(node1.z-node2.z)*(node1.z-node2.z))<<" c:"<<PlatformState::SPEED_COST[node2.speed]<<"="<<sqrt((node1.x-node2.x)*(node1.x-node2.x)+(node1.y-node2.y)*(node1.y-node2.y)+(node1.z-node2.z)*(node1.z-node2.z))*PlatformState::SPEED_COST[node2.speed]<<"\n";
      return /*sqrt((node1.x-node2.x)*(node1.x-node2.x)+
                  (node1.y-node2.y)*(node1.y-node2.y)+
                  (node1.z-node2.z)*(node1.z-node2.z))**/
             cruiseBurnRate*PlatformState::SPEED_COST[node2.speed];
}

double AirplaneGridlessEnvironment::GCost(const PlatformState &node1, const PlatformAction &act) const
{
    PlatformState node2(node1);
    ApplyAction(node2,act);
    return GCost(node1,node2);
}


bool AirplaneGridlessEnvironment::GoalTest(const PlatformState &node, const PlatformState &goal) const
{
    return fless(fabs(node.x-goal.x),1.0) &&
    fless(fabs(node.y-goal.y),1.0) &&
    fless(fabs(node.z-goal.z),1.0) &&
    fmod(fabs(node.hdg()-goal.hdg())+360,360) < 6.1 &&
    node.speed == goal.speed;
}

double AirplaneGridlessEnvironment::GetPathLength(const std::vector<PlatformState> &sol) const
{
    double gcost(0.0);
    if(sol.size()>1)
      for(auto n(sol.begin()+1); n!=sol.end(); ++n)
        gcost += GCost(*(n-1),*n);
    return gcost;
}


uint64_t AirplaneGridlessEnvironment::GetStateHash(const PlatformState &node) const
{
    return node.key();
}

void AirplaneGridlessEnvironment::GetStateFromHash(uint64_t hash, PlatformState& s) const
{
    assert(!"Not implemented yet.");
}

uint64_t AirplaneGridlessEnvironment::GetActionHash(PlatformAction act) const
{

    uint64_t h = 0;
    h |= act.turnHalfDegs;
    h = h << 8;
    h |= act.speed;
    h = h << 8;
    h |= act.pitchHalfDegs;
    return h;
}

recVec AirplaneGridlessEnvironment::GetCoordinate(int x, int y, int z) const
{
    return {(x-width/2.0)/(width/2.0), (y-width/2.0)/(width/2.0), -4.0*z/(255.0*width)};
}

void AirplaneGridlessEnvironment::OpenGLDraw() const
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

void AirplaneGridlessEnvironment::OpenGLDraw(const PlatformState &l) const
{
    {
        GLfloat r, g, b, t;
        GetColor(r, g, b, t);
        glColor3f(r, g, b);
    }
    // x & y range from 20*4 = 0 to 80 = -1 to +1
    // z ranges from 0 to 20 which is 0...
    GLfloat x = (l.x-width/2.0)/width/2.0;
    GLfloat y = (l.y-length)/length;
    GLfloat z = -l.z/width;
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(360*l.hdg()/8.0, 0, 0, 1);
    DrawAirplane();
    glPopMatrix();
    
    //DrawCylinder(l.x, l.y, l.height, 0, 0.001, 0.01);
}

void AirplaneGridlessEnvironment::OpenGLDraw(const PlatformState& o, const PlatformState &n, float perc) const
{
    {
        GLfloat r, g, b, t;
        GetColor(r, g, b, t);
        glColor3f(r, g, b);
    }
    
    GLfloat x1 = (o.x-width/2.0)/width/2.0;
    GLfloat y1 = (o.y-length)/length;
    GLfloat z1 = -o.z/width;
    GLfloat h1 = 360*o.hdg()/8.0;

    GLfloat x2 = (n.x-width/2.0)/width/2.0;
    GLfloat y2 = (n.y-length)/length;
    GLfloat z2 = -n.z/width;
    GLfloat h2 = 360*n.hdg()/8.0;
    if (o.hdg() < 2 && n.hdg() >= 6)
        h2 -= 360;
    if (o.hdg() >= 6 && n.hdg() < 2)
        h1 -= 360;

    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef((1-perc)*x1+perc*x2, (1-perc)*y1+perc*y2, (1-perc)*z1+perc*z2);
    glRotatef((1-perc)*h1+perc*h2, 0, 0, 1);

    if(n.z>o.z) glRotatef(-45, 1, 1, 0);
    else if(n.z<o.z) glRotatef(45, 1, 1, 0);

    DrawAirplane();
    glPopMatrix();
}

void AirplaneGridlessEnvironment::OpenGLDraw(const PlatformState &, const PlatformAction &) const
{
    //TODO: Implement this
}

void AirplaneGridlessEnvironment::DrawAirplane() const
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

void AirplaneGridlessEnvironment::GLDrawLine(const PlatformState &a, const PlatformState &b) const
{
    glColor4f(1.0, 1.0, 1.0, .5); // Make it partially opaque gray

    // Normalize coordinates between (-1, 1)
    GLfloat x_start((a.x-width/2.0)/width/2.0);
    GLfloat y_start((a.y-length)/length);
    GLfloat z_start(-a.z/width);

    GLfloat x_end((b.x-width/2.0)/width/2.0);
    GLfloat y_end((b.y-length)/length);
    GLfloat z_end(-b.z/width);

    glDisable(GL_LIGHTING);
    glPushMatrix();

    // Draw the edge line segments
    glBegin(GL_LINES);

    glVertex3f(x_start,y_start,z_start);
    glVertex3f(x_end,y_end,z_end);

    glEnd();

    glPopMatrix();
}

void AirplaneGridlessEnvironment::GLDrawPath(const std::vector<PlatformState> &p, const std::vector<PlatformState> &waypoints) const
{
        if(p.size()<2) return;
        int wpt(0);
        //TODO Draw waypoints as cubes.
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}


std::vector<uint8_t> AirplaneGridlessEnvironment::getGround() 
{
    return std::vector<uint8_t>(ground);
}
std::vector<recVec> AirplaneGridlessEnvironment::getGroundNormals()
{
    return std::vector<recVec>(groundNormals);
}
std::vector<PlatformAction> AirplaneGridlessEnvironment::getInternalActions()
{
    return std::vector<PlatformAction>(internalActions);
}

void AirplaneGridlessEnvironment::AddLandingStrip(gridlessLandingStrip& strip)
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

void AirplaneGridlessEnvironment::ClearStaticConstraints(){static_constraints.resize(0);}
double AirplaneGridlessEnvironment::ViolatesConstraint(const PlatformState &from, const PlatformState &to, int time) const{
  PlatformAction act = GetAction(from,to);
  PlatformState toState(from);
  ApplyAction(toState,act);
  return ViolatesConstraint(from,toState);
}

double AirplaneGridlessEnvironment::ViolatesConstraint(const PlatformState &from, const PlatformState &to) const{
 // Generate a well formed set of boxes for the action box
  // which we will need later to compare
  float a_minx = std::min(from.x, to.x);
  float a_maxx = std::max(from.x, to.x);

  float a_miny = std::min(from.y, to.y);
  float a_maxy = std::max(from.y, to.y);

  float a_minz = std::min(from.z, to.z);
  float a_maxz = std::max(from.z, to.z);

  // Allocate some temp variables
  float c_minx, c_maxx, c_miny, c_maxy, c_minz, c_maxz;


  // If both are landed, then the plane's not going anywhere - and we're good
  //if (from.landed && to.landed)
    //return false;

  //Check if the action box violates any of the constraints that are in the constraints list
  for (auto const& c : constraints)
  {

    // Check if the range of the constraint overlaps in time
    if (max(c->start().t, from.t) <= min(c->end().t, to.t) + 0.00001)
    {
      // Generate a well formed set of boxes for the constraint box
      c_minx = c->start().x < c->end().x ? c->start().x : c->end().x;
      c_maxx = c->start().x > c->end().x ? c->start().x : c->end().x;

      c_miny = c->start().y < c->end().y ? c->start().y : c->end().y;
      c_maxy = c->start().y > c->end().y ? c->start().y : c->end().y;

      c_minz = c->start().z < c->end().z ? c->start().z : c->end().z;
      c_maxz = c->start().z > c->end().z ? c->start().z : c->end().z;


      if (std::max(c_minx, a_minx) <= std::min(c_maxx, a_maxx) && // Check if overlapping on the X axis
          std::max(c_miny, a_miny) <= std::min(c_maxy, a_maxy) && // Check if overlapping on the Y axis
          std::max(c_minz, a_minz) <= std::min(c_maxz, a_maxz)    // Check if overlapping on the Z axis
         )
      {
        // If we overlap on all three axis, then there must be a common point, and thus
        // we can return that the constraint was violated
        return max(c->start().t, from.t)-TOLERANCE;
      }

    }
  }


  // If landing, we don't want to check the static constraints - but we don't want planes taking off and landing
  // at the same time
  //if (from.landed || to.landed)
  //{
    //return false;
  //}
//Check if the action box violates any of the constraints that are in the static constraints list
  for (auto const& c : static_constraints)
  {
    // Check if the range of the constraint overlaps in time
    if (max(c->start().t, from.t) <= min(c->end().t, to.t) + 0.00001)
    {
      // Generate a well formed set of boxes for the constraint box
      c_minx = c->start().x < c->end().x ? c->start().x : c->end().x;
      c_maxx = c->start().x > c->end().x ? c->start().x : c->end().x;

      c_miny = c->start().y < c->end().y ? c->start().y : c->end().y;
      c_maxy = c->start().y > c->end().y ? c->start().y : c->end().y;

      c_minz = c->start().z < c->end().z ? c->start().z : c->end().z;
      c_maxz = c->start().z > c->end().z ? c->start().z : c->end().z;


      if (std::max(c_minx, a_minx) <= std::min(c_maxx, a_maxx) && // Check if overlapping on the X axis
          std::max(c_miny, a_miny) <= std::min(c_maxy, a_maxy) && // Check if overlapping on the Y axis
          std::max(c_minz, a_minz) <= std::min(c_maxz, a_maxz)    // Check if overlapping on the Z axis
         )
      {
        // If we overlap on all three axis, then there must be a common point, and thus
        // we can return that the constraint was violated
        return max(c->start().t, from.t)-TOLERANCE;
      }

    }
  }

  // If no constraint is violated, return false
  return 0;
}
