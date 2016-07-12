//
//  Airplane.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include "Airplane.h"
#include <iostream>
#include "TemplateAStar.h"
#include "Heuristic.h"

bool operator==(const airplaneState &s1, const airplaneState &s2)
{
    return (s1.x==s2.x && s1.y==s2.y && s1.height==s2.height && s1.heading == s2.heading && s1.speed == s2.speed);
    //return (fequal(s1.x,s2.x) && fequal(s1.y,s2.y) && s1.height==s2.height && s1.speed == s2.speed && s1.heading == s2.heading);
}

bool operator==(const airplaneAction &a1, const airplaneAction &a2)
{
  return a1.turn == a2.turn && a1.speed==a2.speed && a1.height==a2.height;
}

AirplaneEnvironment::AirplaneEnvironment(
  unsigned width,
  unsigned length,
  unsigned height,
  double climbRate,
  double minSpeed,
  double maxSpeed,
  uint8_t numSpeeds,
  double cruiseBurnRate,
  double speedBurnDelta,
  double climbCost,
  double descendCost,
  double gridSize,
  std::string const& perimeterFile
): width(width),
  length(length),
  height(height),
  climbRate(climbRate),
  minSpeed(minSpeed),
  maxSpeed(maxSpeed),
  numSpeeds(numSpeeds),
  gridSize(gridSize),
  cruiseBurnRate(cruiseBurnRate),
  speedBurnDelta(speedBurnDelta),
  climbCost(climbCost),
  descendCost(descendCost),
  perimeterLoaded(false),
  perimeterFile(perimeterFile)
{
    // Load the perimeter heuristic before we add any obstacles to the environment...
    srandom(time(0));
    ground.resize((width+1)*(length+1));
    groundNormals.resize((width+1)*(length+1));
    int value = random()%255;
    int offset = 5;
    int steps = 5;

    // initial strip
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
    // smooth
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
    // extend
    for (int y = 0; y < length; y++)
    {
        for (int x = 0; x <= width; x++)
        {
            SetGround(x, y, (255*tmp[x+y*(length+1)])/maxVal);
        }
    }
    
    
    // build normals
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

  airplaneState launchLoc(18, 29, 7, 1, 4);
  airplaneState landingLoc(18, 29, 7, 1, 0);
  airplaneState goalLoc(18, 23, 0, 0, 0);
  landingStrip l(15, 20, 17, 28, launchLoc, landingLoc, goalLoc);
  AddLandingStrip(l);
    
    //SetGround(x, y, (random()%60)-30+255*(sin(0.01*cos(x*y+y^2+3))+sin(0.04*sin(x+y))+2.0)/4.0);
    //SetGround(x, y, random()%255);
    
//  for (int y = 0; y <= length; y++)
//  {
//      for (int x = 0; x <= width; x++)
//      {
//          if (x < width && y < length)
//              SetGround(x, y, (GetGround(x, y)+GetGround(x+1, y)+GetGround(x, y+1))/3.0);
//          else if (x > 0 && y > 0)
//              SetGround(x, y, (GetGround(x, y)+GetGround(x-1, y)+GetGround(x, y-1))/3.0);
//          else if (x > 0)
//              SetGround(x, y, (GetGround(x, y)+GetGround(x-1, y))/2.0);
//          else if (y > 0)
//              SetGround(x, y, (GetGround(x, y)+GetGround(x, y-1))/2.0);
//      }
//  }

    // set 0,0  width,width  length,0  length,width
//  SetGround(0, 0, random()%256);
//  SetGround(width, 0, random()%256);
//  SetGround(0, length, random()%256);
//  SetGround(width, length, random()%256);
//  RecurseGround(0, 0, width+1, length+1);
}

void AirplaneEnvironment::loadPerimeterDB(){
  // Note, the ref state must be free and clear of obstacles within the perimeter
  airplaneState ref(40, 40, 10, 1, 1);
  perimeterLoaded = perimeter.loadGCosts(getRef(),ref,perimeterFile);
}

void AirplaneEnvironment::SetGround(int x, int y, uint8_t val)
{
    ground[x + y*(length+1)] = val;
}

uint8_t AirplaneEnvironment::GetGround(int x, int y) const
{
    return ground[x + y*(length+1)];
}

bool AirplaneEnvironment::Valid(int x, int y)
{
    return x >= 0 && x <= width && y >= 0 && y <= length;
}

recVec &AirplaneEnvironment::GetNormal(int x, int y)
{
    return groundNormals[x + y*(length+1)];
}

recVec AirplaneEnvironment::GetNormal(int x, int y) const
{
    return groundNormals[x + y*(length+1)];
}

void AirplaneEnvironment::RecurseGround(int x1, int y1, int x2, int y2)
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


void AirplaneEnvironment::GetSuccessors(const airplaneState &nodeID, std::vector<airplaneState> &neighbors) const
{
    GetActions(nodeID, internalActions);
    for (auto &act : internalActions)
    {
        airplaneState s;
        GetNextState(nodeID, act, s);
        neighbors.push_back(s);
    }
}

void AirplaneEnvironment::GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
    // 45, 90, 0, shift
    // speed:
    // faster, slower
    // height:
    // up / down
    actions.resize(0);

  // If the airplane is on the ground, the only option is to takeoff
  if (nodeID.landed)
  {
    // Figure out which landing strip we're at
    for (landingStrip st : landingStrips)
    {
      //std::cout << "Comparing " << nodeID << " and " << st.goal_state << std::endl;
      if (nodeID == st.goal_state)
      {
        // Add the takeoff action
        actions.push_back(airplaneAction(0,0,0,1));
        return;
      }
    }
    // There should never be a situation where we get here
    assert(false && "Airplane trying to takeoff, but not at a landing strip");
  } 
  else 
  {
    // Check to see if we can land
    for (landingStrip st : landingStrips)
    {
      if (nodeID == st.landing_state)
      {
        // Add the takeoff action
        actions.push_back(airplaneAction(0,0,0,2));
      }
    }
    // We don't have to land though, so we keep going.
  }

  // no change
    actions.push_back(airplaneAction(0, 0, 0));

    // each type of turn
    actions.push_back(airplaneAction(k45, 0, 0));
    actions.push_back(airplaneAction(-k45, 0, 0));
    actions.push_back(airplaneAction(k90, 0, 0));
    actions.push_back(airplaneAction(-k90, 0, 0));
    actions.push_back(airplaneAction(kShift, 0, 0));
    actions.push_back(airplaneAction(-kShift, 0, 0));

    if (nodeID.height > 1)
        {
          // decrease height
          actions.push_back(airplaneAction(0, 0, -1));
          actions.push_back(airplaneAction(k45, 0, -1));
          actions.push_back(airplaneAction(-k45, 0, -1));
          actions.push_back(airplaneAction(k90, 0, -1));
          actions.push_back(airplaneAction(-k90, 0, -1));
          actions.push_back(airplaneAction(kShift, 0, -1));
          actions.push_back(airplaneAction(-kShift, 0, -1));

          // decrease height, decrease speed
          if (nodeID.speed > minSpeed)
          {
            actions.push_back(airplaneAction(0, -1, -1));
            actions.push_back(airplaneAction(k45, -1, -1));
            actions.push_back(airplaneAction(-k45, -1, -1));
            actions.push_back(airplaneAction(k90, -1, -1));
            actions.push_back(airplaneAction(-k90, -1, -1));
            actions.push_back(airplaneAction(kShift, -1, -1));
            actions.push_back(airplaneAction(-kShift, -1, -1));
          }

          // increase height, decrease speed
          if (nodeID.speed < numSpeeds+minSpeed)
          {
            actions.push_back(airplaneAction(0, +1, -1));
            actions.push_back(airplaneAction(k45, +1, -1));
            actions.push_back(airplaneAction(-k45, +1, -1));
            actions.push_back(airplaneAction(k90, +1, -1));
            actions.push_back(airplaneAction(-k90, +1, -1));
            actions.push_back(airplaneAction(kShift, +1, -1));
            actions.push_back(airplaneAction(-kShift, +1, -1));
          }
        }

    if (nodeID.height < height)
        {
          // increase height
          actions.push_back(airplaneAction(0, 0, +1));
          actions.push_back(airplaneAction(k45, 0, +1));
          actions.push_back(airplaneAction(-k45, 0, +1));
          actions.push_back(airplaneAction(k90, 0, +1));
          actions.push_back(airplaneAction(-k90, 0, +1));
          actions.push_back(airplaneAction(kShift, 0, +1));
          actions.push_back(airplaneAction(-kShift, 0, +1));

          if (nodeID.speed > minSpeed)
          {
            // increase height, decrease speed
            actions.push_back(airplaneAction(0, -1, +1));
            actions.push_back(airplaneAction(k45, -1, +1));
            actions.push_back(airplaneAction(-k45, -1, +1));
            actions.push_back(airplaneAction(k90, -1, +1));
            actions.push_back(airplaneAction(-k90, -1, +1));
            actions.push_back(airplaneAction(kShift, -1, +1));
            actions.push_back(airplaneAction(-kShift, -1, +1));
          }

          if (nodeID.speed < numSpeeds+minSpeed)
          {
            // increase height, increase speed
            actions.push_back(airplaneAction(0, +1, +1));
            actions.push_back(airplaneAction(k45, +1, +1));
            actions.push_back(airplaneAction(-k45, +1, +1));
            actions.push_back(airplaneAction(k90, +1, +1));
            actions.push_back(airplaneAction(-k90, +1, +1));
            actions.push_back(airplaneAction(kShift, +1, +1));
            actions.push_back(airplaneAction(-kShift, +1, +1));
          }
        }
    
    if (nodeID.speed > minSpeed)
        {
                // decrease speed
        actions.push_back(airplaneAction(0, -1, 0));
                actions.push_back(airplaneAction(k45, -1, 0));
                actions.push_back(airplaneAction(-k45, -1, 0));
                actions.push_back(airplaneAction(k90, -1, 0));
                actions.push_back(airplaneAction(-k90, -1, 0));
                actions.push_back(airplaneAction(kShift, -1, 0));
                actions.push_back(airplaneAction(-kShift, -1, 0));
        }

    if (nodeID.speed < numSpeeds+minSpeed)
        {
                // increase speed
        actions.push_back(airplaneAction(0, +1, 0));
                actions.push_back(airplaneAction(k45, +1, 0));
                actions.push_back(airplaneAction(-k45, +1, 0));
                actions.push_back(airplaneAction(k90, +1, 0));
                actions.push_back(airplaneAction(-k90, +1, 0));
                actions.push_back(airplaneAction(kShift, +1, 0));
                actions.push_back(airplaneAction(-kShift, +1, 0));
        }
    
}

void AirplaneEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
    // 45, 90, 0, shift
    // speed:
    // faster, slower
    // height:
    // up / down
    actions.resize(0);

  // no change
    actions.push_back(airplaneAction(0, 0, 0));

    // each type of turn
    actions.push_back(airplaneAction(k45, 0, 0));
    actions.push_back(airplaneAction(-k45, 0, 0));
    actions.push_back(airplaneAction(k90, 0, 0));
    actions.push_back(airplaneAction(-k90, 0, 0));
    actions.push_back(airplaneAction(kShift, 0, 0));
    actions.push_back(airplaneAction(-kShift, 0, 0));

    if (nodeID.height < height)
        {
          // decrease height
          actions.push_back(airplaneAction(0, 0, -1));
          actions.push_back(airplaneAction(k45, 0, -1));
          actions.push_back(airplaneAction(-k45, 0, -1));
          actions.push_back(airplaneAction(k90, 0, -1));
          actions.push_back(airplaneAction(-k90, 0, -1));
          actions.push_back(airplaneAction(kShift, 0, -1));
          actions.push_back(airplaneAction(-kShift, 0, -1));

          // decrease height, decrease speed
          if (nodeID.speed > minSpeed)
          {
            actions.push_back(airplaneAction(0, +1, -1));
            actions.push_back(airplaneAction(k45, +1, -1));
            actions.push_back(airplaneAction(-k45, +1, -1));
            actions.push_back(airplaneAction(k90, +1, -1));
            actions.push_back(airplaneAction(-k90, +1, -1));
            actions.push_back(airplaneAction(kShift, +1, -1));
            actions.push_back(airplaneAction(-kShift, +1, -1));
          }

          // increase height, decrease speed
          if (nodeID.speed < numSpeeds+minSpeed)
          {
            actions.push_back(airplaneAction(0, -1, -1));
            actions.push_back(airplaneAction(k45, -1, -1));
            actions.push_back(airplaneAction(-k45, -1, -1));
            actions.push_back(airplaneAction(k90, -1, -1));
            actions.push_back(airplaneAction(-k90, -1, -1));
            actions.push_back(airplaneAction(kShift, -1, -1));
            actions.push_back(airplaneAction(-kShift, -1, -1));
          }
        }

    if (nodeID.height > 0)
        {
          // increase height
          actions.push_back(airplaneAction(0, 0, +1));
          actions.push_back(airplaneAction(k45, 0, +1));
          actions.push_back(airplaneAction(-k45, 0, +1));
          actions.push_back(airplaneAction(k90, 0, +1));
          actions.push_back(airplaneAction(-k90, 0, +1));
          actions.push_back(airplaneAction(kShift, 0, +1));
          actions.push_back(airplaneAction(-kShift, 0, +1));

          if (nodeID.speed > minSpeed)
          {
            // increase height, decrease speed
            actions.push_back(airplaneAction(0, +1, +1));
            actions.push_back(airplaneAction(k45, +1, +1));
            actions.push_back(airplaneAction(-k45, +1, +1));
            actions.push_back(airplaneAction(k90, +1, +1));
            actions.push_back(airplaneAction(-k90, +1, +1));
            actions.push_back(airplaneAction(kShift, +1, +1));
            actions.push_back(airplaneAction(-kShift, +1, +1));
          }

          if (nodeID.speed < numSpeeds+minSpeed)
          {
            // increase height, increase speed
            actions.push_back(airplaneAction(0, -1, +1));
            actions.push_back(airplaneAction(k45, -1, +1));
            actions.push_back(airplaneAction(-k45, -1, +1));
            actions.push_back(airplaneAction(k90, -1, +1));
            actions.push_back(airplaneAction(-k90, -1, +1));
            actions.push_back(airplaneAction(kShift, -1, +1));
            actions.push_back(airplaneAction(-kShift, -1, +1));
          }
        }
    
    if (nodeID.speed > minSpeed)
        {
                // decrease speed
                actions.push_back(airplaneAction(0, +1, 0));
                actions.push_back(airplaneAction(k45, +1, 0));
                actions.push_back(airplaneAction(-k45, +1, 0));
                actions.push_back(airplaneAction(k90, +1, 0));
                actions.push_back(airplaneAction(-k90, +1, 0));
                actions.push_back(airplaneAction(kShift, +1, 0));
                actions.push_back(airplaneAction(-kShift, +1, 0));
        }

    if (nodeID.speed < numSpeeds+minSpeed)
        {
                // increase speed
                actions.push_back(airplaneAction(0, -1, 0));
                actions.push_back(airplaneAction(k45, -1, 0));
                actions.push_back(airplaneAction(-k45, -1, 0));
                actions.push_back(airplaneAction(k90, -1, 0));
                actions.push_back(airplaneAction(-k90, -1, 0));
                actions.push_back(airplaneAction(kShift, -1, 0));
                actions.push_back(airplaneAction(-kShift, -1, 0));
        }
    
}

/** Gets the action required to go from node1 to node2 */
airplaneAction AirplaneEnvironment::GetAction(const airplaneState &node1, const airplaneState &node2) const
{
    airplaneAction a;

  // Deal with actions that setup landing
  if (node1.landed && !node2.landed){
    return(airplaneAction(0,0,0,1));
  } else if (node2.landed && !node1.landed) {
    return(airplaneAction(0,0,0,2));
  }

    a.height = node2.height - node1.height;
        a.turn = node2.heading - node1.heading;
        if(a.turn>2){
          if(a.turn == 6) {a.turn = -2;}
          if(a.turn == 7) {a.turn = -1;}
          if(a.turn == -6) {a.turn = 2;}
          if(a.turn == -7) {a.turn = 1;}
        }
        
        // Detect a kshift
        if (node1.heading == node2.heading)
        {
          if(node1.heading%2 == 0 && !fequal(node1.x, node2.x) && !fequal(node1.y, node2.y)){
            a.turn = kShift;
            switch(node1.heading){
              case 4:
                if(node1.x<node2.x) a.turn *=-1;
              break;
              case 2:
                if(node1.y>node2.y) a.turn *=-1;
              break;
              case 0:
                if(node1.x>node2.x) a.turn *=-1;
              break;
              case 6:
                if(node1.y<node2.y) a.turn *=-1;
              break;
            }
          } else if(node1.heading%2 == 1 && 
              ((!fequal(node1.x, node2.x) && fequal(node1.y, node2.y)) || 
               (fequal(node1.x, node2.x) && !fequal(node1.y, node2.y)))){
            a.turn = kShift;
            switch(node1.heading){
              case 3:
                if(node1.x<node2.x) a.turn *=-1;
              break;
              case 1:
                if(node1.y<node2.y) a.turn *=-1;
              break;
              case 7:
                if(node1.x>node2.x) a.turn *=-1;
              break;
              case 5:
                if(node1.y>node2.y) a.turn *=-1;
              break;
            }
          }
        }
        a.speed = node2.speed-node1.speed; // As of right now
        return a;
}


// Note action application does not account for speed
// Also, turn is performed, and then the offset is applied
void AirplaneEnvironment::ApplyAction(airplaneState &s, airplaneAction dir) const
{

  if (dir.takeoff == 1) {
    // Takeoff action
    // Find the airstrip that we're at
    for (landingStrip st : landingStrips)
    {
      
      if (s == st.goal_state)
      {
        s = st.launch_state;
        s.landed = false;
        return;
      }
    }
    assert (false && "Tried to takeoff from a non-existant landing strip");
  }

  if (dir.takeoff == 2) {
    // Landing action
    // Find the airstrip that we're at
    for (landingStrip st : landingStrips)
    {
      if (s == st.landing_state)
      {
        s = st.goal_state;
        s.landed = true;
        return;
      }
    }
    assert (false && "Tried to land at a non-existant landing strip");
  }



    static const double offset[8][2] = {
        { 0, -1},
        { 1, -1},
        { 1,  0},
        { 1,  1},
        { 0,  1},
        {-1,  1},
        {-1,  0},
        {-1, -1}
  };
  

  //std::cout << "Apply turn: " << signed(dir.turn) << ", height: " << signed(dir.height) << ", speed: " << signed(dir.speed) << " to " << s << "\n";
  uint8_t heading(s.heading);
  if(dir.turn == kShift) {heading = (s.heading+8+k45)%8;}
  else if(dir.turn == -kShift) {heading = (s.heading+8-k45)%8;}
  else { heading = s.heading = (s.heading+8+dir.turn)%8;}
  s.speed += dir.speed;
  s.x += offset[heading][0];
  s.y += offset[heading][1];
  // Note: speed represents ground speed (2D speed) not 3D speed
  s.height += dir.height;
  //std::cout << "Moved to " << s << "\n";
}

void AirplaneEnvironment::UndoAction(airplaneState &s, airplaneAction dir) const
{
    static const double offset[8][2] = {
        { 0, -1},
        { 1, -1},
        { 1,  0},
        { 1,  1},
        { 0,  1},
        {-1,  1},
        {-1,  0},
        {-1, -1} };
  
  s.height -= dir.height;
  s.speed -= dir.speed;

  uint8_t heading(s.heading);
  if(dir.turn == kShift) {heading = (s.heading+8+k45+4)%8;}
  else if(dir.turn == -kShift) {heading = (s.heading+8-k45+4)%8;}
  else { heading = (s.heading+4)%8;
    s.heading = (s.heading+8-dir.turn)%8;
  }
  s.x += offset[heading][0];
  s.y += offset[heading][1];
}

void AirplaneEnvironment::GetNextState(const airplaneState &currents, airplaneAction dir, airplaneState &news) const
{
    news = currents;
    ApplyAction(news, dir);
}

double AirplaneEnvironment::myHCost(const airplaneState &node1, const airplaneState &node2) const
{
  // Estimate fuel cost...
  static const int cruise(3);
  int diffx(abs(node1.x-node2.x));
  int diffy(abs(node1.y-node2.y));
  int diff(abs(diffx-diffy));
  int diag(abs((diffx+diffy)-diff)/2);
  int vertDiff(node2.height-node1.height);
  int speedDiff1(std::max(0,abs(cruise-node1.speed)-1));
  int speedDiff2(abs(cruise-node2.speed));
  int speedDiff(abs(node2.speed-node1.speed));
  double vcost(vertDiff>0?climbCost:descendCost);
  vertDiff=abs(vertDiff);
  int maxMove(std::max(std::max(speedDiff,speedDiff1+speedDiff2),vertDiff));
  int travel(node1.headingTo(node2));
  //int numTurns1(hdgDiff<8>(node1.heading,travel));
  //int numTurns2(hdgDiff<8>(node2.heading,travel));

  // Change as many diagonal moves into horizontal as we can
  while(maxMove>diff+diag && diag>0){
    diag--;
    diff+=2;
  }

  int speedChanges=std::max(0,(diff>=speedDiff1+speedDiff2)?(speedDiff1+speedDiff2):(speedDiff));

  
  //std::cout << "speed:"<<speedChanges<<"v:"<<vertDiff<<"\n";
  double total(diff*cruiseBurnRate+std::min(speedChanges,diff)*speedBurnDelta+std::min(vertDiff,diff)*vcost);
  speedChanges-=std::min(speedChanges,diff);
  vertDiff-=std::min(vertDiff,diff);
  return total+(diag*cruiseBurnRate+speedChanges*speedBurnDelta+vertDiff*vcost)*M_SQRT2;
}

double AirplaneEnvironment::HCost(const airplaneState &node1, const airplaneState &node2) const
{
  // We want to estimate the heuristic to the landing state
  // Figure out which landing strip we're going to
  for (landingStrip st : landingStrips)
  {
    if (node2 == st.goal_state)
    {
      return HCost(node1, st.landing_state);
    } else if (node1 == st.goal_state)
    {
      return HCost(st.landing_state, node2);
    }
  }
  
  airplaneState pNode = node2;
  double perimeterVal(0);
  if(perimeterLoaded)
  {
    if(abs(node1.x-node2.x)<=2 && abs(node1.y-node2.y)<=2 && abs(node2.height-node1.height) <=2)
      return perimeter.GCost(node1,node2);
    // Select a perimeter node
    double best(9999999);
    static const int perimeterSize(2);
    for(int x(std::max(0,node2.x-perimeterSize)); x<std::min(node2.x+perimeterSize,width); ++x){
      for(int y(std::max(0,node2.y-perimeterSize)); y<std::min(node2.y+perimeterSize,length); ++y){
        double dist( sqrt((x-node1.x)*(x-node1.x)+(y-node1.y)*(y-node1.y)));
        if(dist<best){
          pNode.x = x;
          pNode.y = y;
        }
      }
    }
    perimeterVal = perimeter.GCost(pNode,node2);
  }
  return myHCost(node1,pNode)+ perimeterVal;
}



double AirplaneEnvironment::GCost(const airplaneState &node1, const airplaneState &node2) const
{
    // Compute cost according to fuel consumption
    int vertDiff(node2.height-node1.height);
    double vcost(vertDiff>0?climbCost:descendCost);
    vertDiff=abs(vertDiff);
    double cost(cruiseBurnRate+speedBurnDelta*abs(((numSpeeds+1)/2.0)-(node2.speed))+vertDiff*vcost);
    return cost*(((node1.x-node2.x) && (node1.y-node2.y))?M_SQRT2:1.0);
}

double AirplaneEnvironment::GCost(const airplaneState &node1, const airplaneAction &act) const
{
    airplaneState node2(node1);
    ApplyAction(node2,act);
    return GCost(node1,node2);
}


bool AirplaneEnvironment::GoalTest(const airplaneState &node, const airplaneState &goal) const
{
    return (node.x==goal.x && node.y==goal.y && node.height == goal.height && node.heading == goal.heading && node.speed == goal.speed);
    //return (fequal(node.x,goal.x) && fequal(node.y, goal.y) && node.height == goal.height && node.heading == goal.heading); //&& node.speed == goal.speed
}

double AirplaneEnvironment::GetPathLength(const std::vector<airplaneState> &sol) const
{
    double gcost(0.0);
    if(sol.size()>1)
      for(auto n(sol.begin()+1); n!=sol.end(); ++n)
        gcost += GCost(*(n-1),*n);
    return gcost;
}


uint64_t AirplaneEnvironment::GetStateHash(const airplaneState &node) const
{
    uint64_t h = 0;
    
    // Assume x,y discretization of 3 meters
    h |= node.x;
    h = h << 16;
    h |= node.y;
    h = h << 10;

    // Assume height discretization of 25 meters
    h |= node.height & (0x400-1); // 10 bits
    h = h << 5;

    // Speed increments are in 1 m/sec
    h |= node.speed & (0x20-1); // 4 bits
    h = h << 3;

    // Heading increments are in 45 degrees
    h |= node.heading & (0x8-1); // 3 bits
  
    h = h << 1;
    h |= node.landed;

    return h;
}

uint64_t AirplaneEnvironment::GetActionHash(airplaneAction act) const
{

    uint64_t h = 0;
    h |= act.turn;
    h = h << 8;
    h |= act.speed;
    h = h << 8;
    h |= act.height;
    h = h << 8;
    h |= act.takeoff;
    return h;
}

recVec AirplaneEnvironment::GetCoordinate(int x, int y, int z) const
{
    return {(x-width/2.0)/(width/2.0), (y-width/2.0)/(width/2.0), -4.0*z/(255.0*80)};
}

void AirplaneEnvironment::OpenGLDraw() const
{
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

      for (landingStrip st : landingStrips) 
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


      for (landingStrip st : landingStrips) 
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

  for (landingStrip st : landingStrips)
  {
    //Draw the dashed line down the middle
    
    float xval = (max(st.x1, st.x2)-min(st.x1, st.x2))/2.0f + min(st.x1, st.x2);
  
    glLineStipple(1, 0x00FF);
    glLineWidth(100);
    glEnable(GL_LINE_STIPPLE);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
      glVertex3f((xval-width/2.0)/(width/2.0),(min(st.y1,st.y2)-width/2.0)/(width/2.0) ,-4.0*st.z/(255.0*80));
      glVertex3f((xval-width/2.0)/(width/2.0),(max(st.y1,st.y2)-width/2.0)/(width/2.0) ,-4.0*st.z/(255.0*80));
    glEnd();
    glLineStipple(0, 0xFFFF);
    glLineWidth(1);

    // Draw the launch location
    this->SetColor(0.5,0.5,0.5, 0.3);
    this->OpenGLDraw(st.launch_state);

  }



}

void AirplaneEnvironment::OpenGLDraw(const airplaneState &l) const
{
    {
        GLfloat r, g, b, t;
        GetColor(r, g, b, t);
        glColor3f(r, g, b);
    }
    // x & y range from 20*4 = 0 to 80 = -1 to +1
    // z ranges from 0 to 20 which is 0...
    GLfloat x = (l.x-40.0)/40.0;
    GLfloat y = (l.y-40.0)/40.0;
    GLfloat z = -l.height/80.0;
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(360*l.heading/8.0, 0, 0, 1);
    DrawAirplane();
    glPopMatrix();
    
    //DrawCylinder(l.x, l.y, l.height, 0, 0.001, 0.01);
}

void AirplaneEnvironment::OpenGLDraw(const airplaneState& o, const airplaneState &n, float perc) const
{
    {
        GLfloat r, g, b, t;
        GetColor(r, g, b, t);
        glColor3f(r, g, b);
    }
    
    GLfloat x1 = (o.x-40.0)/40.0;
    GLfloat y1 = (o.y-40.0)/40.0;
    GLfloat z1 = -o.height/80.0;
    GLfloat h1 = 360*o.heading/8.0;

    GLfloat x2 = (n.x-40.0)/40.0;
    GLfloat y2 = (n.y-40.0)/40.0;
    GLfloat z2 = -n.height/80.0;
    GLfloat h2 = 360*n.heading/8.0;
    if (o.heading < 2 && n.heading >= 6)
        h2 -= 360;
    if (o.heading >= 6 && n.heading < 2)
        h1 -= 360;
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef((1-perc)*x1+perc*x2, (1-perc)*y1+perc*y2, (1-perc)*z1+perc*z2);
    glRotatef((1-perc)*h1+perc*h2, 0, 0, 1);
        if(n.height>o.height) glRotatef(-45, 1, 1, 0);
        else if(n.height<o.height) glRotatef(45, 1, 1, 0);
    DrawAirplane();
    glPopMatrix();
}

void AirplaneEnvironment::OpenGLDraw(const airplaneState &, const airplaneAction &) const
{
    //TODO: Implement this
}

void AirplaneEnvironment::DrawAirplane() const
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
  glVertex3f(0,0.000,0);
  glVertex3f(0,0.005,0);
  glVertex3f(0,0.005,-0.004);
  glEnd();

}

void AirplaneEnvironment::GLDrawLine(const airplaneState &a, const airplaneState &b) const
{
    glColor4f(1.0, 1.0, 1.0, .5); // Make it partially opaque gray

    // Normalize coordinates between (-1, 1)
    GLfloat x_start((a.x-40.0)/40.0);
    GLfloat y_start((a.y-40.0)/40.0);
    GLfloat z_start(-a.height/80.0);

    GLfloat x_end((b.x-40.0)/40.0);
    GLfloat y_end((b.y-40.0)/40.0);
    GLfloat z_end(-b.height/80.0);

    glDisable(GL_LIGHTING);
    glPushMatrix();

    // Draw the edge line segments
    glBegin(GL_LINES);

    glVertex3f(x_start,y_start,z_start);
    glVertex3f(x_end,y_end,z_end);

    glEnd();

    glPopMatrix();
}

void AirplaneEnvironment::GLDrawPath(const std::vector<airplaneState> &p) const
{
        if(p.size()<2) return;
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}


std::vector<uint8_t> AirplaneEnvironment::getGround() 
{
    return std::vector<uint8_t>(ground);
}
std::vector<recVec> AirplaneEnvironment::getGroundNormals()
{
    return std::vector<recVec>(groundNormals);
}
std::vector<airplaneAction> AirplaneEnvironment::getInternalActions()
{
    return std::vector<airplaneAction>(internalActions);
}

void AirplaneEnvironment::AddLandingStrip(landingStrip strip)
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
      SetGround(i, j, (int) avgh);
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
  strip.z = (uint16_t) avgh;
  this->landingStrips.push_back(strip);

}
