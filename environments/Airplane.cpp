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
    return (s1.x==s2.x && s1.y==s2.y && s1.height==s2.height && s1.heading == s2.heading && s1.speed == s2.speed && s1.landed == s2.landed);
    //return (fequal(s1.x,s2.x) && fequal(s1.y,s2.y) && s1.height==s2.height && s1.speed == s2.speed && s1.heading == s2.heading);
}

bool operator==(const airplaneAction &a1, const airplaneAction &a2)
{
  return a1.turn == a2.turn && a1.speed==a2.speed && a1.height==a2.height && a1.takeoff == a2.takeoff;
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
  airplaneState launchLoc(18, 29, 7, 1, 4, false);
  airplaneState landingLoc(18, 29, 7, 1, 0, false);
  airplaneState goalLoc(18, 23, 0, 0, 0, true);
  landingStrip l(15, 20, 17, 28, launchLoc, landingLoc, goalLoc);
  AddLandingStrip(l);

  // Setup turns vetor
  turns.push_back(0);
  turns.push_back(k45);
  turns.push_back(-k45);
  turns.push_back(k90);
  turns.push_back(-k90);
  turns.push_back(kShift);
  turns.push_back(-kShift);

  // Quad Turns vector
  quad_turns.push_back(0);
  quad_turns.push_back(k45);
  quad_turns.push_back(-k45);
  quad_turns.push_back(k90);
  quad_turns.push_back(-k90);
  quad_turns.push_back(k135);
  quad_turns.push_back(-k135);
  quad_turns.push_back(k180);
  quad_turns.push_back(kWait);
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
  actions.resize(0);
  switch(nodeID.type) {
    case AirplaneType::QUAD:
      if (AppendLandingActionsQuad(nodeID, actions))
        GetActionsQuad(nodeID, actions);
      break;
    case AirplaneType::PLANE:
      if (AppendLandingActionsPlane(nodeID, actions))
        GetActionsPlane(nodeID, actions);
      break;
    default: break;
  }
}

void AirplaneEnvironment::GetActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
  // Do Nothing 
 // std::cout << "Getting quadcopter actions from state " << nodeID << ": " << std::endl;

  // Push back all quad_turns at current speed and height (including no turn)
  for (auto x : quad_turns) if (x != kWait || nodeID.speed == minSpeed) actions.push_back(airplaneAction(x, 0, 0));

  // Increase speed at current height
  if (nodeID.speed < numSpeeds + minSpeed) for (auto x : quad_turns) if (x != kWait) actions.push_back(airplaneAction(x, 1, 0));
  // Decrease speed at current height
  if (nodeID.speed > minSpeed) for (auto x : quad_turns) if (x != kWait) actions.push_back(airplaneAction(x, -1, 0));

  // Check for decreasing height
  if (nodeID.height > 1) {
    // Decrease height
    for (auto x : quad_turns) actions.push_back(airplaneAction(x, 0, -1));
    // Decrease height and decrease speed
    if (nodeID.speed > minSpeed) for (auto x : quad_turns) actions.push_back(airplaneAction(x, -1, -1));
    // Decrease height increase speed
    if (nodeID.speed < numSpeeds+minSpeed) for (auto x : quad_turns) actions.push_back(airplaneAction(x, 1, -1));
  }

  // Check for increasing height
  if (nodeID.height < height) {
    // Increase height
    for (auto x : quad_turns) actions.push_back(airplaneAction(x, 0, 1));
    // Increase height and decrease speed
    if (nodeID.speed > minSpeed) for (auto x : quad_turns) actions.push_back(airplaneAction(x, -1, 1));
    // Increase height increase speed
    if (nodeID.speed < numSpeeds+minSpeed) for (auto x : quad_turns) actions.push_back(airplaneAction(x, 1, 1));
  }

  //for (auto x : actions) {
   // std::cout << "\t" << x << std::endl;
 // }
}

void AirplaneEnvironment::GetActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
    // Push back all turns at current speed and height (including no turn)
    for (auto x : turns) 
      actions.push_back(airplaneAction(x, 0, 0));

    // Increase speed at current height
    if (nodeID.speed < numSpeeds + minSpeed)
      for (auto x : turns) actions.push_back(airplaneAction(x, 1, 0));
    // Decrease speed at current height
    if (nodeID.speed > minSpeed) 
      for (auto x : turns) actions.push_back(airplaneAction(x, -1, 0));

    // Check for decreasing height
    if (nodeID.height > 1) {
      // Decrease height
      for (auto x : turns) actions.push_back(airplaneAction(x, 0, -1));
      // Decrease height and speed
      if (nodeID.speed > minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, -1, -1));
      // Decrease height decrease speed
      if (nodeID.speed < numSpeeds+minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, 1, -1));
    }

    // Check for increasing height
    if (nodeID.height < height) {
      // Decrease height
      for (auto x : turns) actions.push_back(airplaneAction(x, 0, 1));
      // Decrease height and speed
      if (nodeID.speed > minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, -1, 1));
      // Decrease height decrease speed
      if (nodeID.speed < numSpeeds+minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, 1, 1));
    }
}

bool AirplaneEnvironment::AppendLandingActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
  if (nodeID.landed)
  {
    // Figure out which landing strip we're at
    for (landingStrip st : landingStrips)
    {
      //std::cout << "Comparing " << nodeID << " and " << st.goal_state << std::endl;
      if (nodeID.x == st.goal_state.x && nodeID.y == st.goal_state.y && nodeID.height == st.goal_state.height &&
          nodeID.heading == st.goal_state.heading && nodeID.speed == st.goal_state.speed)
      {
        // Add the takeoff action
        actions.push_back(airplaneAction(0,0,0,1));
        // Add the landed no-op action
        actions.push_back(airplaneAction(0,0,0,3));
        return false;
      }
    }
    // There should never be a situation where we get here
    std::cout << nodeID << std::endl;
    assert(false && "Airplane trying to takeoff, but not at a landing strip");
  } 
  else 
  {
    // Check to see if we can land
    for (landingStrip st : landingStrips)
    {
      if (nodeID.x == st.landing_state.x && nodeID.y == st.landing_state.y && nodeID.height == st.landing_state.height &&
          nodeID.heading == st.landing_state.heading && nodeID.speed == st.landing_state.speed)
      {
        // Add the landing action
        actions.push_back(airplaneAction(0,0,0,2));
      }
    }
    return true;
  }
}

bool AirplaneEnvironment::AppendLandingActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const {
    // Do Nothing
    return true;
}



void AirplaneEnvironment::GetReverseActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const
{
  actions.resize(0);
  switch(nodeID.type) {
    case AirplaneType::QUAD:
      GetReverseActionsQuad(nodeID, actions);
      break;
    case AirplaneType::PLANE:
      GetReverseActionsPlane(nodeID, actions);
      break;
    default: break;
  }
}


void AirplaneEnvironment::GetReverseActionsPlane(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const 
{
    for (auto x : turns) 
      actions.push_back(airplaneAction(x, 0, 0));

    if (nodeID.speed <= numSpeeds + minSpeed)
      for (auto x : turns) actions.push_back(airplaneAction(x, -1, 0));
    if (nodeID.speed >= minSpeed) 
      for (auto x : turns) actions.push_back(airplaneAction(x, 1, 0));

    if (nodeID.height >= 1) {
      for (auto x : turns) actions.push_back(airplaneAction(x, 0, 1));
      if (nodeID.speed >= minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, 1, 1));
      if (nodeID.speed <= numSpeeds+minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, -1, 1));
    }

    if (nodeID.height <= height) {
      for (auto x : turns) actions.push_back(airplaneAction(x, 0, -1));
      if (nodeID.speed >= minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, 1, -1));
      if (nodeID.speed <= numSpeeds+minSpeed) 
        for (auto x : turns) actions.push_back(airplaneAction(x, -1, -1));
    }
}

void AirplaneEnvironment::GetReverseActionsQuad(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const 
{
  // Push back all quad_turns at current speed and height (including no turn)
  for (auto x : quad_turns) if (x != kWait || nodeID.speed == minSpeed) actions.push_back(airplaneAction(x, 0, 0));

  if (nodeID.speed <= numSpeeds + minSpeed)
    for (auto x : quad_turns) if (x != kWait) actions.push_back(airplaneAction(x, -1, 0));
  if (nodeID.speed >= minSpeed) 
    for (auto x : quad_turns) if (x != kWait) actions.push_back(airplaneAction(x, 1, 0));

  if (nodeID.height >= 1) {
    for (auto x : quad_turns) actions.push_back(airplaneAction(x, 0, 1));
    if (nodeID.speed >= minSpeed) 
      for (auto x : quad_turns) actions.push_back(airplaneAction(x, 1, 1));
    if (nodeID.speed <= numSpeeds+minSpeed) 
      for (auto x : quad_turns) actions.push_back(airplaneAction(x, -1, 1));
  }

  if (nodeID.height <= height) {
    for (auto x : quad_turns) actions.push_back(airplaneAction(x, 0, -1));
    if (nodeID.speed >= minSpeed) 
      for (auto x : quad_turns) actions.push_back(airplaneAction(x, 1, -1));
    if (nodeID.speed <= numSpeeds+minSpeed) 
      for (auto x : quad_turns) actions.push_back(airplaneAction(x, -1, -1));
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
  } else if (node1.landed && node2.landed) {
    return(airplaneAction(0,0,0,3));
  }

  a.height = node2.height - node1.height;
  a.turn = node2.heading - node1.heading;
  a.speed = node2.speed-node1.speed;
  
  if(a.turn>2){
    if(a.turn == 6) {a.turn = -2;}
    if(a.turn == 7) {a.turn = -1;}
    if(a.turn == -6) {a.turn = 2;}
    if(a.turn == -7) {a.turn = 1;}
  }

  if (node1.type == AirplaneType::QUAD) {
    // Detect a no-move action
    if (node1.x == node2.x && node1.y == node2.y) {
      a.turn = kWait;
    }
  }

  if (node1.type == AirplaneType::PLANE) {
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
  }      
  return a;
}

void AirplaneEnvironment::ApplyAction(airplaneState &s, airplaneAction dir) const
{
  // Manage Landing
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
    assert (!"Tried to takeoff from a non-existant landing strip");
  } else if (dir.takeoff == 2) {
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
    assert (!"Tried to land at a non-existant landing strip");
  } else if (dir.takeoff == 3) {
    // Landed no-op action
    // Do nothing if there's no action - as the state remains the same.
    if (!s.landed)
      assert(!"Tried to do nothing while not landed");
    return;
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
  
  int8_t heading(s.heading);

  if (dir.turn == kShift) {
    heading = (s.heading + kCircleSize + k45) % kCircleSize;
  } else if (dir.turn == -kShift) {
    heading = (s.heading + kCircleSize - k45) % kCircleSize;
  } else {
    heading = s.heading = (s.heading + kCircleSize + dir.turn) % kCircleSize;
  }

  if (dir.turn != kWait) {
    s.x += offset[heading][0];
    s.y += offset[heading][1];
  }
  
  s.speed += dir.speed;
  s.height += dir.height;
}

void AirplaneEnvironment::UndoAction(airplaneState &s, airplaneAction dir) const
{

    // Manage Landing
    if (dir.takeoff == 1) {
    // Takeoff action
    // Find the airstrip that we're at
    for (landingStrip st : landingStrips)
    { 
      if (s == st.launch_state)
      {
        s = st.goal_state;
        s.landed = true;
        return;
      }
    }
    assert (!"Tried to UNDO takeoff from a non-existant landing strip");
  } else if (dir.takeoff == 2) {
    // Landing action
    // Find the airstrip that we're at
    for (landingStrip st : landingStrips)
    {
      if (s == st.goal_state)
      {
        s = st.landing_state;
        s.landed = false;
        return;
      }
    }
    assert (!"Tried to UNDO land at a non-existant landing strip");
  } else if (dir.takeoff == 3) {
    // Landed no-op action
    // Do nothing if there's no action - as the state remains the same.
    if (!s.landed)
      assert(!"Tried to UNDO do nothing while not landed");
    return;
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
  
  s.height -= dir.height;
  s.speed -= dir.speed;

  int8_t heading(s.heading);

  if (dir.turn == kShift) {
    heading = (s.heading + kCircleSize + k45 + kCircleSize/2 ) % kCircleSize;
  } else if (dir.turn == -kShift) {
    heading = (s.heading + kCircleSize - k45 + kCircleSize/2 ) % kCircleSize;
  } else {
    heading = (s.heading + kCircleSize/2) % kCircleSize;
    s.heading = (s.heading + kCircleSize - dir.turn) % kCircleSize;
  }

  if (dir.turn != kWait) {
     s.x += offset[heading][0];
     s.y += offset[heading][1];
  }
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

  // Estimate cost of doing nothing
  double noop_cost = (node1.landed && node2.landed) ? 1.0 : 0.0;
  total += noop_cost;

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

    // Estimate cost of doing nothing
  double noop_cost = (node1.landed && node2.landed) ? 1.0 : 0.0;
  cost += noop_cost;


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
    if (l.landed)
      return;
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
    if (l.type == AirplaneType::QUAD) DrawQuadCopter();
    else if (l.type == AirplaneType::PLANE) DrawAirplane();
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

    if (o.type == AirplaneType::QUAD) DrawQuadCopter();
    else if (o.type == AirplaneType::PLANE) DrawAirplane();
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

void AirplaneEnvironment::DrawQuadCopter() const 
{
   // Body
  glRotatef(90,1,1,0);
  DrawCylinder(0, 0, 0, 0, 0.01/2.0, 0.01/10);
  glRotatef(-90,1,1,0);
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
