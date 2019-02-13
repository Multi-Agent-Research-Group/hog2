//
//  Map2DConstrainedEnvironment.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//
//  Modified by Thayne Walker 2017.
//

#include "Map2DConstrainedEnvironment.h"
#include "PositionalUtils.h"

Map2DConstrainedEnvironment::Map2DConstrainedEnvironment(Map *m):ignoreTime(false),ignoreHeading(false)
{
	mapEnv = new MapEnvironment(m);
	mapEnv->SetFourConnected();
}

Map2DConstrainedEnvironment::Map2DConstrainedEnvironment(MapEnvironment *m):ignoreTime(false),ignoreHeading(false)
{
	mapEnv = m;
}

void Map2DConstrainedEnvironment::ClearConstraints()
{
  ConstrainedEnvironment<xytLoc,tDirection>::ClearConstraints();
  vconstraints.resize(0);
}

bool Map2DConstrainedEnvironment::GetNextSuccessor(const xytLoc &currOpenNode, const xytLoc &goal, xytLoc &next, double &currHCost, uint64_t &special, bool &validMove){
  return mapEnv->GetNextSuccessor(currOpenNode,goal,next,currHCost,special,validMove);
}
  
double Map2DConstrainedEnvironment::GetPathLength(std::vector<xytLoc> &neighbors)
{
  // There is no cost for waiting at the goal
  double cost(0);
  for(int j(neighbors.size()-1); j>0; --j){
    if(!neighbors[j-1].sameLoc(neighbors[j])){
      cost += neighbors[j].t;
      break;
    }
  }
  return cost;
}

bool Map2DConstrainedEnvironment::collisionCheck(xytLoc const& s1, xytLoc const& d1, float radius1, xytLoc const& s2, xytLoc const& d2,float radius2){
  if(!mapEnv->collisionPreCheck(s1,d1,radius1,s2,d2,radius2)) return false;
  Vector2D A(s1);
  Vector2D B(s2);
  Vector2D VA(d1);
  VA-=A;
  VA.Normalize();
  Vector2D VB(d2);
  VB-=B;
  VB.Normalize();
  return collisionImminent(A,VA,radius1,s1.t,d1.t,B,VB,radius2,s2.t,d2.t);
}

void Map2DConstrainedEnvironment::GetSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const
{
  std::vector<xyLoc> n;
  mapEnv->GetSuccessors(nodeID, n);

  // TODO: remove illegal successors
  for (unsigned int x = 0; x < n.size(); x++)
  {
    float inc(n[x].sameLoc(nodeID)?WaitTime():mapEnv->GetConnectedness()>5?(Util::distance(nodeID.x,nodeID.y,n[x].x,n[x].y)):1.0);
    xytLoc newLoc(n[x],
        (uint16_t)Util::heading<1024>(nodeID.x,nodeID.y,n[x].x,n[x].y), // hdg
        nodeID.t+inc);
    if (!ViolatesConstraint(nodeID, newLoc)){
      neighbors.push_back(newLoc);
    }
  }
}

void Map2DConstrainedEnvironment::GetAllSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const
{
  std::vector<xyLoc> n;
  mapEnv->GetSuccessors(nodeID, n);

  // TODO: remove illegal successors
  for (unsigned int x = 0; x < n.size(); x++)
  {
    float inc(n[x].sameLoc(nodeID)?WaitTime():mapEnv->GetConnectedness()>5?(Util::distance(nodeID.x,nodeID.y,n[x].x,n[x].y)):1.0);
    xytLoc newLoc(n[x],
        (uint16_t)Util::heading<1024>(nodeID.x,nodeID.y,n[x].x,n[x].y), // hdg
        nodeID.t+inc);
    neighbors.push_back(newLoc);
  }
}

/*double Map2DConstrainedEnvironment::ViolatesConstraint(const xytLoc &from, const xytLoc &to) const{
  
  // Check motion constraints first
  //if(maxTurnAzimuth&&fless(maxTurnAzimuth,fabs(from.h-to.h))) return false;
  double ctime(ConstrainedEnvironment<xytLoc,tDirection>::ViolatesConstraint(from,to));

  if(ctime){return ctime;}

  if(vconstraints.size()){
    TemporalVector3D A(from);
    TemporalVector3D VA(to);
    VA-=A; // Direction vector
    VA.Normalize();
    double ctime(0);
    for(unsigned int x = 0; x < vconstraints.size(); x++)
    {
      Vector3D B(vconstraints[x]->start_state);
      Vector3D VB(vconstraints[x]->end_state);
      VB-=B; // Direction vector
      VB.Normalize();
      if(ctime=collisionImminent(A,VA,agentRadius,from.t,to.t,B,VB,agentRadius,vconstraints[x]->start_state.t,vconstraints[x]->end_state.t)){
        //std::cout << from << " --> " << to << " collides with " << vconstraints[x]->start_state << "-->" << vconstraints[x]->end_state << "\n";
        return ctime;
      }else{
        //std::cout << from << " --> " << to << " does not collide with " << vconstraints[x]->start_state << "-->" << vconstraints[x]->end_state << "\n";
      }
    }
  }
  return 0;
}*/

void Map2DConstrainedEnvironment::GetActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const
{
	mapEnv->GetActions(nodeID, actions);

	// TODO: remove illegal actions
}

tDirection Map2DConstrainedEnvironment::GetAction(const xytLoc &s1, const xytLoc &s2) const
{
	return mapEnv->GetAction(s1, s2);
}

void Map2DConstrainedEnvironment::ApplyAction(xytLoc &s, tDirection a) const
{
	mapEnv->ApplyAction(s, a);
	s.t+=1;
}

void Map2DConstrainedEnvironment::UndoAction(xytLoc &s, tDirection a) const
{
	mapEnv->UndoAction(s, a);
	s.t-=1;
}

void Map2DConstrainedEnvironment::AddConstraint(Constraint<TemporalVector3D> const* c)
{
	vconstraints.push_back(c);
}

void Map2DConstrainedEnvironment::GetReverseActions(const xytLoc &nodeID, std::vector<tDirection> &actions) const
{
	// Get the action information from the hidden env
	//this->mapEnv->GetReverseActions(nodeID, actions);
}
bool Map2DConstrainedEnvironment::InvertAction(tDirection &a) const
{
	return mapEnv->InvertAction(a);
}


/** Heuristic value between two arbitrary nodes. **/
double Map2DConstrainedEnvironment::HCost(const xytLoc &node1, const xytLoc &node2) const
{
        return mapEnv->HCost(node1, node2);
	double res1 = mapEnv->HCost(node1, node2);
	double res2 = (node2.t>node1.t)?(node2.t-node1.t):0;
	//std::cout << "h(" << node1 << ", " << node2 << ") = " << res1 << " " << res2 << std::endl;
	return max(res1, res2);
}

bool Map2DConstrainedEnvironment::GoalTest(const xytLoc &node, const xytLoc &goal) const
{
	return (node.x == goal.x && node.y == goal.y && node.t >= goal.t);
}


uint64_t Map2DConstrainedEnvironment::GetStateHash(const xytLoc &node) const
{
  uint64_t hash;
  hash = node.x;
  hash <<= 11;
  hash |= node.y&(2047);
  if(!ignoreHeading){
    hash <<= 10;
    hash |= node.h&(1023);
  }
  if(!ignoreTime){
    hash <<= 32;
    hash |= *(uint32_t*)&node.t;
    //std::cout << "time to hash" << (*(uint32_t*)&node.t) << "\n";
  }
  return hash;
}

void Map2DConstrainedEnvironment::GetStateFromHash(uint64_t hash, xytLoc &s) const
{
  if(!ignoreTime){
    uint32_t tmp=(hash&(0x100000000-1));
    //std::cout << "time from hash" << (hash&(0x100000000-1)) << "\n";
    s.t=*(float*)&tmp;
    s.y=(hash>>32)&(0x10000-1);
    s.x=(hash>>48)&(0x10000-1);
  }else{
    s.y=(hash)&(0x10000-1);
    s.x=(hash>>16)&(0x10000-1);
  }
}

uint64_t Map2DConstrainedEnvironment::GetActionHash(tDirection act) const
{
	return act;
}


void Map2DConstrainedEnvironment::OpenGLDraw() const
{
	mapEnv->OpenGLDraw();
}

void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc& s, const xytLoc& e, float perc) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  Map *map = mapEnv->GetMap();
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord((1-perc)*s.x+perc*e.x, (1-perc)*s.y+perc*e.y, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz, rad*2.0*agentRadius); // zz-s.t*2*rad
}

void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc& l) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  Map *map = mapEnv->GetMap();
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz-l.t*rad, rad*2.0*agentRadius); // zz-l.t*2*rad
}

void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc&, const tDirection&) const
{
       std::cout << "Draw move\n";
}

void Map2DConstrainedEnvironment::GLDrawLine(const xytLoc &x, const xytLoc &y) const
{
	GLdouble xx, yy, zz, rad;
	Map *map = mapEnv->GetMap();
	map->GetOpenGLCoord(x.x, x.y, xx, yy, zz, rad);
	
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
        if(x.sameLoc(y)){
          GLDrawCircle(x.x,x.y,rad/2.0);
        }else{
          glBegin(GL_LINES);
          glVertex3f(xx, yy, zz);
          map->GetOpenGLCoord(y.x, y.y, xx, yy, zz, rad);
          glVertex3f(xx, yy, zz);
          glEnd();
        }
}

void Map2DConstrainedEnvironment::GLDrawPath(const std::vector<xytLoc> &p, const std::vector<xytLoc> &waypoints) const
{
        if(p.size()<2) return;
        //TODO Draw waypoints as cubes.
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}

const float Map2DConstrainedEnvironment::HDG_RESOLUTON=1023.0/360.;
