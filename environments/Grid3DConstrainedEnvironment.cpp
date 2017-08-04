//
//  Grid3DConstrainedEnvironment.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#include "Grid3DConstrainedEnvironment.h"

bool operator==(const xyztLoc &l1, const xyztLoc &l2)
{
	return fequal(l1.t,l2.t) && (l1.x == l2.x) && (l1.y==l2.y);
}

Grid3DConstrainedEnvironment::Grid3DConstrainedEnvironment(Map3D *m):ignoreTime(false)
{
	mapEnv = new Grid3DEnvironment(m);
}

Grid3DConstrainedEnvironment::Grid3DConstrainedEnvironment(Grid3DEnvironment *m):ignoreTime(false)
{
	mapEnv = m;
}

/*void Grid3DConstrainedEnvironment::AddConstraint(xyztLoc loc)
{
	AddConstraint(loc, kTeleport);
}*/

void Grid3DConstrainedEnvironment::AddConstraint(xyztLoc const& loc, t3DDirection dir)
{
        xyztLoc tmp(loc);
        ApplyAction(tmp,dir);
	constraints.emplace_back(loc,tmp);
}

void Grid3DConstrainedEnvironment::ClearConstraints()
{
	constraints.resize(0);
	vconstraints.resize(0);
}

void Grid3DConstrainedEnvironment::GetSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const
{
  std::vector<xyzLoc> n;
  mapEnv->GetSuccessors(nodeID, n);

  // TODO: remove illegal successors
  for (unsigned int x = 0; x < n.size(); x++)
  {
    float inc(mapEnv->GetConnectedness()>0?(Util::distance(nodeID.x,nodeID.y,n[x].x,n[x].y)):1.0);
    if (!ViolatesConstraint(nodeID, n[x], nodeID.t, inc))
    {
      xyztLoc newLoc(n[x],nodeID.t+inc);
      neighbors.push_back(newLoc);
    }else{
      //std::cout << n[x] << " violates\n";
    }
  }
}

void Grid3DConstrainedEnvironment::GetAllSuccessors(const xyztLoc &nodeID, std::vector<xyztLoc> &neighbors) const
{
  std::vector<xyzLoc> n;
  mapEnv->GetSuccessors(nodeID, n);

  // TODO: remove illegal successors
  for (unsigned int x = 0; x < n.size(); x++)
  {
    float inc(mapEnv->GetConnectedness()>5?(Util::distance(nodeID.x,nodeID.y,n[x].x,n[x].y)):1.0);
    xyztLoc newLoc(n[x],nodeID.t+inc);
    neighbors.push_back(newLoc);
  }
}

bool Grid3DConstrainedEnvironment::ViolatesConstraint(const xyztLoc &from, const xyztLoc &to) const{
  
  /*for(unsigned int x = 0; x < constraints.size(); x++)
  {
    if(from.x==constraints[x].start_state.x && from.y==constraints[x].start_state.y &&
        to.x==constraints[x].end_state.x && to.y==constraints[x].end_state.y)
      return true;
  }
  return false;
  */

  Vector3D A(from);
  Vector3D VA(to);
  VA-=A; // Direction vector
  VA.Normalize();
  static double aradius(0.25);
  static double bradius(0.25);
  // TODO: Put constraints into a kD tree so that we only need to compare vs relevant constraints
  for(unsigned int x = 0; x < constraints.size(); x++)
  {
    Vector3D B(constraints[x].start_state);
    Vector3D VB(constraints[x].end_state);
    VB-=B; // Direction vector
    VB.Normalize();
    if(collisionImminent(A,VA,aradius,from.t,to.t,B,VB,bradius,constraints[x].start_state.t,constraints[x].end_state.t)){
      //std::cout << from << " --> " << to << " collides with " << vconstraints[x].start_state << "-->" << vconstraints[x].end_state << "\n";
      return true;
    }else{
      //std::cout << from << " --> " << to << " does not collide with " << vconstraints[x].start_state << "-->" << vconstraints[x].end_state << "\n";
    }
  }
  for(unsigned int x = 0; x < vconstraints.size(); x++)
  {
    Vector3D B(vconstraints[x].start_state);
    Vector3D VB(vconstraints[x].end_state);
    VB-=B; // Direction vector
    VB.Normalize();
    if(collisionImminent(A,VA,aradius,from.t,to.t,B,VB,bradius,vconstraints[x].start_state.t,vconstraints[x].end_state.t)){
      //std::cout << from << " --> " << to << " collides with " << vconstraints[x].start_state << "-->" << vconstraints[x].end_state << "\n";
      return true;
    }else{
      //std::cout << from << " --> " << to << " does not collide with " << vconstraints[x].start_state << "-->" << vconstraints[x].end_state << "\n";
    }
  }
  return false;
}

bool Grid3DConstrainedEnvironment::ViolatesConstraint(const xyzLoc &from, const xyzLoc &to, float time, float inc) const
{
  return ViolatesConstraint(xyztLoc(from,time),xyztLoc(to,time+inc));
}

void Grid3DConstrainedEnvironment::GetActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const
{
	mapEnv->GetActions(nodeID, actions);

	// TODO: remove illegal actions
}

t3DDirection Grid3DConstrainedEnvironment::GetAction(const xyztLoc &s1, const xyztLoc &s2) const
{
	return mapEnv->GetAction(s1, s2);
}

void Grid3DConstrainedEnvironment::ApplyAction(xyztLoc &s, t3DDirection a) const
{
	mapEnv->ApplyAction(s, a);
	s.t+=1;
}

void Grid3DConstrainedEnvironment::UndoAction(xyztLoc &s, t3DDirection a) const
{
	mapEnv->UndoAction(s, a);
	s.t-=1;
}

void Grid3DConstrainedEnvironment::AddConstraint(Constraint<xyztLoc> const& c)
{
	constraints.push_back(c);
}

void Grid3DConstrainedEnvironment::AddConstraint(Constraint<TemporalVector3D> const& c)
{
	vconstraints.push_back(c);
}

void Grid3DConstrainedEnvironment::GetReverseActions(const xyztLoc &nodeID, std::vector<t3DDirection> &actions) const
{
	// Get the action information from the hidden env
	//this->mapEnv->GetReverseActions(nodeID, actions);
}
bool Grid3DConstrainedEnvironment::InvertAction(t3DDirection &a) const
{
	return mapEnv->InvertAction(a);
}


/** Heuristic value between two arbitrary nodes. **/
double Grid3DConstrainedEnvironment::HCost(const xyztLoc &node1, const xyztLoc &node2) const
{
        return mapEnv->HCost(node1, node2);
	double res1 = mapEnv->HCost(node1, node2);
	double res2 = (node2.t>node1.t)?(node2.t-node1.t):0;
	//std::cout << "h(" << node1 << ", " << node2 << ") = " << res1 << " " << res2 << std::endl;
	return max(res1, res2);
}

bool Grid3DConstrainedEnvironment::GoalTest(const xyztLoc &node, const xyztLoc &goal) const
{
	return (node.x == goal.x && node.y == goal.y && node.t >= goal.t);
}


uint64_t Grid3DConstrainedEnvironment::GetStateHash(const xyztLoc &node) const
{
  uint64_t h1;
  h1 = node.x;
  h1 <<= 16;
  h1 |= node.y;
  h1 <<= 16;
  h1 |= node.z;
  h1 <<= 16;
  h1 |= node.v;
  uint64_t h2(*(uint32_t*)&node.t);
  h2 <<=32;
  h2 |= h2;
  return (h1 * 16777619) ^ h2;
}

void Grid3DConstrainedEnvironment::GetStateFromHash(uint64_t hash, xyztLoc &s) const
{
  assert(!"Hash is irreversible...");
  /*if(!ignoreTime){
    uint32_t tmp=(hash&(0x100000000-1));
    //std::cout << "time from hash" << (hash&(0x100000000-1)) << "\n";
    s.t=*(float*)&tmp;
    s.y=(hash>>32)&(0x10000-1);
    s.x=(hash>>48)&(0x10000-1);
  }else{
    s.y=(hash)&(0x10000-1);
    s.x=(hash>>16)&(0x10000-1);
  }*/
}

uint64_t Grid3DConstrainedEnvironment::GetActionHash(t3DDirection act) const
{
	return act;
}


void Grid3DConstrainedEnvironment::OpenGLDraw() const
{
	mapEnv->OpenGLDraw();
}

void Grid3DConstrainedEnvironment::OpenGLDraw(const xyztLoc& s, const xyztLoc& e, float perc) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  Map3D *map = mapEnv->GetMap();
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord((1-perc)*s.x+perc*e.x, (1-perc)*s.y+perc*e.y, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz, rad/2.0); // zz-s.t*2*rad
}

void Grid3DConstrainedEnvironment::OpenGLDraw(const xyztLoc& l) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  Map3D *map = mapEnv->GetMap();
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz-l.t*rad, rad/2.0); // zz-l.t*2*rad
}

void Grid3DConstrainedEnvironment::OpenGLDraw(const xyztLoc&, const t3DDirection&) const
{
       std::cout << "Draw move\n";
}

void Grid3DConstrainedEnvironment::GLDrawLine(const xyztLoc &x, const xyztLoc &y) const
{
	GLdouble xx, yy, zz, rad;
	Map3D *map = mapEnv->GetMap();
	map->GetOpenGLCoord(x.x, x.y, xx, yy, zz, rad);
	
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	glBegin(GL_LINES);
	glVertex3f(xx, yy, zz);
	map->GetOpenGLCoord(y.x, y.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz);
	glEnd();
}

void Grid3DConstrainedEnvironment::GLDrawPath(const std::vector<xyztLoc> &p, const std::vector<xyztLoc> &waypoints) const
{
        if(p.size()<2) return;
        int wpt(0);
        //TODO Draw waypoints as cubes.
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}

template<>
bool Constraint<xyztLoc>::ConflictsWith(const xyztLoc &state) const
{
  /*if(state.landed || end_state.landed && start_state.landed) return false;
  Vector3D A(state);
  Vector3D VA(0,0);
  //VA.Normalize();
  static double aradius(0.25);
  static double bradius(0.25);
  Vector3D B(start_state);
  Vector3D VB(end_state);
  VB-=B; // Direction vector
  //VB.Normalize();
  if(collisionImminent(A,VA,aradius,state.t,state.t+1.0,B,VB,bradius,start_state.t,end_state.t)){
    return true;
  }*/
  return false; // There is really no such thing as a vertex conflict in continuous time domains.
}

// Check both vertex and edge constraints
template<>
bool Constraint<xyztLoc>::ConflictsWith(const xyztLoc &from, const xyztLoc &to) const
{
  Vector3D A(from);
  Vector3D VA(to);
  VA-=A; // Direction vector
  VA.Normalize();
  static double aradius(0.25);
  static double bradius(0.25);
  Vector3D B(start_state);
  Vector3D VB(end_state);
  VB-=B; // Direction vector
  VB.Normalize();
  if(collisionImminent(A,VA,aradius,from.t,to.t,B,VB,bradius,start_state.t,end_state.t)){
    return true;
  }
  return false;
}

template<>
bool Constraint<xyztLoc>::ConflictsWith(const Constraint<xyztLoc> &x) const
{
  return ConflictsWith(x.start_state, x.end_state);
}


template<>
void Constraint<xyztLoc>::OpenGLDraw(MapInterface* map) const 
{
  GLdouble xx, yy, zz, rad;
  glColor3f(1, 0, 0);
  glLineWidth(12.0);
  glBegin(GL_LINES);
  map->GetOpenGLCoord(start_state.x, start_state.y, xx, yy, zz, rad);
  glVertex3f(xx, yy, -rad);
  map->GetOpenGLCoord(end_state.x, end_state.y, xx, yy, zz, rad);
  glVertex3f(xx, yy, -rad);
  glEnd();
}

template<>
bool Constraint<TemporalVector3D>::ConflictsWith(const TemporalVector3D &state) const
{
  /*if(state.landed || end_state.landed && start_state.landed) return false;
  Vector3D A(state);
  Vector3D VA(0,0);
  //VA.Normalize();
  static double aradius(0.25);
  static double bradius(0.25);
  Vector3D B(start_state);
  Vector3D VB(end_state);
  VB-=B; // Direction vector
  //VB.Normalize();
  if(collisionImminent(A,VA,aradius,state.t,state.t+1.0,B,VB,bradius,start_state.t,end_state.t)){
    return true;
  }*/
  return false; // There is really no such thing as a vertex conflict in continuous time domains.
}

// Check both vertex and edge constraints
template<>
bool Constraint<TemporalVector3D>::ConflictsWith(const TemporalVector3D &from, const TemporalVector3D &to) const
{
  Vector3D A(from);
  Vector3D VA(to);
  VA-=A; // Direction vector
  VA.Normalize();
  static double aradius(0.25);
  static double bradius(0.25);
  Vector3D B(start_state);
  Vector3D VB(end_state);
  VB-=B; // Direction vector
  VB.Normalize();
  if(collisionImminent(A,VA,aradius,from.t,to.t,B,VB,bradius,start_state.t,end_state.t)){
    return true;
  }
  return false;
}

template<>
bool Constraint<TemporalVector3D>::ConflictsWith(const Constraint<TemporalVector3D> &x) const
{
  return ConflictsWith(x.start_state, x.end_state);
}

template<>
void Constraint<TemporalVector3D>::OpenGLDraw(MapInterface* map) const 
{
	GLdouble xx, yy, zz, rad;
	glColor3f(1, 0, 0);
	map->GetOpenGLCoord(start_state.x, start_state.y, xx, yy, zz, rad);
	//DrawSphere(xx, yy, zz, rad/2.0); // zz-l.t*2*rad
	
        glLineWidth(12.0);
        //glDrawCircle(xx,yy,.5/map->GetMapWidth());
	glBegin(GL_LINES);
	glVertex3f(xx, yy, zz-.1);
        //std::cout << "Line from " << xx <<"("<<start_state.x << ")," << yy<<"("<<start_state.y << ")," << zz;
	map->GetOpenGLCoord(end_state.x, end_state.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-.1);
        //std::cout << " to " << xx <<"("<<end_state.x << ")," << yy<<"("<<end_state.y << ")," << zz << "\n";
	glEnd();
        //glDrawCircle(xx,yy,.5/map->GetMapWidth());
	//DrawSphere(xx, yy, zz, rad/2.0); // zz-l.t*2*rad
}

