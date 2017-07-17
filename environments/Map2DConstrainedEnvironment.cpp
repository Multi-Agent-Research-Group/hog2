//
//  Map2DConstrainedEnvironment.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/3/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#include "Map2DConstrainedEnvironment.h"

bool operator==(const xytLoc &l1, const xytLoc &l2)
{
	return fequal(l1.t,l2.t) && (l1.x == l2.x) && (l1.y==l2.y);
}

Map2DConstrainedEnvironment::Map2DConstrainedEnvironment(Map *m):ignoreTime(false)
{
	mapEnv = new MapEnvironment(m);
	mapEnv->SetFourConnected();
}

Map2DConstrainedEnvironment::Map2DConstrainedEnvironment(MapEnvironment *m):ignoreTime(false)
{
	mapEnv = m;
}

/*void Map2DConstrainedEnvironment::AddConstraint(xytLoc loc)
{
	AddConstraint(loc, kTeleport);
}*/

void Map2DConstrainedEnvironment::AddConstraint(xytLoc const& loc, tDirection dir)
{
        xytLoc tmp(loc);
        ApplyAction(tmp,dir);
	constraints.emplace_back(loc,tmp);
}

void Map2DConstrainedEnvironment::ClearConstraints()
{
	constraints.resize(0);
}

bool Map2DConstrainedEnvironment::GetNextSuccessor(const xytLoc &currOpenNode, const xytLoc &goal, xytLoc &next, double &currHCost, uint64_t &special, bool &validMove){
  return mapEnv->GetNextSuccessor(currOpenNode,goal,next,currHCost,special,validMove);
}
  

void Map2DConstrainedEnvironment::GetSuccessors(const xytLoc &nodeID, std::vector<xytLoc> &neighbors) const
{
  std::vector<xyLoc> n;
  mapEnv->GetSuccessors(nodeID, n);

  // TODO: remove illegal successors
  for (unsigned int x = 0; x < n.size(); x++)
  {
    float inc(mapEnv->GetConnectedness()>5?(Util::distance(nodeID.x,nodeID.y,n[x].x,n[x].y)):1.0);
    if (!ViolatesConstraint(nodeID, n[x], nodeID.t, inc))
    {
      xytLoc newLoc(n[x],nodeID.t+inc);
      neighbors.push_back(newLoc);
    }else{
      std::cout << n[x] << " violates\n";
    }
  }
}
bool Map2DConstrainedEnvironment::ViolatesConstraint(const xytLoc &from, const xytLoc &to) const{
  Vector2D A(from);
  Vector2D VA(to);
  VA-=A; // Direction vector
  //VA.NormalIize();
  static double aradius(0.25);
  static double bradius(0.25);
  // TODO: Put constraints into a kD tree so that we only need to compare vs relevant constraints
  for(unsigned int x = 0; x < constraints.size(); x++)
  {
    Vector2D B(constraints[x].start_state);
    Vector2D VB(constraints[x].end_state);
    VB-=B; // Direction vector
    //VB.Normalize();
    if(collisionImminent(A,VA,aradius,from.t,to.t,B,VB,bradius,constraints[x].start_state.t,constraints[x].end_state.t)){
      std::cout << from << " collides with " << to << "\n";
      return true;
    }
  }
  return false;
}

bool Map2DConstrainedEnvironment::ViolatesConstraint(const xyLoc &from, const xyLoc &to, float time, float inc) const
{
  return ViolatesConstraint(xytLoc(from,time),xytLoc(to,time+inc));
}

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

void Map2DConstrainedEnvironment::AddConstraint(Constraint<xytLoc> const& c)
{
	constraints.push_back(c);
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
  hash <<= 16;
  hash |= node.y;
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
	// draw constrains
	Map *map = mapEnv->GetMap();
	for (unsigned int x = 0; x < constraints.size(); x++)
	{
		GLdouble xx, yy, zz, rad;
		map->GetOpenGLCoord(constraints[x].start_state.x, constraints[x].start_state.y, xx, yy, zz, rad);
		glColor4f(1.0, 0.0, 0.0, 0.5);
		//glColor3f(0.5, 0.5, 0.5);
		DrawSphere(xx, yy, zz-constraints[x].start_state.t*rad, rad);
	}
}

void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc& s, const xytLoc& e, float perc) const
{
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	Map *map = mapEnv->GetMap();
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	glColor4f(r, g, b, t);
	DrawSphere(xx, yy, zz-s.t*rad, rad); // zz-s.t*2*rad
}

void Map2DConstrainedEnvironment::OpenGLDraw(const xytLoc& l) const
{
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	Map *map = mapEnv->GetMap();
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	glColor4f(r, g, b, t);
	DrawSphere(xx, yy, zz-l.t*rad, rad); // zz-l.t*2*rad
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
	glBegin(GL_LINES);
	glVertex3f(xx, yy, zz-x.t*rad);
	map->GetOpenGLCoord(y.x, y.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-y.t*rad);
	glEnd();
}

void Map2DConstrainedEnvironment::GLDrawPath(const std::vector<xytLoc> &p, const std::vector<xytLoc> &waypoints) const
{
        if(p.size()<2) return;
        int wpt(0);
        //TODO Draw waypoints as cubes.
        for(auto a(p.begin()+1); a!=p.end(); ++a){
          GLDrawLine(*(a-1),*a);
        }
}
template<>
bool Constraint<xytLoc>::ConflictsWith(const xytLoc &state) const
{
  /*if(state.landed || end_state.landed && start_state.landed) return false;
  Vector2D A(state);
  Vector2D VA(0,0);
  //VA.NormalIize();
  static double aradius(0.25);
  static double bradius(0.25);
  Vector2D B(start_state);
  Vector2D VB(end_state);
  VB-=B; // Direction vector
  //VB.Normalize();
  if(collisionImminent(A,VA,aradius,state.t,state.t+1.0,B,VB,bradius,start_state.t,end_state.t)){
    return true;
  }*/
  return false; // There is really no such thing as a vertex conflict in continuous time domains.
}

// Check both vertex and edge constraints
template<>
bool Constraint<xytLoc>::ConflictsWith(const xytLoc &from, const xytLoc &to) const
{
  Vector2D A(from);
  Vector2D VA(to);
  VA-=A; // Direction vector
  //VA.NormalIize();
  static double aradius(0.25);
  static double bradius(0.25);
  Vector2D B(start_state);
  Vector2D VB(end_state);
  VB-=B; // Direction vector
  //VB.Normalize();
  if(collisionImminent(A,VA,aradius,from.t,to.t,B,VB,bradius,start_state.t,end_state.t)){
    return true;
  }
  return false;
}

template<>
bool Constraint<xytLoc>::ConflictsWith(const Constraint<xytLoc> &x) const
{
  return ConflictsWith(x.start_state, x.end_state);
}


template<>
void Constraint<xytLoc>::OpenGLDraw() const 
{
  return;
}

