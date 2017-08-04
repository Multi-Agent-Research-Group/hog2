/*
 *  Grid3DEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */
#include "Grid3DEnvironment.h"
#include "FPUtil.h"
#include <cstring>
#include "Graphics2D.h"
#include "PositionalUtils.h"

using namespace Graphics2D;

Grid3DEnvironment::Grid3DEnvironment(Map3D *_m,bool w):h(nullptr),map(_m),connectedness(0),waitAllowed(w){
}

Grid3DEnvironment::Grid3DEnvironment(Grid3DEnvironment *me)
{
	map = me->map->Clone();
	h = 0;
	connectedness = me->connectedness;
}

Grid3DEnvironment::~Grid3DEnvironment()
{
}

GraphHeuristic *Grid3DEnvironment::GetGraphHeuristic()
{
	return h;
}

void Grid3DEnvironment::SetGraphHeuristic(GraphHeuristic *gh)
{
	h = gh;
}

void Grid3DEnvironment::GetSuccessors(const xyzLoc &loc, std::vector<xyzLoc> &neighbors) const
{
  if(connectedness==0){
    if(map->IsTraversable(loc.x,loc.y,loc.z+1))
      neighbors.emplace_back(loc.x,loc.y,loc.z+1);
    if(map->IsTraversable(loc.x,loc.y,loc.z-1))
      neighbors.emplace_back(loc.x,loc.y,loc.z-1);
    if(map->IsTraversable(loc.x,loc.y+1,loc.z))
      neighbors.emplace_back(loc.x,loc.y+1,loc.z);
    if(map->IsTraversable(loc.x,loc.y-1,loc.z))
      neighbors.emplace_back(loc.x,loc.y-1,loc.z);
    if(map->IsTraversable(loc.x+1,loc.y,loc.z))
      neighbors.emplace_back(loc.x+1,loc.y,loc.z);
    if(map->IsTraversable(loc.x-1,loc.y,loc.z))
      neighbors.emplace_back(loc.x-1,loc.y,loc.z);
    if(waitAllowed)neighbors.emplace_back(loc);
  }else{
    for(int i(-connectedness); i<=connectedness; ++i){
      for(int j(-connectedness); j<=connectedness; ++j){
        for(int k(-connectedness); k<=connectedness; ++k){
          if(!waitAllowed && i==0 && j==0 && k==0)continue;
          neighbors.emplace_back(loc.x+i,loc.y+j,loc.z+k);
        }
      }
    }
  }
}

void Grid3DEnvironment::GetActions(const xyzLoc &loc, std::vector<t3DDirection> &actions) const
{
// TODO: Implement this
 assert(false && "Not implemented");
}

t3DDirection Grid3DEnvironment::GetAction(const xyzLoc &s1, const xyzLoc &s2) const
{
// TODO: Implement this
 assert(false && "Not implemented");
return kU;
}

bool Grid3DEnvironment::InvertAction(t3DDirection &a) const
{
 // TODO
 assert(false && "Not implemented");
	return true;
}

void Grid3DEnvironment::ApplyAction(xyzLoc &s, t3DDirection dir) const
{
// TODO
 assert(false && "Not implemented");
}

double Grid3DEnvironment::HCost(const xyzLoc &l1, const xyzLoc &l2) const
{
}

double Grid3DEnvironment::GCost(const xyzLoc &l, const t3DDirection &act) const
{
  assert(false&&"Not implemented");
  return 0;
}

double Grid3DEnvironment::GCost(const xyzLoc &l1, const xyzLoc &l2) const
{
  double multiplier = 1.0;
  static const float SQRT_2(sqrt(2));
  static const float SQRT_3(sqrt(3));
  static const float SQRT_5(sqrt(5));
  static const float SQRT_6(sqrt(6));
  static const float SQRT_8(sqrt(8));
  static const float SQRT_10(sqrt(10));
  static const float SQRT_11(sqrt(11));
  static const float SQRT_12(sqrt(12));
  static const float SQRT_13(sqrt(13));
  static const float SQRT_15(sqrt(15));
  static const float SQRT_17(sqrt(17));
  static const float SQRT_18(sqrt(18));
  static const float SQRT_19(sqrt(19));
  static const float SQRT_22(sqrt(22));
  static const float SQRT_27(sqrt(27));
  switch(connectedness){
    case 0:{return 1;}
    case 1:{
             unsigned v(abs(l1.x-l2.x)+abs(l1.y-l2.y)+abs(l1.z-l2.z));
             switch(v){
               case 0: return multiplier;
               case 1: return multiplier;
               case 2: return multiplier*SQRT_2;
               case 3: return multiplier*SQRT_3;
               default: return multiplier;
             }
           }
    case 2:{
             unsigned dx(abs(l1.x-l2.x));
             unsigned dy(abs(l1.y-l2.y));
             unsigned dz(abs(l1.z-l2.z));
             unsigned v(dx+dy+dz);

             if(v==6){
               return multiplier*SQRT_12;
             }else if(v==5){
               return multiplier*3.0; // sqrt(4+4+1)=3
             }else if(v==4){
               if(dx==0||dy==0||dz==0) return multiplier*SQRT_8;
               else return multiplier*SQRT_6;
             }else if(v==3){
               if(dx==0||dy==0||dz==0) return multiplier*SQRT_5;
               else return multiplier*SQRT_3;
             }else if(v==2){
               if(dx==2||dy==2||dz==2) return multiplier*2.0;
               else return multiplier*SQRT_2;
             }else{return 1;}
           }
    case 3:{
             unsigned dx(abs(l1.x-l2.x));
             unsigned dy(abs(l1.y-l2.y));
             unsigned dz(abs(l1.z-l2.z));
             unsigned v(dx+dy+dz);

             if(v==9){
               return multiplier*SQRT_27;
             }else if(v==8){
               return multiplier*SQRT_22;
             }else if(v==7){
               if(dx==1||dy==1||dz==1) return multiplier*SQRT_19;
               else return multiplier*SQRT_17;
             }else if(v==6){
               if(dx==0||dy==0||dz==0) return multiplier*SQRT_18;
               else if(dx==3||dy==3||dz==3) return multiplier*SQRT_15;
               else return multiplier*SQRT_12;
             }else if(v==5){
               if(dx==0||dy==0||dz==0) return multiplier*SQRT_13;
               else if(dx==3||dy==3||dz==3) return multiplier*SQRT_11;
               else return multiplier*3.0;
             }else if(v==4){
               if(dx==3||dy==3||dz==3) return multiplier*SQRT_10;
               else if(dx==0||dy==0||dz==0) return multiplier*SQRT_8;
               else return multiplier*SQRT_6;
             }else if(v==3){
               if(dx==1&&dy==1&&dz==1) return multiplier*SQRT_3;
               else if(dx==3||dy==3||dz==3) return multiplier*3.0;
               else return multiplier*SQRT_5;
             }else if(v==2){
               if(dx==2||dy==2||dz==2) return multiplier*2.0;
               else return multiplier*SQRT_2;
             }else{return 1;}
           }
  }
}

bool Grid3DEnvironment::LineOfSight(const xyzLoc &node, const xyzLoc &goal) const{
  return map->LineOfSight(node.x,node.y,node.z,goal.x,goal.y,goal.z);
}

bool Grid3DEnvironment::GoalTest(const xyzLoc &node, const xyzLoc &goal) const
{
	return ((node.x == goal.x) && (node.y == goal.y));
}

uint64_t Grid3DEnvironment::GetMaxHash() const
{
	return map->GetMapWidth()*map->GetMapHeight();
}

uint64_t Grid3DEnvironment::GetStateHash(const xyzLoc &node) const
{
	//return (((uint64_t)node.x)<<16)|node.y;
	return node.y*map->GetMapWidth()+node.x;
	//	return (node.x<<16)|node.y;
}

uint64_t Grid3DEnvironment::GetActionHash(t3DDirection act) const
{
	return (uint32_t) act;
}

void Grid3DEnvironment::OpenGLDraw() const
{
	map->OpenGLDraw();
}

void Grid3DEnvironment::OpenGLDraw(const xyzLoc &l) const
{
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	//glColor3f(0.5, 0.5, 0.5);
	DrawSphere(xx, yy, zz, rad);
}

void Grid3DEnvironment::OpenGLDraw(const xyzLoc &l1, const xyzLoc &l2, float v) const
{
	GLdouble xx, yy, zz, rad;
	GLdouble xx2, yy2, zz2;
//	map->GetOpenGLCoord((float)((1-v)*l1.x+v*l2.x),
//						(float)((1-v)*l1.y+v*l2.y), xx, yy, zz, rad);
//	printf("%f between (%d, %d) and (%d, %d)\n", v, l1.x, l1.y, l2.x, l2.y);
	map->GetOpenGLCoord(l1.x, l1.y, xx, yy, zz, rad);
	map->GetOpenGLCoord(l2.x, l2.y, xx2, yy2, zz2, rad);
	//	map->GetOpenGLCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
	xx = (1-v)*xx+v*xx2;
	yy = (1-v)*yy+v*yy2;
	zz = (1-v)*zz+v*zz2;
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	DrawSphere(xx, yy, zz, rad);
}

//void Grid3DEnvironment::OpenGLDraw(const xyzLoc &l, GLfloat r, GLfloat g, GLfloat b) const
//{
//	GLdouble xx, yy, zz, rad;
//	map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
//	glColor3f(r,g,b);
//	DrawSphere(xx, yy, zz, rad);
//}


void Grid3DEnvironment::OpenGLDraw(const xyzLoc& initial, const t3DDirection &dir) const
{
        assert(false&&"not implemented");
	
	xyzLoc s = initial;
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	
	glColor3f(0.5, 0.5, 0.5);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
		

	
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-rad/2);
	glEnd();
	
}

void Grid3DEnvironment::GLDrawLine(const xyzLoc &a, const xyzLoc &b) const
{
	GLdouble xx1, yy1, zz1, rad;
	GLdouble xx2, yy2, zz2;
	map->GetOpenGLCoord(a.x, a.y, xx1, yy1, zz1, rad);
	map->GetOpenGLCoord(b.x, b.y, xx2, yy2, zz2, rad);
	
	double angle = atan2(yy1-yy2, xx1-xx2);
	double xoff = sin(2*PI-angle)*rad*0.1;
	double yoff = cos(2*PI-angle)*rad*0.1;

	
	
	GLfloat rr, gg, bb, t;
	GetColor(rr, gg, bb, t);
	glColor4f(rr, gg, bb, t);

	
	glBegin(GL_LINES);
	glVertex3f(xx1, yy1, zz1-rad/2);
	glVertex3f(xx2, yy2, zz2-rad/2);
	glEnd();

//	glEnable(GL_BLEND);
//	glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
	//glEnable(GL_POLYGON_SMOOTH);
//	glBegin(GL_TRIANGLE_STRIP);
//	//glBegin(GL_QUADS);
//	glVertex3f(xx1+xoff, yy1+yoff, zz1-rad/2);
//	glVertex3f(xx2+xoff, yy2+yoff, zz2-rad/2);
//	glVertex3f(xx1-xoff, yy1-yoff, zz1-rad/2);
//	glVertex3f(xx2-xoff, yy2-yoff, zz2-rad/2);
//	glEnd();

	//	glDisable(GL_POLYGON_SMOOTH);
	//
//	glBegin(GL_LINES);
//	glVertex3f(xx, yy, zz-rad/2);
//	map->GetOpenGLCoord(b.x, b.y, xx, yy, zz, rad);
//	glVertex3f(xx, yy, zz-rad/2);
//	glEnd();
}

void Grid3DEnvironment::GLLabelState(const xyzLoc &s, const char *str, double scale) const
{
	glPushMatrix();
	
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	
	glTranslatef(xx-rad, yy+rad/2, zz-2*rad);
	glScalef(scale*rad/(300.0), scale*rad/300.0, 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
	glDisable(GL_LIGHTING);
	for (int which = 0; which < strlen(str); which++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, str[which]);
	glEnable(GL_LIGHTING);
	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
	glPopMatrix();
}

void Grid3DEnvironment::GLLabelState(const xyzLoc &s, const char *str) const
{
	glPushMatrix();

	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	
	glTranslatef(xx-rad, yy+rad/2, zz-rad);
	glScalef(rad/(300.0), rad/300.0, 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
	glDisable(GL_LIGHTING);
	for (int which = 0; which < strlen(str); which++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, str[which]);
	glEnable(GL_LIGHTING);
	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
	glPopMatrix();
}

void Grid3DEnvironment::Draw() const
{
}

void Grid3DEnvironment::Draw(const xyzLoc &l) const
{
	GLdouble px, py, t, rad;
	map->GetOpenGLCoord(l.x, l.y, px, py, t, rad);

	//if (map->GetTerrainType(l.x, l.y) == kGround)
	{
		recColor c;// = {0.5, 0.5, 0};
		GLfloat t;
		GetColor(c.r, c.g, c.b, t);

		rect r;
		r.left = px-rad;
		r.top = py-rad;
		r.right = px+rad;
		r.bottom = py+rad;

		//s += SVGDrawCircle(l.x+0.5+1, l.y+0.5+1, 0.5, c);
		::FillCircle(r, c);
		//stroke-width="1" stroke="pink" />
	}
}

void Grid3DEnvironment::DrawLine(const xyzLoc &a, const xyzLoc &b, double width) const
{
	GLdouble xx1, yy1, zz1, rad;
	GLdouble xx2, yy2, zz2;
	map->GetOpenGLCoord(a.x, a.y, xx1, yy1, zz1, rad);
	map->GetOpenGLCoord(b.x, b.y, xx2, yy2, zz2, rad);

	recColor c;// = {0.5, 0.5, 0};
	GLfloat t;
	GetColor(c.r, c.g, c.b, t);
	
	::DrawLine({xx1, yy1}, {xx2, yy2}, width, c);
}



double Grid3DEnvironment::GetPathLength(std::vector<xyzLoc> &neighbors)
{
	double length = 0;
	for (unsigned int x = 1; x < neighbors.size(); x++)
	{
		length += HCost(neighbors[x-1], neighbors[x]);
	}
	return length;
}

/************************************************************/

/*AbsGrid3DEnvironment::AbsGrid3DEnvironment(MapAbstraction *_ma)
:Grid3DEnvironment(_ma->GetMap())
{
	ma = _ma;
	
}

AbsGrid3DEnvironment::~AbsGrid3DEnvironment()
{
	map = 0;
	//delete ma;
}*/
