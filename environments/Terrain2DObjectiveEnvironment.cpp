/*
 *  TerrainObjectiveEnvironment.cpp
 *  hog2
 *
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
 *
 */
#include "Terrain2DObjectiveEnvironment.h"
#include "Map2DEnvironment.h"
#include "dtedreader.h"
#include "Utilities.h"
#include <algorithm>

Terrain2DObjectiveEnvironment::Terrain2DObjectiveEnvironment(unsigned w, unsigned h, char* filename, float s):width(w),height(h),scale(s),connectivity(5){
  loadCosts(filename);
  buildHeuristic();
}

Terrain2DObjectiveEnvironment::~Terrain2DObjectiveEnvironment(){
  for(int i(0); i<width; ++i)
    delete[] elevation[i];
  delete [] elevation;
}

// I don't think landmark heuristics will work for this...
void Terrain2DObjectiveEnvironment::buildHeuristic(){
  // Find the min height along all columns and rows.
  // In order to go from x_1 to x_2, an agent absolutely must visit at least
  // the minimum height in the columns between them.
  rowMins.resize(height);
  colMins.resize(width);
  for(int x(0); x<width; ++x){
    colMins[x]=9999999;
  }
  for(int y(0); y<height; ++y){
    rowMins[y]=9999999;
  }
  for(int x(0); x<width; ++x){
    for(int y(0); y<height; ++y){
      rowMins[y]=std::min(rowMins[y],elevation[x][y]);
      colMins[x]=std::min(colMins[x],elevation[x][y]);
    }
  }
}

void Terrain2DObjectiveEnvironment::loadCosts(char* filename){
  elevation = new float*[width];
  for(int i = 0; i <width; i++)
    elevation[i] = new float[height];
  readdted1(filename,elevation,width,height,0,0,scale);
}

double Terrain2DObjectiveEnvironment::GCost(xytLoc const& node1, xytLoc const& node2) const{
  if(node1.sameLoc(node2))return 1.0/scale; // Waiting has a cost
  return cumulativeCost(node1.x,node1.y,node2.x,node2.y);
  // Find maximum height along line of sight trajectory...
  int x(node1.x);
  int y(node1.y);
  int x2(node2.x);
  int y2(node2.y);
  int delta_x(std::abs(x - x2));
  int delta_y(std::abs(y - y2));
  int step_x(x < x2 ? 1 : -1);
  int step_y(y < y2 ? 1 : -1);
  int error(0);
  int sep_value = delta_x*delta_x + delta_y*delta_y;
  //float maxElev(elevation[x][y]);
  //float minElev(elevation[x][y]);
  float val(0.0);
  if(delta_x == 0)
  {
    for(; y != y2; y += step_y){
      val+=std::max(0.0f,elevation[x][y+step_y]-elevation[x][y]);
      //maxElev=std::max(maxElev,elevation[x][y]);
      //minElev=std::min(minElev,elevation[x][y]);
    }
  }
  else if(delta_y == 0)
  {
    for(; x != x2; x += step_x){
      val+=std::max(0.0f,elevation[x+step_x][y]-elevation[x][y]);
      //maxElev=std::max(maxElev,elevation[x][y]);
      //minElev=std::min(minElev,elevation[x][y]);
    }
  }
  else if(delta_x > delta_y)
  {
    for(; x != x2; x += step_x)
    {
      //maxElev=std::max(maxElev,elevation[x][y]);
      //maxElev=std::max(maxElev,elevation[x][y+step_y]);
      //minElev=std::min(minElev,elevation[x][y]);
      //minElev=std::min(minElev,elevation[x][y+step_y]);
      val+=std::max(0.0f,elevation[x][y+step_y]-elevation[x][y]);
      error += delta_y;
      if(error > delta_x)
      {
        if(((error << 1) - delta_x - delta_y)*((error << 1) - delta_x - delta_y) < sep_value){
          val+=std::max(0.0f,elevation[x+step_x][y]-elevation[x][y]);
          //maxElev=std::max(maxElev,elevation[x+step_x][y]);
          //minElev=std::min(minElev,elevation[x+step_x][y]);
        }
        if((3*delta_x - ((error << 1) - delta_y))*(3*delta_x - ((error << 1) - delta_y)) < sep_value){
          val+=std::max(0.0f,elevation[x][y+2*step_y]-elevation[x][y]);
          //maxElev=std::max(maxElev,elevation[x][y+2*step_y]);
          //minElev=std::min(minElev,elevation[x][y+2*step_y]);
        }
        y += step_y;
        error -= delta_x;
      }
    }
    val+=std::max(0.0f,elevation[x][y+step_y]-elevation[x][y]);
    //maxElev=std::max(maxElev,elevation[x][y]);
    //maxElev=std::max(maxElev,elevation[x][y+step_y]);
    //minElev=std::min(minElev,elevation[x][y]);
    //minElev=std::min(minElev,elevation[x][y+step_y]);
  }
  else
  {
    for(; y != y2; y += step_y)
    {
      //maxElev=std::max(maxElev,elevation[x][y]);
      //maxElev=std::max(maxElev,elevation[x+step_x][y]);
      //minElev=std::min(minElev,elevation[x][y]);
      //minElev=std::min(minElev,elevation[x+step_x][y]);
      val+=std::max(0.0f,elevation[x+step_x][y]-elevation[x][y]);
      error += delta_x;
      if(error > delta_y)
      {
        if(((error << 1) - delta_x - delta_y)*((error << 1) - delta_x - delta_y) < sep_value){
          val+=std::max(0.0f,elevation[x][y+step_y]-elevation[x][y]);
          //maxElev=std::max(maxElev,elevation[x][y+step_y]);
          //minElev=std::min(minElev,elevation[x][y+step_y]);
        }
        if((3*delta_y - ((error << 1) - delta_x))*(3*delta_y - ((error << 1) - delta_x)) < sep_value){
          val+=std::max(0.0f,elevation[x+2*step_x][y]-elevation[x][y]);
          //maxElev=std::max(maxElev,elevation[x+2*step_x][y]);
          //minElev=std::min(minElev,elevation[x+2*step_x][y]);
        }
        x += step_x;
        error -= delta_y;
      }
    }
    val+=std::max(0.0f,elevation[x+step_x][y]-elevation[x][y]);
    //maxElev=std::max(maxElev,elevation[x][y]);
    //maxElev=std::max(maxElev,elevation[x+step_x][y]);
    //minElev=std::min(minElev,elevation[x][y]);
    //minElev=std::min(minElev,elevation[x+step_x][y]);
  }
  return val;//std::max(maxElev-minElev,0.0f);//+Util::distance(node1.x,node1.y,node2.x,node2.y);
}

// Based on Wu's Antialiasing Algorithm
// Code adapted from: http://www.geeksforgeeks.org/anti-aliased-line-xiaolin-wus-algorithm/
float
Terrain2DObjectiveEnvironment::cumulativeCost(int x0, int y0, int x1, int y1) const
{
  double total(0.0);
  bool steep(abs(y1 - y0) > abs(x1 - x0));

  // swap the co-ordinates if slope > 1 or we
  // draw backwards
  if (steep)
  {
    Util::swap(&x0 , &y0);
    Util::swap(&x1 , &y1);
  }
  bool swapped(false);
  if (x0 > x1)
  {
    swapped=true;
    Util::swap(&x0 ,&x1);
    Util::swap(&y0 ,&y1);
  }

  //compute the slope
  float dx(x1-x0);
  float dy(y1-y0);
  float gradient(dx?dy/dx:1.0f);

  float intersectY(y0);

  // main loop
  if (steep)
  {
    float prev(elevation[y0][x0]);
    for(int x(x0); x<=x1; ++x)
    {
      // pixel coverage is determined by fractional
      // part of y co-ordinate
      float val(elevation[Util::iPartOfNumber(intersectY)][x]);
      if(Util::iPartOfNumber(intersectY)+1<width)
        val=val*Util::rfPartOfNumber(intersectY)+elevation[Util::iPartOfNumber(intersectY)+1][x]*Util::fPartOfNumber(intersectY);
      if(swapped) total+=std::max(0.0f,prev-val);
      else total+=std::max(0.0f,val-prev);
      prev=val;
      intersectY += gradient;
    }
  }
  else
  {
    float prev(elevation[x0][y0]);
    for(int x(x0); x<=x1; ++x)
    {
      // pixel coverage is determined by fractional
      // part of y co-ordinate
      float val(elevation[x][Util::iPartOfNumber(intersectY)]);
      if(Util::iPartOfNumber(intersectY)+1<height)
        val=val*Util::rfPartOfNumber(intersectY)+elevation[x][Util::iPartOfNumber(intersectY)+1]*Util::fPartOfNumber(intersectY);
      if(swapped) total+=std::max(0.0f,prev-val);
      else total+=std::max(0.0f,val-prev);
      prev=val;
      intersectY += gradient;
    }
  }
  return total;
}


double Terrain2DObjectiveEnvironment::HCost(xytLoc const& node1, xytLoc const& node2) const{
  float increase(std::max(0.0f,elevation[node2.x][node2.y]-elevation[node1.x][node1.y]));
  // Sum up the mins between our current row,col and the goal row,col
  float val(0);
  int xmax(std::max(node1.x,node2.x));
  int ymax(std::max(node1.y,node2.y));
  for(int x(std::min(node1.x,node2.x)); x<xmax; ++x){
    val=std::max(val,colMins[x]);
  }
  for(int y(std::min(node1.y,node2.y)); y<ymax; ++y){
    val=std::max(val,rowMins[y]);
  }
  int dist(0.0);
  switch(connectivity){
    case 4:
    case 5:
      dist=MapEnvironment::h4(node1,node2);
      break;
    case 8:
    case 9:
      dist=MapEnvironment::h8(node1,node2);
      break;
    case 24:
    case 25:
      dist=MapEnvironment::h24(node1,node2);
      break;
    case 48:
    case 49:
      dist=MapEnvironment::h48(node1,node2);
      break;
    default:
      dist=Util::distance(node1.x,node1.y,node2.x,node2.y);
      break;
  }

  return std::max(val,increase);//+dist;
}

void Terrain2DObjectiveEnvironment::OpenGLDraw(Map* map)const{
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  GLdouble xx, yy, zz, rr;
  
  for(int y(0); y<map->GetMapHeight(); ++y){
    for(int x(0); x<map->GetMapWidth(); ++x){
      double v(elevation[x][y]);
      if(v){
        glColor4f(0., 0., 1.,v/scale);
        glBegin(GL_QUADS);

        map->GetOpenGLCoord(x, y, xx, yy, zz, rr);
        zz=-.005;
        //zz=v/scale;
        glVertex3f(xx-rr, yy-rr, zz);
        glVertex3f(xx-rr, yy+rr, zz);
        glVertex3f(xx+rr, yy+rr, zz);
        glVertex3f(xx+rr, yy-rr, zz);
        glEnd();
      }
    }
  }
  glDisable(GL_BLEND);
}
