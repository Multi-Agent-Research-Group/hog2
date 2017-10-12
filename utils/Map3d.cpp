/*
 * $Id: map.cpp,v 1.28 2007/03/07 22:01:36 nathanst Exp $
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ 

#include "Map3d.h"
#include "GLUtil.h"
#include <cstdlib>
#include <cstring>
#include "dtedreader.h"


using namespace std;

/** 
* Create a new map of a particular size.
*
* A map is an array of tiles according to the height and width of the map.
*/
Map3D::Map3D(long _width, long _height, long _depth)
:width(_width), height(_height), depth(_depth)
{
  elev = new uint8_t *[width];
  for (int x = 0; x < width; x++){
    elev[x] = new uint8_t [height];
    for(int y(0); y<height; ++y)
      elev[x][y]=0;
  }

  type = new tTerrain *[width];
  for (int x = 0; x < width; x++){
    type[x] = new tTerrain [height];
    for(int y(0); y<height; ++y)
      type[x][y]=kGround;
  }

}

/** 
* Create a new map by copying it from another map.
*
* Creates a new map and initializes it with the map passed to it.
*/
Map3D::Map3D(Map3D *m)
{
  width = m->width;
  height = m->height;
  depth = m->depth;
  maxDepth=m->maxDepth;
  memcpy(elev,m->elev,sizeof(elev));
  memcpy(type,m->type,sizeof(type));
  
}

/** 
* Create a new map by loading it from a file.
*
* Creates a new map and initializes it with the file passed to it.
*/
Map3D::Map3D(const char *filename, const char *dtedfile, unsigned md):maxDepth(md),depth(md){
	Load(filename,dtedfile);
}

/** 
* Resets the current map by loading the file passed in.
*
* Resets the current map by loading the file passed in.
*/
void Map3D::Load(const char *filename, const char *dtedfile)
{
  FILE *f(fopen(filename, "r"));
  char format[32];
  int num = fscanf(f, "type %s\nheight %d\nwidth %d\nmap\n", format, &height, &width);

  float** elevation = new float*[width];
  for(int i = 0; i <width; i++){
    elevation[i] = new float[height];
    memset(elevation[i],0,height); // Set to zero for now...
  }

  if(!readdted1(dtedfile,elevation,width,height,0,0,depth)){
    printf("Dted not loaded\n");
  }

  if(num == 3){
    loadOctile(f,elevation);
  }else{
    printf("Unknown map type; aborting load!\n");
  }
  fclose(f);

  for(int i(0); i<width; ++i)
    delete[] elevation[i];
  delete [] elevation;

}

void Map3D::loadOctile(FILE *f, float** elevation){
  elev = new uint8_t *[width];
  for (int x = 0; x < width; x++)
    elev[x] = new uint8_t [height];

  type = new tTerrain *[width];
  for (int x = 0; x < width; x++)
    type[x] = new tTerrain [height];

  for (int x = 0; x < width; x++)
  {
    for (int y = 0; y < height; y++)
    {
      char what;
      fscanf(f, "%c", &what);
      switch (toupper(what))
      {
        case '@':
        case 'O': // Out of bounds/obstructed
          elev[x][y]=0;
          type[x][y]=kOutOfBounds;
          break;
        case 'L': // Water; above sea level
          elev[x][y]=elevation[x][y];
          type[x][y]=kWater;
          break;
        case 'W': // Water; at sea level
        case 'S': // Swamp; at sea level
          elev[x][y]=0;
          type[x][y]=kWater;
          break;
        case 'T': // Elevated terrain
          elev[x][y]=elevation[x][y];
          type[x][y]=kGround;
          break;
        default:
          elev[x][y]=0;
          type[x][y]=kGround;
          break;
      }
    }
  }
}

/**
* Does actual OpenGL drawing of the map
 *
 * If drawLand has been set (on by default) the ground will be drawn using
 * the appropriate mode:   kPolygons, kLines, kPoints
 * kPolygon is the default mode. The map is cached in a display list unless
 * it changes.
 */
void Map3D::setColor(int x, int y) const{
  switch(type[x][y]){
    case kOutOfBounds:
      glColor3f(0, 0, 0);
      break;
    case kGround:
      glColor3f(0.9, 0.5, 0.5);
      break;
    case kWater:
      glColor3f(0.0, 0.1, 0.9);
      break;
    default:
      glColor3f(0.1, 0.9, 0.1);
      break;
  }
}

void Map3D::OpenGLDraw() const
{
  if(dList){
    glCallList(dList);
  }else{
    dList = glGenLists(1);
    glNewList(dList, GL_COMPILE_AND_EXECUTE);
    GLdouble xx, yy, zz, rr;
    glBegin(GL_QUADS);
    glColor3f(0.5, 0.5, 0.5);
    for(int y = 0; y < height; y++)
    {

      GetOpenGLCoord(0, y, elev[0][y], xx, yy, zz, rr);
      setColor(0, y);
      glVertex3f(xx-rr, yy-rr, zz);
      glVertex3f(xx-rr, yy+rr, zz);
      for(int x = 1; x < width; x++)
      {
        if(type[x][y] != type[x-1][y])
        {
          setColor(x-1, y);
          GetOpenGLCoord(x, y, elev[x][y], xx, yy, zz, rr);
          glVertex3f(xx-rr, yy+rr, zz);
          glVertex3f(xx-rr, yy-rr, zz);

          setColor(x, y);
          GetOpenGLCoord(x, y, elev[x][y], xx, yy, zz, rr);
          glVertex3f(xx-rr, yy-rr, zz);
          glVertex3f(xx-rr, yy+rr, zz);
        }
      }
      setColor(width-1, y);
      GetOpenGLCoord(width-1, y, elev[width-1][y], xx, yy, zz, rr);
      glVertex3f(xx+rr, yy+rr, zz);
      glVertex3f(xx+rr, yy-rr, zz);
    }
    glEnd();
    glEndList();
  }
}

/**
 * Get the openGL coordinates of a given tile.
 *
 * Given a tile in (x, y) coordinates, it returns the OpenGL space coordinates of
 * that tile along with the radius of the tile square. The map is drawn in the
 * x<->z plane, with the y plane up.
 */
bool Map3D::GetOpenGLCoord(float _x, float _y, float _z, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const
{
  if (_x >= width || _y >= height || _z>= depth)return false;
  if (_x <0 || _y <0 || _z<0)return false;
  double _scale(1.0/std::max(std::max(height,width),depth));
  x = (2*_x-width)*_scale;
  y = (2*_y-height)*_scale;
  z = (2*_z-depth)*_scale;
  radius=_scale;
  return true;
}

bool Map3D::LineOfSight2D(int x, int y, int x2, int y2, AgentType agentType) const{
  int delta_x(std::abs(x - x2));
  int delta_y(std::abs(y - y2));
  int step_x(x < x2 ? 1 : -1);
  int step_y(y < y2 ? 1 : -1);
  int error(0);
  int sep_value = delta_x*delta_x + delta_y*delta_y;
  if(delta_x == 0)
  {
    for(; y != y2; y += step_y)
      if(!IsTraversable(x, y, 0, agentType))
        return false;
  }
  else if(delta_y == 0)
  {
    for(; x != x2; x += step_x)
      if(!IsTraversable(x, y, 0, agentType))
        return false;
  }
  else if(delta_x > delta_y)
  {
    for(; x != x2; x += step_x)
    {
      if(!IsTraversable(x, y, 0, agentType))
        return false;
      if(!IsTraversable(x, y + step_y, 0, agentType))
        return false;
      error += delta_y;
      if(error > delta_x)
      {
        if(((error << 1) - delta_x - delta_y)*((error << 1) - delta_x - delta_y) < sep_value)
          if(!IsTraversable(x + step_x, y, 0, agentType))
            return false;
        if((3*delta_x - ((error << 1) - delta_y))*(3*delta_x - ((error << 1) - delta_y)) < sep_value)
          if(!IsTraversable(x, y + 2*step_y, 0, agentType))
            return false;
        y += step_y;
        error -= delta_x;
      }
    }
    if(!IsTraversable(x, y, 0, agentType))
      return false;
    if(!IsTraversable(x, y + step_y, 0, agentType))
      return false;
  }
  else
  {
    for(; y != y2; y += step_y)
    {
      if(!IsTraversable(x, y, 0, agentType))
        return false;
      if(!IsTraversable(x + step_x, y, 0, agentType))
        return false;
      error += delta_x;
      if(error > delta_y)
      {
        if(((error << 1) - delta_x - delta_y)*((error << 1) - delta_x - delta_y) < sep_value)
          if(!IsTraversable(x, y + step_y, 0, agentType))
            return false;
        if((3*delta_y - ((error << 1) - delta_x))*(3*delta_y - ((error << 1) - delta_x)) < sep_value)
          if(!IsTraversable(x + 2*step_x, y, 0, agentType))
            return false;
        x += step_x;
        error -= delta_y;
      }
    }
    if(!IsTraversable(x, y, 0, agentType))
      return false;
    if(!IsTraversable(x + step_x, y, 0, agentType))
      return false;
  }
  return true;
}


bool Map3D::LineOfSight(int x1, int y1, int z1, int const x2, int const y2, int const z2) const{
  int dx = x2 - x1;
  int dy = y2 - y1;
  int dz = z2 - z1;
  int x_inc = (dx < 0) ? -1 : 1;
  int l = abs(dx);
  int y_inc = (dy < 0) ? -1 : 1;
  int m = abs(dy);
  int z_inc = (dz < 0) ? -1 : 1;
  int n = abs(dz);
  int dx2 = l << 1;
  int dy2 = m << 1;
  int dz2 = n << 1;

  if ((l >= m) && (l >= n)) {
    int err_1 = dy2 - l;
    int err_2 = dz2 - l;
    for (int i = 0; i < l; i++) {
      if(HasObstacle(x1, y1, z1)){return false;}
      if (err_1 > 0) {
        y1 += y_inc;
        err_1 -= dx2;
      }
      if (err_2 > 0) {
        z1 += z_inc;
        err_2 -= dx2;
      }
      err_1 += dy2;
      err_2 += dz2;
      x1 += x_inc;
    }
  } else if ((m >= l) && (m >= n)) {
    int err_1 = dx2 - m;
    int err_2 = dz2 - m;
    for (int i = 0; i < m; i++) {
      if(HasObstacle(x1, y1, z1)){return false;}
      if (err_1 > 0) {
        x1 += x_inc;
        err_1 -= dy2;
      }
      if (err_2 > 0) {
        z1 += z_inc;
        err_2 -= dy2;
      }
      err_1 += dx2;
      err_2 += dz2;
      y1 += y_inc;
    }
  } else {
    int err_1 = dy2 - n;
    int err_2 = dx2 - n;
    for (int i = 0; i < n; i++) {
      if(HasObstacle(x1, y1, z1)){return false;}
      if (err_1 > 0) {
        y1 += y_inc;
        err_1 -= dz2;
      }
      if (err_2 > 0) {
        x1 += x_inc;
        err_2 -= dz2;
      }
      err_1 += dy2;
      err_2 += dx2;
      z1 += z_inc;
    }
  }
  return true;
}

bool Map3D::CanStep(long x1, long y1, long x2, long y2, AgentType a) const
{
  switch (x1-x2) {
    case 0: //return true;
      switch (y1-y2) {
        case 0: return true;
        case 1:
        case -1: return IsTraversable(x2,y2,a);
      }
      break;
    case 1: //return AdjacentEdges(x1, y1, kLeftEdge);
      switch (y1-y2) {
        case 0: return IsTraversable(x2,y2,a);
        case 1:
        case -1: return IsTraversable(x2,y1,a)&&IsTraversable(x1,y2,a)&&IsTraversable(x2,y2,a);
      }
      break;
    case -1: //return AdjacentEdges(x1, y1, kRightEdge);
      switch (y1-y2) {
        case 0: return IsTraversable(x2,y2,a);
        case 1:
        case -1: return IsTraversable(x2,y1,a)&&IsTraversable(x1,y2,a)&&IsTraversable(x2,y2,a);
      }
      break;
  }
  return false;
}
