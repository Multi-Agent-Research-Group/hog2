/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "Map3d.h"
#include "GLUtil.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include "dtedreader.h"
#include "Vector3D.h"


using namespace std;

/** 
* Create a new map of a particular size.
*
* A map is an array of tiles according to the height and width of the map.
*/
Map3D::Map3D(long _width, long _height, long _depth)
:width(_width), height(_height), depth(_depth), maxDepth(_depth), dList(0)
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
  dList=m->dList;
  width = m->width;
  height = m->height;
  depth = m->depth;
  maxDepth=m->maxDepth;
  memcpy(elev,m->elev,sizeof(uint8_t)*width*height);
  memcpy(type,m->type,sizeof(tTerrain)*width*height);
  
}

/** 
* Create a new map by loading it from a file.
*
* Creates a new map and initializes it with the file passed to it.
*/
Map3D::Map3D(const char *filename, const char *dtedfile, unsigned md):maxDepth(md),depth(md),dList(0){
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
  std::string fmt(format);

  float** elevation = new float*[width];
  for(int i = 0; i <width; i++){
    elevation[i] = new float[height];
    for(int j(0); j<height; ++j)
      elevation[i][j]=1;
  }

  if(!readdted1(dtedfile,elevation,width,height,0,0,depth)){
    fprintf(stderr,"Dted not loaded\n");
  }


  if(num == 3 && fmt == "octile"){
    loadOctile(f,elevation);
  }else if(num == 3 && fmt == "octile-corner"){
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        char what;
        fscanf(f, "%c", &what);
      }
      fscanf(f, "\n");
    }
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

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      char what;
      fscanf(f, "%c", &what);
      switch (toupper(what))
      {
        case '@':
        case 'O': // Out of bounds/obstructed
          elev[x][y]=elevation[x][y];
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
          type[x][y]=kTrees;
          break;
        case '\n':
        case '\r':
        break;
        default:
          elev[x][y]=0;
          type[x][y]=kGround;
          break;
      }
    }
    fscanf(f, "\n");
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
      glColor3f(0.6, 0.4, 0.3);
      break;
    case kWater:
      glColor3f(0.0, 0.1, 0.9);
      break;
    default:
      glColor3f(0.1, 0.9, 0.1);
      break;
  }
}

void Map3D::doNormal(float x1, float y1, float z1,float x2, float y2, float z2,float x3, float y3, float z3)const{
  Vector3D p1(x1,y1,z1);
  Vector3D p2(x2,y2,z2);
  Vector3D p3(x3,y3,z3);
  Vector3D U=p2-p1;
  Vector3D V=p3-p1;
  glNormal3f(U.y*V.z - U.z*V.y, U.z*V.x - U.x*V.z, U.x*V.y - U.y*V.x);
}

void Map3D::OpenGLDraw() const
{
  if(dList){
    glCallList(dList);
  }else{
    dList = glGenLists(1);
    glNewList(dList, GL_COMPILE_AND_EXECUTE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    float ambientColour[4] = {0.2f, 0.2f, 0.2f, 1.0f};
    float diffuseColour[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float specularColour[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColour);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColour);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, specularColour);

    float position[4] = {0.0f, 0.0f, -2.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    GLdouble xx, yy, zz, rr;
    glBegin(GL_TRIANGLES);
    for(int y = 0; y < height; y++)
    {
      setColor(0, y);
      GetOpenGLCoord(0, y, elev[0][y], xx, yy, zz, rr);

      GLdouble x1(xx-rr);
      GLdouble x2(xx+rr);
      GLdouble y1(yy-rr);
      GLdouble y2(yy+rr);
      GLdouble z11(0);
      GLdouble z12(0);
      GLdouble z21(y?elev[0][y-1]*-rr:0);
      GLdouble z22(elev[0][y]*-rr);
      GLdouble z11n(0);
      GLdouble z12n(0);
      GLdouble z21n(y?elev[0][y-1]*-rr:0);
      GLdouble z22n(elev[0][y]*-rr);
      
        doNormal(x1,y1,z11n, x1,y2,z12n, x2,y1,z21n);
        glVertex3f(x1, y1, z11n);
        glVertex3f(x1, y2, z12n);
        glVertex3f(x2, y1, z21n);

        doNormal(x2,y1,z21n, x2,y2,z22n, x1,y2,z12n);
        glVertex3f(x2, y1, z21n);
        glVertex3f(x2, y2, z22n);
        glVertex3f(x1, y2, z12n);

      /*if(z21||z22){
        // Level change

        // Make a west vertical edge
        doNormal(x1,y1,z11, x1,y2,z12, x1,y1,z21);
        glVertex3f(x1, y1, z11);
        glVertex3f(x1, y1, z21);
        glVertex3f(x1, y2, z12);

        doNormal(x1,y1,z11, x1,y2,z12, x1,y2,z22);
        glVertex3f(x1, y1, z21);
        glVertex3f(x1, y2, z12);
        glVertex3f(x1, y2, z22);

        // Make the surface
        doNormal(x1,y1,z21, x1,y2,z22, x2,y1,z21);
        glVertex3f(x1, y1, z21);
        glVertex3f(x1, y2, z22);
        glVertex3f(x2, y1, z21);

        doNormal(x2,y1,z21, x2,y2,z22, x1,y2,z22);
        glVertex3f(x2, y1, z21);
        glVertex3f(x2, y2, z22);
        glVertex3f(x1, y2, z22);
      }else{
        // All on a level
        doNormal(x1,y1,z11, x1,y2,z12, x2,y1,z21);
        glVertex3f(x1, y1, z11);
        glVertex3f(x1, y2, z12);
        glVertex3f(x2, y1, z21);

        doNormal(x2,y1,z21, x2,y2,z22, x1,y2,z12);
        glVertex3f(x2, y1, z21);
        glVertex3f(x2, y2, z22);
        glVertex3f(x1, y2, z12);
      }
      if(y==0){
        // Make a north vertical edge
        doNormal(x1,y1,z11, x2,y1,z21, x1,y1,z12);
        glVertex3f(x1, y1, z11);
        glVertex3f(x2, y1, z11);
        glVertex3f(x1, y1, z12);
        
        doNormal(x1,y1,z12, x2,y1,z22, x2,y1,z21);
        glVertex3f(x1, y1, z12);
        glVertex3f(x2, y1, z22);
        glVertex3f(x2, y1, z21);
      }*/

      for(int x = 1; x < width; x++)
      {
        setColor(x, y);
        GetOpenGLCoord(x, y, elev[x][y], xx, yy, zz, rr);
        x1=xx-rr;
        x2=xx+rr;
        y1=yy-rr;
        y2=yy+rr;
        z11n=z11=y?elev[x-1][y-1]*-rr:0;
        z12n=z12=elev[x-1][y]*-rr;
        z21n=z21=y?elev[x][y-1]*-rr:0;
        z22n=z22=elev[x][y]*-rr;

        /*if(!z11&&z22){
          if(z12){ // y rises
            if(z21){ // x rises
              z11n=(z21n+z12n)/2.0;
              z12n=z22;
              // Make a west vertical edge
              doNormal(x1,y1,0, x1,y2,z12n, x1,y1,z11n);
              glVertex3f(x1, y1, 0);
              glVertex3f(x1, y2, z12n);
              glVertex3f(x1, y1, z11n);
            }
              if(!z11n)z11n=z12;
              // Make a north vertical edge
              doNormal(x1,y1,0, x2,y1,z21n, x1,y1,z11n);
              glVertex3f(x1, y1, 0);
              glVertex3f(x2, y1, z21n);
              glVertex3f(x1, y1, z11n);
          }else if(z21){ // x rises
            z11n=z21;
            z12n=z22;
            // Make a west vertical edge
            doNormal(x1,y1,0, x1,y2,z12n, x1,y1,z11n);
            glVertex3f(x1, y1, 0);
            glVertex3f(x1, y2, z12n);
            glVertex3f(x1, y1, z11n);
          }
        }*/

        if(z11){
          if(!z12&&z21){
            // Make a south vertical edge
            doNormal(x1,y1,z11n, x2,y1,z21n, x1,y1,0);
            glVertex3f(x1, y1, z11n);
            glVertex3f(x2, y1, z21n);
            glVertex3f(x1, y1, 0);
            z11n=0;
          }else if(!z21){
            // Make a east vertical edge
            if(z12n){
            doNormal(x1,y1,z11n, x1,y2,z12n, x1,y1,0);
            glVertex3f(x1, y1, z11n);
            glVertex3f(x1, y2, z12n);
            glVertex3f(x1, y1, 0);
            }
            z11n=0;
          }
        }
        if(!z22){
          if(z21){
            if(z12){
              // Make a east vertical edge
              doNormal(x1,y1,z11n, x1,y2,z12n, x1,y2,0);
              glVertex3f(x1, y1, z11n);
              glVertex3f(x1, y2, z12n);
              glVertex3f(x1, y2, 0);
              z12n=0;
            }
            // Make a south vertical edge
            doNormal(x1,y1,z11n, x2,y1,z21n, x2,y1,z22n);
            glVertex3f(x1, y1, z11n);
            glVertex3f(x2, y1, z21n);
            glVertex3f(x2, y1, z22n);
            z21n=0;
          }else if(z12){
            // Make a east vertical edge
            doNormal(x1,y1,z11n, x1,y2,z12n, x1,y2,z22n);
            glVertex3f(x1, y1, z21n);
            glVertex3f(x1, y2, z12n);
            glVertex3f(x1, y2, z22n);
            z12n=0;
          }
        }

        if(z11&&!z12&&!z21&&!z22&&type[x-1][y]!=kGround){
          doNormal(x1,y1,z11, x1,y1,0, x1,y2,0);
          glVertex3f(x1, y1, z11);
          glVertex3f(x1, y1, 0);
          glVertex3f(x1, y2, 0);
        }

/*
        if(!z21 && z22){
          z21n=z22;
          // Make a north vertical edge
          doNormal(x1,y1,z11n, x2,y1,z21n, x2,y1,0);
          glVertex3f(x1, y1, z11n);
          glVertex3f(x2, y1, z21n);
          glVertex3f(x2, y1, 0);
        }

        if(!z12 && z22){
          z12n=z22;
          // Make a west vertical edge
          doNormal(x1,y1,0, x1,y2,0, x1,y2,z12n);
          glVertex3f(x1, y1, 0);
          glVertex3f(x1, y2, 0);
          glVertex3f(x1, y2, z12n);
        }
*/
        doNormal(x1,y1,z11n, x1,y2,z12n, x2,y1,z21n);
        glVertex3f(x1, y1, z11n);
        glVertex3f(x1, y2, z12n);
        glVertex3f(x2, y1, z21n);

        doNormal(x2,y1,z21n, x2,y2,z22n, x1,y2,z12n);
        glVertex3f(x2, y1, z21n);
        glVertex3f(x2, y2, z22n);
        glVertex3f(x1, y2, z12n);
      }
      /*setColor(width-1, y);
        GetOpenGLCoord(width-1, y, elev[width-1][y], xx, yy, zz, rr);
        x1=xx-rr;
        x2=xx+rr;
        y1=yy-rr;
        y2=yy+rr;
        z11=elev[x-1][y-1]*-rr;
        z12=elev[x-1][y]*-rr;
        z21=elev[x][y-1]*-rr;
        z22=elev[x][y]*-rr;

        doNormal(x1,y1,z11, x1,y2,z12, x2,y1,z21);
        glVertex3f(x1, y1, z11);
        glVertex3f(x1, y2, z12);
        glVertex3f(x2, y1, z21);

        doNormal(x2,y1,z21, x2,y2,z22, x1,y2,z12);
        glVertex3f(x2, y1, z21);
        glVertex3f(x2, y2, z22);
        glVertex3f(x1, y2, z12);

        doNormal(xx-rr, yy-rr, elev[width-1][y]*-rr, xx-rr, yy+rr, 0, xx+rr, yy-rr, 0);
        glVertex3f(xx-rr, yy-rr, elev[width-1][y]*-rr);
        glVertex3f(xx-rr, yy+rr, y<height-1?elev[width-1][y+1]*-rr:0.0f);
        glVertex3f(xx+rr, yy-rr, 0);

        doNormal(xx+rr, yy-rr, 0, xx+rr, yy+rr, 0, xx-rr, yy+rr, 0);
        glVertex3f(xx+rr, yy-rr, 0);
        glVertex3f(xx+rr, yy+rr, 0);
        glVertex3f(xx-rr, yy+rr, y<height-1?elev[width-1][y+1]*-rr:0.0f);
       */
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
  z = _z*-_scale; // Negative because minus approaches the camera
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
  return LineOfSight(Vector3D(x1,y1,z1),Vector3D(x2,y2,z2));
  /*
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
    for(;;){
      if(!Visible(x1, y1, z1)){return false;}
      if(x1==x2 && y1==y2 && z1==z2){return true;}
      if (err_1 > 0) {
        y1 += y_inc;
        err_1 -= dx2;
      }else if (err_2 > 0) {
        z1 += z_inc;
        err_2 -= dx2;
      }else{
        err_1 += dy2;
        err_2 += dz2;
        x1 += x_inc;
      }
    }
  } else if ((m >= l) && (m >= n)) {
    int err_1 = dx2 - m;
    int err_2 = dz2 - m;
    for(;;){
      if(!Visible(x1, y1, z1)){return false;}
      if(x1==x2 && y1==y2 && z1==z2){return true;}
      if (err_1 > 0) {
        x1 += x_inc;
        err_1 -= dy2;
      }else if (err_2 > 0) {
        z1 += z_inc;
        err_2 -= dy2;
      }else{
        err_1 += dx2;
        err_2 += dz2;
        y1 += y_inc;
      }
    }
  } else {
    int err_1 = dy2 - n;
    int err_2 = dx2 - n;
    for(;;){
      if(!Visible(x1, y1, z1)){return false;}
      if(x1==x2 && y1==y2 && z1==z2){return true;}
      if (err_1 > 0) {
        y1 += y_inc;
        err_1 -= dz2;
      } else if (err_2 > 0) {
        x1 += x_inc;
        err_2 -= dz2;
      }else{
        err_1 += dy2;
        err_2 += dx2;
        z1 += z_inc;
      }
    }
  }
  return true;
  */
}

double FRAC(double x) {return x>=0?x-floor(x):1.0-x+floor(x);}

// Using "voxel" implementation from http://www.cse.yorku.ca/~amana/research/grid.pdf
// This accounts for xyz coordinates "off the grid"
bool Map3D::LineOfSight(Vector3D const& a, Vector3D const& b)const{
  // Make sure that permuting the order does not give different results
  Vector3D const& ray_start(a<b?a:b);
  Vector3D const& ray_end(a<b?b:a);
  static double offset(-0.5); // Voxel boundaries actually occur .5 before the center
  int x(round(ray_start.x));
  int y(round(ray_start.y));
  int z(round(ray_start.z));
  int const X(round(ray_end.x));
  int const Y(round(ray_end.y));
  int const Z(round(ray_end.z));
  Vector3D ray=ray_end-ray_start;

  double stepX(ray.x >= 0 ?1.0:-1.0);
  double stepY(ray.y >= 0 ?1.0:-1.0);
  double stepZ(ray.z >= 0 ?1.0:-1.0);

  double tDeltaX(ray.x ? stepX/ray.x : DBL_MAX);
  double tDeltaY(ray.y ? stepY/ray.y : DBL_MAX);
  double tDeltaZ(ray.z ? stepZ/ray.z : DBL_MAX);

  //double tMaxX(ray.x!=0 ? ((x+stepX) - ray_start.x)/ray.x : DBL_MAX);
  //double tMaxY(ray.y!=0 ? ((y+stepY) - ray_start.y)/ray.y : DBL_MAX);
  //double tMaxZ(ray.z!=0 ? ((z+stepZ) - ray_start.z)/ray.z : DBL_MAX);

  double tMaxX(ray.x ? ray.x>0 ? (tDeltaX*(1.0-FRAC(ray_start.x+offset))):(tDeltaX*(FRAC(ray_start.x+offset))) : DBL_MAX);
  double tMaxY(ray.y ? ray.y>0 ? (tDeltaY*(1.0-FRAC(ray_start.y+offset))):(tDeltaY*(FRAC(ray_start.y+offset))) : DBL_MAX);
  double tMaxZ(ray.z ? ray.z>0 ? (tDeltaZ*(1.0-FRAC(ray_start.z+offset))):(tDeltaZ*(FRAC(ray_start.z+offset))) : DBL_MAX);


  while(X!=x || Y!=y || Z!=z){
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        x += stepX;
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    if(!Visible(x,y,z)){return false;}
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
