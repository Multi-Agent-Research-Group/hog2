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
#include "BitMap.h"


using namespace std;

/** 
* Create a new map of a particular size.
*
* A map is an array of tiles according to the height and width of the map.
*/
Map3D::Map3D(long _width, long _height, long _depth)
:width(_width), height(_height), depth(_depth)
{
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
        //obstacles = m->obstacles;
}

/** 
* Create a new map by loading it from a file.
*
* Creates a new map and initializes it with the file passed to it.
*/
Map3D::Map3D(const char *filename)
{
	Load(filename);
}

/** 
* Create a new map by loading it from a file pointer.
*
* Creates a new map and initializes it with the file pointer passed to it.
*/
Map3D::Map3D(FILE *f)
{
	Load(f);
}

/** 
* Not implemented.
*
* This function is not implemented.
*/
Map3D::Map3D(std::istringstream &/*data*/)
{
}

/** 
* Resets the current map by loading the file passed in.
*
* Resets the current map by loading the file passed in.
*/
void Map3D::Load(const char *filename)
{
  assert(false && "Not implemented");
}

/** 
* Resets the current map by loading the file from the pointer passed in.
*/
void Map3D::Load(FILE *f)
{
  assert(false && "Not implemented");
}

#include <vector>

/** 
* unimplemented.
*/
void Map3D::Save(std::stringstream &/*data*/) {}

/** 
* Saves the current map out to the designated file.
*
* Saves the current map out to the designated file.
*/
void Map3D::Save(const char *filename)
{
}

/** 
* Saves the current map out to the designated file.
*
* Saves the current map out to the designated file.
*/
void Map3D::Save(FILE *f)
{
}

/**
* Does actual OpenGL drawing of the map
 *
 * If drawLand has been set (on by default) the ground will be drawn using
 * the appropriate mode:   kPolygons, kLines, kPoints
 * kPolygon is the default mode. The map is cached in a display list unless
 * it changes.
 */
void Map3D::OpenGLDraw(tDisplay how) const
{
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

/**
* Returns the scale multiplier between openGL coordinates and map coordinates.
 * If you measure a distance in openGL coordinate space, you can multiply it by
 * this value to convert it to map space, where the distance between adjacent tiles
 * is 1.
 */
double Map3D::GetCoordinateScale()
{
	if (height > width)
		return (double)height/2.0;
	return (double)width/2.0;
}

void Map3D::GetPointFromCoordinate(point3d loc, int &px, int &py) const
{
}
