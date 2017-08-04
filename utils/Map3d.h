/*
 * $Id: map.h,v 1.20 2007/03/07 22:01:05 nathanst Exp $
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

// HOG File

#ifndef MAP3D_H
#define MAP3D_H

#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <stdint.h>
#include <unordered_map>
#include "GLUtil.h"
#include "MapInterface.h"

/**
 * A 3D grid-based representation of the world.
 */

class Map3D : public MapInterface{
  enum tDisplay {
    kPolygons,
    kLines,
    kPoints
  };
  public:
  Map3D(long width, long height, long depth);
  Map3D(const char *filename);
  Map3D(Map3D *);
  Map3D(FILE *);
  Map3D(std::istringstream &data);
  void Load(const char *filename);
  void Load(FILE *f);
  void Save(std::stringstream &data);
  void Save(const char *filename);
  void Save(FILE *f);
  Map3D *Clone() { return new Map3D(this); }
  /** return the width of the map */
  inline long GetMapWidth() const { return width; }
  /** return the height of the map */
  inline long GetMapHeight() const { return height; }
  inline long GetMapDepth() const { return depth; }

  inline bool InMap(unsigned x, unsigned y, unsigned z)const{ return x<width&&y<height&&z<depth;} // Relies on unsigned params to check <0

  void OpenGLDraw(tDisplay how = kPolygons) const;
  bool GetOpenGLCoord(int _x, int _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const{assert(!"not implemented");}
  bool GetOpenGLCoord(float _x, float _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const{assert(!"not implemented");}
  bool GetOpenGLCoord(int _x, int _y, int _z, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const{return GetOpenGLCoord((float)_x,(float)_y,(float)_z,x,y,z,radius);}
  bool GetOpenGLCoord(float _x, float _y, float _z, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;
  void GetPointFromCoordinate(point3d loc, int &px, int &py) const;
  double GetCoordinateScale();
  bool LineOfSight2D(int x, int y, int _x, int _y) const;
  bool LineOfSight(int x, int y, int z, int _x, int _y, int _z) const;
  void AddObstacle(int x, int y, int z){obstacles[toIndex(x,y,z)]=true;}
  bool IsTraversable(unsigned x, unsigned y, unsigned z)const{return InMap(x,y,z)&&obstacles.find(toIndex(x,y,z))==obstacles.end();}
  bool HasObstacle(unsigned x, unsigned y, unsigned z)const{return !IsTraversable(x,y,z);}

  private:
  uint64_t toIndex(int x, int y, int z)const{
    return x*height*depth+ y*depth+ z;
  }
  void fromIndex(uint64_t index, int& x, int& y, int& z)const{
    z=index%depth;
    y=((index-z)/depth)%height;
    x=((index-y*depth-z)/(height*depth))%width;
  }
  int width, height, depth;
  std::unordered_map<uint64_t,bool> obstacles;
};

#endif
