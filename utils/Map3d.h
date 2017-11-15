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
  public:
  Map3D(long width, long height, long depth);
  Map3D(const char *filename, const char *dtedfile, unsigned md=20);
  Map3D(Map3D *);
  Map3D *Clone() { return new Map3D(this); }
  enum tTerrain {
    kOutOfBounds =0x0, // not part of map
    kOutOfBounds2=0x1, // not part of map - different color
    kWater=0x10,     // water
    kGround=0x20,     // ground
    kSwamp=0x21,
    kGrass=0x22,
    kBlight=0x23,
    kTrees=0x30,
    kUndefined=0x40 // mixed type due to split tile
  };

  enum AgentType {
    surface,ground,air,other
  };
  void Load(const char *filename, const char *dtedfile);
  void loadOctile(FILE *f, float** elevation);
  inline long GetMapWidth() const { return width; }
  inline long GetMapHeight() const { return height; }
  inline long GetMapDepth() const { return depth; }

  inline bool InMap(unsigned x, unsigned y, unsigned z)const{ return x<width&&y<height&&z<depth;} // Relies on unsigned params to check <0

  void doNormal(float x1, float y1, float z1,float x2, float y2, float z2,float x3, float y3, float z3) const;
  void OpenGLDraw() const;
  void setColor(int x, int y) const;
  bool GetOpenGLCoord(float _x, float _y, float _z, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;
  bool GetOpenGLCoord(float _x, float _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const{return GetOpenGLCoord(_x,_y,0,x,y,z,radius);}
  bool LineOfSight2D(int x, int y, int _x, int _y, AgentType agentType) const;
  bool LineOfSight(int x, int y, int z, int _x, int _y, int _z) const;
  void SetGrid(int x, int y, uint8_t elevation, tTerrain terrain){type[x][y]=terrain;elev[x][y]=std::min(maxDepth,elevation);}
  bool CanStep(long x1, long y1, long x2, long y2, AgentType atype) const;
  bool IsTraversable(unsigned x, unsigned y, AgentType atype)const{
    if(!InMap(x,y,0))return false;
    if(atype==surface&&type[x][y]!=kWater)return false;
    if(atype==ground&&type[x][y]!=kGround)return false;
    return elev[x][y]==0;
  }
  bool IsTraversable(unsigned x, unsigned y, unsigned z, AgentType atype)const{
    if(!InMap(x,y,z))return false;
    switch(atype){
      case surface:
        return type[x][y]==kWater && elev[x][y]<=z;
        break;
      case ground:
        return type[x][y]==kGround && elev[x][y]<=z;
        break;
      default:
        return elev[x][y]<z;
        break;
    }
  }
  bool Visible(unsigned x, unsigned y, unsigned z)const{
    return InMap(x,y,z) && elev[x][y]<=z;
      
  }
  bool HasObstacle(unsigned x, unsigned y, unsigned z)const{return !IsTraversable(x,y,z,air);}
  inline tTerrain GetTerrain(int x, int y)const{return type[x][y];}

  private:
  int width, height, depth;
  uint8_t maxDepth;
  uint8_t** elev;
  tTerrain** type;
  mutable GLuint dList;
};

#endif
