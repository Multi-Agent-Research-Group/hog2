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

#include "GLUtil.h"

/**
 * A 3D grid-based representation of the world.
 */

class Map3D {
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
	
        inline bool IsTraversable(long x, long y, long z)const{ return x>=0&&x<width&&y>=0&&y<height&&z>=0&&z<depth&&true;} // TODO: check obstacles
	
	void OpenGLDraw(tDisplay how = kPolygons) const;
	bool GetOpenGLCoord(int _x, int _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;
	bool GetOpenGLCoord(float _x, float _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;
	void GetPointFromCoordinate(point3d loc, int &px, int &py) const;
	double GetCoordinateScale();
        bool LineOfSight(int x, int y, int z, int _x, int _y, int _z) const;
	
private:
	int width, height, depth;
        // TODO: Add scene tree
};

#endif
