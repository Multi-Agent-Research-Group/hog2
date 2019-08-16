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
#ifndef VELOCITYOBSTACLE_H
#define VELOCITYOBSTACLE_H

#include "Vector2D.h"
#include "Vector3D.h"
#include "PositionalUtils.h"
//#include <CGAL/Polygon_2.h>
#include <vector>

class VelocityObstacle{
  public:
    //typedef std::vector<Point_2> Points;
    //typedef CGAL::Polygon_2<K> Polygon_2;
    VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, double r1, double r2=DBL_MAX);
    //VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, std::vector<Vector2D>const& polyA, std::vector<Vector2D>const& polyB);
    static bool AgentOverlap(Vector2D const& A,Vector2D const& B,double ar,double br);
    //static bool AgentOverlap(Vector2D const& A,Vector2D const& B,std::vector<Vector2D>const& polyA, std::vector<Vector2D>const& polyB);
    bool IsInside(Vector2D const& point) const;
  private:
    Vector2D VO,VL,VR; // Pos,vel,pos,vel,VO-apex,VO-left,VO-right
};

bool getTangentOfCircle(Vector2D const& center, double radius, Vector2D const& point, std::vector<Vector2D>& tangents);

#endif
