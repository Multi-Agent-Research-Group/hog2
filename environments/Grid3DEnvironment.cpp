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
#include "Grid3DEnvironment.h"
#include "FPUtil.h"
#include <cstring>
#include "Graphics2D.h"
#include "PositionalUtils.h"

using namespace Graphics2D;

Grid3DEnvironment::Grid3DEnvironment(Map3D *_m,bool w,Map3D::AgentType a):h(nullptr),map(_m),connectedness(0),waitAllowed(w),agentType(a){
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

void Grid3DEnvironment::GetSuccessors(const xyztLoc &loc, std::vector<xyztLoc> &neighbors) const
{
  if(agentType==Map3D::air){
    // TODO add LOS checks
    if(connectedness==0){
      if(map->IsTraversable(loc.x,loc.y,loc.z+1,agentType))
        neighbors.emplace_back(loc.x,loc.y,loc.z+1);
      if(map->IsTraversable(loc.x,loc.y,loc.z-1,agentType))
        neighbors.emplace_back(loc.x,loc.y,loc.z-1);
      if(map->IsTraversable(loc.x,loc.y+1,loc.z,agentType))
        neighbors.emplace_back(loc.x,loc.y+1,loc.z);
      if(map->IsTraversable(loc.x,loc.y-1,loc.z,agentType))
        neighbors.emplace_back(loc.x,loc.y-1,loc.z);
      if(map->IsTraversable(loc.x+1,loc.y,loc.z,agentType))
        neighbors.emplace_back(loc.x+1,loc.y,loc.z);
      if(map->IsTraversable(loc.x-1,loc.y,loc.z,agentType))
        neighbors.emplace_back(loc.x-1,loc.y,loc.z);
      if(waitAllowed)neighbors.emplace_back(loc);
    }else{
      for(int i(-connectedness); i<=connectedness; ++i){
        for(int j(-connectedness); j<=connectedness; ++j){
          for(int k(-connectedness); k<=connectedness; ++k){
            if(!waitAllowed && i==0 && j==0 && k==0)continue;
            if(map->IsTraversable(loc.x+i,loc.y+j,loc.z+k,agentType))
              neighbors.emplace_back(loc.x+i,loc.y+j,loc.z+k);
          }
        }
      }
    }
  }else{
    bool u=false, d=false, /*l=false, r=false,*/ u2=false, d2=false, l2=false, r2=false, /*ur=false, ul=false, dr=false, dl=false,*/ u2l=false, d2l=false, u2r=false, d2r=false, ul2=false, ur2=false, dl2=false, dr2=false, u2r2=false, u2l2=false, d2r2=false, d2l2=false;
    // 
    if ((map->CanStep(loc.x, loc.y, loc.x, loc.y+1,agentType)))
    {
      d = true;
      neighbors.emplace_back(loc.x, loc.y+1);
      if(connectedness>1 && map->IsTraversable(loc.x, loc.y+2,agentType)){
        d2=true;
        if(fullBranching)neighbors.emplace_back(loc.x, loc.y+2);
      }
    }
    if ((map->CanStep(loc.x, loc.y, loc.x, loc.y-1,agentType)))
    {
      u = true;
      neighbors.emplace_back(loc.x, loc.y-1);
      if(connectedness>1 && map->IsTraversable(loc.x, loc.y-2,agentType)){
        u2=true;
        if(fullBranching)neighbors.emplace_back(loc.x, loc.y-2);
      }
    }
    if ((map->CanStep(loc.x, loc.y, loc.x-1, loc.y,agentType)))
    {
      //l=true;
      neighbors.emplace_back(loc.x-1, loc.y);
      if (connectedness>0){
        // Left is open ...
        if(connectedness>1 && map->IsTraversable(loc.x-2, loc.y,agentType)){ // left 2
          l2=true;
          if(fullBranching)neighbors.emplace_back(loc.x-2, loc.y);
        }
        if(u && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y-1,agentType))){
          //ul=true;
          neighbors.emplace_back(loc.x-1, loc.y-1);
          if(connectedness>1){
            // Left, Up, Left2 and UpLeft are open...
            if(l2 && map->IsTraversable(loc.x-2, loc.y-1,agentType)){
              ul2=true;
              neighbors.emplace_back(loc.x-2, loc.y-1);
            }
            // Left, Up2, Up and UpLeft are open...
            if(u2 && map->IsTraversable(loc.x-1, loc.y-2,agentType)){
              u2l=true;
              neighbors.emplace_back(loc.x-1, loc.y-2);
            }
            if(ul2 && u2l && map->IsTraversable(loc.x-2, loc.y-2,agentType)){
              u2l2=true;
              if(fullBranching)neighbors.emplace_back(loc.x-2, loc.y-2);
            }
          }
        }

        if (d && (map->CanStep(loc.x, loc.y, loc.x-1, loc.y+1,agentType))){
          neighbors.emplace_back(loc.x-1, loc.y+1);
          //dl=true;
          if(connectedness>1){
            // Left, Down, Left2 and UpLeft are open...
            if(l2 && map->IsTraversable(loc.x-2, loc.y+1,agentType)){
              dl2=true;
              neighbors.emplace_back(loc.x-2, loc.y+1);
            }
            // Left, Up2, Up and UpLeft are open...
            if(d2 && map->IsTraversable(loc.x-1, loc.y+2,agentType)){
              d2l=true;
              neighbors.emplace_back(loc.x-1, loc.y+2);
            }
            if(dl2 && d2l && map->IsTraversable(loc.x-2, loc.y+2,agentType)){
              d2l2=true;
              if(fullBranching)neighbors.emplace_back(loc.x-2, loc.y+2);
            }
          }
        } // down && downleft
      } // connectedness>5
    } // left

    if ((map->CanStep(loc.x, loc.y, loc.x+1, loc.y,agentType)))
    {
      //r=true;
      neighbors.emplace_back(loc.x+1, loc.y);
      if (connectedness>0){
        // Right is open ...
        if(connectedness>1 && map->IsTraversable(loc.x+2, loc.y,agentType)){ // right 2
          r2=true;
          if(fullBranching)neighbors.emplace_back(loc.x+2, loc.y);
        }
        if(u && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y-1,agentType))){
          //ur=true;
          neighbors.emplace_back(loc.x+1, loc.y-1);
          if(connectedness>1){
            // Right, Up, Right2 and UpRight are open...
            if(r2 && map->IsTraversable(loc.x+2, loc.y-1,agentType)){
              ur2=true;
              neighbors.emplace_back(loc.x+2, loc.y-1);
            }
            // Right, Up2, Up and UpRight are open...
            if(u2 && map->IsTraversable(loc.x+1, loc.y-2,agentType)){
              u2r=true;
              neighbors.emplace_back(loc.x+1, loc.y-2);
            }
            if(ur2 && u2r && map->IsTraversable(loc.x+2, loc.y-2,agentType)){
              u2r2=true;
              if(fullBranching)neighbors.emplace_back(loc.x+2, loc.y-2);
            }
          }
        }

        if (d && (map->CanStep(loc.x, loc.y, loc.x+1, loc.y+1,agentType))){
          //dr=true;
          neighbors.emplace_back(loc.x+1, loc.y+1);
          if(connectedness>1){
            // Right, Down, Right2 and UpRight are open...
            if(r2 && map->IsTraversable(loc.x+2, loc.y+1,agentType)){
              dr2=true;
              neighbors.emplace_back(loc.x+2, loc.y+1);
            }
            // Right, Up2, Up and UpRight are open...
            if(d2 && map->IsTraversable(loc.x+1, loc.y+2,agentType)){
              d2r=true;
              neighbors.emplace_back(loc.x+1, loc.y+2);
            }
            if(dr2 && d2r && map->IsTraversable(loc.x+2, loc.y+2,agentType)){
              d2r2=true;
              if(fullBranching)neighbors.emplace_back(loc.x+2, loc.y+2);
            }
          }
        } // down && downright
      } // connectedness>0
    } // right

    if(connectedness>3){
      if(fullBranching){
        if(d2 && map->IsTraversable(loc.x, loc.y+3,agentType))
          neighbors.emplace_back(loc.x, loc.y+3);
        if(u2 && map->IsTraversable(loc.x, loc.y-3,agentType))
          neighbors.emplace_back(loc.x, loc.y-3);
        if(r2 && map->IsTraversable(loc.x+3, loc.y,agentType))
          neighbors.emplace_back(loc.x+3, loc.y);
        if(l2 && map->IsTraversable(loc.x-3, loc.y,agentType))
          neighbors.emplace_back(loc.x-3, loc.y);
      }

      // ul3
      //if(l2 && map->IsTraversable(loc.x-2, loc.y-1,agentType) && map->IsTraversable(loc.x-3, loc.y-1,agentType))
      if(l2 && ul2 && map->IsTraversable(loc.x-3, loc.y-1,agentType))
        neighbors.emplace_back(loc.x-3, loc.y-1);
      // dl3
      //if(l2 && map->IsTraversable(loc.x-2, loc.y+1,agentType) && map->IsTraversable(loc.x-3, loc.y+1,agentType))
      if(l2 && dl2  && map->IsTraversable(loc.x-3, loc.y+1,agentType))
        neighbors.emplace_back(loc.x-3, loc.y+1);
      // ur3
      //if(r2 && map->IsTraversable(loc.x+2, loc.y-1,agentType) && map->IsTraversable(loc.x+3, loc.y-1,agentType))
      if(r2 && ur2 && map->IsTraversable(loc.x+3, loc.y-1,agentType))
        neighbors.emplace_back(loc.x+3, loc.y-1);
      // dr3
      //if(r2 && map->IsTraversable(loc.x+2, loc.y+1,agentType) && map->IsTraversable(loc.x+3, loc.y+1,agentType))
      if(r2 && dr2 && map->IsTraversable(loc.x+3, loc.y+1,agentType))
        neighbors.emplace_back(loc.x+3, loc.y+1);

      // u3l
      //if(u2 && map->IsTraversable(loc.x-1, loc.y-2,agentType) && map->IsTraversable(loc.x-1, loc.y-3,agentType))
      if(u2 && u2l && map->IsTraversable(loc.x-1, loc.y-3,agentType))
        neighbors.emplace_back(loc.x-1, loc.y-3);
      // d3l
      //if(d2 && map->IsTraversable(loc.x-1, loc.y+2,agentType) && map->IsTraversable(loc.x-1, loc.y+3,agentType))
      if(d2 && d2l && map->IsTraversable(loc.x-1, loc.y+3,agentType))
        neighbors.emplace_back(loc.x-1, loc.y+3);
      // u3r
      //if(u2 && map->IsTraversable(loc.x+1, loc.y-2,agentType) && map->IsTraversable(loc.x+1, loc.y-3,agentType))
      if(u2 && u2r && map->IsTraversable(loc.x+1, loc.y-3,agentType))
        neighbors.emplace_back(loc.x+1, loc.y-3);
      // d3r
      //if(d2 && map->IsTraversable(loc.x+1, loc.y+2,agentType) && map->IsTraversable(loc.x+1, loc.y+3,agentType))
      if(d2 && d2r && map->IsTraversable(loc.x+1, loc.y+3,agentType))
        neighbors.emplace_back(loc.x+1, loc.y+3);

      // u2l3
      if(u2l2 && map->IsTraversable(loc.x-3,loc.y-1,agentType) && map->IsTraversable(loc.x-3, loc.y-2,agentType))
        neighbors.emplace_back(loc.x-3, loc.y-2);
      // d2l3
      if(d2l2 && map->IsTraversable(loc.x-3,loc.y+1,agentType) && map->IsTraversable(loc.x-3, loc.y+2,agentType))
        neighbors.emplace_back(loc.x-3, loc.y+2);
      // u2r3
      if(u2r2 && map->IsTraversable(loc.x+3,loc.y-1,agentType) && map->IsTraversable(loc.x+3, loc.y-2,agentType))
        neighbors.emplace_back(loc.x+3, loc.y-2);
      // d2r3
      if(d2r2 && map->IsTraversable(loc.x+3,loc.y+1,agentType) && map->IsTraversable(loc.x+3, loc.y+2,agentType))
        neighbors.emplace_back(loc.x+3, loc.y+2);

      // u3l2
      if(u2l2 && map->IsTraversable(loc.x-1,loc.y-3,agentType) && map->IsTraversable(loc.x-2, loc.y-3,agentType))
        neighbors.emplace_back(loc.x-2, loc.y-3);
      // d3l2
      if(d2l2 && map->IsTraversable(loc.x-1,loc.y+3,agentType) && map->IsTraversable(loc.x-2, loc.y+3,agentType))
        neighbors.emplace_back(loc.x-2, loc.y+3);
      // u3r2
      if(u2r2 && map->IsTraversable(loc.x+1,loc.y-3,agentType) && map->IsTraversable(loc.x+2, loc.y-3,agentType))
        neighbors.emplace_back(loc.x+2, loc.y-3);
      // d3r2
      if(d2r2 && map->IsTraversable(loc.x+1,loc.y+3,agentType) && map->IsTraversable(loc.x+2, loc.y+3,agentType))
        neighbors.emplace_back(loc.x+2, loc.y+3);

      if(fullBranching){
        // u3l3
        if(u2l2 && map->IsTraversable(loc.x-2,loc.y-3,agentType) && map->IsTraversable(loc.x-3,loc.y-2,agentType) && map->IsTraversable(loc.x-3, loc.y-3,agentType))
          neighbors.emplace_back(loc.x-3, loc.y-3);
        // d3l3
        if(d2l2 && map->IsTraversable(loc.x-2,loc.y+3,agentType) && map->IsTraversable(loc.x-3,loc.y+2,agentType) && map->IsTraversable(loc.x-3, loc.y+3,agentType))
          neighbors.emplace_back(loc.x-3, loc.y+3);
        // u3r3
        if(u2r2 && map->IsTraversable(loc.x+2,loc.y-3,agentType) && map->IsTraversable(loc.x+3,loc.y-2,agentType) && map->IsTraversable(loc.x+3, loc.y-3,agentType))
          neighbors.emplace_back(loc.x+3, loc.y-3);
        // d3r3
        if(d2r2 && map->IsTraversable(loc.x+2,loc.y+3,agentType) && map->IsTraversable(loc.x+3,loc.y+2,agentType) && map->IsTraversable(loc.x+3, loc.y+3,agentType))
          neighbors.emplace_back(loc.x+3, loc.y+3);
      }
    }
    if(waitAllowed) // Is waiting allowed?
    {
      neighbors.push_back(loc);
    }
  }
}

void Grid3DEnvironment::GetActions(const xyztLoc &loc, std::vector<t3DDirection> &actions) const
{
// TODO: Implement this
 assert(false && "Not implemented");
}

t3DDirection Grid3DEnvironment::GetAction(const xyztLoc &s1, const xyztLoc &s2) const
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

void Grid3DEnvironment::ApplyAction(xyztLoc &s, t3DDirection dir) const
{
// TODO
 assert(false && "Not implemented");
}

double Grid3DEnvironment::_h4(unsigned dx, unsigned dy, double result){
  return dx+dy+result;
}

double Grid3DEnvironment::h4(const xyztLoc &l1, const xyztLoc &l2){
  return _h4(abs(l1.x-l2.x),abs(l1.y-l2.y));
}

double Grid3DEnvironment::_h6(unsigned dx, unsigned dy, unsigned dz, double result){
  return dx+dy+dz+result;
}

double Grid3DEnvironment::h6(const xyztLoc &l1, const xyztLoc &l2){
  return _h6(abs(l1.x-l2.x),abs(l1.y-l2.y),abs(l1.z-l2.z));
}

double Grid3DEnvironment::_h8(unsigned dx,unsigned dy,double result){
  static double const SQRT_2(std::sqrt(2.0));
  if(dx>dy){ // Swap
    unsigned tmp(dx); dx=dy; dy=tmp;
  }
  if(dy>=dx){
    result += dx*SQRT_2;
      dy -= dx;
      dx = 0;
  }
  return _h4(dy,dx,result);
}

double Grid3DEnvironment::h8(const xyztLoc &l1, const xyztLoc &l2){
  return _h8(abs(l1.x-l2.x),abs(l1.y-l2.y));
}

double Grid3DEnvironment::_h26(unsigned dx,unsigned dy,unsigned dz,double result){
  static double const SQRT_3(std::sqrt(3.0));
  if(dx==0)return _h8(dy,dz,result);
  if(dy==0)return _h8(dx,dz,result);
  if(dz==0)return _h8(dx,dy,result);

  if(dx>dy){ // Swap
    unsigned tmp(dx); dx=dy; dy=tmp;
  }
  if(dz>dy){ // Swap
    unsigned tmp(dz); dz=dy; dy=tmp;
  }
  if(dx>dz){ // Swap
    unsigned tmp(dx); dx=dz; dz=tmp;
  }
  
  result += dx*SQRT_3;
  dy -= dx;
  dz -= dx;
  dx = 0;
  return _h8(dy,dz,result);
}

double Grid3DEnvironment::h26(const xyztLoc &l1, const xyztLoc &l2){
  return _h26(abs(l1.x-l2.x),abs(l1.y-l2.y),abs(l1.z-l2.z));
}

double Grid3DEnvironment::_h24(unsigned dx,unsigned dy,double result){
  static double const SQRT_5(sqrt(5));
  if(dx>dy){ // Swap
    unsigned tmp(dx); dx=dy; dy=tmp;
  }
  if(dy>1){
    unsigned diff(dy-dx);
    if(diff >=1){
      unsigned s5=std::min(diff,dx);
      result += s5*SQRT_5;
      dy -= s5*2; //up/down 2
      dx -= s5; //over 1
    }
  }
  return _h8(dy,dx,result);
}

double Grid3DEnvironment::h24(const xyztLoc &l1, const xyztLoc &l2){
  return _h24(abs(l1.x-l2.x),abs(l1.y-l2.y));
}

double Grid3DEnvironment::_h48(unsigned dx,unsigned dy,double result){
  static double const SQRT_2(std::sqrt(2.0));
  static double const SQRT_5(std::sqrt(5.0));
  static double const SQRT_10(sqrt(10));
  static double const SQRT_13(sqrt(13));
  if(dx==dy) return dx*SQRT_2;
  if(dx>dy){ // Swap
    unsigned tmp(dx); dx=dy; dy=tmp;
  }
  if(2*dx==dy) return dx*SQRT_5;
  if(3*dx==dy) return dx*SQRT_10;
  if(3*dx==2*dy) return (dx/2)*SQRT_13;
  unsigned steps(dy/3);
  unsigned ss2(dy/2);
  if(steps>=dx){
    //std::cout << dx << " " << dy << " " << "case 1\n";
    result += SQRT_10*dx;
    dy-=dx*3;
    dx=0;
  }else if(dx>2*steps){
    //std::cout << steps << "-" << (dx%steps) << "+" << (dy%steps) << " " << dx << " " << dy << " " << "case 2\n";
    unsigned ddx(dx-2*steps);
    unsigned ddy(dy-3*steps);
    if(ddx>ddy)
      steps-=ddx-ddy;
    result += steps*SQRT_13;
    dx-=2*steps;
    dy-=steps*3;
  }else if(steps<dx && ss2>=dx){
    //std::cout << dx << " " << dy << " " << "case n\n";
    unsigned ddx(dx-steps);
    unsigned ddy(dy-3*steps);
    steps-=(2*ddx-ddy);
    result += steps*SQRT_10;
    dx-=steps;
    dy-=steps*3;
  }else if(ss2<dx && 2*steps>=dx){
    //std::cout << dx << " " << dy << " " << "case p\n";
    unsigned ddx(2*steps-dx);
    unsigned ddy(dy-3*steps);
    steps-=(2*ddx+ddy);
    result += steps*SQRT_13;
    dx-=steps*2;
    dy-=steps*3;
  }
  return _h24(dy,dx,result);
}

double Grid3DEnvironment::h48(const xyztLoc &l1, const xyztLoc &l2){
  return _h48(abs(l1.x-l2.x),abs(l1.y-l2.y));
}

double Grid3DEnvironment::_h124(unsigned dx,unsigned dy,unsigned dz,double result){
  static double const SQRT_3(sqrt(3));
  static double const SQRT_6(sqrt(6));
  static double const SQRT_9(3.0);
  if(dx==0)return _h24(dy,dz,result);
  if(dy==0)return _h24(dx,dz,result);
  if(dz==0)return _h24(dx,dy,result);
  if(dx==dy&&dx==dz)return result+dx*SQRT_3;

  if(dx>dy){ // Swap
    unsigned tmp(dx); dx=dy; dy=tmp;
  }
  if(dz>dy){ // Swap
    unsigned tmp(dz); dz=dy; dy=tmp;
  }
  if(dx>dz){ // Swap
    unsigned tmp(dx); dx=dz; dz=tmp;
  }

  if(dy>dx){
    if(dz>dx){
      unsigned diff(dz-dx);
      if(diff >=1){
        unsigned s9=std::min(diff,dx);
        result += s9*SQRT_9;
        dy -= s9*2; //move 2
        dz -= s9*2; //move 2
        dx -= s9; //over 1
      }
    }
    if(dy>dz){
      unsigned diff(dy-dz);
      if(diff >=1){
        unsigned s6=std::min(diff,dx);
        result += s6*SQRT_6;
        dy -= s6*2; //move 2
        dz -= s6*2; //move 2
        dx -= s6; //over 1
      }
    }
  }
  return _h26(dy,dz,dx,result);
}

double Grid3DEnvironment::h124(const xyztLoc &l1, const xyztLoc &l2){
  return _h124(abs(l1.x-l2.x),abs(l1.y-l2.y),abs(l1.z-l2.z));
}


double Grid3DEnvironment::HCost(const xyztLoc &l1, const xyztLoc &l2)const{
  //if(l1.sameLoc(l2))return 1.0;
  switch(connectedness){
    case 0:
      return h6(l1,l2);
    case 1:
      return h26(l1,l2);
    case 2:
      return h124(l1,l2);
    case 3:
      return h48(l1,l2);
    default:
      return Util::distance(l1.x,l1.y,l1.z,l2.x,l2.y,l2.z);
  }
}

double Grid3DEnvironment::GCost(const xyztLoc &l, const t3DDirection &act) const
{
  assert(false&&"Not implemented");
  return 0;
}

double Grid3DEnvironment::GCost(const xyztLoc &l1, const xyztLoc &l2) const
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
    case 0:{return l1.sameLoc(getGoal()) && l1.sameLoc(l2)?0:multiplier;}
    case 1:{
             unsigned v(abs(l1.x-l2.x)+abs(l1.y-l2.y)+abs(l1.z-l2.z));
             switch(v){
               case 0: return l1.sameLoc(getGoal()) && l1.sameLoc(l2)?0:multiplier;
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
             }else{return l1.sameLoc(getGoal()) && l1.sameLoc(l2)?0:multiplier;}
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
             }else{return l1.sameLoc(getGoal()) && l1.sameLoc(l2)?0:multiplier;}
           }
  }
}

bool Grid3DEnvironment::LineOfSight(const xyztLoc &node, const xyztLoc &goal) const{
  return map->LineOfSight(node.x,node.y,node.z,goal.x,goal.y,goal.z);
}

bool Grid3DEnvironment::GoalTest(const xyztLoc &node, const xyztLoc &goal) const
{
	return ((node.x == goal.x) && (node.y == goal.y));
}

uint64_t Grid3DEnvironment::GetMaxHash() const
{
	return map->GetMapWidth()*map->GetMapHeight();
}

uint64_t Grid3DEnvironment::GetStateHash(const xyztLoc &node) const
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

void Grid3DEnvironment::OpenGLDraw(const xyztLoc &l) const
{
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(l.x, l.y, l.z, xx, yy, zz, rad);
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	//glColor3f(0.5, 0.5, 0.5);
	DrawSphere(xx, yy, zz, rad);
}

void Grid3DEnvironment::OpenGLDraw(const xyztLoc &l1, const xyztLoc &l2, float v) const
{
	GLdouble xx, yy, zz, rad;
	GLdouble xx2, yy2, zz2;
//	map->GetOpenGLCoord((float)((1-v)*l1.x+v*l2.x),
//						(float)((1-v)*l1.y+v*l2.y), xx, yy, zz, rad);
//	printf("%f between (%d, %d) and (%d, %d)\n", v, l1.x, l1.y, l2.x, l2.y);
	map->GetOpenGLCoord(l1.x, l1.y, l1.z, xx, yy, zz, rad);
	map->GetOpenGLCoord(l2.x, l2.y, l2.z, xx2, yy2, zz2, rad);
	//	map->GetOpenGLCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
	xx = (1-v)*xx+v*xx2;
	yy = (1-v)*yy+v*yy2;
	zz = (1-v)*zz+v*zz2;
	GLfloat r, g, b, t;
	GetColor(r, g, b, t);
	glColor4f(r, g, b, t);
	DrawSphere(xx, yy, zz, rad);
}

//void Grid3DEnvironment::OpenGLDraw(const xyztLoc &l, GLfloat r, GLfloat g, GLfloat b) const
//{
//	GLdouble xx, yy, zz, rad;
//	map->GetOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
//	glColor3f(r,g,b);
//	DrawSphere(xx, yy, zz, rad);
//}


void Grid3DEnvironment::OpenGLDraw(const xyztLoc& initial, const t3DDirection &dir) const
{
        assert(false&&"not implemented");
	
	xyztLoc s = initial;
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	
	glColor3f(0.5, 0.5, 0.5);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
		

	
	map->GetOpenGLCoord(s.x, s.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-rad/2);
	glEnd();
	
}

void Grid3DEnvironment::GLDrawLine(const xyztLoc &a, const xyztLoc &b) const
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

void Grid3DEnvironment::GLLabelState(const xyztLoc &s, const char *str, double scale) const
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

void Grid3DEnvironment::GLLabelState(const xyztLoc &s, const char *str) const
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

void Grid3DEnvironment::Draw(const xyztLoc &l) const
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

void Grid3DEnvironment::DrawLine(const xyztLoc &a, const xyztLoc &b, double width) const
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



double Grid3DEnvironment::GetPathLength(std::vector<xyztLoc> &neighbors)
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
