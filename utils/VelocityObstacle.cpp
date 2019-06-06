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
#include "VelocityObstacle.h"
#include "Timer.h"
#include <assert.h>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include "string.h"
//#include <CGAL/Boolean_set_operations_2.h>
//#include <CGAL/Polygon_2.h>

VelocityObstacle::VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, double r1, double r2)
  : VO(a+vb), VL(0,0), VR(0,0)
{
  if(r2==DBL_MAX){r2=r1;}
  // Compute VL and VR
  std::vector<Vector2D> tangents;
  if(getTangentOfCircle(b+vb,r1+r2,VO,tangents)){
    VL=tangents[1];
    VR=tangents[0];
  }else{ // Punt. This should *never* happen
    VL=normal(VO,b+vb);
    VR=-VL;
  }
}

// Polygon points are relative to the center fo the body (a, b respectively)
/*VelocityObstacle::VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, std::vector<Vector2D>const& polyA, std::vector<Vector2D>const& polyB)
  : VO(a+vb), VL(0,0), VR(0,0)
{
  // Compute points in minkowski sum
  std::vector<Vector2D> points;
  points.reserve(polyA.size()*polyB.size()+1);

  Vector2D center(b+vb);
  for(auto const& pa:polyA){
    for(auto const& pb:polyB){
      points.push_back(center+pa+pb);
    }
  }
  
  std::vector<Vector2D> result;
  points.push_back(VO); // Add apex
  Util::convexHull(points,result);

  // Compute VL and VR
  int i=0;
  // Find the apex in the convex hull
  for(; i<result.size(); ++i){
    if(result[i] == VO){
      break;
    }
  }
  // VL and VR are on either side of the apex
  if(i==0){
    VL=result.back();
    VR=result[1];
  }
  else if(i==result.size()-1){
    VL=result[result.size()-2];
    VR=result.front();
  }else{
    VL=result[i-1];
    VR=result[i+1];
  }

  // If the center point is not between the tangents, swap them
  if(!IsInside(center)){
    Vector2D tmp(VL);
    VR=tmp;
    VL=VR;
  }
}*/

bool getTangentOfCircle(Vector2D const& center, double radius, Vector2D const& point, std::vector<Vector2D>& tangents){
  assert(fless(0,radius)); //If the agent has no size, there's no VOB to compute
 
  Vector2D n((point-center)/radius);
  double xy=n.sq();

  if(fleq(xy,1.0)){return false;} // point is in or on circle

  tangents.resize(2);
  double d(n.y*sqrt(xy-1.0));
  double tx0((n.x-d)/xy);
  double tx1((n.x+d)/xy);
  if(!fequal(0.0,n.y)){
    tangents[0].y=center.y+radius*(1.0-tx0*n.x)/n.y;
    tangents[1].y=center.y+radius*(1.0-tx1*n.x)/n.y;
  }else{
    d=radius*sqrt(1.0-tx0*tx0);
    tangents[0].y=center.y+d;
    tangents[1].y=center.y-d;
  }
  tangents[0].x=center.x+radius*tx0;
  tangents[1].x=center.x+radius*tx1;
  return true;
}

// Input is the two center points and their radiuses
/*bool VelocityObstacle::AgentOverlap(Vector2D const& A,Vector2D const& B,std::vector<Vector2D>const& polyA,std::vector<Vector2D>const& polyB){
  Points a;
  a.reserve(polyA.size());
  for(auto const& aa:polyA){
    a.push_back(A+aa);
  }
  Points b;
  b.reserve(polyB.size());
  for(auto const& bb:polyB){
    b.push_back(B+bb);
  }
  return CGAL::do_intersect(Polygon_2(a.begin(),a.end()),Polygon_2(b.begin(),b.end()));
}*/

// Input is the two center points and their radiuses
bool VelocityObstacle::AgentOverlap(Vector2D const& A,Vector2D const& B,double ar,double br){
  return fless((B-A).sq(),(ar+br)*(ar+br));
}

// Input should be a point relative to A after a normalized time step.
// e.g. VOB velocities are normalized, so the input point should be normalized
// from: http://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
bool VelocityObstacle::IsInside(Vector2D const& point) const{
     Vector2D apexDiff(point-VO);
     //Check if it is strictly right of the left vector
     return ((VL.x - VO.x)*(apexDiff.y) < (VL.y - VO.y)*(apexDiff.x)) &&
     // Check if it is strictly left of the right vector
       ((VR.x - VO.x)*(apexDiff.y) > (VR.y - VO.y)*(apexDiff.x));
}

/*bool detectCollisionPolygonalAgents(Vector2D A, Vector2D const& VA, std::vector<Vector2D>const& polyA, double startTimeA, double endTimeA,
Vector2D B, Vector2D const& VB, std::vector<Vector2D>const& polyB, double startTimeB, double endTimeB){

  // check for time overlap
  if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return false;}

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }

  // Check for immediate collision
  if(VelocityObstacle::AgentOverlap(A,B,polyA,polyB)){return true;}

  // Check for collision in future
  if(!VelocityObstacle(A,VA,B,VB,polyA,polyB).IsInside(A+VA)){return false;}
  
  // If we got here, we have established that a collision will occur
  // if the agents continue indefinitely. However, we can now check
  // the end of the overlapping interval to see if the collision is
  // still in the future. If so, no collision has occurred in the interval.
  double duration(std::min(endTimeB,endTimeA)-startTimeA);
  A+=VA*duration;
  B+=VB*duration;
  
  // Check for immediate collision at end of time interval
  if(VelocityObstacle::AgentOverlap(A,B,polyA,polyB)){return true;}

  // Finally, if the collision is still in the future we're ok.
  return !VelocityObstacle(A,VA,B,VB,polyA,polyB).IsInside(A+VA);
}*/

// Detect whether a collision will occur between agent A and B inside the
// given time intervals if they continue at constant velocity with no turns
//
// Inputs: agent A position,
//         agent A velocity vector - normalized to 1 unit time step
//         agent A radius
//         agent A movement start time
//         agent A movement end time
//         same for agent B
//
bool detectCollision(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return false;}

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }

  // Check for immediate collision
  if(VelocityObstacle::AgentOverlap(A,B,radiusA,radiusB)){return true;}

  // Check for collision in future
  if(!VelocityObstacle(A,VA,B,VB,radiusA,radiusB).IsInside(A+VA)){return false;}
  
  // If we got here, we have established that a collision will occur
  // if the agents continue indefinitely. However, we can now check
  // the end of the overlapping interval to see if the collision is
  // still in the future. If so, no collision has occurred in the interval.
  double duration(std::min(endTimeB,endTimeA)-startTimeA);
  A+=VA*duration;
  B+=VB*duration;
  
  // Check for immediate collision at end of time interval
  if(VelocityObstacle::AgentOverlap(A,B,radiusA,radiusB)){return true;}

  // Finally, if the collision is still in the future we're ok.
  return !VelocityObstacle(A,VA,B,VB,radiusA,radiusB).IsInside(A+VA);
}

// Detect whether collision is occurring or will occur between 2 agents
// placed at pi and pj with velocity and radius.
// Return time of collision or zero otherwise
double collisionImminent(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return 0;}

  if(A==B&&VA==VB&&((VA.x==0&&VA.y==0)||(VB.x==0&&VB.y==0))){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return 0;}
    else{return std::max(startTimeA,startTimeB)-TOLERANCE;}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  //if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return 0;}

  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector2D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0.0)){return startTimeA-TOLERANCE;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector2D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return 0; }

  double ctime((b-sqrt(dscr))/a); // Collision time
  if(fless(ctime,0)){ return 0; }

  // Collision will occur if collision time is before the end of the shortest segment
  return fleq(ctime,std::min(endTimeB,endTimeA)-startTimeA)?ctime+startTimeA:0;
}

void get2DSuccessors(Vector2D const& c, std::vector<Vector2D>& s, unsigned conn){
  s.reserve((conn*2+1)*(conn*2+1));
  for(int x(c.x-conn); x<conn*2+1+c.x; ++x)
    for(int y(c.y-conn); y<conn*2+1+c.y; ++y)
      s.push_back(Vector2D(x,y));
}

void get9Coeffs(){
  int p[9][2]={{0,0},{1,0},{2,0},{0,1},{1,1},{2,1},{0,2},{1,2},{2,2}};
  std::unordered_set<unsigned> moves;
  std::unordered_map<unsigned,bool> map;
  for(unsigned a(0); a<9; ++a){
    for(unsigned b(0); b<9; ++b){
      if(abs(p[a][0]-p[b][0])<2 && abs(p[a][1]-p[b][1])<2){
        for(unsigned c(0); c<9; ++c){
          for(unsigned d(0); d<9; ++d){
            if(abs(p[c][0]-p[d][0])<2 && abs(p[c][1]-p[d][1])<2){
              if(std::max(p[a][0],p[b][0])>=std::min(p[c][0],p[d][0]) &&
                  std::max(p[c][0],p[d][0])>=std::min(p[a][0],p[b][0]) &&
                  std::max(p[a][1],p[b][1])>=std::min(p[c][1],p[d][1]) &&
                  std::max(p[c][1],p[d][1])>=std::min(p[a][1],p[b][1])){
                unsigned index(p[a][0]+p[a][1]*3+p[b][0]*9+p[b][1]*27+p[c][0]*81+p[c][1]*243+p[d][0]*729+p[d][1]*2187
                    );
                if(p[a][0]==p[c][0]?p[a][1]==p[c][1]?p[b][0]==p[d][0]?p[b][1]<p[d][1]:p[b][0]<p[d][0]:p[a][1]<p[c][1]:p[a][0]<p[c][0]){
                  moves.insert(index);
                  for(int i(0); i<28; ++i){
                    float t(10);
                    TemporalVector3D a1(p[a][0],p[a][1],0,t+float(i-14)/10);
                    TemporalVector3D a2(p[b][0],p[b][1],0,a1.t + abs(p[a][0]-p[b][0])+abs(p[a][1]-p[b][1])==2?1.4:1.0);
                    TemporalVector3D b1(p[c][0],p[c][1],0,t);
                    TemporalVector3D b2(p[d][0],p[d][1],0,a1.t + abs(p[c][0]-p[d][0])+abs(p[c][1]-p[d][1])==2?1.4:1.0);
                    map[index+i*6561]=collisionCheck3D(a1,a2,b1,b2,.25);
                  }
                }else{
                  index=p[c][0]+p[c][1]*3+p[d][0]*9+p[d][1]*27+p[a][0]*81+p[a][1]*243+p[b][0]*729+p[b][1]*2187;
                  moves.insert(index);
                  for(int i(0); i<28; ++i){
                    float t(10);
                    TemporalVector3D a1(p[a][0],p[a][1],0,t+float(i-14)/10);
                    TemporalVector3D a2(p[b][0],p[b][1],0,a1.t + abs(p[a][0]-p[b][0])+abs(p[a][1]-p[b][1])==2?1.4:1.0);
                    TemporalVector3D b1(p[c][0],p[c][1],0,t);
                    TemporalVector3D b2(p[d][0],p[d][1],0,a1.t + abs(p[c][0]-p[d][0])+abs(p[c][1]-p[d][1])==2?1.4:1.0);
                    map[index+i*6561]=collisionCheck3D(a1,a2,b1,b2,.25);
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  for(int i=0; i<10000; ++i){
    Vector2D a1(rand()%3,rand()%3);
    Vector2D a2(rand()%3-1,rand()%3-1);
    Vector2D b1(rand()%3,rand()%3);
    Vector2D b2(rand()%3-1,rand()%3-1);
    if(collisionImminent(a1,a2,.25,0,a2.x+a2.y==2?1.4:1.0,b1,b2,.25,0,b2.x+b2.y==2?1.4:1.0)){
      Vector2D a3=a1+a2;
      Vector2D b3=b1+b2;
      if(a3.x>2 || a3.x<0 || a3.y>2 || a3.y<0 || b3.x>2 || b3.x<0 || b3.y>2 || b3.y<0) continue;
      Vector2D a(a1);
      Vector2D b(a3);
      Vector2D c(b1);
      Vector2D d(b3);
      //std::cout << "Collision, " << a1 << "-->" << a3 << "x" << b1 << "-->" << b3 << "\n";
      /*int xmin=std::min(std::min(a1.x,a3.x),std::min(b1.x,b3.x));
      int ymin=std::min(std::min(a1.y,a3.y),std::min(b1.y,b3.y));
      Vector2D a(a1.x-xmin,a1.y-ymin);
      Vector2D b(a3.x-xmin,a3.y-ymin);
      Vector2D c(b1.x-xmin,b1.y-ymin);
      Vector2D d(b3.x-xmin,b3.y-ymin);
      std::cout << "Shifted, " << a << "-->" << b << "x" << c << "-->" << d << "\n";
      if(a.x+a.y>2 || b.x+b.y>2 || c.x+c.y>2 || d.x+d.y>2){
        if(a.x+a.y>2){
          if(a.y==2){ // mirror vertical
            if(a.y==2)a.y=0;
            else if(a.y==0)a.y=2;
            if(b.y==2)b.y=0;
            else if(b.y==0)b.y=2;
            if(c.y==2)c.y=0;
            else if(c.y==0)c.y=2;
            if(d.y==2)d.y=0;
            else if(d.y==0)d.y=2;
          }else{
            if(a.x==2)a.x=0;
            else if(a.x==0)a.x=2;
            if(b.x==2)b.x=0;
            else if(b.x==0)b.x=2;
            if(c.x==2)c.x=0;
            else if(c.x==0)c.x=2;
            if(d.x==2)d.x=0;
            else if(d.x==0)d.x=2;
          }
        }
        else if(b.x+b.y>2){
          if(b.y==2){ // mirror vertical
            if(a.y==2)a.y=0;
            else if(a.y==0)a.y=2;
            if(b.y==2)b.y=0;
            else if(b.y==0)b.y=2;
            if(c.y==2)c.y=0;
            else if(c.y==0)c.y=2;
            if(d.y==2)d.y=0;
            else if(d.y==0)d.y=2;
          }else{
            if(a.x==2)a.x=0;
            else if(a.x==0)a.x=2;
            if(b.x==2)b.x=0;
            else if(b.x==0)b.x=2;
            if(c.x==2)c.x=0;
            else if(c.x==0)c.x=2;
            if(d.x==2)d.x=0;
            else if(d.x==0)d.x=2;
          }
        }
        else if(c.x+c.y>2){
          if(c.y==2){ // mirror vertical
            if(a.y==2)a.y=0;
            else if(a.y==0)a.y=2;
            if(b.y==2)b.y=0;
            else if(b.y==0)b.y=2;
            if(c.y==2)c.y=0;
            else if(c.y==0)c.y=2;
            if(d.y==2)d.y=0;
            else if(d.y==0)d.y=2;
          }else{
            if(a.x==2)a.x=0;
            else if(a.x==0)a.x=2;
            if(b.x==2)b.x=0;
            else if(b.x==0)b.x=2;
            if(c.x==2)c.x=0;
            else if(c.x==0)c.x=2;
            if(d.x==2)d.x=0;
            else if(d.x==0)d.x=2;
          }
        }
        else if(d.x+d.y>2){
          if(d.y==2){ // mirror vertical
            if(a.y==2)a.y=0;
            else if(a.y==0)a.y=2;
            if(b.y==2)b.y=0;
            else if(b.y==0)b.y=2;
            if(c.y==2)c.y=0;
            else if(c.y==0)c.y=2;
            if(d.y==2)d.y=0;
            else if(d.y==0)d.y=2;
          }else{
            if(a.x==2)a.x=0;
            else if(a.x==0)a.x=2;
            if(b.x==2)b.x=0;
            else if(b.x==0)b.x=2;
            if(c.x==2)c.x=0;
            else if(c.x==0)c.x=2;
            if(d.x==2)d.x=0;
            else if(d.x==0)d.x=2;
          }
        }
        std::cout << "Mirrored, " << a << "-->" << b << "x" << c << "-->" << d << "\n";
      }*/
      if(a.x==c.x?a.y==c.y?b.x==d.x?b.y<d.y:b.x<d.x:a.y<c.y:a.x<c.x){
        if(moves.find(a.x+a.y*3+b.x*9+b.y*27+c.x*81+c.y*243+d.x*729+d.y*2187)==moves.end()){
          std::cout << "Error, " << a << "-->" << b << "x" << c << "-->" << d << "==" << (a.x+a.y*3+b.x*9+b.y*27+c.x*81+c.y*243+d.x*729+d.y*2187) << " not found\n";
          assert(false);
        }
      }else{
        if(moves.find(c.x+c.y*3+d.x*9+d.y*27+a.x*81+a.y*243+b.x*729+b.y*2187)==moves.end()){
          std::cout << "Error, " << c << "-->" << d << "x" << a << "-->" << b << "==" << (c.x+c.y*3+d.x*9+d.y*27+a.x*81+a.y*243+b.x*729+b.y*2187) << " not found\n";
          assert(false);
        }
      }
    }
  }
  TemporalVector3D tests[10000][4];
  for(int i=0; i<10000; ++i){
    unsigned t(rand()%200+14);
    tests[i][0] = TemporalVector3D(double(rand()%3),double(rand()%3),0,float(t)/10.0);
    tests[i][1] = TemporalVector3D(double(rand()%3),double(rand()%3),0,0);
    unsigned plus(abs(tests[i][1].x-tests[i][0].x)+abs(tests[i][1].y-tests[i][0].y)==2?14:10);
    tests[i][1].t = tests[i][0].t + float(plus)/10.0;
    tests[i][2] = TemporalVector3D(double(rand()%3),double(rand()%3),0,tests[i][0].t+float(rand()%(plus*2)-plus)/10.0);
    tests[i][3] = TemporalVector3D(double(rand()%3),double(rand()%3),0,0);
    plus = abs(tests[i][3].x-tests[i][2].x)+abs(tests[i][3].y-tests[i][2].y)==2?14:10;
    tests[i][3].t = tests[i][2].t + float(plus)/10.0;
  }
  Timer t;
  t.StartTimer();
  for(int i=0; i<10000; ++i){
    collisionCheck3D(tests[i][0],tests[i][1],tests[i][2],tests[i][3],.25);
  }
  std::cout << "Full computation took: " << t.EndTimer() << std::endl;
  std::unordered_map<unsigned,bool> fixedLookup;
  std::unordered_map<unsigned,double*> dynamicLookup;
  
  Timer t2;
  t2.StartTimer();
  for(int i=0; i<10000; ++i){
    unsigned mx(std::min(std::min(tests[i][0].x,tests[i][1].x),std::min(tests[i][2].x,tests[i][3].x)));
    unsigned my(std::min(std::min(tests[i][0].y,tests[i][1].y),std::min(tests[i][2].y,tests[i][3].y)));
    unsigned x1(tests[i][0].x-mx);
    unsigned y1(tests[i][0].y-my);
    unsigned x2(tests[i][1].x-mx);
    unsigned y2(tests[i][1].y-my);
    unsigned x3(tests[i][2].x-mx);
    unsigned y3(tests[i][2].y-my);
    unsigned x4(tests[i][3].x-mx);
    unsigned y4(tests[i][3].y-my);
    if(x1==x3?y1==y3?x2==x4?y2<y4:x2<x4:y1<y3:x1<x3){
      unsigned index(x1+y1*3+x2*9+y2*27+x3*81+y3*243+x4*729+y4*2187+6561*((tests[i][2].t-tests[i][0].t)*10+14));
      int cc=map[index];
    }else{
      unsigned index(x1+y1*3+x2*9+y2*27+x3*81+y3*243+x4*729+y4*2187+6561*((tests[i][2].t-tests[i][0].t)*10+14));
      int cc=map[index];
    }
    //collisionCheckAPriori(tests[i][0],tests[i][1],.25,0,tests[i][1].x+tests[i][1].y==2?1.4:1.0,tests[i][2],tests[i][3],.25,0,tests[i][3].x+tests[i][3].y==2?1.4:1.0);
  }
  std::cout << "Cache computation took: " << t2.EndTimer() << std::endl;
  //std::cout << "There are " << moves.size() << " unique combinations\n";
}

void get16Coeffs(){
  const signed moves[17][2]={{-1,-1},{-2,-1},{-1,0},{-2,1},{-1,1},{-1,2},{0,1},{1,2},{1,1},{2,1},{1,0},{2,-1},{1,-1},{1,-2},{0,-1},{-1,-2},{0,0}};
  int p[25][2]={{0,0},{1,0},{2,0},{3,0},{4,0},{0,1},{1,1},{2,1},{3,1},{4,1},{0,2},{1,2},{2,2},{3,2},{4,2},{0,3},{1,3},{2,3},{3,3},{4,3},{0,4},{1,4},{2,4},{3,4},{4,4}};
  std::unordered_set<unsigned> s;
  for(unsigned a(0); a<25; ++a){
    for(unsigned m(0); m<17; ++m){
      signed x1=p[a][0]+moves[m][0];
      signed y1=p[a][1]+moves[m][1];
      if(x1>=0 && y1>=0 && x1<5 && y1<5){
        for(unsigned b(0); b<25; ++b){
          for(unsigned n(0); n<17; ++n){
            signed x2=p[b][0]+moves[n][0];
            signed y2=p[b][1]+moves[n][1];
            if(x2>=0 && y2>=0 && x2<5 && y2<5){
              if(std::max(p[a][0],x1)>=std::min(p[b][0],x2) &&
                  std::max(p[b][0],x2)>=std::min(p[a][0],x1) &&
                  std::max(p[a][1],y1)>=std::min(p[b][1],y2) &&
                  std::max(p[b][1],y2)>=std::min(p[a][1],y1)){
                if(p[a][0]==p[b][0]?p[a][1]==p[b][1]?x1==x2?y1<y2:x1<x2:p[a][1]<p[b][1]:p[a][0]<p[b][0])
                  s.insert(p[a][0]+p[a][1]*3+x1*9+y1*27+p[b][0]*81+p[b][1]*243+x2*729+y2*2187);
                else
                  s.insert(p[b][0]+p[b][1]*3+x2*9+y2*27+p[a][0]*81+p[a][1]*243+x1*729+y1*2187);
              }
            }
          }
        }
      }
    }
  }
  //std::cout << "There are " << s.size() << " unique combinations\n";
}

void get32Coeffs(){
    const signed moves32[33][2]={{-1,-1},{-3,-2},{-2,-1},{-3,-1},{-1,0},{-3,1},{-2,1},{-3,2},{-1,1},{-2,3},{-1,2},{-1,3},{0,1},
      {1,3},{1,2},{2,3},{1,1},{3,2},{2,1},{3,1},{1,0},{3,-1},{2,-1},{3,-2},{1,-1},{2,-3},{1,-2},{1,-3},{0,-1},{-1,-3},{-1,-2},{-2,-3},{0,0}};
}

void get3DSuccessors(Vector3D const& c, std::vector<Vector3D>& s, unsigned conn){
  s.reserve((conn*2+1)*(conn*2+1)*(conn*2+1));
  for(int x(c.x-conn); x<c.x+conn; ++x)
    for(int y(c.y-conn); y<c.y+conn; ++y)
      for(int z(c.z-conn); z<c.z+conn; ++z)
        s.push_back(Vector3D(x,y,z));
}

unsigned index9(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-1;
  d1.y-=s1.y-1;
  // Translate d2 relative to s2
  d2.x-=s2.x-1;
  d2.y-=s2.y-1;
  s2.x-=s1.x-1;
  s2.y-=s1.y-1;
  return d1.y*3+d1.x + 9*(s2.y*3+s2.x) + 81*(d2.y*3+d2.x);
}

unsigned index25(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-2;
  d1.y-=s1.y-2;
  // Translate d2 relative to s2
  d2.x-=s2.x-2;
  d2.y-=s2.y-2;
  s2.x-=s1.x-2;
  s2.y-=s1.y-2;
  return d1.y*5+d1.x + 25*(s2.y*5+s2.x) + 625*(d2.y*5+d2.x);
}

unsigned index49(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-3;
  d1.y-=s1.y-3;
  d2.x-=s2.x-3;
  d2.y-=s2.y-3;
  s2.x-=s1.x-3;
  s2.y-=s1.y-3;
  return d1.y*7+d1.x + 49*(s2.y*7+s2.x) + 2401*(d2.y*7+d2.x);
}

unsigned index93(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
  
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-3;
  d1.y-=s1.y-3;
  d2.x-=s2.x-3;
  d2.y-=s2.y-3;
  s2.x-=s1.x-3;
  s2.y-=s1.y-3;
  return d1.y*7+d1.x + 49*(s2.y*7+s2.x) + 2401*(d2.y*7+d2.x);
}

unsigned index493(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-3;
  d1.y-=s1.y-3;
  d2.x-=s2.x-3;
  d2.y-=s2.y-3;
  s2.x-=s1.x-3;
  s2.y-=s1.y-3;
  return d1.y*7+d1.x + 49*(s2.y*7+s2.x) + 2401*(d2.y*7+d2.x);
}

unsigned index27(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-1;
  d1.y-=s1.y-1;
  d1.z-=s1.z-1;
  d2.x-=s2.x-1;
  d2.y-=s2.y-1;
  d2.z-=s2.z-1;
  s2.x-=s1.x-1;
  s2.y-=s1.y-1;
  s2.z-=s1.z-1;
  return d1.z*9+d1.y*3+d1.x + 27*(s2.z*9+s2.y*3+s2.x) + 729*(d2.z*9+d2.y*3+d2.x);
}

unsigned index125(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-2;
  d1.y-=s1.y-2;
  d1.z-=s1.z-2;
  d2.x-=s2.x-2;
  d2.y-=s2.y-2;
  d2.z-=s2.z-2;
  s2.x-=s1.x-2;
  s2.y-=s1.y-2;
  s2.z-=s1.z-2;
  return d1.z*25+d1.y*5+d1.x + 125*(s2.z*25+s2.y*3+s2.x) + 15625*(d2.z*25+d2.y*5+d2.x);
}


void fillArray(unsigned* bitarray,unsigned (*index)(Vector2D const&,Vector2D, Vector2D,Vector2D),unsigned conn,double radius){
  std::vector<Vector2D> n;
  Vector2D center(6,6);
  get2DSuccessors(center,n,conn);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<Vector2D> p;
      get2DSuccessors(o,p,conn);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,radius,o,q,radius)){
          set(bitarray,(*index)(center,m,o,q));
        }
      }
    }
  }
}

void fillArray(unsigned* bitarray,unsigned (*index)(Vector3D const&,Vector3D, Vector3D,Vector3D),unsigned conn, double radius){
  std::vector<Vector3D> n;
  Vector3D center(6,6,6);
  get3DSuccessors(center,n,conn);
  for(auto const& m:n){
    for(auto const& o:n){
      std::vector<Vector3D> p;
      get3DSuccessors(o,p,conn);
      for(auto const& q:p){
        if(Util::fatLinesIntersect(center,m,radius,o,q,radius)){
          set(bitarray,(*index)(center,m,o,q));
        }
      }
    }
  }
}

void load3DCollisionTable9(){
  char const*fname = "3dcollision9.dat";
  FILE* f(fopen(fname,"rb"));
  if(f){fread(fltarray,sizeof(uint64_t),25*4*14,f);
    fclose(f);
  }else{
    fillArray(index93,3,.25);
    FILE* f=fopen(fname,"wb");
    fwrite(fltarray,sizeof(float),49*48*49,f);
    fclose(f);
  }
}

void load3DCollisionTable(){
  get9Coeffs();
  get16Coeffs();
  char const*fname = "3dcollision.dat";
  FILE* f(fopen(fname,"rb"));
  if(f){fread(fltarray,sizeof(float),49*49*49,f);
    fclose(f);
  }else{
    fillArray(index493,3,.25);
    FILE* f=fopen(fname,"wb");
    fwrite(fltarray,sizeof(float),49*49*49,f);
    fclose(f);
  }
}

void fillArray(unsigned (*index)(Vector3D const&,Vector3D, Vector3D,Vector3D),unsigned conn, double radius){
  std::vector<Vector3D> n;
  std::vector<Vector3D> p;
  Vector3D center(6,6,6);
  get3DSuccessors(center,n,conn);
  for(auto const& m:n){
    for(auto const& o:n){
      p.resize(0);
      get3DSuccessors(o,p,conn);
      for(auto const& q:p){
        fltarray[(*index)(center,m,o,q)]=(float)getCollisionInterval3D(center,m,o,q,radius,radius);
      }
    }
  }
}

// Check for collision between entities moving from A1 to A2 and B1 to B2
// Speed is optional. If provided, should be in grids per unit time; time should also be pre adjusted to reflect speed.
bool collisionCheck(TemporalVector const& A1, TemporalVector const& A2, TemporalVector const& B1, TemporalVector const& B2, double radiusA, double radiusB, double speedA, double speedB){
  unsigned dim(std::max(std::max(fabs(A1.x-A2.x),fabs(B1.x-B2.x)),std::max(fabs(A1.y-A2.y),fabs(B1.y-B2.y))));
  unsigned ssx(fabs(A1.x-B1.x));
  unsigned sdx(fabs(A1.x-B2.x));
  unsigned ssy(fabs(A1.y-B1.y));
  unsigned sdy(fabs(A1.y-B2.y));

  switch(dim){
    case 0:
    case 1:
      if(std::max(radiusA,radiusB)>.25){
        static unsigned bitarray9r_5[9*9*9/WORD_BITS+1];
        if(!bitarray9r_5[0]){
          fillArray(bitarray9r_5,*index9,1,.5);
        }

        if(ssx<2 && ssy<2 && sdx <3 && sdy<3){
          if(!get(bitarray9r_5,index9(A1,A2,B1,B2)))
             return false;
        }else if(sdx<2 && sdy<2 && ssx <3 && ssy<3){
          if(!get(bitarray9r_5,index9(A1,A2,B2,B1)))
             return false;
        }else{return false;}
      }else{
        static unsigned bitarray9r_25[9*9*9/WORD_BITS+1];
        if(!bitarray9r_25[0]){
          fillArray(bitarray9r_25,*index9,1,.25);
        }

        if(ssx<2 && ssy<2 && sdx <3 && sdy<3){
          if(!get(bitarray9r_25,index9(A1,A2,B1,B2)))
             return false;
        }else if(sdx<2 && sdy<2 && ssx <3 && ssy<3){
          if(!get(bitarray9r_25,index9(A1,A2,B2,B1)))
             return false;
        }else{return false;}
      }
      break;
    case 2:
      if(std::max(radiusA,radiusB)>.25){
        static unsigned bitarray25r_5[25*25*25/WORD_BITS+1];
        if(!bitarray25r_5[0]){
          fillArray(bitarray25r_5,*index25,2,.5);
        }

        if(ssx<3 && ssy<3 && sdx <5 && sdy<5){
          if(!get(bitarray25r_5,index25(A1,A2,B1,B2)))
             return false;
        }else if(sdx<3 && sdy<3 && ssx <5 && ssy<5){
          if(!get(bitarray25r_5,index25(A1,A2,B2,B1)))
             return false;
        }else{return false;}
      }else{
        static unsigned bitarray25r_25[25*25*25/WORD_BITS+1];
        if(!bitarray25r_25[0]){
          fillArray(bitarray25r_25,*index25,2,.25);
        }

        if(ssx<3 && ssy<3 && sdx <5 && sdy<5){
          if(!get(bitarray25r_25,index25(A1,A2,B1,B2)))
             return false;
        }else if(sdx<3 && sdy<3 && ssx <5 && ssy<5){
          if(!get(bitarray25r_25,index25(A1,A2,B2,B1)))
             return false;
        }else{return false;}
      }
      break;
    case 3:
      if(std::max(radiusA,radiusB)>.25){
        static unsigned bitarray49r_5[49*49*49/WORD_BITS+1];
        if(!bitarray49r_5[0]){
          fillArray(bitarray49r_5,*index49,3,.5);
        }

        if(ssx<4 && ssy<4 && sdx <7 && sdy<7){
          if(!get(bitarray49r_5,index49(A1,A2,B1,B2)))
             return false;
        }else if(sdx<4 && sdy<4 && ssx <7 && ssy<7){
          if(!get(bitarray49r_5,index49(A1,A2,B2,B1)))
             return false;
        }else{return false;}
      }else{
        static unsigned bitarray49r_25[49*49*49/WORD_BITS+1];
        if(!get(bitarray49r_25,CENTER_IDX49)){
          fillArray(bitarray49r_25,*index49,3,.25);
        }

        if(ssx<4 && ssy<4 && sdx <7 && sdy<7){
          if(!get(bitarray49r_25,index49(A1,A2,B1,B2)))
             return false;
        }else if(sdx<4 && sdy<4 && ssx <7 && ssy<7){
          if(!get(bitarray49r_25,index49(A1,A2,B2,B1)))
             return false;
        }else{return false;}
      }
      break;
    default:
      break;
  };
  Vector2D VA(A2-A1);
  VA.Normalize();
  VA*=speedA;
  Vector2D VB(B2-B1);
  VB.Normalize();
  VB*=speedA;
  return collisionImminent(A1,VA,radiusA,A1.t,A2.t,B1,VB,radiusB?radiusB:radiusA,B1.t,B2.t);
}

double collisionTime(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return 0;}

  if(A==B&&VA==VB&&((VA.x==0&&VA.y==0)||(VB.x==0&&VB.y==0))){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return 0;}
    else{return std::max(startTimeA,startTimeB)-TOLERANCE;}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  //if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return 0;}

  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector3D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0)){return startTimeA-TOLERANCE;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return 0; }

  double ctime((b-sqrt(dscr))/a); // Collision time
  if(fless(ctime,0)){ return 0; }

  // Collision will occur if collision time is before the end of the shortest segment
  return fleq(ctime,std::min(endTimeB,endTimeA)-startTimeA)?ctime+startTimeA:0;
}

bool collisionImminent(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
  // assume time overlap
  //if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return false;}

  if(A==B&&VA==VB&&(VA.x==0&&VA.y==0)){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return false;}
    else{return true;}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  //if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return 0;}

  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector3D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0)){return true;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return false; }

  double ctime((b-sqrt(dscr))/a); // Collision time
  if(fless(ctime,0)){ return false; }

  // Collision will occur if collision time is before the end of the shortest segment
  return fleq(ctime,std::min(endTimeB,endTimeA)-startTimeA);
}


double collisionCheck3DSlow(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA, double radiusB, double speedA, double speedB){
  Vector3D VA(A2-A1);
  VA.Normalize();
  VA*=speedA;
  Vector3D VB(B2-B1);
  VB.Normalize();
  VB*=speedA;
  return collisionImminent(A1,VA,radiusA,A1.t,A2.t,B1,VB,radiusB?radiusB:radiusA,B1.t,B2.t);
}

// Check for collision between entities moving from A1 to A2 and B1 to B2
// Speed is optional. If provided, should be in grids per unit time; time should also be pre adjusted to reflect speed.
double collisionTime3D(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA, double radiusB, double speedA, double speedB){
  unsigned sdx(fabs(A1.x-B2.x));
  unsigned sdy(fabs(A1.y-B2.y));
  unsigned sdz(fabs(A1.z-B2.z));
  // Same edge in reverse?
  if(sdx==0&&sdy==0&&sdz==0&&B1.x==A2.x&&B1.y==A2.y&&B1.z==A2.z){
    double dist(sqrt(distanceSquared(A1,B1))-(A1.t>B1.t?(A1.t-B1.t)*speedA:(B1.t-A1.t)*speedB));
    double radius((radiusA*speedA+radiusB*speedB));
    double speed(speedA+speedB);
    double ctime(std::max(A1.t,B1.t)+(dist-radius)/speed);
    return ctime;
  }

  unsigned ssx(fabs(A1.x-B1.x));
  unsigned ssy(fabs(A1.y-B1.y));
  unsigned ssz(fabs(A1.z-B1.z));
  // Same start? 
  double diam(radiusA+radiusB);
  if(ssx==0 && ssy==0 && ssz==0){
    if(A1.t==B1.t){
      return A1.t;
    }
    double tdist(fgreater(A1.t,B1.t)?speedA*(A1.t-B1.t):speedB*(B1.t-A1.t));
    if(tdist<diam){
      return std::min(A1.t,B1.t);
    }
  }
  //same end?
  if(A2.x==B2.x && A2.y==B2.y && A2.z==B2.z){
    if(A2.t==B2.t){
      return A2.t;
    }
    double tdist(fgreater(A2.t,B2.t)?speedA*(A2.t-B2.t):speedB*(B2.t-A2.t));
    // Distance based on time offset
    if(tdist<diam){
      // collides at min-tdist
      return std::min(A1.t,B1.t)-tdist;
    }
  }


  unsigned dim(std::max(std::max(std::max(fabs(A1.x-A2.x),fabs(B1.x-B2.x)),std::max(fabs(A1.y-A2.y),fabs(B1.y-B2.y))),std::max(fabs(A1.z-A2.z),fabs(B1.z-B2.z))));

  switch(dim){
    case 0:
    case 1:
        if(ssx<2 && ssy<2 && ssz<2 && sdx<3 && sdy<3 && sdz<3){
        }else if(sdx<2 && sdy<2 && sdz<2 && ssx<3 && ssy<3 && ssz<3){
        }else{return 0;}
      break;
    case 2:
        if(ssx<3 && ssy<3 && ssz<3 && sdx<5 && sdy<5 && sdz<5){
        }else if(sdx<3 && sdy<3 && sdz<3 && ssx<5 && ssy<5 && ssz<5){
        }else{return 0;}
      break;
    default:
      break;
  };
  Vector3D VA(A2-A1);
  VA.Normalize();
  VA*=speedA;
  Vector3D VB(B2-B1);
  VB.Normalize();
  VB*=speedA;
  return collisionImminent(A1,VA,radiusA,A1.t,A2.t,B1,VB,radiusB?radiusB:radiusA,B1.t,B2.t);
}

bool collisionCheck3DAPriori(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA, double radiusB){
  float tdiff(A1.t>B1.t?A1.t-B1.t:B1.t-A1.t);
  return tdiff>fltarray[index493(A1,A2,B1,B2)];
}

// Check for collision between entities moving from A1 to A2 and B1 to B2
// Speed is optional. If provided, should be in grids per unit time; time should also be pre adjusted to reflect speed.
bool collisionCheck3D(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA, double radiusB, double speedA, double speedB){
  unsigned sdx(fabs(A1.x-B2.x));
  unsigned sdy(fabs(A1.y-B2.y));
  unsigned sdz(fabs(A1.z-B2.z));
  // Same edge in reverse?
  if(sdx==0&&sdy==0&&sdz==0&&B1.x==A2.x&&B1.y==A2.y&&B1.z==A2.z){
    return true;
  }

  unsigned ssx(fabs(A1.x-B1.x));
  unsigned ssy(fabs(A1.y-B1.y));
  unsigned ssz(fabs(A1.z-B1.z));
  // Same start? 
  double diam(radiusA+radiusB);
  if(ssx==0 && ssy==0 && ssz==0){
    if(A1.t==B1.t){
      return true;
    }
    double tdist(fgreater(A1.t,B1.t)?speedA*(A1.t-B1.t):speedB*(B1.t-A1.t));
    if(tdist<diam){
      return true;
    }
  }
  //same end?
  if(A2.x==B2.x && A2.y==B2.y && A2.z==B2.z){
    if(A2.t==B2.t){
      return true;
    }
    double tdist(fgreater(A2.t,B2.t)?speedA*(A2.t-B2.t):speedB*(B2.t-A2.t));
    // Distance based on time offset
    if(tdist<diam){
      // collides at min-tdist
      return true;
    }
  }


  unsigned dim(std::max(std::max(std::max(fabs(A1.x-A2.x),fabs(B1.x-B2.x)),std::max(fabs(A1.y-A2.y),fabs(B1.y-B2.y))),std::max(fabs(A1.z-A2.z),fabs(B1.z-B2.z))));

  switch(dim){
    case 0:
    case 1:
        if(ssx<2 && ssy<2 && ssz<2 && sdx<3 && sdy<3 && sdz<3){
        }else if(sdx<2 && sdy<2 && sdz<2 && ssx<3 && ssy<3 && ssz<3){
        }else{return false;}
      break;
    case 2:
        if(ssx<3 && ssy<3 && ssz<3 && sdx<5 && sdy<5 && sdz<5){
        }else if(sdx<3 && sdy<3 && sdz<3 && ssx<5 && ssy<5 && ssz<5){
        }else{return false;}
      break;
    default:
      break;
  };
  Vector3D VA(A2-A1);
  VA.Normalize();
  VA*=speedA;
  Vector3D VB(B2-B1);
  VB.Normalize();
  VB*=speedA;
  return collisionImminent(A1,VA,radiusA,A1.t,A2.t,B1,VB,radiusB?radiusB:radiusA,B1.t,B2.t);
}

std::pair<double,double> collisionInterval3D(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA, double radiusB, double speedA, double speedB){
  unsigned sdx(fabs(A1.x-B2.x));
  unsigned sdy(fabs(A1.y-B2.y));
  unsigned sdz(fabs(A1.z-B2.z));
  // Same edge in reverse?
  if(sdx==0&&sdy==0&&sdz==0&&B1.x==A2.x&&B1.y==A2.y&&B1.z==A2.z){
    double dist(sqrt(distanceSquared(A1,B1))-(A1.t>B1.t?(A1.t-B1.t)*speedA:(B1.t-A1.t)*speedB));
    double radius((radiusA*speedA+radiusB*speedB));
    double speed(speedA+speedB);
    double ctime(std::max(A1.t,B1.t)+(dist-radius)/speed);
    return {ctime,ctime+radius/speed};
  }

  // Same start && same end?
  if(A1.x==B1.x && A1.y==B1.y && A1.z==B1.z
      && A2.x==B2.x && A2.y==B2.y && A2.z==B2.z){
    double radius((radiusA*speedA+radiusB*speedB));
    // Distance based on time offset
    double tdist(fgreater(A1.t,B1.t)?speedA*(A1.t-B1.t):speedB*(B1.t-A1.t));
    if(radius > tdist){
      // immediately colliding
      double begin(std::max(A1.t,B1.t));
      double end(std::min(A2.t,B2.t));
      if(fequal(speedA,speedB)){
        return {begin,end}; // Collision continues for entire duration
      }else if(fgreater(speedA,speedB)){ // A is moving faster
        // How much distance is required?
        double reqd(fgreater(A1.t,B1.t)?tdist-radius:tdist+radius);
        end = std::min(end,begin + reqd*(speedA-speedB));
      }else{ // B is moving faster
        double reqd(fless(A1.t,B1.t)?tdist-radius:tdist+radius);
        end = std::min(end,begin + reqd*(speedB-speedA));
      }
      return {begin,end};
    }
    // Determine if collision is going to happen in the future
  }

  unsigned ssx(fabs(A1.x-B1.x));
  unsigned ssy(fabs(A1.y-B1.y));
  unsigned ssz(fabs(A1.z-B1.z));

  unsigned dim(std::max(std::max(std::max(fabs(A1.x-A2.x),fabs(B1.x-B2.x)),std::max(fabs(A1.y-A2.y),fabs(B1.y-B2.y))),std::max(fabs(A1.z-A2.z),fabs(B1.z-B2.z))));

  switch(dim){
    case 0:
    case 1:
        if(ssx<2 && ssy<2 && ssz<2 && sdx<3 && sdy<3 && sdz<3){
        }else if(sdx<2 && sdy<2 && sdz<2 && ssx<3 && ssy<3 && ssz<3){
        }else{return {0,0};}
      break;
    case 2:
        if(ssx<3 && ssy<3 && ssz<3 && sdx<5 && sdy<5 && sdz<5){
        }else if(sdx<3 && sdy<3 && sdz<3 && ssx<5 && ssy<5 && ssz<5){
        }else{return {0,0};}
      break;
    default:
      break;
  };
  Vector3D VA(A2-A1);
  VA.Normalize();
  VA*=speedA;
  Vector3D VB(B2-B1);
  VB.Normalize();
  VB*=speedA;
  return getCollisionInterval(A1,VA,radiusA,A1.t,A2.t,B1,VB,radiusB?radiusB:radiusA,B1.t,B2.t);
}

// Get collision time between two agents - that is the time that they will "start" colliding.
// Agents have a position, velocity, start time, end time and radius.
// Note: this is analogous to an agent traversing an edge from time "start" to "end" at constant velocity.
// -1 is a seminal value meaning "no collision in the future."
double getCollisionTime(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return -1;}

  if(A==B&&VA==VB&&((VA.x==0&&VA.y==0)||(VB.x==0&&VB.y==0))){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return -1;}
    else{return std::max(startTimeA,startTimeB);}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A forward to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return -1;}


  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector2D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0.0)){return startTimeA-TOLERANCE;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector2D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return -1; } // No collision will occur in the future

  return (b-sqrt(dscr))/a + startTimeA; // Absolute collision time
}

double getCollisionTime(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return -1;}

  if(A==B&&VA==VB&&((VA.x==0&&VA.y==0)||(VB.x==0&&VB.y==0))){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return -1;}
    else{return std::max(startTimeA,startTimeB);}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A forward to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return -1;}

  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector3D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0.0)){return startTimeA-TOLERANCE;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return -1; } // No collision will occur in the future

  return (b-sqrt(dscr))/a + startTimeA; // Absolute collision time
}

std::pair<float,float> getForbiddenInterval(Vector2D const& A,
Vector2D const& A2,
Vector2D const& B,
Vector2D const& B2,
double radiusA,
double radiusB){
  Vector2D VA(A2-A);
  Vector2D VB(B2-B);
  Vector2D DAB(B-A);
  // Assume unit speed...
  VA.Normalize();
  VB.Normalize();
  double r(radiusA+radiusB);
  double rsq(r*r);
  Vector2D DVAB(VB-VA);

  // Are they parallel?
  if(fequal(DVAB.x,0.0) && fequal(DVAB.y,0.0)){
    // If they are currently overlapping, they will do so forever, otherwise, they never will
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      double dc(sqrt(DAB.sq()-d)); // Distance between centers along trajectory line
      return {dc-v,dc+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Are they opposing?
  if(VA==-VB){
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Is one agent waiting?
  if(VA.x==0 && VA.y==0){
    if(VB.x==0 && VB.y==0){
      if(DAB.sq()<rsq) return {-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()}; //Overlapping and waiting
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Not overlapping and waiting
    // Compute the interval until agent B passes over agent A
    double d(Util::distanceOfPointToLine(B,B2,A));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }else if(VB.x==0 && VB.y==0){
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }

  double AVA(A*VA);
  double BVA(B*VA);
  double AVB(A*VB);
  double BVB(B*VB);

  double a(DVAB*DVAB); // Dot product
  //b has the effect of rotating the conic
  double b(2.0*(VA*VA - VA*VB));
  double c(VA*VA);
  double d(2.0*(BVB - BVA + AVA - AVB));
  double e(-2.0*BVA+2.0*AVA);
  double f(DAB*DAB - rsq);

  // this conic is guaranteed to be an ellipse because both a and c are positive (they are squares)
  double dscr2(4.0*a*c-b*b);
  // The following equations are from:
  // https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form
  double g(b*d-2.0*a*e);
  double center_y(g/dscr2);
  double g2(2.0*g);
  double delta(sqrt(g2*g2+4.0*dscr2*(d*d-4*a*f))/(2*dscr2));
  // interval for agent A:
  return {-center_y-delta,-center_y+delta};
  // interval for agent B:
  //return {center_y-delta,center_y+delta};
}

std::pair<float,float> getForbiddenInterval(Vector3D const& A,
Vector3D const& A2,
Vector3D const& B,
Vector3D const& B2,
double radiusA,
double radiusB){
  Vector3D VA(A2-A);
  Vector3D VB(B2-B);
  Vector3D DAB(B-A);
  // Assume unit speed...
  VA.Normalize();
  VB.Normalize();
  double r(radiusA+radiusB);
  double rsq(r*r);
  Vector3D DVAB(VB-VA);

  if(A==B && A2==B2){
    return {-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()}; //Overlapping and waiting
  }
  // Are they parallel?
  if(fequal(DVAB.x,0.0) && fequal(DVAB.y,0.0)){
    // If they are currently overlapping, they will do so forever, otherwise, they never will
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      double dc(sqrt(DAB.sq()-d)); // Distance between centers along trajectory line
      return {dc-v,dc+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Are they opposing?
  if(VA==-VB){
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Is one agent waiting?
  if(VA.x==0 && VA.y==0){
    if(VB.x==0 && VB.y==0){
      if(DAB.sq()<rsq) return {-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()}; //Overlapping and waiting
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Not overlapping and waiting
    // Compute the interval until agent B passes over agent A
    double d(Util::distanceOfPointToLine(B,B2,A));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }else if(VB.x==0 && VB.y==0){
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }


  double AVA(A*VA);
  double BVA(B*VA);
  double AVB(A*VB);
  double BVB(B*VB);

  double a(DVAB*DVAB); // Dot product
  //b has the effect of rotating the conic
  double b(2.0*(VA*VA - VA*VB));
  double c(VA*VA);
  double d(2.0*(BVB - BVA + AVA - AVB));
  double e(-2.0*BVA+2.0*AVA);
  double f(DAB*DAB - rsq);

  // this conic is guaranteed to be an ellipse because both a and c are positive (they are squares)
  double dscr2(4.0*a*c-b*b);
  // The following equations are from:
  // https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form
  double g(b*d-2.0*a*e);
  double center_y(g/dscr2);
  double g2(2.0*g);
  double delta(sqrt(g2*g2+4.0*dscr2*(d*d-4*a*f))/(2*dscr2));
  // interval for agent A:
  return {-center_y-delta,-center_y+delta};
  // interval for agent B:
  //return {center_y-delta,center_y+delta};
}

std::pair<float,float> getForbiddenInterval(Vector3D const& A,
Vector3D const& A2,
float startTimeA,
float endTimeA,
float radiusA,
Vector3D const& B,
Vector3D const& B2,
float startTimeB,
float endTimeB,
float radiusB){
  Vector3D VA(A2-A);
  Vector3D VB(B2-B);
  Vector3D DAB(B-A);
  // Assume unit speed...
  VA.Normalize();
  VB.Normalize();
  double r(radiusA+radiusB);
  double rsq(r*r);
  Vector3D DVAB(VB-VA);

  if(A==B && A2==B2){
    return {-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()}; //Overlapping and waiting
  }
  // Are they parallel?
  if(fequal(DVAB.x,0.0) && fequal(DVAB.y,0.0)){
    // If they are currently overlapping, they will do so forever, otherwise, they never will
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      double dc(sqrt(DAB.sq()-d)); // Distance between centers along trajectory line
      return {startTimeB+dc-v,dc+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Are they opposing?
  if(VA==-VB){
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Is one agent waiting?
  if(VA.x==0 && VA.y==0){
    if(VB.x==0 && VB.y==0){
      if(DAB.sq()<rsq) return {-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()}; //Overlapping and waiting
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Not overlapping and waiting
    // Compute the interval until agent B passes over agent A
    double d(Util::distanceOfPointToLine(B,B2,A));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }else if(VB.x==0 && VB.y==0){
    double d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      double v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }


  double AVA(A*VA);
  double BVA(B*VA);
  double AVB(A*VB);
  double BVB(B*VB);

  double a(DVAB*DVAB); // Dot product
  //b has the effect of rotating the conic
  double b(2.0*(VA*VA - VA*VB));
  double c(VA*VA);
  double d(2.0*(BVB - BVA + AVA - AVB));
  double e(-2.0*BVA+2.0*AVA);
  double f(DAB*DAB - rsq);

  // this conic is guaranteed to be an ellipse because both a and c are positive (they are squares)
  double dscr2(4.0*a*c-b*b);
  // The following equations are from:
  // https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form
  double g(b*d-2.0*a*e);
  double center_y(g/dscr2);
  double g2(2.0*g);
  double delta(sqrt(g2*g2+4.0*dscr2*(d*d-4*a*f))/(2*dscr2));
  // interval for agent A:
  return {-center_y-delta,-center_y+delta};
  // interval for agent B:
  //return {center_y-delta,center_y+delta};
}

double getCollisionInterval3D(Vector3D const& A1, Vector3D const& A2, Vector3D const& B1, Vector3D const& B2, double radiusA, double radiusB){
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector3D VA(A2-A1);
  VA.Normalize();
  Vector3D VB(B2-B1);
  VB.Normalize();

  Vector3D w(B1-A1);
  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);
  double c(w.sq()-r*r);
  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return -1.0; } // No collision will occur in the future
  double sqrtd(sqrt(dscr));
  return (b+sqrtd)/a-(b-sqrtd)/a; // Collision interval
}

// Get continuous collision interval for two agents (return -1,-1 if such does not exist)
std::pair<double,double> getCollisionInterval(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return {-1,-1};}

  // Detect a wait action...
  if(A==B&&VA==VB&&((VA.x==0&&VA.y==0)||(VB.x==0&&VB.y==0))){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return {-1,-1};}
    else{return std::make_pair(std::max(startTimeA,startTimeB),std::max(startTimeA,startTimeB)+radiusA+radiusB);}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A forward to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return {-1,-1};}

  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector3D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0.0)){return {startTimeA-TOLERANCE,startTimeA+r+TOLERANCE*3};} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return {-1,-1}; } // No collision will occur in the future
  double sqrtd(sqrt(dscr));
  return std::make_pair((b-sqrtd)/a + startTimeA,(b+sqrtd)/a + startTimeA); // Absolute collision time
}

std::pair<double,double> getCollisionInterval(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return {-1,-1};}

  // Detect a wait action...
  if(A==B&&VA==VB&&((VA.x==0&&VA.y==0)||(VB.x==0&&VB.y==0))){
    if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return {-1,-1};}
    else{return std::make_pair(std::max(startTimeA,startTimeB),std::max(startTimeA,startTimeB)+radiusA+radiusB);}
  }

  if(fgreater(startTimeB,startTimeA)){
    // Move A forward to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  if(fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return {-1,-1};}

  // Assume an open interval just at the edge of the agents...
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  Vector3D w(B-A);
  double c(w.sq()-r*r);
  if(fless(c,0.0)){return {startTimeA-TOLERANCE,startTimeA+r+TOLERANCE*3};} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector3D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return {-1,-1}; } // No collision will occur in the future
  double sqrtd(sqrt(dscr));
  return std::make_pair((b-sqrtd)/a + startTimeA,(b+sqrtd)/a + startTimeA); // Absolute collision time
}
float fltarray[49*49*49];
