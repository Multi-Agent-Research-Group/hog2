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
#include "PrintUtils.h"
#include <iostream>
#include "BiClique.h"
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>

#define WORD_BITS (8 * sizeof(unsigned))

inline bool get(unsigned const*const bitarray, size_t idx) {
  return bitarray[idx / WORD_BITS] & (1 << (idx % WORD_BITS));
}

inline void set(unsigned* bitarray, size_t idx) {
  bitarray[idx / WORD_BITS] |= (1 << (idx % WORD_BITS));
}

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
/*static bool detectCollision(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA,
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
}*/

// Detect whether collision is occurring or will occur between 2 agents
// placed at pi and pj with velocity and radius.
// Return time of collision or zero otherwise
static double collisionImminent(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
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
  Vector2D v(VA-VB); // Note: the subtraction order is deliberate...
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
//std::cout << "w,v " << w << " " << v << " " << r << "\n";
//std::cout << "coeffs " << a << " " << b << " " << c << "\n";
  if(fleq(dscr,0)){ return 0; }

  double ctime((b-sqrt(dscr))/a); // Collision time
//std::cout << "dscr " << dscr << " " << ctime <<"\n";
  if(fless(ctime,0)){ return 0; }

  // Collision will occur if collision time is before the end of the shortest segment
  return fleq(ctime,std::min(endTimeB,endTimeA)-startTimeA)?ctime+startTimeA:0;
}

static void get2DSuccessors(Vector2D const& c, std::vector<Vector2D>& s, unsigned conn){
  s.reserve((conn*2+1)*(conn*2+1));
  for(int x(c.x-conn); x<conn*2+1+c.x; ++x)
    for(int y(c.y-conn); y<conn*2+1+c.y; ++y)
      s.push_back(Vector2D(x,y));
}

// Assume agents appear/disappear at start/end of time intervals
static bool collisionImminent(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
  // assume time overlap
  if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)||fequal(startTimeA,endTimeA)||fequal(startTimeB,endTimeB)){return false;}

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
  Vector3D w(B-A);
  double r(radiusA+radiusB-2*TOLERANCE); // Combined radius
  double c(w.sq()-r*r);
  if(c<0.0){return true;} // Agents are currently colliding

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


// Check for collision between entities moving from A1 to A2 and B1 to B2
// Speed is optional. If provided, should be in grids per unit time; time should also be pre adjusted to reflect speed.
static bool collisionCheck3D(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA=0.25, double radiusB=0.25, double speedA=1.0, double speedB=1.0){
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

/*
static void get9Coeffs(){
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
      //unsigned index(x1+y1*3+x2*9+y2*27+x3*81+y3*243+x4*729+y4*2187+6561*((tests[i][2].t-tests[i][0].t)*10+14));
      //int cc=map[index];
    }else{
      //unsigned index(x1+y1*3+x2*9+y2*27+x3*81+y3*243+x4*729+y4*2187+6561*((tests[i][2].t-tests[i][0].t)*10+14));
      //int cc=map[index];
    }
    //collisionCheckAPriori(tests[i][0],tests[i][1],.25,0,tests[i][1].x+tests[i][1].y==2?1.4:1.0,tests[i][2],tests[i][3],.25,0,tests[i][3].x+tests[i][3].y==2?1.4:1.0);
  }
  std::cout << "Cache computation took: " << t2.EndTimer() << std::endl;
  //std::cout << "There are " << moves.size() << " unique combinations\n";
}

static void get16Coeffs(){
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

static void get32Coeffs(){
    //const signed moves32[33][2]={{-1,-1},{-3,-2},{-2,-1},{-3,-1},{-1,0},{-3,1},{-2,1},{-3,2},{-1,1},{-2,3},{-1,2},{-1,3},{0,1},
      //{1,3},{1,2},{2,3},{1,1},{3,2},{2,1},{3,1},{1,0},{3,-1},{2,-1},{3,-2},{1,-1},{2,-3},{1,-2},{1,-3},{0,-1},{-1,-3},{-1,-2},{-2,-3},{0,0}};
}
*/

static void get3DSuccessors(Vector3D const& c, std::vector<Vector3D>& s, unsigned conn){
  s.reserve((conn*2+1)*(conn*2+1)*(conn*2+1));
  for(int x(c.x-conn); x<c.x+conn; ++x)
    for(int y(c.y-conn); y<c.y+conn; ++y)
      for(int z(c.z-conn); z<c.z+conn; ++z)
        s.push_back(Vector3D(x,y,z));
}

static unsigned index9(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2){
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

static unsigned index25(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2){
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

static unsigned index49(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-3;
  d1.y-=s1.y-3;
  d2.x-=s2.x-3;
  d2.y-=s2.y-3;
  s2.x-=s1.x-3;
  s2.y-=s1.y-3;
  return d1.y*7+d1.x + 49*(s2.y*7+s2.x) + 2401*(d2.y*7+d2.x);
}

static unsigned index93(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
  
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-3;
  d1.y-=s1.y-3;
  d2.x-=s2.x-3;
  d2.y-=s2.y-3;
  s2.x-=s1.x-3;
  s2.y-=s1.y-3;
  return d1.y*7+d1.x + 49*(s2.y*7+s2.x) + 2401*(d2.y*7+d2.x);
}

static unsigned index493(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
  // Translate d1 and s2 relative to s1
  d1.x-=s1.x-3;
  d1.y-=s1.y-3;
  d2.x-=s2.x-3;
  d2.y-=s2.y-3;
  s2.x-=s1.x-3;
  s2.y-=s1.y-3;
  return d1.y*7+d1.x + 49*(s2.y*7+s2.x) + 2401*(d2.y*7+d2.x);
}

static unsigned index27(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
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

static unsigned index125(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2){
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


static void fillArray(unsigned* bitarray,unsigned (*index)(Vector2D const&,Vector2D, Vector2D,Vector2D),unsigned conn,double radius){
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

static void fillArray(unsigned* bitarray,unsigned (*index)(Vector3D const&,Vector3D, Vector3D,Vector3D),unsigned conn, double radius){
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

/*static void load3DCollisionTable9(){
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

static void load3DCollisionTable(){
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
static void fillArray(unsigned (*index)(Vector3D const&,Vector3D, Vector3D,Vector3D),unsigned conn, double radius){
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
*/

// Check for collision between entities moving from A1 to A2 and B1 to B2
// Speed is optional. If provided, should be in grids per unit time; time should also be pre adjusted to reflect speed.
/*static bool collisionCheck(TemporalVector const& A1, TemporalVector const& A2, TemporalVector const& B1, TemporalVector const& B2, double radiusA, double radiusB, double speedA, double speedB){
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
}*/

static double collisionTime(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
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

static double collisionCheck3DSlow(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA, double radiusB, double speedA, double speedB){
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
static double collisionTime3D(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA=0.25, double radiusB=0.25, double speedA=1.0, double speedB=1.0){
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

static double getCollisionInterval3D(Vector3D const& A1, Vector3D const& A2, Vector3D const& B1, Vector3D const& B2, double radiusA, double radiusB){
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
static std::pair<double,double> getCollisionInterval(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
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

static std::pair<double,double> getCollisionInterval(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
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


static std::pair<double,double> collisionInterval3D(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2, double radiusA=0.25, double radiusB=0.25, double speedA=1.0, double speedB=1.0){
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

static bool collisionCheckLLA(TemporalVector3D A1, TemporalVector3D A2, TemporalVector3D B1, TemporalVector3D B2, double radiusA, double radiusB){
  A1.z /= 364566.929f; // feet to degs (60 NMI units)
  A2.z /= 364566.929f; // feet to degs (60 NMI units)
  B1.z /= 364566.929f; // feet to degs (60 NMI units)
  B2.z /= 364566.929f; // feet to degs (60 NMI units)
  double speedA(Util::distance(A1.x,A1.y,A1.z,A2.x,A2.y,A2.z)/(A2.t-A1.t));
  double speedB(Util::distance(B1.x,B1.y,B1.z,B2.x,B2.y,B2.z)/(B2.t-B1.t));
  Vector3D VA(A2-A1);
  VA.Normalize();
  VA*=speedA;
  Vector3D VB(B2-B1);
  VB.Normalize();
  VB*=speedB;
  return collisionImminent(A1,VA,radiusA,A1.t,A2.t,B1,VB,radiusB?radiusB:radiusA,B1.t,B2.t);
}

// Get collision time between two agents - that is the time that they will "start" colliding.
// Agents have a position, velocity, start time, end time and radius.
// Note: this is analogous to an agent traversing an edge from time "start" to "end" at constant velocity.
// -1 is a seminal value meaning "no collision in the future."
static double getCollisionTime(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB){
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

static double getCollisionTime(Vector3D A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA, Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB){
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

/*static std::pair<float,float> getForbiddenInterval(Vector2D const& A,
Vector2D const& A2,
Vector2D const& B,
Vector2D const& B2,
float radiusA,
float radiusB){
  Vector2D VA(A2-A);
  Vector2D VB(B2-B);
  Vector2D DAB(B-A);
  // Assume unit speed...
  VA.Normalize();
  VB.Normalize();
  float r(radiusA+radiusB);
  float rsq(r*r);
  Vector2D DVAB(VB-VA);

  // Are they parallel?
  if(fequal(DVAB.x,0.0) && fequal(DVAB.y,0.0)){
    // If they are currently overlapping, they will do so forever, otherwise, they never will
    float d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      float v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      float dc(sqrt(DAB.sq()-d)); // Distance between centers along trajectory line
      return {dc-v,dc+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Are they opposing?
  if(VA==-VB){
    float d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      float v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
      return {-std::numeric_limits<float>::infinity(),sqrt(DAB.sq()-d)+v}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Is one agent waiting?
  if(VA.x==0 && VA.y==0){
    if(VB.x==0 && VB.y==0){
      if(DAB.sq()<rsq) return {-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()}; //Overlapping and waiting
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Not overlapping and waiting
    // Compute the interval that agent B passes over agent A
    float d(Util::distanceOfPointToLine(B,B2,A));
    d*=d;
    float v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
    float dc(sqrt(DAB.sq()-d)); // Distance between centers along trajectory line
    return {dc-v,dc+v}; // Distance between lines is close enough to crash.
  }else if(VB.x==0 && VB.y==0){
    float d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    float v(sqrt(rsq-d)); // orthogonal distance between centers at start of impact
    float dc(sqrt(DAB.sq()-d)); // Distance between centers along trajectory line
    return {dc-v,dc+v}; // Distance between lines is close enough to crash.
  }

  float AVA(A*VA);
  float BVA(B*VA);
  float AVB(A*VB);
  float BVB(B*VB);

  float a(DVAB*DVAB); // Dot product
  //b has the effect of rotating the conic
  float b(2.0*(VA*VA - VA*VB));
  float c(VA*VA);
  float d(2.0*(BVB - BVA + AVA - AVB));
  float e(-2.0*BVA+2.0*AVA);
  float f(DAB*DAB - rsq);

  // this conic is guaranteed to be an ellipse because both a and c are positive (they are squares)
  float dscr(4.0*a*c-b*b);
  // The following equations are from:
  // https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form
  float g(b*d-2.0*a*e);
  float center_y(g/dscr);
  float g2(2.0*g);
  float delta(sqrt(g2*g2+4.0*dscr*(d*d-4*a*f))/(2*dscr));
  // interval for agent A:
  return {-center_y-delta,-center_y+delta};
  // interval for agent B:
  //return {center_y-delta,center_y+delta};
}*/

static void getFoci(float q,
                    float a,
                    float b,
                    float c,
                    float center_x,
                    float center_y,
                    float& fx1,
                    float& fy1,
                    float& fx2,
                    float& fy2){
  float s(.25*sqrt(fabs(q)*sqrt(b*b + (a-c)*(a-c)))); // Dist from center to foci
  float dscr(q*a-q*c);
  float dscr2(q*b);
  float theta(0);
  //if((dscr==0) && (dscr2==0)) θ=0
  if(dscr==0){
    if(dscr2>0) theta=.25*M_PI;
    else if(dscr2<0) theta=.75*M_PI;
  }else if(dscr>0){
    theta=.5*atan(b/(a-c));
    if(dscr2<0) theta+=M_PI;
  }else{
    theta=.5*atan(b/(a-c))+M_PI/2.0;
  }
  fx1=center_x+cos(theta)*s;
  fx2=center_x-cos(theta)*s;
  fy1=center_y+sin(theta)*s;
  fy2=center_y-sin(theta)*s;
}

static std::pair<float,float> getDelay(float fx1, // focal point 1 x coord
                       float fy1, // focal point 1 y coord
                       float fx2, // focal point 2 x coord
                       float fy2, // focal point 2 y coord
                       float s, // segment length
                       float r){ // radius
  float fx12(fx1*fx1);
  float fx22(fx2*fx2);
  float fy12(fy1*fy1);
  float fy22(fy2*fy2);
  float fxy1(fx1*fy1);
  float fx1x2(fx1*fx2);
  float fxy2(fx2*fy2);
  float fx1y2(fx1*fy2);
  float fy1x2(fy1*fx2);
  float r2=r*r;
  float s2=s*s;
  float c1(-4*fx12 + 8*fxy1 + 8*fx1x2 - 8*fx1y2 - 4*fy12 - 8*fy1x2 + 8*fy1*fy2 - 4*fx22 + 8*fxy2 - 4*fy22 + 8*r2);
  float num(-4*fx12*fx1 + 4*fx12*fy1 + 4*fx12*fx2 - 4*fx12*fy2 + 8*fx12*s - 4*fx1*fy12 - 8*fxy1*s + 4*fx1*fx22 - 16*fx1x2*s + 4*fx1*fy22 + 8*fx1y2*s + 4*fx1*r2 + 4*fy12*fy1 + 4*fy12*fx2 - 4*fy12*fy2 - 4*fy1*fx22 + 8*fy1x2*s - 4*fy1*fy22 - 4*fy1*r2 - 4*fx22*fx2 + 4*fx2*fxy2 + 8*fx22*s - 4*fx2*fy22 - 8*fxy2*s + 4*fx2*r2 + 4*fy22*fy2 - 4*fy2*r2 - 8*r2*s);
  float v1(4*fx12*fx1 - 4*fx12*fy1 - 4*fx12*fx2 + 4*fx12*fy2 - 8*fx12*s);
  float v2(sqrt(num*num - 4*c1*(-fx12*fx12 + 4*fx12*fx1*s - 2*fx12*fy12 + 2*fx12*fx22 - 4*fx12*fx2*s + 2*fx12*fy22 + 2*fx12*r2 - 4*fx12*s2 + 4*fx1*fy12*s - 4*fx1*fx22*s + 8*fx1x2*s2 - 4*fx1*fy22*s - 4*fx1*r2*s - fy12*fy12 + 2*fy12*fx22 - 4*fy12*fx2*s + 2*fy12*fy22 + 2*fy12*r2 - fx22*fx22 + 4*fx22*fx2*s - 2*fx22*fy22 + 2*fx22*r2 - 4*fx22*s2 + 4*fx2*fy22*s - 4*fx2*r2*s - fy22*fy22 + 2*fy22*r2 - r2*r2 + 4*r2*s2)));
  float v3(4*fx1*fy12 + 8*fxy1*s - 4*fx1*fx22 + 16*fx1x2*s - 4*fx1*fy22 - 8*fx1y2*s - 4*fx1*r2 - 4*fy12*fy1 - 4*fy12*fx2 + 4*fy12*fy2 + 4*fy1*fx22 - 8*fy1x2*s + 4*fy1*fy22 + 4*fy1*r2 + 4*fx22*fx2 - 4*fx2*fxy2 - 8*fx22*s + 4*fx2*fy22 + 8*fxy2*s - 4*fx2*r2 - 4* fy22*fy2 + 4*fy2*r2 + 8*r2*s);
  return {(v1 - v2 + v3)/(2*c1), (v1 + v2 + v3)/(2*c1)};
}

static std::pair<float,float>
getForbiddenIntervalIncremental(Vector3D const& A,
                     Vector3D const& VA,
                     float startTimeA,
                     float endTimeA,
                     float radiusA,
                     Vector3D const& B,
                     Vector3D const& VB,
                     float startTimeB,
                     float endTimeB,
                     float radiusB,
                     float delayStart,
                     float delayEnd,
                     float res=0.001){
  float dur(endTimeA-startTimeA);
  float durB(endTimeB-startTimeB);
    for(float i(delayStart+res); i<delayEnd; i+=res){
      if(collisionImminent(A,VA,radiusA,i,i+dur,B,VB,radiusB,0,durB)){
        delayStart=i-res;
        break;
      }
    }
    for(float i(delayEnd-res); i>delayStart; i-=res){
      if(collisionImminent(A,VA,radiusA,i,i+dur,B,VB,radiusB,0,durB)){
        delayEnd=i-res;
        break;
      } 
    }
    return {delayStart,delayEnd};
}

// General case for interval - does not consider the corner cases... (use getForbiddenInterval())
static std::pair<float,float>
getForbiddenIntervalGeneralCase(Vector3D const& A,
                     Vector3D const& A2,
                     Vector3D const& VA,
                     float dur, // Duration of action A
                     Vector3D const& B,
                     Vector3D const& B2,
                     Vector3D const& VB,
                     float durB,
                     Vector3D const& DAB, // A-B
                     Vector3D const& DVAB, // VA-VB
                     float r, // (rA+rB)
                     float res=0.001){
  float rsq(r*r);
  float AVA(A*VA);
  float BVA(B*VA);
  float AVB(A*VB);
  float BVB(B*VB);

  float a(DVAB*DVAB); // Dot product
  //b has the effect of rotating the conic
  float b(2.0*(VA*VA - VA*VB));
  float c(VA*VA);
  float d(2.0*(BVB - BVA + AVA - AVB));
  float e(-2.0*BVA+2.0*AVA);
  float f(DAB*DAB - rsq);

  // this conic is guaranteed to be an ellipse because both a and c are >= 0 (because they are squares)
  // the case of a or c being zero is covered by the above cases (opposing, parallel and waiting)
  float dscr(4.0*a*c-b*b);


  // Many of the following equations are from:
  // https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form
  float g(b*d-2.0*a*e);
  float center_y(g/dscr);
  g*=2;
  float delta(sqrt(g*g+4.0*dscr*(d*d-4*a*f))/(2*dscr));
  float delayStart(center_y-delta);
  float delayEnd(center_y+delta);
  //std::cout << "############### segs " << dur << "," << durB << "\n";
  //std::cout << "############### delay " << delayEnd << ", " << delayStart << "\n";

  float da1b(Util::distanceOfPointToLine(B,B2,A));
  float da2b(Util::distanceOfPointToLine(B,B2,A2));
  float db1a(Util::distanceOfPointToLine(A,A2,B));
  float db2a(Util::distanceOfPointToLine(A,A2,B2));
  if(da1b<r || da2b<r || db1a<r || db2a<r){
    //std::cout << da1b << " " << da2b << " " << db1a << " " << db2a << "\n";
    return getForbiddenIntervalIncremental(A,VA,0,dur,r/2.0,B,VB,0,durB,r/2.0,-delayEnd,-delayStart,res);
    //std::cout << "SPECIAL CASE\n";
    //for(float i(delayStart+res); i<delayEnd; i+=res){
      //if(collisionImminent(A,VA,r/2.0,0,dur,B,VB,r/2.0,i,i+durB)){
        //delayStart=i-res;
        //break;
      //}
    //}
    //for(float i(delayEnd-res); i>delayStart; i-=res){
      //if(collisionImminent(A,VA,r/2.0,0,dur,B,VB,r/2.0,i,i+durB)){
        //delayEnd=i-res;
        //break;
      //} 
    //}
  }else if(distanceSquared(A,B2)<rsq && da2b>r){
    delayStart=-durB;
  }else{
    // Check the collision time for delayEnd.
    // If it occurs after the common interval, we must truncate it to the delay associated with
    // the end of the common interval (find the y-coordinate on the ellipse for x=tEnd)
    float collisionTimeApex((-b*delayEnd-d)/(2*a));
    float collisionTimeBase((-b*delayStart-d)/(2*a));
    // Determine if segments end before being able to collide
    float h(b*e - 2*c*d);
    float center_x(h/dscr);
    h*=2;
    float leftx(center_x-sqrt(h*h+4.0*dscr*(e*e-4*c*f))/(2*dscr));
    //float rightx(center_x+sqrt(h*h+4.0*dscr*(e*e-4*c*f))/(2*dscr));
    //float lefty((-b*leftx-e)/(2*c));
    //std::cout << "############### apex " << collisionTimeApex << ", " << collisionTimeBase << "\n";
    //std::cout << "############### left/right " << leftx << ", " << rightx << "\n";
    //std::cout << "len " << (dur+delayStart) << "<" << collisionTimeApex << "=" << ((dur+delayStart)<collisionTimeApex) << ", " << (dur+delayEnd) << "<" << collisionTimeApex << "=" << ((dur+delayEnd)<collisionTimeApex) << "\n";
    // len A > len B and A1 is near B2 and orthogonal distance from A2 to B is large, then the forbidden interval ends when agent B disappears (len B)
    if(!fequal(leftx,collisionTimeApex) && (collisionTimeApex+delayEnd>dur || collisionTimeBase>durB)){
      float q(64*(f*dscr-a*e*e+b*d*e-c*d*d)/(dscr*dscr));
      float fx1;
      float fy1;
      float fx2;
      float fy2;
      getFoci(q,a,b,c,center_x,center_y,fx1,fy1,fx2,fy2);
      float dx1(collisionTimeApex-fx1);
      float dx2(collisionTimeApex-fx2);
      float dy1(delayEnd-fy1);
      float dy2(delayEnd-fy2);
      float focrad(sqrt(dx1*dx1 + dy1*dy1) + sqrt(dx2*dx2 + dy2*dy2));
      //std::cout << "Start was " << delayStart << "\n";
      //std::cout << "End was " << delayEnd << "\n";
      if(collisionTimeApex+delayEnd>dur){
        auto delays(getDelay(fx1,fy1,fx2,fy2,dur,focrad));
        //std::cout << "begin: " << delays.first << "," << delays.second << "\n";
        delayEnd=delays.second;
      }
      if(collisionTimeApex>(durB-dur)){
        auto delays(getDelay(fx1,fy1,fx2,fy2,(durB-dur)+delayEnd,focrad));
        //std::cout << "diff: " << delays.first << "," << delays.second << "\n";
        if(!isnan(delays.first)){
          delayEnd=delays.first;
        }
      }
      if(collisionTimeBase>durB){
        // Does applying the delay make the collision time end too early?
        auto delays(getDelay(fx1,fy1,fx2,fy2,durB+delayStart,focrad));
        //std::cout << "end: " << delays.first << "," << delays.second << "\n";
        if(!isnan(delays.first)){
          delayStart=delays.first;
        }else{
          auto delays(getDelay(fx1,fy1,fx2,fy2,durB+delayEnd,focrad));
          //std::cout << "end2: " << delays.first << "," << delays.second << "\n";
          if(!isnan(delays.first))
            delayStart=delays.first;
        }
      }
      //std::cout << "Start now " << delayStart << "\n";
      //std::cout << "End now " << delayEnd << "\n";
    }
  }
  return {-delayEnd,-delayStart};
}

static std::pair<float,float>
getForbiddenInterval(Vector3D const& A,
                     Vector3D const& A2,
                     float startTimeA,
                     float endTimeA,
                     float radiusA,
                     Vector3D const& B,
                     Vector3D const& B2,
                     float startTimeB,
                     float endTimeB,
                     float radiusB,
                     float res=0.001){
  Vector3D VA(A2-A);
  Vector3D VB(B2-B);
  float dur(endTimeA-startTimeA);
  Vector3D DAB(B-A);
  // Assume unit speed...
  VA.Normalize();
  VB.Normalize();
  float r(radiusA+radiusB);
  float rsq(r*r);
  Vector3D DVAB(VB-VA);

  if(!Util::fatLinesIntersect(A,A2,radiusA,B,B2,radiusB)){
    return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()}; // never conflicting
  }
  // Is one agent waiting?
  if(VA.x==0 && VA.y==0){
    if(VB.x==0 && VB.y==0){ // Both waiting
      if(DAB.sq()<rsq){return {startTimeB-dur,endTimeB}; //Overlapping and waiting
      }else{
        return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Not overlapping and waiting
    }
    // Compute the interval until agent B passes over agent A
    float d(Util::distanceOfPointToLine(B,B2,A));
    d*=d;
    if(d<rsq){
      float durB(endTimeB-startTimeB);
      float v(sqrt(rsq-d)); // translational distance between centers at start of impact
      float dist(sqrt(DAB.sq()-d)); // distance traveled by either agent
      // Backoff=time of collision - total time of intersection
      //std::cout << "v " << v << " dur " << dur << "\n";
      //std::cout << "i " << startTimeB+(dist-v-dur) << "," << startTimeB+(dist+v) << "\n";
      //return {std::max(startTimeB-dur,startTimeB+(dist-v-dur)),std::min(startTimeB+(dist+v),endTimeB)}; // Distance from line is close enough to crash.
      return getForbiddenIntervalIncremental(A,VA,0,dur,radiusA,B,VB,0,durB,radiusB,std::max(startTimeB-dur,startTimeB+(dist-v-dur)),std::min(startTimeB+(dist+v),endTimeB),res);
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }else if(VB.x==0 && VB.y==0){
    float d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      //float v(sqrt(rsq-d)); // translational distance between centers at start of impact
      //float dist(sqrt(DAB.sq()-d)); // distance traveled by either agent
      float durB(endTimeB-startTimeB);
      return getForbiddenIntervalIncremental(A,VA,0,dur,radiusA,B,VB,0,durB,radiusB,startTimeA-std::max(dur,durB),endTimeB,res);
      //std::cout << v << " " << dist << "\n";
      //std::cout << (startTimeB-durB) << " " << (startTimeB+(-durB+dist+v)) << "," << (startTimeB+(dist+v)) << " " << (endTimeB) << "\n";
      //std::cout << (startTimeA-durB) << " " << (startTimeA+(dist-v)) << "," << (startTimeA+(dist+v)) << " " << (endTimeB) << "\n";
      //return {std::max(startTimeA-durB,startTimeA+(dist-v)),std::max(startTimeA+(dist+v),endTimeB)}; // Distance from line is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // never conflicting
  }

  // Are they parallel?
  if(fequal(DVAB.x,0.0) && fequal(DVAB.y,0.0)){
    // If they are currently overlapping, they will do so forever, otherwise, they never will
    float d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      float v(sqrt(rsq-d)); // translational orthogonal distance between centers at start of impact
      float ds(DAB.sq()); // Sq. distance between starts
      float dc(sqrt(ds-d)); // Distance between starts along trajectory line
      // Determine if A is "in front of" B
      Vector3D DA(A2-A);
      float costheta(DAB*DA/(DAB.len()*DA.len()));
      if(costheta>0){
        return {std::max(startTimeB-dc-v,-endTimeA),std::min(startTimeB-dc+v,endTimeB)};
      }else if(costheta<0){
        return {std::max(startTimeB+dc-v,-endTimeA),std::min(startTimeB+dc+v,endTimeB)};
      }
      return {startTimeB-v,std::min(startTimeB+v,endTimeB)}; // Distance between lines is close enough to crash.
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }
  // Are they opposing?
  if(VA==-VB){
    float d(Util::distanceOfPointToLine(A,A2,B));
    d*=d;
    if(d<rsq){
      float v(sqrt(rsq-d)); // translational distance between centers at start of impact

      auto ES(B-A2);
      auto SE(B2-A);

      float dep(sqrt(ES.sq()-d)); // distance required for A to be orthogonal with start of B
      float dp(sqrt(SE.sq()-d)); // distance required for A to be orthogonal with end of B

      float start(0.0f);
      float end(0.0f);

      Vector3D DA(A2-A);
      if(SE.len()<=r){
        end=endTimeB;
      }else{
        float costheta(SE*DA/(SE.len()*DA.len()));
        if(costheta>=0){
          end=endTimeB-dp+v;
        }else{
          end=endTimeB-dp+v;
        }
      }

      // Determine if A2 is "behind" B
      if(ES.len()<=r){
        start=startTimeB-dur;
      }else{
        float costheta(ES*DA/(ES.len()*DA.len()));
        if(costheta>0){ // Acute angle
          start=startTimeB-dur+dep-v;
        }else{
          start=startTimeB-dur+dep-v;
        }
      }
      return {start,end};
    }else{ return {std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};} // Parallel, never conflicting
  }


  // General case
  float durB(endTimeB-startTimeB);
  if(durB<dur){
    //std::cout << "SWAPPED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    auto intvl(getForbiddenIntervalGeneralCase(B,B2,VB,durB,A,A2,VA,dur,DAB,DVAB,r));
    return {-intvl.second,-intvl.first};
  }
  return getForbiddenIntervalGeneralCase(A,A2,VA,dur,B,B2,VB,durB,DAB,DVAB,r);
}

// Get mirrored move number assuming that the start point is mirrored
// movenum is numbered clockwise starting with zero at bottom left (pointing toward origin)
//# 2  3  4
//#  \ | /
//# 1--8--5
//#  / | .
//# 0  7  6
// conn is the number of moves
static unsigned getMirroredMove(unsigned movenum, bool swap, bool ortho, bool y, unsigned conn){
  conn=conn-1;
  if(movenum==conn){ // wait action...
    return movenum;
  }
  unsigned half(conn/2);
  if(ortho){
    if(movenum >0 && movenum<half){ // Mirror across the diagonal (positive slope line)
      movenum += (half-movenum)*2;
      movenum%=conn;
    }else if(movenum>half){
      movenum -= (movenum-half)*2;
    }
  }
  if(swap){ // Mirror across the orthogonal (negative slope line)
    unsigned quarter(conn/4);
    unsigned threequarter(quarter*3);
    if(movenum>quarter){
      if(movenum <= half){
        movenum -= (movenum-quarter)*2;
      }else if(movenum<threequarter){
        movenum += (threequarter-movenum)*2;
      }else if(movenum>threequarter){
        movenum -= (movenum-threequarter)*2;
      }
    }else if(movenum<quarter){
        movenum += (quarter-movenum)*2;
    }
  }
  if(y){ // Mirror over y axis
    unsigned eighth(conn/8);
    if(movenum<(3*eighth)){
      movenum+=(3*eighth-movenum)*2;
      movenum%=conn;
    }else if(movenum>(3*eighth) && movenum<(7*eighth)){
      movenum-=(movenum-3*eighth)*2-conn;
      movenum%=conn;
    }else if(movenum>(7*eighth)){
      movenum-=(movenum-7*eighth)*2;
    }
  }
  return movenum;
}

//Invert mirroring
static unsigned invertMirroredMove(unsigned movenum, bool swap, bool ortho, bool y, unsigned conn){
  conn=conn-1;
  if(movenum==conn){ // wait action...
    return movenum;
  }
  if(y){ // Mirror over y axis
    unsigned eighth(conn/8);
    if(movenum<(3*eighth)){
      movenum+=(3*eighth-movenum)*2;
    }else if(movenum>(3*eighth) && movenum<(7*eighth)){
      movenum-=(movenum-3*eighth)*2-conn;
      movenum%=conn;
    }else if(movenum>(7*eighth)){
      movenum-=(movenum-7*eighth)*2;
    }
  }
  unsigned half(conn/2);
  if(swap){ // Mirror across the orthogonal (negative slope line)
    unsigned quarter(conn/4);
    unsigned threequarter(quarter*3);
    if(movenum>quarter){
      if(movenum <= half){
        movenum -= (movenum-quarter)*2;
      }else if(movenum<threequarter){
        movenum += (threequarter-movenum)*2;
      }else if(movenum>threequarter){
        movenum -= (movenum-threequarter)*2;
      }
    }else if(movenum<quarter){
        movenum += (quarter-movenum)*2;
    }
  }
  if(ortho){
    if(movenum >0 && movenum<half){ // Mirror across the diagonal (positive slope line)
      movenum += (half-movenum)*2;
      movenum%=conn;
    }else if(movenum>half){
      movenum -= (movenum-half)*2;
    }
  }
  return movenum;
}

static inline unsigned getDist(unsigned bf){
  switch(bf){
    case 32:
    case 33:
    case 48:
    case 49:
      return 6;
      break;
    case 16:
    case 17:
    case 24:
    case 25:
      return 4;
        break;
    default:
      return 2;
  }
}

// Number of mirrored start points
static unsigned getNumCases(unsigned conn){
  switch(conn){
    case 48:
    case 32:
    case 33:
    case 49:
      return 28;
      break;
    case 16:
    case 17:
    case 24:
    case 25:
      return 15;
      break;
    case 8:
    case 9:
      return 6;
      break;
    default:
      unsigned n(sqrt(conn-1));
      return ((n+1)*(n+2))/2;
      break;
  }
}

// Get index in grid:
// Steps: mirror over diagonal (swap agent locations)
// .
// .  .
// .  .  .
// .  .  .  .
// .  .  .  .  .
//
// Mirror over orthogonal (swap x and y)
//       .
//    .  .  .
// .  .  .  .  .
//
// Mirror over y
//
//       .
//    .  .
// .  .  .
//
// Map to indices:
//       5
//    3  4
// 0  1  2
//----------------
// Simultaneously, return ops that happened to translate (swap,ortho,y)
// 3-1 TTPs = basic fighter maneuvers
template <typename state>
signed locationIndex(state a, state b, bool& swap, bool& ortho, bool& y, unsigned conn){
  const unsigned dist(getDist(conn));
  if(a==b){
    return getNumCases(conn)-1;
  }
  signed dx(a.x-b.x);
  signed dy(a.y-b.y);
  if(abs(dx)>dist || abs(dy)>dist){return -1;}
  // Make B=(dist,dist) and A relative to it.
  a.x+=dist-b.x;
  if(a.x<0)return -1;
  a.y+=dist-b.y;
  if(a.y<0)return -1;
  b.x=b.y=dist;
  if((a.x>=b.x && a.y>=b.y) || (dx<0 && dy>0 && -dy<dx) || (dx>0 && dy<0 && dy>-dx)){ // Mirror over the diagonal
    auto tmp(a.y);
    a.y=-(a.x-b.x)+b.y;
    a.x=-(tmp-b.y)+b.x;
    swap=true;
  }
  if(a.y>a.x){ // Mirror over the orthogonal
    auto tmp(a.y);
    a.y=a.x;
    a.x=tmp;
    ortho=true;
  }
  // Finally, mirror over the y-axis
  if(a.x>dist){
    a.x-=(a.x-dist)*2;
    y=true;
  }
  if(dist==2){
    return a.x+a.y*dist;
  }
  else{
    dx=abs(dx);
    dy=abs(dy);
    if(dx<dy){
      auto tmp(dx);
      dx=dy;
      dy=tmp;
    }
    static const unsigned moves17[5][5]={{14, 0,0,0,0},
                                         {13,12,0,0,0},
                                         {11,10,9,0,0},
                                         {8, 7, 6,5,0},
                                         {4, 3, 2,1,0}};
    static const unsigned moves33[7][7]={{27, 0, 0, 0, 0, 0,0},
                                         {26,25, 0, 0, 0, 0,0},
                                         {24,23,22, 0, 0, 0,0},
                                         {21,20,19,18, 0, 0,0},
                                         {17,16,15,14,13, 0,0},
                                         {12,11,10, 9, 8, 7,0},
                                         { 6, 5, 4, 3, 2, 1,0}};
    if(dist==4)return moves17[dx][dy];
    if(dist==6)return moves33[dx][dy];
  }
}

template <typename state>
static void fromLocationIndex(signed ix, state& a, state& b, unsigned conn){
  unsigned dist(getDist(conn));
  b.x=b.y=dist;
  a.x=a.y=0;
  for(int i(0); i<dist+1; ++i){
    a.y=i;
    for(int j(0); j<dist+1-i; ++j){
      a.x=j+i;
      if(--ix<0)break;
    }
    if(ix<0)break;
  }
}

template <typename state>
static unsigned hdg8(state const& a, state const&b, int offset=0){
  static const unsigned rev[9]={0,1,2,7,8,3,6,5,4};
  return (rev[(int(b.x-a.x+1)*3+int(b.y-a.y+1))]+offset+9)%9;
}
template <typename state>
static float fetch8(state const& a, int offset, state& c){
  static const signed moves[9][2]={{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{0,0}};
  static const float dur[9]={M_SQRT2,1,
    M_SQRT2,1,
    M_SQRT2,1,
    M_SQRT2,1,1};
  c.x=a.x+moves[offset][0];
  c.y=a.y+moves[offset][1];
  return dur[offset];
}

template <typename state>
static unsigned hdg16(state const& a, state const&b, int offset=0){
  static const unsigned rev[25]={16,1,16,3,16,15,0,2,4,5,16,14,16,6,16,13,12,10,8,7,16,11,16,9,16};
  return (rev[(int(b.x-a.x+2)*5+int(b.y-a.y+2))]+offset+17)%17;
}
template <typename state>
float fetch16(state const& a, int offset, state& c){
  static const signed moves[17][2]={{-1,-1},{-2,-1},{-1,0},{-2,1},{-1,1},{-1,2},{0,1},{1,2},{1,1},{2,1},{1,0},{2,-1},{1,-1},{1,-2},{0,-1},{-1,-2},{0,0}};
  static const float dur[17]={M_SQRT2,sqrt(5),1,sqrt(5),
    M_SQRT2,sqrt(5),1,sqrt(5),
    M_SQRT2,sqrt(5),1,sqrt(5),
    M_SQRT2,sqrt(5),1,sqrt(5),1};
  c.x=a.x+moves[offset][0];
  c.y=a.y+moves[offset][1];
  return dur[offset];
}

template <typename state>
static unsigned hdg32(state const& a, state const&b, int offset=0){
  static const unsigned rev[49]={32,1,3,32,5,7,32,31,32,2,32,6,32,9,29,30,0,4,8,10,11,32,32,28,32,12,32,32,27,26,24,20,16,14,13,25,32,22,32,18,32,15,32,23,21,32,19,17,32};
  return (rev[(int(b.x-a.x+3)*7+int(b.y-a.y+3))]+offset+33)%33;
}
template <typename state>
static float fetch32(state const& a, int offset, state& c){
  static const signed moves[33][2]={{-1,-1},{-3,-2},{-2,-1},{-3,-1},{-1,0},{-3,1},{-2,1},{-3,2},{-1,1},{-2,3},{-1,2},{-1,3},{0,1},
          {1,3},{1,2},{2,3},{1,1},{3,2},{2,1},{3,1},{1,0},{3,-1},{2,-1},{3,-2},{1,-1},{2,-3},{1,-2},{1,-3},{0,-1},{-1,-3},{-1,-2},{-2,-3},{0,0}};
  static const float dur[33]={M_SQRT2,sqrt(13),sqrt(5),sqrt(10),1,sqrt(10),sqrt(5),sqrt(13),
    M_SQRT2,sqrt(13),sqrt(5),sqrt(10),1,sqrt(10),sqrt(5),sqrt(13),
    M_SQRT2,sqrt(13),sqrt(5),sqrt(10),1,sqrt(10),sqrt(5),sqrt(13),
    M_SQRT2,sqrt(13),sqrt(5),sqrt(10),1,sqrt(10),sqrt(5),sqrt(13),
    1};
  c.x=a.x+moves[offset][0];
  c.y=a.y+moves[offset][1];
  return dur[offset];
}

template <typename state>
static unsigned moveNum(state const& a, state const& b, int offset=0, unsigned conn=8){
  switch(conn){
    case 48:
    case 32:
    case 33:
    case 49:
      return hdg32(a,b,offset);
      break;
    case 16:
    case 17:
    case 24:
    case 25:
      return hdg16(a,b,offset);
      break;
    default:
      return hdg8(a,b,offset);
      break;
  }
}

template <typename state>
static float fetch(state const& a, int offset, state& b, unsigned conn=8U){
  switch(conn){
    case 48:
    case 32:
    case 33:
    case 49:
      return fetch32(a,offset,b);
      break;
    case 16:
    case 17:
    case 24:
    case 25:
      return fetch16(a,offset,b);
      break;
    default:
      return fetch8(a,offset,b);
      break;
  }
}

// A ranking function for two actions
template <typename state>
static signed getCaseNumber(state const& A1, state const& A2, state const& B1, state const& B2, unsigned& moveA, unsigned& moveB, unsigned bf){
  unsigned num(getNumCases(bf));
  bool swap=false,ortho=false,y=false;
  auto li(locationIndex(A1,B1,swap,ortho,y,bf));
  if(li<0) return -1;
  //unsigned moveA((swap)?getMirroredMove(moveNum(B1,B2,0,conn),swap,ortho,y,conn):getMirroredMove(moveNum(A1,A2,0,conn),swap,ortho,y,conn));
  //unsigned moveB((swap)?getMirroredMove(moveNum(A1,A2,0,conn),swap,ortho,y,conn):getMirroredMove(moveNum(B1,B2,0,conn),swap,ortho,y,conn));
  moveA=getMirroredMove(moveNum(A1,A2,0,bf),swap,ortho,y,bf);
  moveB=getMirroredMove(moveNum(B1,B2,0,bf),swap,ortho,y,bf);
  //std::cout << "case num " << li << " " << moveA << " " << num << " " << moveB << " " << bf << " : " << (li + moveA*num + moveB*num*bf) <<"\n";
  return li + moveA*num + moveB*num*bf; 
}

static void encodeIvls(std::vector<std::vector<std::pair<float,float>>> const& input,
                       std::vector<float>& output){
  if(input.size()){
    output.reserve(output.size()+input.size()*input[0].size()*2);
    for(auto const& l:input){
      for(auto const& r:l){
        output.push_back(r.first);
        output.push_back(r.second);
      }
    }
  }
}

static bool collisionCheck3DAPriori(std::vector<unsigned> const& array,
                                std::vector<unsigned> const& indices,
                                std::vector<float> const& ivls,
                                unsigned branchingFactor,
                                TemporalVector3D const& a1,
                                TemporalVector3D const& a2,
                                TemporalVector3D const& b1,
                                TemporalVector3D const& b2){
  unsigned moveA=0,moveB=0;
  auto ix(getCaseNumber(a1,a2,b1,b2,moveA,moveB,branchingFactor)); // Index into the biclique array
  unsigned num(indices[ix]); // Index into the intervals array
  unsigned numCases(indices.size());
  unsigned aix(0);
  unsigned bix(0);
  // TODO: a fast way to count number of bits...
  for(unsigned i(0); i<branchingFactor; ++i){
    if(get(array.data(),ix+i*numCases)){
      //std::cout << "get " << (ix+i*numCases) << "\n";
      if(i==moveA){
        break;
      }
      aix++;
    }
  }
  for(unsigned i(0); i<branchingFactor; ++i){
    if(get(array.data(),ix+(branchingFactor+i)*numCases)){
      //std::cout << "get " << (ix+(branchingFactor+i)*numCases) << "\n";
      if(i==moveB){
        break;
      }
      bix++;
    }
  }
  auto delay(b1.t-a1.t);
  return ivls[num+aix]>delay || ivls[num+branchingFactor+bix]<delay;
}

/*static void encodeIvls(std::vector<std::pair<float,float>> const& input1, std::vector<std::pair<float,float>> const& input2, float& output, float res=10.0){
  float prec(res*10); // Captures the number of trailing digits plus leading digits
  output=0;
  float ix(1.0);
  std::cout << std::setprecision(73) ;
  for(auto const& f:input1){
    output += round((f.first*prec)/res)*ix;
    ix*=prec;
    std::cout << output << "\n";
    output += round((f.second*prec)/res)*ix;
    std::cout << output << "\n";
    ix*=prec;
  }
  for(auto const& f:input2){
    output += round((f.first*prec)/res)*ix;
    std::cout << output << "\n";
    ix*=prec;
    output += round((f.second*prec)/res)*ix;
    std::cout << output << "\n";
    ix*=prec;
  }
  std::cout << std::setprecision(6) << "\n";
}*/

static void decodeIvls(std::vector<float>& input,
                       unsigned left,
                       std::vector<std::vector<std::pair<float,float>>>& output){
  output.resize(left);
  unsigned right(input.size()/left/2);
  for(unsigned i(0); i<left; ++i){
    output[i].reserve(right);
    for(unsigned j(0); j<right; ++j){
      output[i].emplace_back(input[i*right*2+j*2],input[i*right*2+j*2+1]);
    }
  }
}

/*static void decodeIvls(float input, unsigned num, std::vector<std::pair<float,float>>& output1, std::vector<std::pair<float,float>>& output2, float res=10.0){
  float prec(res*10);
  res*=10.0;
  output1.resize(num/2);
  output2.resize(num/2);
  for(unsigned ix(0); ix<num/2; ++ix){
    output1[ix].first = fmod(input,prec)/10.0;
    input /= prec;
    output1[ix].second = fmod(input,prec)/10.0;
    input /= prec;
  }
  for(unsigned ix(0); ix<num/2; ++ix){
    output2[ix].first = fmod(input,prec)/10.0;
    input /= prec;
    output2[ix].second = fmod(input,prec)/10.0;
    input /= prec;
  }
}*/

template <typename state>
void getBiclique(state const& a1,
                 state const& a2,
                 state const& b1,
                 state const& b2,
                 float startTimeA,
                 float endTimeA,
                 float startTimeB,
                 float endTimeB,
                 std::vector<unsigned>& left,
                 std::vector<unsigned>& right,
                 unsigned branchingFactor,
                 float rA=0.25,
                 float rB=0.25){
  auto core(getForbiddenInterval(a1,a2,0,endTimeA-startTimeA,rA,b1,b2,0,endTimeB-startTimeB,rB));
  if(core.first<core.second){
    // There is a collision. Now, we have to check surrounding moves in order to build the biclique and set the bits.
    // Set the core actions
    // Determine the mutually conflicting set...
    static std::vector<std::vector<unsigned>> fwd;
    fwd.resize(0);
    fwd.resize(1,std::vector<unsigned>(1)); // The 0th element is the collsion between a1,b1.
    fwd.reserve(branchingFactor);
    static std::vector<std::vector<unsigned>> rwd;
    rwd.resize(0);
    rwd.resize(1,std::vector<unsigned>(1));
    rwd.reserve(branchingFactor);
    unsigned amap[64];
    unsigned bmap[64];
    static std::vector<unsigned> armap;
    armap.resize(1,1);
    armap.reserve(branchingFactor);
    static std::vector<unsigned> brmap;
    brmap.resize(1,1);
    brmap.reserve(branchingFactor);
    static std::pair<unsigned,unsigned> conf(0,0);
    for(unsigned i(0); i<branchingFactor; ++i){
      state a;
      float end(fetch(a1,i,a,branchingFactor));
      auto ivl(getForbiddenInterval(a1,a,0,end,rA,b1,b2,0,endTimeB-startTimeB,rB));
      if(ivl.first<ivl.second && !(core.first>ivl.second || core.second < ivl.first)){
        //std::cout << ax1 << "<->" << as[a] << " " << bx1 << "<->" << bx2 << " => "; 
        //if(collisionCheck3D(ax1, as[a], bx1, bx2, agentRadius))
        //if(collisionCheck3DAPriori(ax1, as[a], bx1, bx2, agentRadius))
        //std::cout << "CRASH\n";
        if(a2.sameLoc(a)){
          amap[i]=0;
          armap[0]=i;
        }else{
          amap[i]=fwd.size();
          armap.push_back(i);
          fwd.push_back(std::vector<unsigned>(1));
          rwd[0].push_back(amap[i]);
        }
      }
      //else{std::cout << "NO CRASH\n";}
    }
    for(unsigned i(0); i<branchingFactor; ++i){
      state b;
      float end(fetch(b1,i,b,branchingFactor));
      auto ivl(getForbiddenInterval(a1,a2,0,endTimeA-startTimeA,rA,b1,b,0,end,rB));
      if(ivl.first<ivl.second && !(core.first>ivl.second || core.second < ivl.first)){
        //std::cout << ax1 << "<->" << as[a] << " " << bx1 << "<->" << bx2 << " => "; 
        //if(collisionCheck3D(ax1, as[a], bx1, bx2, agentRadius))
        //if(collisionCheck3DAPriori(ax1, as[a], bx1, bx2, agentRadius))
        //std::cout << "CRASH\n";
        if(b2.sameLoc(b)){
          bmap[i]=0;
          brmap[0]=i;
        }else{
          bmap[i]=rwd.size();
          brmap.push_back(i);
          rwd.push_back(std::vector<unsigned>(1));
          fwd[0].push_back(bmap[i]);
        }
      }
      //else{std::cout << "NO CRASH\n";}
    }
    for(unsigned ii(1); ii<armap.size(); ++ii){
      unsigned i(armap[ii]);
      state a;
      float endA(fetch(a1,i,a,branchingFactor));
      for(unsigned jj(1); jj<brmap.size(); ++jj){
        unsigned j(brmap[jj]);
        state b;
        float endB(fetch(b1,j,b,branchingFactor));
        auto ivl(getForbiddenInterval(a1,a,0,endA,rA,b1,b,0,endB,rB));
        if(ivl.first<ivl.second && !(core.first>ivl.second || core.second < ivl.first)){
          //std::cout << ax1 << "<->" << as[a] << " " << bx1 << "<->" << bs[b] << " => "; 
          //if(collisionCheck3D(ax1, as[a], bx1, bs[b], agentRadius))
          //if(collisionCheck3DAPriori(ax1, as[a], bx1, bs[b], agentRadius))
          //std::cout << "CRASH: ["<<amap[a]<<"]="<<bmap[b] <<"\n";
          fwd[amap[i]].push_back(bmap[j]);
          rwd[bmap[j]].push_back(amap[i]);
        }
        //else{std::cout << "NO CRASH\n";}
      }
    }
    //std::cout << "\n";
    left.resize(0);
    left.reserve(fwd.size());
    right.resize(0);
    right.reserve(rwd.size());
    if(fwd.size()<=rwd.size()){
      BiClique::findBiClique(fwd,rwd,conf,left,right);
    }else{
      BiClique::findBiClique(rwd,fwd,{conf.second,conf.first},right,left);
    }
    // Revert back to the original indices
    for(auto& m:left){
      m=armap[m];
    }
    for(auto& m:right){
      m=brmap[m];
    }
  }
}

template <typename state>
void getEdgeAnnotatedBiclique(state const& a,
                              state const& a2,
                              state const& b,
                              state const& b2,
                              std::vector<unsigned> const& left,
                              std::vector<unsigned> const& right,
                              std::vector<std::vector<std::pair<float,float>>>& ivls,
                              unsigned branchingFactor,
                              float rA=0.25,
                              float rB=0.25){
  ivls.resize(left.size());
  for(unsigned l(0); l<left.size(); ++l){
    state A2;
    float endA(fetch(a,left[l],A2,branchingFactor));
    ivls[l].resize(right.size(),{-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()});
    for(unsigned r(0); r<right.size(); ++r){
      state B2;
      float endB(fetch(b,right[r],B2,branchingFactor));
      ivls[l][r]=getForbiddenInterval(a,A2,0,endA,rA,b,B2,0,endB,rB);
    }
  }
}

static void getVertexAnnotatedBiclique(unsigned coreA,
                                       unsigned coreB,
                                       float startTimeA,
                                       float endTimeA,
                                       float startTimeB,
                                       float endTimeB,
                                       std::vector<unsigned>& left,
                                       std::vector<unsigned>& right,
                                       std::vector<std::vector<std::pair<float,float>>> const& ivls,
                                       std::vector<std::pair<float,float>>& livls,
                                       std::vector<std::pair<float,float>>& rivls,
                                       unsigned branchingFactor,
                                       float rA=0.25,
                                       float rB=0.25){
  auto delay(startTimeA-startTimeB);
  auto const& core(ivls[coreA][coreB]);
  if(delay<core.first || delay>core.second){ // We shouldn't be calling this function with a bad delay, but just in case...
    left.clear();
    right.clear();
    livls.clear();
    rivls.clear();
    //delay=-delay;
    //if(delay<core.first || delay>core.second){ // We shouldn't be calling this function with a bad delay, but just in case...
    std::cerr << "Warning: You called getVertexAnnotatedBiclique() on a pair of actions that do not conflict! --> delay " << delay << " ivl " << core.first << "," << core.second << std::endl;
    //assert(!"Delay doesn't match interval!");
    //}
    return;
  }
  livls.resize(left.size(),{-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()});
  rivls.resize(right.size(),{-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()});
  if(left.size()==1){
    for(unsigned l(0); l<left.size(); ++l){
      unsigned r1(0);
      for(unsigned r(0); r<right.size(); ++r, ++r1){
        auto const& ivl(ivls[l][r1]);
        if(ivl.first<delay && ivl.second>delay){
          livls[l].first=std::max(ivl.first,livls[l].first);
          livls[l].second=std::min(ivl.second,livls[l].second);
          rivls[r].first=std::max(-ivl.second,rivls[r].first);
          rivls[r].second=std::min(-ivl.first,rivls[r].second);
        }else{
          // remove from right
          if(r<coreB)coreB--;
          right.erase(right.begin()+r);
          rivls.erase(rivls.begin()+r);
          --r;
        }
      }
    }
  }else if(right.size()==1){
    unsigned l1(0);
    for(unsigned l(0); l<left.size(); ++l, ++l1){
      for(unsigned r(0); r<right.size(); ++r){
        auto const& ivl(ivls[l1][r]);
        if(ivl.first<delay && ivl.second>delay){
          livls[l].first=std::max(ivl.first,livls[l].first);
          livls[l].second=std::min(ivl.second,livls[l].second);
          rivls[r].first=std::max(-ivl.second,rivls[r].first);
          rivls[r].second=std::min(-ivl.first,rivls[r].second);
        }else{
          if(l<coreA)coreA--;
          // remove from left
          left.erase(left.begin()+l);
          livls.erase(livls.begin()+l);
          --l;
        }
      }
    }
  }else{
    static std::vector<std::vector<unsigned>> fwd;
    fwd.resize(0);
    fwd.resize(left.size());
    static std::vector<std::vector<unsigned>> rwd;
    rwd.resize(0);
    rwd.resize(right.size());
    bool rebuildBiclique(false);
    for(unsigned l(0); l<left.size(); ++l){
      fwd[l].reserve(right.size());
      for(unsigned r(0); r<right.size(); ++r){
        auto const& ivl(ivls[l][r]);
        if(ivl.first<delay && ivl.second>delay){
          fwd[l].push_back(r);
          rwd[r].push_back(l);
          if(!rebuildBiclique){
            livls[l].first=std::max(ivl.first,livls[l].first);
            livls[l].second=std::min(ivl.second,livls[l].second);
            rivls[r].first=std::max(-ivl.second,rivls[r].first);
            rivls[r].second=std::min(-ivl.first,rivls[r].second);
          }
        }else{
          rebuildBiclique=true;
        }
      }
    }
    if(rebuildBiclique){
      auto lorig(left); // Capture the original move numbers
      auto rorig(right);
      left.clear();
      right.clear();
      livls.clear();
      rivls.clear();
      // Populate left,right with the biclique indices
      if(fwd.size()<=rwd.size()){
        BiClique::findBiClique(fwd,rwd,{coreA,coreB},left,right);
      }else{
        BiClique::findBiClique(rwd,fwd,{coreB,coreA},right,left);
      }
      livls.resize(left.size(),{-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()});
      rivls.resize(right.size(),{-std::numeric_limits<float>::infinity(),std::numeric_limits<float>::infinity()});
      for(unsigned l(0); l<left.size(); ++l){
        for(unsigned r(0); r<right.size(); ++r){
          // Get intersection of vertex-wise intervals
          auto const& ivl(ivls[left[l]][right[r]]);
          livls[l].first=std::max(ivl.first,livls[l].first);
          livls[l].second=std::min(ivl.second,livls[l].second);
          rivls[r].first=std::max(-ivl.second,rivls[r].first);
          rivls[r].second=std::min(-ivl.first,rivls[r].second);
        }
        // Convert to move number
        left[l]=lorig[left[l]];
      }
      // Convert to move numbers
      for(auto& r:right){
        r=rorig[r];
      }
    }
  }
  std::cout << "After filter on delay: " << delay << "\ncore: " << core << "\nleft: " << left << "\nright: " << right << "\nlivls: " << livls << "\nrivls: " << rivls << "\n";
}

template <typename state>
static void getVertexAnnotatedBiclique(state const& a,
                                state const& a2,
                                state const& b,
                                state const& b2,
                                float startTimeA,
                                float endTimeA,
                                float startTimeB,
                                float endTimeB,
                                std::vector<unsigned>& left,
                                std::vector<unsigned>& right,
                                std::vector<std::pair<float,float>>& livls,
                                std::vector<std::pair<float,float>>& rivls,
                                unsigned branchingFactor,
                                float rA=0.25,
                                float rB=0.25){
  getBiclique(a,a2,b,b2,startTimeA,endTimeA,startTimeB,endTimeB,left,right,branchingFactor,rA,rB);
  if(left.empty())return;
  std::sort(left.begin(),left.end());
  std::sort(right.begin(),right.end());
  //auto s(moveNum(b,b2,0,branchingFactor));
  //auto f(std::lower_bound(right.begin(),right.end(),moveNum(b,b2,0,branchingFactor)));
  //std::cout<<"core: " << a << "-->" << a2 << ", " << b << "-->" << b2 << "\n";
  unsigned ixA(std::lower_bound(left.begin(),left.end(),moveNum(a,a2,0,branchingFactor))-left.begin());
  unsigned ixB(std::lower_bound(right.begin(),right.end(),moveNum(b,b2,0,branchingFactor))-right.begin());
  std::vector<std::vector<std::pair<float,float>>> ivls;
  getEdgeAnnotatedBiclique(a,a2,b,b2,left,right,ivls,branchingFactor,rA,rB);
  std::cout << "From scratch:\n";
  std::cout << "left: " << left << "\nright: " << right << "\nintervals: "<<ivls << "\n";
  getVertexAnnotatedBiclique(ixA,ixB,startTimeA,endTimeA,startTimeB,endTimeB,left,right,ivls,livls,rivls,branchingFactor,rA,rB);
}

template <typename state>
static void loadCollisionTable(std::vector<unsigned>& array, std::vector<unsigned>& indices, std::vector<float>& ivls, unsigned bf=9, unsigned resolution=10, float rA=0.25, float rB=0.25, bool force=false){
  char fname[128];
  char fname2[128];
  unsigned num(getNumCases(bf));
  // Number of mirrored start points X connectivity X connectivity X number of bits in conn
  unsigned numCases(num*bf*bf);
  // Each entry consists of 1 bit for each move from A, 1 bit for each move from B, and all of their forbidden intervals
  array.resize((numCases*(bf*2))/sizeof(unsigned)+1,0);
  indices.resize(numCases);

  sprintf(fname,"collision%02d-bicliques_r%0.2f_r%0.2f.dat",bf-1,rA,rB);
  sprintf(fname2,"collision%02d-intervals_r%0.2f_r%0.2f.dat",bf-1,rA,rB);
  FILE* f(fopen(fname,"rb"));
  FILE* f2(fopen(fname2,"rb"));
  if(!force && f && f2){
    fread(array.data(),sizeof(unsigned),array.size(),f);
    fclose(f);
    unsigned total(0);
    for(unsigned caseNum(0); caseNum<num; ++caseNum){
      for(unsigned i(0); i<bf; ++i){
        for(unsigned j(0); j<bf; ++j){
          unsigned ix(caseNum+i*num+j*num*bf);
          indices[ix]=total;
          unsigned ltot(0);
          unsigned rtot(0);
          for(unsigned l(0); l<bf; ++l){
            ltot+=get(array.data(),ix+l*numCases);
          }
          for(unsigned r(0); r<bf; ++r){
            rtot+=get(array.data(),ix+(r+bf)*numCases);
          }
          total+=2*ltot*rtot;
        }
      }
    }
    ivls.resize(total);
    fread(ivls.data(),sizeof(float),ivls.size(),f2);
    fclose(f2);
  }else{
    //std::map<unsigned,unsigned> assn;
    state a,b;
    fromLocationIndex(0,a,b,bf-1);
    // pre-load bs ...  they are always the same ("b" is the center point, "a" is relative)
    state bs[bf];
    for(int j(0); j<bf; ++j){
      fetch(b,j,bs[j],bf);
    }
    for(unsigned caseNum(0); caseNum<num; ++caseNum){
      fromLocationIndex(caseNum,a,b,bf-1);
      for(unsigned i(0); i<bf; ++i){
        state a2;
        float endA(fetch(a,i,a2,bf));
        for(unsigned j(0); j<bf; ++j){
          state b2;
          float endB(fetch(b,j,b2,bf));
          // Debug
          //bool printme(fequal(b.x,2.0) && fequal(b.y,2.0) && fequal(b2.x,2.0) && fequal(b2.y,1.0) && fequal(a.x,2.0) && fequal(a.y,1.0) && fequal(a2.x,2.0) && fequal(a2.y,2.0));
          unsigned ix(caseNum+i*num+j*num*bf);
          //printme=(ix==75);
          //bool printme(true);
          //if(printme)std::cout << a << "-->" << a2 << ", " << b << "-->" << b2 << "\n";
          std::vector<unsigned> left;
          std::vector<unsigned> right;
          std::vector<std::vector<std::pair<float,float>>> tivls;
          getBiclique(a,a2,b,b2,0,endA,0,endB,left,right,bf,rA,rB);
          std::sort(left.begin(),left.end());
          std::sort(right.begin(),right.end());
          getEdgeAnnotatedBiclique(a,a2,b,b2,left,right,tivls,bf,rA,rB);
          //std::cout << "assign " << ix << "\n";
          for(auto const& l:left){
            set(array.data(),ix+l*numCases);
            //if(printme)std::cout << "setting (left) [" << l << "] " << caseNum << " " << i << " " << j << " : " <<  ix+l*numCases <<"\n";
          }
          for(auto const& r:right){
            set(array.data(),ix+(r+bf)*numCases);
            //if(printme)std::cout << "setting (right) [" << r << "] " << caseNum << " " << i << " " << j << " : " <<  ix+(r+bf)*numCases <<"\n";
          }
          //if(printme)std::cout << "index: " << ivls.size() << "\n";
          //if(assn.find(ix)!=assn.end()&&"This assignment has already taken place!");
          indices[ix]=ivls.size();
          //assn[ix]=ivls.size();
          //if(printme){
            //std::cout << "ivls index: " << ivls.size() << "\n";
            //for(int q(0); q<tivls.size(); ++q){
              //std::cout << q << ":\n";
              //for(int r(0); r<tivls[q].size(); ++r){
                //std::cout << "  j: " << tivls[q][r].first << "," << tivls[q][r].second << "\n";
              //}
            //}
          //}
          encodeIvls(tivls,ivls); // Flatten
        }
      }
    }

    f=fopen(fname,"wb");
    fwrite(array.data(),sizeof(unsigned),array.size(),f);
    fclose(f);
    f2=fopen(fname2,"wb");
    fwrite(ivls.data(),sizeof(float),ivls.size(),f2);
    fclose(f);
  }
}

template <typename state>
static void getVertexAnnotatedBiclique(state const& a,
                                state const& a2,
                                state const& b,
                                state const& b2,
                                float startTimeA,
                                float endTimeA,
                                float startTimeB,
                                float endTimeB,
                                std::vector<unsigned> const& array,
                                std::vector<unsigned> const& indices,
                                std::vector<float> const& ivls,
                                std::vector<unsigned>& left, // out
                                std::vector<unsigned>& right, // out
                                std::vector<std::pair<float,float>>& livls, // out
                                std::vector<std::pair<float,float>>& rivls,
                                unsigned branchingFactor,
                                float rA=0.25,
                                float rB=0.25){
  unsigned moveA=0,moveB=0;
  auto ix(getCaseNumber(a,a2,b,b2,moveA,moveB,branchingFactor));
  if(ix<0){
    left.clear();
    right.clear();
    livls.clear();
    rivls.clear();
    return;
  }
  unsigned numCases(indices.size());
  unsigned aix(0);
  unsigned bix(0);
  for(unsigned i(0); i<branchingFactor; ++i){
    if(get(array.data(),ix+i*numCases)){
      //std::cout << "get " << (ix+i*numCases) << "\n";
      if(i==moveA){
        aix=left.size();
      }
      left.push_back(i);
    }
    if(get(array.data(),ix+(branchingFactor+i)*numCases)){
      //std::cout << "get " << (ix+(branchingFactor+i)*numCases) << "\n";
      if(i==moveB){
        bix=right.size();
      }
      right.push_back(i);
    }
  }
  if(left.size()==0)return;
  static std::vector<std::vector<std::pair<float,float>>> intervals;
  intervals.resize(left.size());
  unsigned num(indices[ix]);
  for(auto& l:intervals){
    l.resize(0);
    l.reserve(right.size());
    for(unsigned r(0); r<right.size(); ++r){
      //std::cout << "ivls["<<num<<","<<(num+1)<<"]="<<ivls[num]<<","<<ivls[num+1]<<"\n";
      l.emplace_back(ivls[num],ivls[num+1]);
      num+=2;
    }
  }
  std::cout << "From disk:\n";
  std::cout << "left: " << left << "\nright: " << right << "\nintervals: "<<intervals << "\n";

  /*{
    //TODO: make sure that the biclique is the same, because the core action is not there...
    std::vector<unsigned> left1;
    std::vector<unsigned> right1;
    getBiclique(a,a2,b,b2,startTimeA,endTimeA,startTimeB,endTimeB,left1,right1,branchingFactor,rA,rB);
    std::sort(left1.begin(),left1.end());
    std::sort(right1.begin(),right1.end());
    if(left!=left1 || right!=right1){
      std::cout << "No match for biclique\n";
      std::cout << left << "!=" << left1 << "\n (or) \n";
      std::cout << right << "!=" << right1 << "\n";
      getBiclique(a,a2,b,b2,startTimeA,endTimeA,startTimeB,endTimeB,left1,right1,branchingFactor,rA,rB);
    }
    if(std::find(left.begin(),left.end(),moveA)==left.end()){
      std::cout << "Left core action not in biclique\n";
      std::cout << left << " missing " << moveA << "\n";
    }
    if(std::find(right.begin(),right.end(),moveB)==right.end()){
      std::cout << "right core action not in biclique\n";
      std::cout << right << " missing " << moveB << "\n";
    }
    auto ivl(getForbiddenInterval(a,a2,0,endTimeA-startTimeA,rA,b,b2,0,endTimeB-startTimeB,rB));
    auto ivlr(getForbiddenInterval(a,a2,0,endTimeA-startTimeA,.25,b,b2,0,endTimeB-startTimeB,.25));
    auto ivl2(intervals[aix][bix]);
    auto delay(startTimeA-startTimeB);
    if(fabs(ivl.first-ivl2.first)>.01 || fabs(ivl.second-ivl2.second)>.01){
      std::cout << "cached interval " << ivl2.first << "," << ivl2.second << "; does not match computed: " <<ivl.first << "," << ivl.second << "\n";
    }
    if(delay<ivl2.first || delay>ivl2.second){
      std::cout << "delay:" << delay << "is not in the interval " << ivl2.first << "," << ivl2.second << "\n";
    }
  }*/
  getVertexAnnotatedBiclique(aix,bix,startTimeA,endTimeA,startTimeB,endTimeA,left,right,intervals,livls,rivls,branchingFactor,rA,rB);
}


#endif
