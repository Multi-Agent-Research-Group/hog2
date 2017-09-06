/*
 *  RadialSafety2DObjectiveEnvironment.cpp
 *  hog2
 *
 *  Copyright 2017 Thayne Walker, University of Denver. All rights reserved.
 *
 */
#include "RadialSafety2DObjectiveEnvironment.h"
#include "PositionalUtils.h"

RadialSafety2DObjectiveEnvironment::RadialSafety2DObjectiveEnvironment(std::vector<xytLoc> const& center, std::vector<double> radius, unsigned w, unsigned h, double scale):c(center),r(radius),logr(radius.size()),width(w),height(h),s(scale){
  for(int i(0);i<r.size(); ++i){
    assert(fgreater(r[i],0.0) && "Radius must be greater than zero");
    logr[i]=log(r[i]);
  }
  buildHeuristic();
}

void RadialSafety2DObjectiveEnvironment::buildHeuristic(){
  // Find the min height along all columns and rows.
  // In order to go from x_1 to x_2, an agent absolutely must visit at least
  // the minimum height in the columns between them.
  rowMins.resize(height);
  colMins.resize(width);
  for(int x(0); x<width; ++x){
    colMins[x]=9999999;
  }
  for(int y(0); y<height; ++y){
    rowMins[y]=9999999;
  }
  for(int x(0); x<width; ++x){
    for(int y(0); y<height; ++y){
      double total(0.0);
      for(int i(0); i<c.size(); ++i){
        double d((Util::distance(x,y,c[i].x,c[i].y))/s); // Assume an edge pointing at the center (average distance is just the edge center)
        total += std::max(0.0,(logr[i]/std::max(TOLERANCE,d)-logr[i]/r[i]));
      }
      rowMins[y]=std::min(rowMins[y],total);
      colMins[x]=std::min(colMins[x],total);
    }
  }
}

double RadialSafety2DObjectiveEnvironment::GCost(xytLoc const& a, xytLoc const& b) const {
  double total(0.0);
  // Cost of waiting in place for 1 second
  if(a.sameLoc(b)){
    for(int i(0); i<c.size(); ++i){
      double len(fabs(b.t-a.t)); // Wait time
      double d(std::max(TOLERANCE,Util::distance(a.x,a.y,c[i].x,c[i].y)/s)); // Distance from center
      total += std::max(TOLERANCE,(logr[i]/d-logr[i]/r[i])*len);
    }
  }else{
    // Cost of traveling an edge
    for(int i(0); i<c.size(); ++i){
      auto secant(Util::secantLine(a,b,c[i],r[i])); // Get portion of edge inside the radius
      double len(Util::distance(secant.first.x,secant.first.y,secant.second.x,secant.second.y)); // Length of the edge
      double d(std::max(TOLERANCE,Util::meanDistanceOfPointToLine(secant.first,secant.second,c[i]))); // Average distance from the center
      total += std::max(0.0,(logr[i]/d-logr[i]/r[i])*len);
    }
  }
  //std::cout << "GCOST " << a << "-->" << b << total <<"\n";
  return total;
}

double RadialSafety2DObjectiveEnvironment::HCost(xytLoc const& n, xytLoc const& g) const {
  double total(0.0);
  for(int i(0); i<c.size(); ++i){
    double dist(Util::distance(g.x,g.y,c[i].x,c[i].y)/s);
    if(dist < r[i]){ // Is goal inside radius?
      total += std::max(0.0,(logr[i]/std::max(TOLERANCE,dist+(r[i]-dist)/2.0)-logr[i]/r[i])*(r[i]-dist)); // Cost to go directly toward the center is cheapest.
    } // else zero... we may not have to enter the dangerous zone at all.
    dist=Util::distance(n.x,n.y,c[i].x,c[i].y)/s;
    if(dist < r[i]){ // Is current loc inside radius?
      total += std::max(0.0,(logr[i]/std::max(TOLERANCE,dist+(r[i]-dist)/2.0)-logr[i]/r[i])*(r[i]-dist)); // Cost to go directly away from the center is cheapest.
    } // else zero... we're not in a dangerous zone.
  }
  //std::cout << "HCOST " << n << "-->" << g << total <<"\n";

  // Will we have to cross the dangerous zone to arrive at our goal???
  double val(0);
  int xmax(std::max(n.x,g.x));
  int ymax(std::max(n.y,g.y));
  for(int x(std::min(n.x,g.x)); x<xmax; ++x){
    val+=colMins[x];
  }
  for(int y(std::min(n.y,g.y)); y<ymax; ++y){
    val+=rowMins[y];
  }
  return std::max(val,total);
}

void RadialSafety2DObjectiveEnvironment::OpenGLDraw(Map* map)const{
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);
  GLdouble xx, yy, zz, rr;
  
  for(int y(0); y<map->GetMapHeight(); ++y){
    for(int x(0); x<map->GetMapWidth(); ++x){
      double v(GCost({x,y,0},{x,y,1}));
      if(v){
        glColor4f(1., 0., 0.,v);
        glBegin(GL_QUADS);

        map->GetOpenGLCoord(x, y, xx, yy, zz, rr);
        zz=-.001;
        glVertex3f(xx-rr, yy-rr, zz);
        glVertex3f(xx-rr, yy+rr, zz);
        glVertex3f(xx+rr, yy+rr, zz);
        glVertex3f(xx+rr, yy-rr, zz);
        glEnd();
      }
    }
  }
  glDisable(GL_BLEND);
  for(int i(0); i<c.size(); ++i){
    map->GetOpenGLCoord(c[i].x, c[i].y, xx, yy, zz, rr);
    GLDrawCircle(xx,yy,rr*r[i]*2.0);
  }
}
