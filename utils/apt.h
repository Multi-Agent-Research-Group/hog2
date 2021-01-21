#ifndef _apt_h__
#define _apt_h__

#include "Utilities.h"
#include "aptdefs.h"

#include <sstream>
#include <fstream>
#include <array>

double min(int arg_count, ...){ 
    double m, a; 
  
    // va_list is a type to hold information about 
    // variable arguments 
    va_list ap; 
  
    // va_start must be called before accessing 
    // variable argument list 
    va_start(ap, arg_count); 
  
    // Now arguments can be accessed one by one 
    // using va_arg macro. Initialize m as first 
    // argument in list 
    m = va_arg(ap, double); 
  
    // traverse rest of the arguments to find out minimum 
    for(int i(1); i<arg_count; i++) 
        if((a = va_arg(ap, double)) < m) m = a; 
  
    // va_end should be executed before the function 
    // returns whenever va_start has been previously 
    // used in that function 
    va_end(ap); 
  
    return m; 
} 

double max(int arg_count, ...){ 
    double m, a; 
  
    va_list ap; 
    va_start(ap, arg_count); 
    m = va_arg(ap, double); 
    for(int i(1); i<arg_count; i++) 
        if((a = va_arg(ap, double)) > m) m = a; 
    va_end(ap); 
  
    return m; 
} 

struct AptFeature{
  std::string name;
  unsigned featuretype;
  std::vector<std::array<float, 4>> nodes;
  float r,g,b;
  float lwd=0; // 0=filled, unfilled otherwise.
  bool closed=false;
};

void lineTypeToRGB(unsigned s, float& r, float& g, float& b){
  switch(s){
    case 102: // Blue lighting
      r=g=.5;
      b=1.0;
      break;
    case 104: // Hold lines (yellow)
      r=g=1.0;
      b=.5;
      break;
    default:
      r=g=b=0.0; // Black
      break;
  }
}

void surfaceToRGB(unsigned s, float& r, float& g, float& b){
  switch(s){
    case 1: // Asphalt
      r=g=b=.8;
      break;
    case 3: // Turf
      r=b=.6;
      g=1.0;
      break;
    case 4: // Dirt
      r=.9;
      g=.8;
      b=.6;
      break;
    case 2: // Concrete
    case 5: // Gravel
    case 512: // Dry lakebed
    default:
      r=g=b=.9;
      break;
  }
}

typedef std::vector<AptFeature> FeatureList;

void rotatePoint(float& x, float& y, float cx, float cy, float theta){
  auto cosT(cos(theta));
  auto sinT(sin(theta));
  auto dx(x-cx);
  auto dy(y-cy);
  x = cosT * dx - sinT * dy + cx;
  y = sinT * dx + cosT * dy + cy;
}

void parseAptFile(std::string const& fname, FeatureList& features, float ytrans=0.0035, float scale=1.08, float theta=0){
  theta*=M_PI/180.0; // to rads
  static float const DEG2M(111319.488);

  std::ifstream ss(fname.c_str());
  std::string line;
  std::string a; // placeholder for throwaways
  float minLat(9999999);
  float maxLat(-9999999);
  float minLon(9999999);
  float maxLon(-9999999);
  while(std::getline(ss, line)){
    std::istringstream is(line);
    unsigned ftype(0);
    is >> ftype;
    switch(ftype){
      case apt_airport:
        {
          signed elev;
          std::string icao,name;
          is >> elev;
          is >> a;
          is >> a;
          is >> icao;
          is >> name;
          std::string tmp;
          while(is >> tmp){
            name.append(" ");
            name.append(tmp);
          }
          //std::cout << "Parsing file for "<<name <<"\n";
        }
        break;
      case apt_rwy_new:
        {
          AptFeature f;
          f.featuretype=ftype;
          f.nodes.resize(2);
          std::string n2;
          unsigned surface;
          is >> f.lwd;
          is >> surface;
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> f.name;
          is >> f.nodes[0][0];
          is >> f.nodes[0][1];
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> n2;
          is >> f.nodes[1][0];
          is >> f.nodes[1][1];
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          is >> a;
          // Convert to polygon (rectangle)
          float dy(f.nodes[0][0]-f.nodes[1][0]);
          float dx(f.nodes[0][1]-f.nodes[1][1]);
          auto t(atan2(dy,dx));
          // Project 90 degrees in both directions
          float tmp(f.nodes[0][0]);
          double prj(f.lwd/DEG2M/2.0);
          f.nodes[0][0]=tmp+cos(t+M_PI/2.0)*prj;
          f.nodes[0][2]=tmp+cos(t-M_PI/2.0)*prj;
          tmp=f.nodes[0][1];
          f.nodes[0][1]=tmp+sin(t+M_PI/2.0)*prj;
          f.nodes[0][3]=tmp+sin(t-M_PI/2.0)*prj;
          tmp=f.nodes[1][0];
          f.nodes[1][0]=tmp+cos(t+M_PI/2.0)*prj;
          f.nodes[1][2]=tmp+cos(t-M_PI/2.0)*prj;
          tmp=f.nodes[1][1];
          f.nodes[1][1]=tmp+sin(t+M_PI/2.0)*prj;
          f.nodes[1][3]=tmp+sin(t-M_PI/2.0)*prj;
          minLat=min(5,minLat,f.nodes[0][0],f.nodes[1][0],f.nodes[0][2],f.nodes[1][2]);
          maxLat=max(5,maxLat,f.nodes[0][0],f.nodes[1][0],f.nodes[0][2],f.nodes[1][2]);
          minLon=min(5,minLon,f.nodes[0][1],f.nodes[1][1],f.nodes[0][3],f.nodes[1][3]);
          maxLon=max(5,maxLon,f.nodes[0][1],f.nodes[1][1],f.nodes[0][3],f.nodes[1][3]);
          surfaceToRGB(surface,f.r,f.g,f.b);
          f.name.append("/");
          f.name.append(n2);
          features.push_back(f);
        }
        break;
      case apt_taxi_new:
        {
          AptFeature f;
          f.featuretype=ftype;
          unsigned surface;
          is >> surface;
          is >> a;
          is >> a;
          is >> f.name;
          surfaceToRGB(surface,f.r,f.g,f.b);
          std::string tmp;
          while(is >> tmp){
            f.name.append(" ");
            f.name.append(tmp);
          }
          features.push_back(f);
        }
        break;
      case apt_free_chain:
        {
          AptFeature f;
          f.featuretype=ftype;
          is >> f.name;
          lineTypeToRGB(104,f.r,f.g,f.b);
          std::string tmp;
          while(is >> tmp){
            f.name.append(" ");
            f.name.append(tmp);
          }
          f.lwd=3;
          features.push_back(f);
        }
        break;
      case apt_lin_seg:
      case apt_rng_seg:
      case apt_end_seg:
        {
          AptFeature& f(features.back());
          decltype(f.nodes)::value_type n;
          is >> n[0];
          is >> n[1];
          n[2]=n[3]=0;
          if(ftype==apt_rng_seg)f.closed=true;
          f.nodes.push_back(n);
          minLat=std::min(minLat,n[0]);
          maxLat=std::max(maxLat,n[0]);
          minLon=std::min(minLon,n[1]);
          maxLon=std::max(maxLon,n[1]);
        }
        break;
      case apt_lin_crv:
      case apt_rng_crv:
      case apt_end_crv:
        {
          AptFeature& f(features.back());
          decltype(f.nodes)::value_type n;
          is >> n[0];
          is >> n[1];
          is >> n[2];
          is >> n[3];
          if(ftype==apt_rng_crv)f.closed=true;
          f.nodes.push_back(n);
          minLat=std::min(minLat,n[0]);
          maxLat=std::max(maxLat,n[0]);
          minLon=std::min(minLon,n[1]);
          maxLon=std::max(maxLon,n[1]);
        }
        break;
      default:
        //std::cout << "Ignoring line in apt file: \""<<line<<"\"\n";
        break;
    }
  }
  // Allow extra translation on y-axis...
  minLat+=ytrans;

  // Convert to local tangent plane coords
  float cLat((maxLat+minLat)/2.0);
  float cLon((maxLon+minLon)/2.0);
  float cosLat(cos((cLat*M_PI)/180.0)); // convert to radians
  float cx((cLon-minLon)*cosLat*DEG2M);
  float cy((cLat-minLat)*DEG2M);

  for(auto& f:features){
    for(auto& n:f.nodes){
      n[0]-=minLat; // Make relative to min
      n[0]*=DEG2M*scale; // Convert to meters
      n[1]-=minLon;
      n[1]*=cosLat*DEG2M*scale;
      rotatePoint(n[1],n[0],cx,cy,theta);
      if(n[2]){ // leave zeroes alone
        n[2]-=minLat;
        n[2]*=DEG2M*scale;
        n[3]-=minLon;
        n[3]*=cosLat*DEG2M*scale;
        rotatePoint(n[3],n[2],cx,cy,theta);
      }
    }
  }
}

#endif
