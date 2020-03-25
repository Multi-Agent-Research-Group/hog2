#ifndef DigraphEnvironment_h_
#define DigraphEnvironment_h_
#include "GLUtil.h"
#include <vector>
#include <unordered_map>
#include "ConstrainedEnvironment.h"
#include "Utilities.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include "Map.h"
#include "apt.h"
//#include <valgrind/memcheck.h>

#ifndef _WIN32
#define __stdcall
#endif

#define Z -.04

void __stdcall tessErrorCB(GLenum errorCode)
{
    const GLubyte *errorStr;

    errorStr = gluErrorString(errorCode);
    std::cerr << "[ERROR]: " << errorStr << std::endl;
}

void __stdcall glVertex3dv( const GLdouble *v )
{
    //std::cout << "{"<<*v<<","<<(*(v+1))<<","<<Z<<"},\n";
    glVertex3d(*v,*(v+1),Z);//(-4.7,-1.3,1,-4.7,-1.4,0)
}


struct node_t{
  node_t():x(-1),y(-1),t(-1),id(-1),nc(0U){}
  node_t(unsigned xx, unsigned yy, unsigned i, unsigned tt=0):x(xx),y(yy),id(i),t(tt),nc(0){}
  node_t(node_t const& n, unsigned tt=0):x(n.x),y(n.y),id(n.id),t(tt?tt:n.t),nc(0){}
  operator TemporalVector3D()const{return TemporalVector3D(x,y,0,t/TIME_RESOLUTION_D);}
  operator Vector3D()const{return Vector3D(x,y,0);}
  explicit operator TemporalVector()const{return TemporalVector(x,y,t/TIME_RESOLUTION_D);}
  explicit operator Vector2D()const{return Vector2D(x,y);}
  bool operator==(node_t const& n)const{return id==n.id && t==n.t;}
  bool operator!=(node_t const& n)const{return id!=n.id || t!=n.t;}
  bool operator<(node_t const& n)const{return id==n.id?t<n.t:id<n.id;}
  bool sameLoc(node_t const& n)const{return id==n.id;}
  unsigned x : 13;
  unsigned y : 13;
  unsigned nc: 6;
  unsigned t : 20;
  unsigned id: 12;
  static float TIME_RESOLUTION;
  static double TIME_RESOLUTION_D;
  static unsigned TIME_RESOLUTION_U;
};
double node_t::TIME_RESOLUTION_D=1000.0;
float node_t::TIME_RESOLUTION=1000.0f;
unsigned node_t::TIME_RESOLUTION_U=1000u;

static inline std::ostream& operator<<(std::ostream& out, node_t const& val){
  out << "{\"x\"=" << val.x << ", \"y\":" << val.y << ", \"t\":" << val.t/node_t::TIME_RESOLUTION_D << ", \"id\":" << val.id << '}';
  return out;
}

struct edge_t{
  edge_t():a1(0xffff),a2(0xffff){}
  edge_t(unsigned i1, unsigned i2):a1(i1),a2(i2){if(a2<a1){a1=i2;a2=i1;}}
  uint16_t a1;
  uint16_t a2;
  size_t operator==(edge_t const&pt)const{return pt.a1==a1 && pt.a2==a2;}
  size_t operator()(edge_t const&pt)const{return pt.a1+pt.a2*0xffff;}
  bool operator<(edge_t const&pt)const{return pt.a1==a1?a2<pt.a2:a1<pt.a1;}
  static uint64_t getKey(unsigned a1, unsigned a2){return a1<a2?a1*0xffff+a2:a2*0xffff+a1;}
};

class DigraphEnvironment: public ConstrainedEnvironment<node_t, int>{
  public:
  DigraphEnvironment(char const* vfile, char const* efile, char const* afile, bool bidir=true, bool selfloops=true){
  //VALGRIND_CHECK_VALUE_IS_DEFINED(nodes);
  //VALGRIND_CHECK_VALUE_IS_DEFINED(adj);
  //VALGRIND_CHECK_VALUE_IS_DEFINED(weight);
  //VALGRIND_CHECK_VALUE_IS_DEFINED(width);
  //VALGRIND_CHECK_VALUE_IS_DEFINED(height);
  //VALGRIND_CHECK_VALUE_IS_DEFINED(map);

    if(afile && features.empty()){
      parseAptFile(afile,features); // Load airport features
    }
    if(nodes.size()){return;} // already loaded!
    nodes.reserve(255);
    adj.reserve(512);
    std::ifstream vs(vfile);
    std::string line;
    int n;
    std::string c,curve;
    while(std::getline(vs, line)){
      std::istringstream is(line);
      is >> n >> c;
      auto cs(Util::split(c,','));
      if(nodes.size()<n+1){nodes.resize(n+1);}
      if(adj.size()<n+1){adj.resize(n+1);}
      nodes[n]=node_t(atoi(cs[0].c_str()),atoi(cs[1].c_str()),n);
      width=std::max(width,nodes[n].x);
      height=std::max(height,nodes[n].y);
      if(selfloops){
        adj[n].push_back(n); // Self-loop
        weight[edge_t::getKey(n,n)]=node_t::TIME_RESOLUTION_U;
        auto const& v(weight.find(edge_t::getKey(n,n)));
        assert(v!=weight.end());
      }
    }
    assert(weight.find(edge_t::getKey(88,88))!=weight.end());
    std::ifstream es(efile);
    while(std::getline(es, line)){
      std::istringstream is(line);
      is >> c >> curve;
      auto cs(Util::split(c,','));
      unsigned n1(atoi(cs[0].c_str()));
      unsigned n2(atoi(cs[1].c_str()));
      assert(n1<nodes.size() && nodes[n1]!=node_t());
      assert(n2<nodes.size() && nodes[n2]!=node_t());
      if(adj.size()<std::max(n1,n2)+1){adj.resize(std::max(n1,n2)+1);}
      adj[n1].push_back(n2);
      if(bidir)
        adj[n2].push_back(n1);
      auto dist(Util::distance(nodes[n1].x,nodes[n1].y,nodes[n2].x,nodes[n2].y)*node_t::TIME_RESOLUTION_D);
      auto hash(edge_t::getKey(n1,n2));
      if(weight.find(edge_t::getKey(n1,n2))!=weight.end()){
        assert(!"already inserted");
      }
      //std::cout << e.a1 << "," << e.a2 << ":" << dist << "[" << node_t::TIME_RESOLUTION_D <<"\n";
      weight[hash]=dist;
    }
    //for(auto const& e:weight){
      //std::cout << e.first/0xffff << "," << e.first%0xffff << ":" << e.second << "[" << node_t::TIME_RESOLUTION_D <<"\n";
    //}
    /*for(int i(0); i<adj.size(); ++i){
      for(auto a:adj[i]){
        //std::cout << i << "," << a << "\n";
        assert(weight.find(edge_t::getKey(i,a))!=weight.end());
      }
    }*/
    map.reset(new Map(width,height));
  }
  static std::vector<node_t> nodes;
  static std::vector<std::vector<uint16_t>> adj;
  static std::unordered_map<uint64_t,uint32_t> weight;
  static unsigned width, height;
  static std::unique_ptr<Map> map;
  static FeatureList features;
  node_t goal;
  bool ignoreTime=false;

  inline void SetIgnoreTime(bool v){ignoreTime=v;}
  inline bool GetIgnoreTime()const{return ignoreTime;}
  inline uint64_t GetStateHash(node_t const& state)const{
    if(ignoreTime)return state.id;
    return (state.id << 20) | state.t;
  }

  virtual void GetSuccessors(const node_t &node, std::vector<node_t>& neighbors) const{
    neighbors.resize(16);
    neighbors.resize(GetSuccessors(node,&neighbors[0]));
  }

  virtual void GetActions(const node_t &node, std::vector<int>& actions) const{
    assert(false);
    // Not implemented
  }

  virtual void GetReverseActions(const node_t &node, std::vector<int>& actions) const{
    assert(false);
    // Not implemented
  }

  virtual void ApplyAction(node_t& node, int) const{
    assert(false);
    // Not implemented
  }

  virtual bool InvertAction(int&) const{
    assert(false);
    // Not implemented
  }

  virtual uint64_t GetActionHash(int) const{
    assert(false);
    // Not implemented
  }

void OpenGLDraw() const
{
  map->OpenGLDraw();
  std::vector<std::array<GLdouble,3>> v;
  v.reserve(100);
  // Draw feature polygons
  GLdouble v1[3]={0,0,0};
  GLdouble v2[3]={0,0,0};
  GLdouble rad=0,z=-.04;
  for(auto const& f:features){
    glColor4f(f.r,f.g,f.b,1);
    switch(f.featuretype){
      case apt_rwy_new:
        //glLineWidth(f.lwd);
        glBegin(GL_POLYGON);
        map->GetOpenGLCoord(f.nodes[0][1],f.nodes[0][0],-.04, v1[0], v1[1], v1[2], rad);
        glVertex3f(v1[0], -v1[1], z);
        map->GetOpenGLCoord(f.nodes[0][3],f.nodes[0][2],-.04, v1[0], v1[1], v1[2], rad);
        glVertex3f(v1[0], -v1[1], z);
        map->GetOpenGLCoord(f.nodes[1][3],f.nodes[1][2],-.04, v1[0], v1[1], v1[2], rad);
        glVertex3f(v1[0], -v1[1], z);
        map->GetOpenGLCoord(f.nodes[1][1],f.nodes[1][0],-.04, v1[0], v1[1], v1[2], rad);
        glVertex3f(v1[0], -v1[1], z);
        glEnd();
      break;
      case apt_taxi_new:
      {
//std::cout << f.name << "\n";
        GLdouble p[3]={0,0,z};
        GLdouble pb[3]={0,0,z};
        GLdouble b[3]={0,0,z};

        std::vector<std::array<GLdouble,3>> pts(11);
        //std::string sep1("");
        //std::string sep2("");
        //std::stringstream cx;
        //std::stringstream cy;


        GLUtesselator *tess(gluNewTess());
        assert(tess);
        gluTessCallback ( tess, GLU_TESS_BEGIN,  ( void ( __stdcall *) ( void ))glBegin     );
        gluTessCallback ( tess, GLU_TESS_VERTEX, ( void ( __stdcall *) ( void ))glVertex3dv );
        gluTessCallback ( tess, GLU_TESS_ERROR, ( void ( __stdcall *) ( void ))tessErrorCB );
        gluTessCallback ( tess, GLU_TESS_END,    ( void ( __stdcall *) ( void ))glEnd       );

        gluTessBeginPolygon(tess, NULL);
        //std::cout << "gluTessBeginContour(tess)\n";
        gluTessBeginContour(tess);
        auto prev(f.nodes.front());
        v2[0]=v2[1]=0.0;
        v.resize(0);
        for(auto const& n:f.nodes){
          //std::cout << "Dist: " << Util::distance(n[0],n[1],prev[0],prev[1]) << "\n";
          prev=n;
          memset(b, 0, sizeof(b)); // Reset bezier point
          pts.resize(0);
          map->GetOpenGLCoord(n[1],n[0],-.04, v1[0], v1[1], v1[2], rad);
          //v2[0]+=v1[0];
          //v2[1]+=v1[1];
          //std::cout << "point " << v1[0] << "," << v1[1];
          if(n[2]){
            map->GetOpenGLCoord(n[3],n[2],-.04, b[0], b[1], b[2], rad);
            GLdouble mx(2.0*v1[0] - b[0]);
            GLdouble my(2.0*v1[1] - b[1]);
            //std::cout << " mirror " << mx << "," << my;
            if(pb[0]){
              drawCubicBezier(p[0],p[1],pb[0],pb[1],mx,my,v1[0],v1[1],pts,z,10);
              for(unsigned i(0); i<pts.size(); ++i){
                //std::cout<<"gluTessVertex(tess, "<<pts[i]<<", NULL);\n";
                v.resize(v.size()+1);
                v.back()[0]=v1[0];
                v.back()[1]=-v1[1];
                v.back()[2]=Z;
                //std::cout<<"gluTessVertex(tess, "<<v.back()<<", NULL);\n";
                gluTessVertex(tess, v.back().data(), v.back().data());
                //gluTessVertex(tess, pts[i].data(), pts[i].data());
                //glVertex3f(xs[i], ys[i], -.04);
              }
              //for(unsigned i(0); i<pts.size(); ++i){
                //cx << sep1 << pts[i][0];
                //sep1=",";
              //}
              //for(unsigned i(0); i<pts.size(); ++i){
                //cy << sep2 << pts[i][1];
                //sep2=",";
              //}
            }else{
              drawQuadBezier(p[0],p[1],mx,my,v1[0],v1[1],pts,z,10);
              for(unsigned i(0); i<pts.size(); ++i){
                //std::cout<<"gluTessVertex(tess, "<<pts[i]<<", NULL);\n";
                v.resize(v.size()+1);
                v.back()[0]=v1[0];
                v.back()[1]=-v1[1];
                v.back()[2]=Z;
                //std::cout<<"gluTessVertex(tess, "<<v.back()<<", NULL);\n";
                gluTessVertex(tess, v.back().data(), v.back().data());
                //gluTessVertex(tess, pts[i].data(), pts[i].data());
                //glVertex3f(xs[i], ys[i], -.04);
              }
              //for(unsigned i(0); i<pts.size(); ++i){
                //cx << sep1 << pts[i][0];
                //sep1=",";
              //}
              //for(unsigned i(0); i<pts.size(); ++i){
                //cy << sep2 << pts[i][1];
                //sep2=",";
              //}
            }
            /*}else if(pb[0]){
              drawQuadBezier(p[0],p[1],pb[0],pb[1],v1[0],v2[0],xs.data(),ys.data(),10);
              for(unsigned i(0); i<xs.size(); ++i){
              glVertex3f(xs[i], ys[i], -.04);
              }
              for(unsigned i(0); i<xs.size(); ++i){
              cx << sep1 << xs[i];
              sep1=",";
              }
              for(unsigned i(0); i<xs.size(); ++i){
              cy << sep2 << ys[i];
              sep2=",";
              }*/
        }else{
          //std::cout<<"gluTessVertex(tess, v1, NULL);\n";
          v.resize(v.size()+1);
          v.back()[0]=v1[0];
          v.back()[1]=-v1[1];
          v.back()[2]=Z;
          //std::cout<<"gluTessVertex(tess, "<<v.back()<<", NULL);\n";
          gluTessVertex(tess, v.back().data(), v.back().data());
          //glVertex3f(v1[0],v1[1],-.04);
          //cx << sep1 << v1[0];
          //sep1=",";
          //cy << sep2 << v1[1];
          //sep2=",";
        }
        std::copy(std::begin(v1), std::end(v1), std::begin(p));
        std::copy(std::begin(b), std::end(b), std::begin(pb));

        //std::cout << "\n";
        }
        //DrawFmtTextCentered(v2[0]/f.nodes.size(),-v2[1]/f.nodes.size(),-.05,0.02,"%s",f.name.c_str());
        if(f.closed){
          //map->GetOpenGLCoord(f.nodes[0][1],f.nodes[0][0],-.04, v1[0], v1[1], v1[2], rad);
          //std::cout<<"gluTessVertex(tess, v1, NULL);\n";
          //gluTessVertex(tess, v1, v1);
          //glVertex3f(v1[0], v1[1], -.04);
          //cx << sep1 << v1[0];
                //sep1=",";
          //cy << sep2 << v1[1];
                //sep2=",";
        }
        //std::cout << "px=c("<<cx.str()<<")\npy=c("<<cy.str();
        //std::cout << ")\nplot(px,py,type='b')\n\n";
        //std::cout << "gluTessEndContour(tess)\n";
        gluTessEndContour(tess);
        gluTessEndPolygon(tess);
        gluDeleteTess(tess);
      }
      break;
    }
  }
  // Draw taxiway lines
  glColor4f(0, 0, 0, 1);
  glLineWidth(1.0);
  signed i(-1);
  for(auto const& a:adj){
    ++i;
    map->GetOpenGLCoord(nodes[i].x, nodes[i].y, -.05, v1[0], v1[1], v1[2], rad);
    for(auto const& n:a){

      glBegin(GL_LINES);
      glVertex3f(v1[0], v1[1], v1[2]);
      map->GetOpenGLCoord(nodes[n].x, nodes[n].y, -.05, v2[0], v2[1], v2[2], rad);
      glVertex3f(v2[0], v2[1], v2[2]);
      glEnd();
    }
    DrawFmtTextCentered(v1[0],v1[1],-.05,0.02,"%d",nodes[i].id);
  }
}

void OpenGLDraw(const node_t& s, const node_t& e, float perc, float radius) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord((1-perc)*s.x+perc*e.x, (1-perc)*s.y+perc*e.y, .1, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz, rad*radius); // zz-s.t*2*rad
}

void OpenGLDraw(const node_t& l) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord(l.x, l.y, .1, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz-l.t*rad, rad/2.0); // zz-l.t*2*rad
}

void OpenGLDraw(const node_t&, const int&) const
{
  std::cout << "Draw move\n";
}

void GLDrawLine(const node_t &x, const node_t &y) const
{
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord(x.x, x.y, .1, xx, yy, zz, rad);

  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  glColor4f(r, g, b, t);
  glBegin(GL_LINES);
  glVertex3f(xx, yy, zz);
  map->GetOpenGLCoord(y.x, y.y, .1, xx, yy, zz, rad);
  glVertex3f(xx, yy, zz);
  glEnd();
}

virtual unsigned GetSuccessors(const node_t &node, node_t* neighbors) const{
  unsigned sz(0);
  // Add wait actions
  std::set<float> times;
  node_t wait(node);
  for(auto const& a:adj[node.id]){
    auto const& v(weight.find(edge_t::getKey(node.id,a)));
    assert(v!=weight.end());
    node_t nn(nodes[a],node.t+v->second);
    if(!ViolatesConstraint(node,nn)){
      neighbors[sz++]=nn;
    }
    WaitTimes(node,nn,times);
    for(auto const& t:times){
      wait.t=node.t+t;
      if(!ViolatesConstraint(node,wait)){
        std::cout << "WAIT " << node << "-->" << wait << "\n";
        neighbors[sz++]=wait;
      }else{
        std::cout << "Can't WAIT " << node << "-->" << wait << "\n";
      }
    }
    times.clear();
  }
  return sz;
}

virtual bool GoalTest(const node_t& node, const node_t& ignored) const{
  return node.id==goal.id;
}

virtual double GCost(const node_t& node, const int& ignored) const{
  assert(false); // not implemented
}

// Staying at the goal costs nothing
virtual double GCost(const node_t& node1, const node_t& node2)const{
  assert(node1.id==node2.id || std::find(adj[node1.id].begin(),adj[node1.id].end(),node2.id)!=adj[node1.id].end());
  return node1.id==node2.id ? node1.id==goal.id ? 0 : node_t::TIME_RESOLUTION_D : weight.find(edge_t::getKey(node1.id,node2.id))->second;
}

virtual double HCost(const node_t& node1, const node_t& node2)const{
  return Util::distance(node1.x,node1.y,node2.x,node2.y);
}

bool collisionCheck(const node_t &s1, const node_t &d1, float r1, const node_t &s2, const node_t &d2, float r2){
  Vector3D A(s1);
  Vector3D VA(d1);
  VA-=A; // Direction vector
  VA.Normalize();
  Vector3D B(s2);
  Vector3D VB(d2);
  VB-=B; // Direction vector
  VB.Normalize();
  return collisionImminent(A,VA,r1,s1.t,d1.t,B,VB,r2,s2.t,d2.t);
  //return collisionCheck(s1,d1,s2,d2,double(r1),double(r2));
}
};

FeatureList DigraphEnvironment::features;
std::vector<node_t> DigraphEnvironment::nodes;
std::vector<std::vector<uint16_t>> DigraphEnvironment::adj;
std::unordered_map<uint64_t,uint32_t> DigraphEnvironment::weight;
unsigned DigraphEnvironment::width=0;
unsigned DigraphEnvironment::height=0;
std::unique_ptr<Map> DigraphEnvironment::map;

template <typename state, typename action>
class TieBreaking3D {
  public:

    bool operator()(const AStarOpenClosedData<state> &ci1, const AStarOpenClosedData<state> &ci2) const
    {
      if (fequal(ci1.g+ci1.h, ci2.g+ci2.h)) // F-cost equal
      {

        if(useCAT && CAT){
          // Make them non-const :)
          AStarOpenClosedData<state>& i1(const_cast<AStarOpenClosedData<state>&>(ci1));
          AStarOpenClosedData<state>& i2(const_cast<AStarOpenClosedData<state>&>(ci2));
          // Compute cumulative conflicts (if not already done)
          ConflictSet matches;
          if(i1.data.nc ==-1){
            //std::cout << "Getting NC for " << i1.data << ":\n";
            CAT->get(i1.data.t,i1.data.t+node_t::TIME_RESOLUTION_U,matches);

            // Get number of conflicts in the parent
            state const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
            unsigned nc1(parent1?parent1->nc:0);
            //std::cout << "  matches " << matches.size() << "\n";

            // Count number of conflicts
            for(auto const& m: matches){
              if(currentAgent == m.agent) continue;
              state p;
              currentEnv->GetStateFromHash(m.hash1,p);
              //p.t=m.start;
              state n;
              currentEnv->GetStateFromHash(m.hash2,n);
              //n.t=m.stop;
              collchecks++;
              nc1+=checkForConflict(parent1,&i1.data,&p,&n,agentRadius);
              //if(!nc1){std::cout << "NO ";}
              //std::cout << "conflict(1): " << i1.data << " " << n << "\n";
            }
            // Set the number of conflicts in the data object
            i1.data.nc=nc1;
          }
          if(i2.data.nc ==-1){
            //std::cout << "Getting NC for " << i2.data << ":\n";
            CAT->get(i2.data.t,i2.data.t+node_t::TIME_RESOLUTION_U,matches);

            // Get number of conflicts in the parent
            state const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
            unsigned nc2(parent2?parent2->nc:0);
            //std::cout << "  matches " << matches.size() << "\n";

            // Count number of conflicts
            for(auto const& m: matches){
              if(currentAgent == m.agent) continue;
              state p;
              currentEnv->GetStateFromHash(m.hash2,p);
              //p.t=m.start;
              state n;
              currentEnv->GetStateFromHash(m.hash2,n);
              //n.t=m.stop;
              collchecks++;
              nc2+=checkForConflict(parent2,&i2.data,&p,&n,agentRadius);
              //if(!nc2){std::cout << "NO ";}
              //std::cout << "conflict(2): " << i2.data << " " << n << "\n";
            }
            // Set the number of conflicts in the data object
            i2.data.nc=nc2;
          }
          if(fequal(i1.data.nc,i2.data.nc)){
            if(fequal(ci1.g,ci2.g)){
              if(randomalg && ci1.data.t==ci2.data.t){
                return rand()%2;
              }
              return ci1.data.t<ci2.data.t;  // Tie-break toward greater time (relevant for waiting at goal)
            }
            return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
          }
          return fgreater(i1.data.nc,i2.data.nc);
        }else{
          if(fequal(ci1.g,ci2.g)){
            if(randomalg && ci1.data.t==ci2.data.t){
              return rand()%2;
            }
            return ci1.data.t<ci2.data.t;  // Tie-break toward greater time (relevant for waiting at goal)
          }
          return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
        }
      }
      return (fgreater(ci1.g+ci1.h, ci2.g+ci2.h));
    }
    static OpenClosedInterface<state,AStarOpenClosedData<state>>* openList;
    static ConstrainedEnvironment<state,action>* currentEnv;
    static uint8_t currentAgent;
    static unsigned collchecks;
    static bool randomalg;
    static bool useCAT;
    static NonUnitTimeCAT<state,action>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action>
OpenClosedInterface<state,AStarOpenClosedData<state>>* TieBreaking3D<state,action>::openList=0;
template <typename state, typename action>
ConstrainedEnvironment<state,action>* TieBreaking3D<state,action>::currentEnv=0;
template <typename state, typename action>
uint8_t TieBreaking3D<state,action>::currentAgent=0;
template <typename state, typename action>
unsigned TieBreaking3D<state,action>::collchecks=0;
template <typename state, typename action>
bool TieBreaking3D<state,action>::randomalg=false;
template <typename state, typename action>
bool TieBreaking3D<state,action>::useCAT=false;
template <typename state, typename action>
NonUnitTimeCAT<state,action>* TieBreaking3D<state,action>::CAT=0;

template <typename state, typename action>
class UnitTieBreaking3D {
  public:
    bool operator()(const AStarOpenClosedData<state> &ci1, const AStarOpenClosedData<state> &ci2) const
    {
      if (fequal(ci1.g+ci1.h, ci2.g+ci2.h)) // F-cost equal
      {
        if(useCAT && CAT){
          // Make them non-const :)
          AStarOpenClosedData<state>& i1(const_cast<AStarOpenClosedData<state>&>(ci1));
          AStarOpenClosedData<state>& i2(const_cast<AStarOpenClosedData<state>&>(ci2));

          // Compute cumulative conflicts (if not already done)
          if(i1.data.nc ==-1){
            //std::cout << "Getting NC for " << i1.data << ":\n";

            // Get number of conflicts in the parent
            state const*const parent1(i1.parentID?&(openList->Lookat(i1.parentID).data):nullptr);
            unsigned nc1(parent1?parent1->nc:0);
            //std::cout << "  matches " << matches.size() << "\n";

            // Count number of conflicts
            for(int agent(0); agent<CAT->numAgents(); ++agent){
              if(currentAgent == agent) continue;
              state const* p(0);
              if(i1.data.t!=0)
                p=&(CAT->get(agent,(i1.data.t-node_t::TIME_RESOLUTION_U)/node_t::TIME_RESOLUTION_D));
              state const& n=CAT->get(agent,i1.data.t/node_t::TIME_RESOLUTION_D);
              collchecks++;
              nc1+=checkForTheConflict(parent1,&i1.data,p,&n);
            }
            // Set the number of conflicts in the data object
            i1.data.nc=nc1;
          }
          if(i2.data.nc ==-1){
            //std::cout << "Getting NC for " << i2.data << ":\n";

            // Get number of conflicts in the parent
            state const*const parent2(i2.parentID?&(openList->Lookat(i2.parentID).data):nullptr);
            unsigned nc2(parent2?parent2->nc:0);
            //std::cout << "  matches " << matches.size() << "\n";

            // Count number of conflicts
            for(int agent(0); agent<CAT->numAgents(); ++agent){
              if(currentAgent == agent) continue;
              state const* p(0);
              if(i2.data.t!=0)
                p=&(CAT->get(agent,(i2.data.t-node_t::TIME_RESOLUTION_U)/node_t::TIME_RESOLUTION_D));
              state const& n=CAT->get(agent,i2.data.t/node_t::TIME_RESOLUTION_D);
              collchecks++;
              nc2+=checkForTheConflict(parent2,&i2.data,p,&n);
            }
            // Set the number of conflicts in the data object
            i2.data.nc=nc2;
          }
          if(fequal(i1.data.nc,i2.data.nc)){
            if(randomalg && fequal(ci1.g,ci2.g)){
              return rand()%2;
            }
            return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
          }
          return fgreater(i1.data.nc,i2.data.nc);
        }else{
          if(randomalg && fequal(ci1.g,ci2.g)){
            return rand()%2;
          }
          return (fless(ci1.g, ci2.g));  // Tie-break toward greater g-cost
        }
      }
      return (fgreater(ci1.g+ci1.h, ci2.g+ci2.h));
    }
    static OpenClosedInterface<state,AStarOpenClosedData<state>>* openList;
    static ConstrainedEnvironment<state,action>* currentEnv;
    static uint8_t currentAgent;
    static unsigned collchecks;
    static bool randomalg;
    static bool useCAT;
    static double agentRadius;
    static UnitTimeCAT<state,action>* CAT; // Conflict Avoidance Table
};

template <typename state, typename action>
OpenClosedInterface<state,AStarOpenClosedData<state>>* UnitTieBreaking3D<state,action>::openList=0;
template <typename state, typename action>
ConstrainedEnvironment<state,action>* UnitTieBreaking3D<state,action>::currentEnv=0;
template <typename state, typename action>
uint8_t UnitTieBreaking3D<state,action>::currentAgent=0;
template <typename state, typename action>
unsigned UnitTieBreaking3D<state,action>::collchecks=0;
template <typename state, typename action>
bool UnitTieBreaking3D<state,action>::randomalg=false;
template <typename state, typename action>
bool UnitTieBreaking3D<state,action>::useCAT=false;
template <typename state, typename action>
double UnitTieBreaking3D<state,action>::agentRadius=0.25;
template <typename state, typename action>
UnitTimeCAT<state,action>* UnitTieBreaking3D<state,action>::CAT=0;

#endif
