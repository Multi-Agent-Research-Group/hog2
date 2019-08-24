#ifndef DigraphEnvironment_h_
#define DigraphEnvironment_h_
#include <vector>
#include <map>
#include "ConstrainedEnvironment.h"
#include "Utilities.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include "Map3d.h"

struct node_t{
  node_t():x(-1),y(-1),t(-1),id(-1),nc(0){}
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
};

class DigraphEnvironment: public ConstrainedEnvironment<node_t, int>{
  public:
  DigraphEnvironment(char const* vfile, char const* efile):width(0),height(0){
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
      nodes[n]=node_t(atoi(cs[0].c_str()),atoi(cs[1].c_str()),n);
      width=std::max(width,nodes[n].x);
      height=std::max(height,nodes[n].y);
    }
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
      adj[n2].push_back(n1);
      weight.emplace(edge_t(n1,n2),Util::distance(nodes[n1].x,nodes[n1].y,nodes[n2].x,nodes[n2].y)*node_t::TIME_RESOLUTION_D);
    }
    map.reset(new Map3D(width,height,1));
  }
  std::vector<node_t> nodes;
  std::vector<std::vector<uint16_t>> adj;
  std::map<edge_t,uint16_t> weight;
  node_t goal;
  unsigned width, height;
  bool ignoreTime=false;
  std::unique_ptr<Map3D> map;

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
}

void OpenGLDraw(const node_t& s, const node_t& e, float perc) const
{
  GLfloat r, g, b, t;
  GetColor(r, g, b, t);
  GLdouble xx, yy, zz, rad;
  map->GetOpenGLCoord((1-perc)*s.x+perc*e.x, (1-perc)*s.y+perc*e.y, .1, xx, yy, zz, rad);
  glColor4f(r, g, b, t);
  DrawSphere(xx, yy, zz, rad/2.0); // zz-s.t*2*rad
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
  // Assume waiting is allowed
  if(!ignoreTime){
    node_t wait(node,node.t+node_t::TIME_RESOLUTION_D);
    if(!ViolatesConstraint(node,wait)){
      neighbors[sz++]=wait;
    }
  }
  for(auto const& a:adj[node.id]){
    node_t nn(nodes[a],node.t+weight.find(edge_t(node.id,a))->second);
    if(!ViolatesConstraint(node,nn)){
      neighbors[sz++]=nn;
    }
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
  return node1.id==node2.id ? node1.id==goal.id ? 0 : node_t::TIME_RESOLUTION_D : weight.find({node1.id,node2.id})->second;
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
