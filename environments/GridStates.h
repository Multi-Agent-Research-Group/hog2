#ifndef GRID_STATES_H__
#define GRID_STATES_H__

#include "Map.h"
#include "VelocityObstacle.h"
#include "ConstrainedEnvironment.h"

// Utility function
void GLDrawCircle(GLfloat x, GLfloat y, GLfloat radius);

struct timecoord {
  timecoord():x(0),y(0),z(0),t(0){}
  timecoord(uint16_t _x,uint16_t _y,uint16_t _z,uint16_t _t):x(0),y(0),z(0),t(0){}

  operator Vector2D()const{return Vector2D(x,y);}
  operator Vector3D()const{return Vector3D(x,y,z);}

  bool operator<(timecoord const& other)const{return x==other.x?(y==other.y?(z==other.z?t<other.t:z<other.z):y<other.y):x<other.x;}
  uint64_t hash(){return (uint64_t(x)<<48)|(uint64_t(y)<<32)|(uint64_t(z)<<16)|t;}
  
  uint16_t x,y,z;
  uint32_t t; // Note: t=millisec
};

struct tLoc {
  tLoc():t(0),nc(-1){}
  tLoc(float time):t(time),nc(-1){}
  float t;
  int16_t nc; // Number of conflicts, for conflict avoidance table
  virtual bool operator==(tLoc const& other)const{return fequal(t,other.t);}
  virtual bool sameLoc(tLoc const& other)const{return true;}
  virtual void print(std::ostream& os)const{os<<"("<<t<<")";}
  virtual uint16_t X()const{return 0;}
  virtual uint16_t Y()const{return 0;}
  virtual uint16_t Z()const{return 0;}
};

struct xyLoc {
  public:
    xyLoc():x(-1),y(-1),landed(false){}
    xyLoc(uint16_t _x, uint16_t _y, bool l=false) :x(_x), y(_y), landed(l) {}
    bool operator<(xyLoc const& other)const{return x==other.x?y<other.y:x<other.x;}
    uint16_t x;
    uint16_t y;
    bool landed; // Have we already arrived at the goal? (always leave this false if agent can block other agents)
    operator Vector2D()const{return Vector2D(x,y);}
    virtual bool sameLoc(xyLoc const& other)const{return x==other.x&&y==other.y;}
    virtual bool operator==(xyLoc const& other)const{return sameLoc(other);}
    virtual bool operator!=(xyLoc const& other)const{return !operator==(other); }

    virtual void print(std::ostream& os)const{os<<"("<<x<<","<<y<<")";}
};

struct xyLocHash
{
	std::size_t operator()(const xyLoc & x) const
	{
		return (x.x<<16)|(x.y);
	}
};

struct xytLoc : xyLoc, tLoc {
  xytLoc(xyLoc loc, float time):xyLoc(loc),tLoc(time), h(0){}
  xytLoc(xyLoc loc, uint16_t _h, float time):xyLoc(loc),tLoc(time), h(_h){}
  xytLoc(uint16_t _x, uint16_t _y):xyLoc(_x,_y),tLoc(), h(0){}
  xytLoc(uint16_t _x, uint16_t _y, float time):xyLoc(_x,_y),tLoc(time), h(0){}
  xytLoc(uint16_t _x, uint16_t _y, uint16_t _h, float time):xyLoc(_x,_y),tLoc(time), h(_h){}
  xytLoc():xyLoc(),tLoc(),h(0){}
  operator TemporalVector3D()const{return TemporalVector3D(x,y,0,t);}
  operator Vector2D()const{return Vector2D(x,y);}
  virtual bool sameLoc(xytLoc const& other)const{return x==other.x&&y==other.y;}
  virtual bool operator==(xytLoc const& other)const{return sameLoc(other)&&tLoc::operator==(other);}
  virtual void print(std::ostream& os)const{os<<"("<<x<<","<<y<<","<<t<<")";}
  uint16_t h; // Heading quantized to epsilon=1/(2**16-1)... 0=north max=north-epsilon
  int16_t nc; // Number of conflicts, for conflict avoidance table
  virtual uint16_t X()const{return x;}
  virtual uint16_t Y()const{return y;}
};

struct xyzLoc {
  public:
    xyzLoc():x(0),y(0),z(0){}
    xyzLoc(unsigned _x, unsigned _y, unsigned _z=0):x(_x),y(_y),z(_z){}
    bool operator<(xyzLoc const& other)const{return x==other.x?(y==other.y?z<other.z:y<other.y):x<other.x;}
    operator Vector3D()const{return Vector3D(x,y,z);}
    explicit operator Vector2D()const{return Vector2D(x,y);}
    virtual bool sameLoc(xyzLoc const& other)const{return x==other.x&&y==other.y&&z==other.z;}
    virtual bool operator==(xyzLoc const& other)const{return sameLoc(other);}
    virtual bool operator!=(xyzLoc const& other)const{return !sameLoc(other);}
    virtual void print(std::ostream& os)const{os<<"("<<x<<","<<y<<","<<z<<")";}
    unsigned x : 12;
    unsigned y : 12;
    unsigned z : 10;
};

struct xyztLoc : xyzLoc {
  xyztLoc(xyzLoc const& loc, unsigned time):xyzLoc(loc),t(time), h(0){}
  xyztLoc(xyzLoc const& loc, double time):xyzLoc(loc),t(round(time*TIME_RESOLUTION_D)), h(0){}
  xyztLoc(xyzLoc const& loc, float time):xyzLoc(loc),t(round(time*TIME_RESOLUTION)), h(0){}
  xyztLoc(xyzLoc const& loc, unsigned _h, float time):xyzLoc(loc),t(round(time*TIME_RESOLUTION)), h(_h){}
  xyztLoc(unsigned _x, unsigned _y):xyzLoc(_x,_y,0),t(0), h(0){}
  xyztLoc(unsigned _x, unsigned _y, unsigned _z, float time):xyzLoc(_x,_y,_z),t(round(time*TIME_RESOLUTION)), h(0){}
  xyztLoc(unsigned _x, unsigned _y, unsigned _z):xyzLoc(_x,_y,_z),t(0), h(0){}
  xyztLoc(unsigned _x, unsigned _y, unsigned _z, unsigned time):xyzLoc(_x,_y,_z),t(time), h(0){}
  xyztLoc():xyzLoc(),h(0){}
  operator TemporalVector3D()const{return TemporalVector3D(x,y,z,t/TIME_RESOLUTION_D);}
  explicit operator TemporalVector()const{return TemporalVector(x,y,t/TIME_RESOLUTION_D);}
  operator Vector3D()const{return Vector3D(x,y,z);}
  explicit operator Vector2D()const{return Vector2D(x,y);}
  unsigned t : 20; // Time (milliseconds)
  unsigned h : 10; // Heading
  //int16_t p; // Pitch
  virtual bool sameLoc(xyztLoc const& other)const{return xyzLoc::sameLoc(other);}
  virtual bool operator==(xyztLoc const& other)const{return sameLoc(other)&&t==other.t;}
  virtual bool operator!=(xyztLoc const& other)const{return x!=other.x||y!=other.y||z!=other.z||t!=other.t;}
  virtual void print(std::ostream& os)const{os<<"("<<x<<","<<y<<","<<z<<","<<float(t)/TIME_RESOLUTION<<")";}
  static const float HDG_RESOLUTION;
  static const float PITCH_RESOLUTION;
  static const float TIME_RESOLUTION;
  static const unsigned TIME_RESOLUTION_U;
  static const double TIME_RESOLUTION_D;
};

struct AANode : xyLoc {
  AANode(uint16_t _x, uint16_t _y):xyLoc(_x,_y),F(0),g(0),Parent(nullptr){}
  AANode():xyLoc(0,0),F(0),g(0),Parent(nullptr){}
  float   F;
  float   g;
  AANode*   Parent;
  std::pair<double,double> interval;
};

struct Hashable{
  virtual uint64_t Hash()const=0;
  virtual uint32_t Depth()const=0;
};

/*std::ostream& operator <<(std::ostream & out, const TemporalVector &loc);
std::ostream& operator <<(std::ostream & out, const xytLoc &loc);
std::ostream& operator <<(std::ostream & out, const xyLoc &loc);
std::ostream& operator <<(std::ostream & out, const tLoc &loc);
*/
static inline std::ostream& operator <<(std::ostream & out, const TemporalVector &loc) {
  out << "(" << loc.x << ", " << loc.y << ": " << loc.t << ")";
  return out;
}

static inline std::ostream& operator <<(std::ostream & out, const xytLoc &loc) {
  loc.print(out);
  return out;
}

static inline std::ostream& operator <<(std::ostream & out, const xyLoc &loc) {
  loc.print(out);
  return out;
}

static inline std::ostream& operator <<(std::ostream & out, const tLoc &loc) {
  loc.print(out);
  return out;
}

static inline std::ostream& operator <<(std::ostream & out, const xyzLoc &loc)
{
  loc.print(out);
  return out;
}

static inline std::ostream& operator <<(std::ostream & out, const xyztLoc &loc)
{
  loc.print(out);
  return out;
}


enum tDirection {
	kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE,
	kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE, kAll = 0xFFF,
		kNN=0x80,
		kSS=0x40,
		kEE=0x20,
		kWW=0x10,
		kNNE=kNN|kE,
		kNEE=kN|kEE,
		kNNEE=kNN|kEE,
		kSSE=kSS|kE,
		kSEE=kS|kEE,
		kSSEE=kSS|kEE,
		kSSW=kSS|kW,
		kSWW=kS|kWW,
		kSSWW=kSS|kWW,
		kNNW=kNN|kW,
		kNWW=kN|kWW,
		kNNWW=kNN|kWW,

		kNNN=0x800,
		kSSS=0x400,
		kEEE=0x200,
		kWWW=0x100,
		kNNNE=kNNN|kE,
		kNEEE=kN|kEEE,
		kNNNEE=kNNN|kEE,
		kNNEEE=kNN|kEEE,
		kNNNEEE=kNNN|kEEE,
		kSSSE=kSSS|kE,
		kSEEE=kS|kEEE,
		kSSEEE=kSS|kEEE,
		kSSSEE=kSSS|kEE,
		kSSSEEE=kSSS|kEEE,
		kSSSW=kSSS|kW,
		kSWWW=kS|kWWW,
		kSSWWW=kSS|kWWW,
		kSSSWW=kSSS|kWW,
		kSSSWWW=kSSS|kWWW,
		kNNNW=kNNN|kW,
		kNWWW=kN|kWWW,
		kNNNWW=kNNN|kWW,
		kNNWWW=kNN|kWWW,
		kNNNWWW=kNNN|kWWW
};

class BaseMapOccupancyInterface : public OccupancyInterface<xyLoc,tDirection>
{
public:
	BaseMapOccupancyInterface(Map* m);
	virtual ~BaseMapOccupancyInterface();
	virtual void SetStateOccupied(const xyLoc&, bool);
	virtual bool GetStateOccupied(const xyLoc&);
	virtual bool CanMove(const xyLoc&, const xyLoc&);
	virtual void MoveUnitOccupancy(const xyLoc &, const xyLoc&);

private:
	//BitVector *bitvec; /// For each map position, set if occupied
	std::vector<bool> bitvec;
	long mapWidth; /// Used to compute index into bitvector
	long mapHeight; /// used to compute index into bitvector

	long CalculateIndex(uint16_t x, uint16_t y);
};

// Ignore time component
static inline double distanceSquared(xyzLoc const& A1, xyzLoc const& B1){
  return (A1.x-B1.x)*(A1.x-B1.x) + (A1.y-B1.y)*(A1.y-B1.y) + (A1.z-B1.z)*(A1.z-B1.z);
}

static inline double distanceSquared(xyztLoc const& A1, xyztLoc const& A2, xyztLoc const& B1, xyztLoc const& B2){
  if(A1.t<B1.t){
    TemporalVector3D vec(A2);
    TemporalVector3D pos1(A1);
    vec-=pos1; // Get pointing vector from A1 to A2
    vec.Normalize();
    TemporalVector3D pos2(B1);
    pos1+=vec*(pos2.t-pos1.t); // Project along pointing vector
    pos2-=pos1; // Take difference
    return pos2.sq(); // Squared distance
  }else if(B1.t<A1.t){
    TemporalVector3D vec(B2);
    TemporalVector3D pos1(B1);
    vec-=pos1; // Get pointing vector from A1 to A2
    vec.Normalize();
    TemporalVector3D pos2(A1);
    pos1+=vec*(pos2.t-pos1.t); // Project along pointing vector
    pos2-=pos1; // Take difference
    return pos2.sq(); // Squared distance
  }

  // If A1.t==B1.t, the solution is much simpler
  return distanceSquared(A1,B1);
}

#endif
