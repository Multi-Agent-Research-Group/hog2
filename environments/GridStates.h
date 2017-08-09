#ifndef GRID_STATES_H__
#define GRID_STATES_H__

#include "Map.h"
#include "VelocityObstacle.h"
#include "ConstrainedEnvironment.h"

struct xyLoc {
public:
	xyLoc():x(-1),y(-1),landed(false){}
	xyLoc(uint16_t _x, uint16_t _y, bool l=false) :x(_x), y(_y), landed(l) {}
        bool operator<(xyLoc const& other)const{return x==other.x?y<other.y:x<other.x;}
	uint16_t x;
	uint16_t y;
        bool landed; // Have we already arrived at the goal? (always leave this false if agent can block other agents)
};

struct xyLocHash
{
	std::size_t operator()(const xyLoc & x) const
	{
		return (x.x<<16)|(x.y);
	}
};

struct xytLoc : xyLoc {
	xytLoc(xyLoc loc, float time):xyLoc(loc), h(0), t(time) ,nc(-1){}
	xytLoc(xyLoc loc, uint16_t _h, float time):xyLoc(loc), h(_h), t(time) ,nc(-1){}
	xytLoc(uint16_t _x, uint16_t _y, float time):xyLoc(_x,_y), h(0), t(time) ,nc(-1){}
	xytLoc(uint16_t _x, uint16_t _y, uint16_t _h, float time):xyLoc(_x,_y), h(_h), t(time) ,nc(-1){}
	xytLoc():xyLoc(),h(0),t(0),nc(-1){}
	float t;
        uint16_t h; // Heading quantized to epsilon=1/(2**16-1)... 0=north max=north-epsilon
        int16_t nc; // Number of conflicts, for conflict avoidance table
        operator Vector2D()const{return Vector2D(x,y);}
        bool sameLoc(xytLoc const& other)const{return x==other.x&&y==other.y;}
};

struct TemporalVector : Vector2D {
	TemporalVector(Vector2D const& loc, double time):Vector2D(loc), t(time){}
	TemporalVector(xytLoc const& loc):Vector2D(loc), t(loc.t){}
	TemporalVector(double _x, double _y, float time):Vector2D(_x,_y), t(time){}
	TemporalVector():Vector2D(),t(0){}
	double t;
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
};

std::ostream& operator <<(std::ostream & out, const TemporalVector &loc);
std::ostream& operator <<(std::ostream & out, const xytLoc &loc);
std::ostream& operator <<(std::ostream & out, const xyLoc &loc);
bool operator==(const xytLoc &l1, const xytLoc &l2);
bool operator!=(const xyLoc &l1, const xyLoc &l2);
bool operator==(const xyLoc &l1, const xyLoc &l2);

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

#endif
