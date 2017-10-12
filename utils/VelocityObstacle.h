#ifndef VELOCITYOBSTACLE_H
#define VELOCITYOBSTACLE_H

#include "Vector2D.h"
#include "Vector3D.h"
#include "PositionalUtils.h"
#include <vector>

class VelocityObstacle{
  public:
    VelocityObstacle(Vector2D const& a, Vector2D const& va, Vector2D const& b, Vector2D const& vb, double r1, double r2=DBL_MAX);
    static bool AgentOverlap(Vector2D const& A,Vector2D const& B,double ar,double br);
    bool IsInside(Vector2D const& point) const;
  private:
    Vector2D VO,VL,VR; // Pos,vel,pos,vel,VO-apex,VO-left,VO-right
};

bool getTangentOfCircle(Vector2D const& center, double radius, Vector2D const& point, std::vector<Vector2D>& tangents);

bool detectCollision(Vector2D A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB);
#define CENTER_IDX9 364
#define CENTER_IDX25 7812
#define CENTER_IDX49 58824
#define CENTER_IDX27 9841
#define CENTER_IDX125 976562
#define WORD_BITS (8 * sizeof(unsigned))
void get2DSuccessors(Vector2D const& c, std::vector<Vector2D>& s, unsigned conn);
void get3DSuccessors(Vector3D const& c, std::vector<Vector3D>& s, unsigned conn);
unsigned index9(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2);
unsigned index25(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2);
unsigned index49(Vector2D const& s1, Vector2D d1, Vector2D s2, Vector2D d2);
unsigned index27(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2);
unsigned index125(Vector3D const& s1, Vector3D d1, Vector3D s2, Vector3D d2);

inline bool get(unsigned* bitarray, size_t idx) {
  return bitarray[idx / WORD_BITS] | (1 << (idx % WORD_BITS));
}

inline void set(unsigned* bitarray, size_t idx) {
  bitarray[idx / WORD_BITS] |= (1 << (idx % WORD_BITS));
}

bool collisionImminent(Vector2D const A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB);
bool collisionCheck(TemporalVector const& A1, TemporalVector const& A2, TemporalVector const& B1, TemporalVector const& B2,
double radiusA, double radiusB=0, double speedA=1.0, double speedB=1.0);

bool collisionImminent(Vector3D const A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector3D const B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB);
bool collisionCheck(TemporalVector3D const& A1, TemporalVector3D const& A2, TemporalVector3D const& B1, TemporalVector3D const& B2,
double radiusA, double radiusB=0, double speedA=1.0, double speedB=1.0);

double  getCollisionTime(Vector2D const A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB);
double  getCollisionTime(Vector3D const A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB);

std::pair<double,double> getCollisionInterval(Vector2D const A, Vector2D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector2D B, Vector2D const& VB, double radiusB, double startTimeB, double endTimeB);
std::pair<double,double> getCollisionInterval(Vector3D const A, Vector3D const& VA, double radiusA, double startTimeA, double endTimeA,
Vector3D B, Vector3D const& VB, double radiusB, double startTimeB, double endTimeB);

#endif
