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
