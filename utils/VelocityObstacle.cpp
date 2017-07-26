#include "VelocityObstacle.h"
#include <assert.h>
#include <iostream>

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
bool detectCollision(Vector2D A, Vector2D VA, double radiusA, double startTimeA, double endTimeA,
Vector2D B, Vector2D VB, double radiusB, double startTimeB, double endTimeB){
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
bool collisionImminent(Vector2D A, Vector2D VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA-radiusA,endTimeB)||fgreater(startTimeB-radiusB,endTimeA)){return false;}

  if(fgreater(startTimeB,startTimeA)){
    // Move A to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }

  double r(radiusA+radiusB); // Combined radius
  Vector2D w(B-A);
  double c(w.sq()-r*r);
  if(c<0){return true;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector2D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return false; }

  double ctime((b-sqrt(dscr))/a); // Collision time
  if(fless(ctime,0)){ return false; }

  // Collision will occur if collision time is before the end of the shortest segment
  return fleq(ctime,std::min(endTimeB,endTimeA)-startTimeA);
}

// Get collision time between two agents - that is the time that they will "start" colliding.
// Agents have a position, velocity, start time, end time and radius.
// Note: this is analogous to an agent traversing an edge from time "start" to "end" at constant velocity.
// -1 is a seminal value meaning "no collision in the future."
double getCollisionTime(Vector2D A, Vector2D VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D VB, double radiusB, double startTimeB, double endTimeB){
  // check for time overlap
  if(fgreater(startTimeA,endTimeB)||fgreater(startTimeB,endTimeA)){return -1;}

  if(fgreater(startTimeB,startTimeA)){
    // Move A forward to the same time instant as B
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }

  double r(radiusA+radiusB); // Combined radius
  Vector2D w(B-A);
  double c(w.sq()-r*r);
  if(c<0){return 0.0;} // Agents are currently colliding

  // Use the quadratic formula to detect nearest collision (if any)
  Vector2D v(VA-VB);
  double a(v.sq());
  double b(w*v);

  double dscr(b*b-a*c);
  if(fleq(dscr,0)){ return -1; } // No collision will occur in the future

  return (b-sqrt(dscr))/a + startTimeA; // Absolute collision time
}

// Get continuous collision interval for two agents (return -1,-1 if such does not exist)
std::pair<double,double> getCollisionInterval(Vector2D A, Vector2D VA, double radiusA, double startTimeA, double endTimeA, Vector2D B, Vector2D VB, double radiusB, double startTimeB, double endTimeB){
  double collisionStart(getCollisionTime(A,VA,radiusA,startTimeA,endTimeA,B,VB,radiusB,startTimeB,endTimeB));
  //std::cout << "Collision Start: " << collisionStart << "\n";
  // Is collision in the past?
  if(fless(collisionStart,0)){
    return std::make_pair(-1.0,-1.0);
  }
  
  // Traverse edges in reverse to see when the collision will end.

  // Reverse the times - make them relative to the end time of the traversal
  if(fgreater(startTimeB,startTimeA)){
    // Move A forward to the same time instant as B
    //std::cout << "Move A by : " << (startTimeB-startTimeA) << "\n";
    A+=VA*(startTimeB-startTimeA);
    startTimeA=startTimeB;
  }else if(fless(startTimeB,startTimeA)){
    //std::cout << "Move B by : " << (startTimeA-startTimeB) << "\n";
    B+=VB*(startTimeA-startTimeB);
    startTimeB=startTimeA;
  }
  //std::cout << "A:"<<A<<"B:"<<B<<"\n";

  double duration(std::min(endTimeA-startTimeA,endTimeB-startTimeB));
  //std::cout << "Move both by : " << duration << "\n";
  Vector2D rA(A+(VA*duration)); // Move A to the end of the edge
  Vector2D rB(B+(VB*duration)); // Move B to the end of the edge
  //std::cout << "rA:"<<rA<<"rB:"<<rB<<"\n";
  double collisionEnd(startTimeA+duration-getCollisionTime(rA,-VA,radiusA,0,duration,rB,-VB,radiusB,0,duration));
  //std::cout << "Collision End: " << collisionEnd << "\n";
  assert(collisionStart<=collisionEnd);
  return std::make_pair(collisionStart,collisionEnd);
}
