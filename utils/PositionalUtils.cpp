#include "PositionalUtils.h"

// From http://www.cplusplus.com/forum/beginner/49408/

double Util::linesIntersect(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, double* out){
  Vector2D a(A2-A1);
  Vector2D b(B2-B1);

  double f(det(a,b));
  if(!f)      // lines are parallel
    return false;

  Vector2D c(B2-A2);
  double aa(det(a,c));
  double bb(det(b,c));

  if(f < 0)
  {
    if(aa > 0)     return false;
    if(bb > 0)     return false;
    if(aa < f)     return false;
    if(bb < f)     return false;
  }
  else
  {
    if(aa < 0)     return false;
    if(bb < 0)     return false;
    if(aa > f)     return false;
    if(bb > f)     return false;
  }

  if(out)
    *out = 1.0 - (aa / f);
  return true;
}

bool Util::intersectionPoint(Vector2D const& A1, Vector2D const& A2, Vector2D const& B1, Vector2D const& B2, Vector2D& out){
  double pct(0);
  if(linesIntersect(A1,A2,B1,B2,&pct)){
    out=((B2 - B1) * pct) + B1;
    return true;
  }
  return false;
}

// Assume "rounded" line with radius (thus the line width is 2*r)
bool Util::fatLinesIntersect(Vector2D const& A1, Vector2D const& A2, double r1, Vector2D const& B1, Vector2D const& B2, double r2){
  // If A and B intersect, we're done
  if(linesIntersect(A1,A2,B1,B2)){return true;}

  Vector2D NA(normal(A1,A2)*r1); // Normal of line from A1 to A2 
  Vector2D NB(normal(B1,B2)*r2); // Normal of line from B1 to B2 

  // If A and B are parallel, then we can just check the distance
  double r(r1+r2);
  if((NA==NB || NA==-NB) && fless(distanceOfPointToLine(A1,A2,B1),r)){return true;}

  // Project along normal in both directions
  Vector2D A11(A1+NB);
  Vector2D A21(A2+NB);
  Vector2D A12(A1-NB);
  Vector2D A22(A2-NB);

  Vector2D B11(B1+NB);
  Vector2D B21(B2+NB);
  Vector2D B12(B1-NB);
  Vector2D B22(B2-NB);

  if(linesIntersect(A11,A21,B11,B21)){return true;}
  if(linesIntersect(A11,A21,B12,B22)){return true;}
  if(linesIntersect(A12,A22,B11,B21)){return true;}
  if(linesIntersect(A12,A22,B12,B22)){return true;}

  // Finally, check endpoints
  if(fless(distanceOfPointToLine(A11,A21,B1),r)){return true;}
  if(fless(distanceOfPointToLine(A12,A22,B1),r)){return true;}
  if(fless(distanceOfPointToLine(A11,A21,B2),r)){return true;}
  if(fless(distanceOfPointToLine(A12,A22,B2),r)){return true;}
  if(fless(distanceOfPointToLine(B11,B21,A1),r)){return true;}
  if(fless(distanceOfPointToLine(B12,B22,A1),r)){return true;}
  if(fless(distanceOfPointToLine(B11,B21,A2),r)){return true;}
  if(fless(distanceOfPointToLine(B12,B22,A2),r)){return true;}

  return false;
}
