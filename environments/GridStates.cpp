#include "GridStates.h"

double agentRadius(.5);

std::ostream& operator <<(std::ostream & out, const TemporalVector &loc) {
  out << "(" << loc.x << ", " << loc.y << ": " << loc.t << ")";
  return out;
}

std::ostream& operator <<(std::ostream & out, const xytLoc &loc) {
  out << "(" << loc.x << ", " << loc.y << ", " << loc.h << ": " << loc.t << ")";
  return out;
}

std::ostream& operator <<(std::ostream & out, const xyLoc &loc) {
  out << "(" << loc.x << ", " << loc.y << ")";
  return out;
}

bool operator==(const xyLoc &l1, const xyLoc &l2) {
  return (l1.x == l2.x) && (l1.y == l2.y);
}

bool operator!=(const xyLoc &l1, const xyLoc &l2) {
  return (l1.x != l2.x) || (l1.y != l2.y);
}

bool operator==(const xytLoc &l1, const xytLoc &l2) {
  return fequal(l1.t,l2.t) && (l1.x == l2.x) && (l1.y==l2.y);
}

// Utility function
void GLDrawCircle(GLfloat x, GLfloat y, GLfloat radius){
  static const int lineAmount(20); //# of segments

  static const GLfloat twicePi(2.0f * PI);

  glBegin(GL_LINE_LOOP);
  for(int i(0); i <= lineAmount;i++) { 
    glVertex3f(
        x + (radius * cos(i *  twicePi / lineAmount)), 
        y + (radius* sin(i * twicePi / lineAmount)),
        -.1);
  }
  glEnd();
}

template<>
bool Constraint<xytLoc>::ConflictsWith(const xytLoc &state) const
{
  /*if(state.landed || end_state.landed && start_state.landed) return false;
  Vector2D A(state);
  Vector2D VA(0,0);
  //VA.Normalize();
  Vector2D B(start_state);
  Vector2D VB(end_state);
  VB-=B; // Direction vector
  //VB.Normalize();
  if(collisionImminent(A,VA,agentRadius,state.t,state.t+1.0,B,VB,agentRadius,start_state.t,end_state.t)){
    return true;
  }*/
  return false; // There is really no such thing as a vertex conflict in continuous time domains.
}

// Check both vertex and edge constraints
template<>
bool Constraint<xytLoc>::ConflictsWith(const xytLoc &from, const xytLoc &to) const
{
  Vector2D A(from);
  Vector2D VA(to);
  VA-=A; // Direction vector
  VA.Normalize();
  Vector2D B(start_state);
  Vector2D VB(end_state);
  VB-=B; // Direction vector
  VB.Normalize();
  if(collisionImminent(A,VA,agentRadius,from.t,to.t,B,VB,agentRadius,start_state.t,end_state.t)){
    return true;
  }
  return false;
}

template<>
bool Constraint<xytLoc>::ConflictsWith(const Constraint<xytLoc> &x) const
{
  return ConflictsWith(x.start_state, x.end_state);
}


template<>
void Constraint<xytLoc>::OpenGLDraw(MapInterface* map) const 
{
  GLdouble xx, yy, zz, rad;
  glColor3f(1, 0, 0);
  glLineWidth(12.0);
  glBegin(GL_LINES);
  map->GetOpenGLCoord((float)start_state.x, (float)start_state.y, xx, yy, zz, rad);
  glVertex3f(xx, yy, -rad);
  map->GetOpenGLCoord((float)end_state.x, (float)end_state.y, xx, yy, zz, rad);
  glVertex3f(xx, yy, -rad);
  glEnd();
}

template<>
bool Constraint<TemporalVector>::ConflictsWith(const TemporalVector &state) const
{
  /*if(state.landed || end_state.landed && start_state.landed) return false;
  Vector2D A(state);
  Vector2D VA(0,0);
  //VA.Normalize();
  Vector2D B(start_state);
  Vector2D VB(end_state);
  VB-=B; // Direction vector
  //VB.Normalize();
  if(collisionImminent(A,VA,agentRadius,state.t,state.t+1.0,B,VB,agentRadius,start_state.t,end_state.t)){
    return true;
  }*/
  return false; // There is really no such thing as a vertex conflict in continuous time domains.
}

// Check both vertex and edge constraints
template<>
bool Constraint<TemporalVector>::ConflictsWith(const TemporalVector &from, const TemporalVector &to) const
{
  Vector2D A(from);
  Vector2D VA(to);
  VA-=A; // Direction vector
  VA.Normalize();
  Vector2D B(start_state);
  Vector2D VB(end_state);
  VB-=B; // Direction vector
  VB.Normalize();
  if(collisionImminent(A,VA,agentRadius,from.t,to.t,B,VB,agentRadius,start_state.t,end_state.t)){
    return true;
  }
  return false;
}

template<>
bool Constraint<TemporalVector>::ConflictsWith(const Constraint<TemporalVector> &x) const
{
  return ConflictsWith(x.start_state, x.end_state);
}


template<>
void Constraint<TemporalVector>::OpenGLDraw(MapInterface* map) const 
{
        if(start_state.x<0 || start_state.x>map->GetMapWidth() || end_state.x<0 || end_state.x>map->GetMapWidth()){
          int x = 2;
        }
	GLdouble xx, yy, zz, rad;
	glColor3f(1, 0, 0);
	map->GetOpenGLCoord((float)start_state.x, (float)start_state.y, xx, yy, zz, rad);
	//DrawSphere(xx, yy, zz, rad/2.0); // zz-l.t*2*rad
	
        glLineWidth(12.0);
        //glDrawCircle(xx,yy,.5/map->GetMapWidth());
	glBegin(GL_LINES);
	glVertex3f(xx, yy, zz-.1);
        //std::cout << "Line from " << xx <<"("<<start_state.x << ")," << yy<<"("<<start_state.y << ")," << zz;
	map->GetOpenGLCoord((float)end_state.x, (float)end_state.y, xx, yy, zz, rad);
	glVertex3f(xx, yy, zz-.1);
        //std::cout << " to " << xx <<"("<<end_state.x << ")," << yy<<"("<<end_state.y << ")," << zz << "\n";
	glEnd();
        //glDrawCircle(xx,yy,.5/map->GetMapWidth());
	//DrawSphere(xx, yy, zz, rad/2.0); // zz-l.t*2*rad
}

/************************************************************/

/** Constructor for the BaseMapOccupancyInterface
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param m The map to which the occupancy interface applies
*/
BaseMapOccupancyInterface::BaseMapOccupancyInterface(Map* m)
{
 	mapWidth = m->GetMapWidth();
 	mapHeight = m->GetMapHeight();
	bitvec.resize(mapWidth*mapHeight);// = new BitVector(mapWidth * mapHeight);
	
	//initialize the bitvector
//	for (int i=0; i<m->GetMapWidth(); i++)
//		for (int j=0; j<m->GetMapHeight(); j++)
//			bitvec->Set(CalculateIndex(i,j), false);
}



/** Destructor for the BaseMapOccupancyInterface
* 
* @author Renee Jansen
* @date 08/22/2007
*/
BaseMapOccupancyInterface::~BaseMapOccupancyInterface()
{
//	delete bitvec;
//	bitvec = 0;
}

/** Sets the occupancy of a state.
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param s The state for which we want to set the occupancy
* @param occupied Whether or not the state is occupied
*/
void BaseMapOccupancyInterface::SetStateOccupied(const xyLoc &s, bool occupied)
{
	// Make sure the location is valid
	// unsigned, so must be greater than 0
	assert(/*(s.x>=0) &&*/ (s.x<mapWidth)/* && (s.y>=0)*/ && (s.y<mapHeight));
//	bitvec->Set(CalculateIndex(s.x,s.y), occupied);
	bitvec[CalculateIndex(s.x,s.y)] = occupied;
}

/** Returns the occupancy of a state.
* 
* @author Renee Jansen
* @date 08/22/2007
*
* @param s The state for which we want to know the occupancy information
* @return True if the state is occupied, false otherwise. 
*/
bool BaseMapOccupancyInterface::GetStateOccupied(const xyLoc &s)
{
	// unsigned, so must be greater than 0
	assert(/*s.x>=0 &&*/ s.x<=mapWidth && /*s.y>=0 && */s.y<=mapHeight);
	//return bitvec->Get(CalculateIndex(s.x,s.y));
	return bitvec[CalculateIndex(s.x,s.y)];
}

/** Gets the index into the bitvector. 
*
* Converts (x,y) locations to a position in the bitvector. 
*
* @author Renee Jansen
* @date 08/22/2007
*
* @param x The x-coordinate of the location
* @param y The y-coordinate of the location
* @return The index into the bit vector
*/
//template <class state, class action>
long BaseMapOccupancyInterface::CalculateIndex(uint16_t x, uint16_t y)
{
	return (y * mapWidth) + x;
}

/** Updates the occupancy interface when a unit moves
*
* Sets the old location to be not occupied, and the new location
* to be occupied.
* 
* @author Renee Jansen
* @date 09/17/2007
*
* @param oldState The unit's previous state
* @param newState The unit's new state
*/
void BaseMapOccupancyInterface::MoveUnitOccupancy(const xyLoc &oldState, const xyLoc &newState)
{
	SetStateOccupied(oldState, false);
	SetStateOccupied(newState, true);
}

bool BaseMapOccupancyInterface::CanMove(const xyLoc &, const xyLoc &l2)
{
	if (!(GetStateOccupied(l2)))
	{
		return true;
	}
	else
	{		
		return false;
	}
	
}

