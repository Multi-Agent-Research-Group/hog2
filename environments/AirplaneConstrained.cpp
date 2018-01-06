//
//  AirplaneConstrained.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//
//  Modified by Thayne Walker 2017
//

#include "AirplaneConstrained.h"
#include <glut.h>
#include <iostream>
#include <algorithm>
 
/////////////////////////////// PUBLIC ////////////////////////////////////////////////////


/** Operator for equivalent PlatformState states */
bool operator==(const airtimeState &l1, const airtimeState &l2)
{
	//return fequal(l1.t,l2.t) && ((airplaneState const&)l1)== ((airplaneState const&)l2);
	return ((airplaneState const&)l1)== ((airplaneState const&)l2);
}


/// CONSTRUCTORS
/** Construct a new airplane constrained environemnt with a random ground map */
/*AirplaneConstrainedEnvironment::AirplaneConstrainedEnvironment() {
	this->ae = new AirplaneEnvironment();
	ClearConstraints();
}*/
/** Construct an Airplane Constrained Environment given a pointer to an Airplane Environment */
AirplaneConstrainedEnvironment::AirplaneConstrainedEnvironment(AirplaneEnvironment *aire,int w,int l, int h)
: ae(aire),width(w),length(l),height(h)
{
  DrawableConstraint::width=width;
  DrawableConstraint::length=length;
  DrawableConstraint::height=height;

	ClearConstraints();

	// Add back the air-strip constraints
	/*for (landingStrip st : ae->GetLandingStrips())
	{
		airplaneState mi(min(st.x1, st.x2), min(st.y1, st.y2), 0, 0,0);
		airtimeState o(mi, 0);
		airplaneState ma(max(st.x1, st.x2), max(st.y1, st.y2), 10, 0,0);
		airtimeState f(ma, std::numeric_limits<float>::max());
		Collision<airtimeState> c(o, f);
		c.strip=true;
		static_constraints.push_back(c);
	}*/

	// Add a constraint for the ground
	airplaneState mi(0,0, 0, 0,0);
	airtimeState o(mi, 0);
	airplaneState ma(width, length, 0, 0,0);
	airtimeState f(ma, std::numeric_limits<float>::max());
	Collision<airtimeState> c(o, f, .25);
	//c.strip=false;
	static_constraints.push_back(c);
}

// Add a constraint of any type
void AirplaneConstrainedEnvironment::AddStaticConstraint(Collision<airtimeState> const& c)
{
	static_constraints.push_back(c);
}

// Add a point constraint
void AirplaneConstrainedEnvironment::AddStaticPointConstraint(const airtimeState &loc)
{
	//Collision<airtimeState> x(loc);
	//static_constraints.push_back(x);
}

void AirplaneConstrainedEnvironment::AddStaticBoxConstraint(const airtimeState &loc1, const airtimeState &loc2)
{
	Collision<airtimeState> x(loc1, loc2, .25);
	static_constraints.push_back(x);
}

void AirplaneConstrainedEnvironment::ClearStaticConstraints()
{
	static_constraints.resize(0);
}


/// STATE MANAGEMENT

void AirplaneConstrainedEnvironment::GetActions(const airtimeState &nodeID, std::vector<airplaneAction> &actions) const
{
	// Get the action information from the hidden AE
	this->ae->GetActions(nodeID, actions);
}
void AirplaneConstrainedEnvironment::GetReverseActions(const airtimeState &nodeID, std::vector<airplaneAction> &actions) const
{
	// Get the action information from the hidden AE
	this->ae->GetReverseActions(nodeID, actions);
}
void AirplaneConstrainedEnvironment::GetSuccessors(const airtimeState &nodeID, std::vector<airtimeState> &neighbors) const
{
  // Create a location to hold the states
  std::vector<airplaneAction> actions;

  // Get the successors from the hidden AE
  this->ae->GetActions(nodeID, actions);

  // Check to see if any constraints are violated, and remove them from the actions that are allowed
  for (airplaneAction act : actions)
  {
    // Construct the followup state
    airtimeState new_state(nodeID, nodeID.t);
    this->ApplyAction(new_state, act);

    // Check to see if it violates any hard constraint. If it does not, push it back.
    if (!ViolatesConstraint(nodeID, new_state))
    {
      //if (ticket_authority->CanObtainTicket(new_state)) {
      neighbors.push_back(new_state);
      //}
    }
  }
}

void AirplaneConstrainedEnvironment::GetReverseSuccessors(const airtimeState &nodeID, std::vector<airtimeState> &neighbors) const
{
  // Create a location to hold the states
  std::vector<airplaneAction> actions;

  // Get the successors from the hidden AE
  this->ae->GetReverseActions(nodeID, actions);

  // Check to see if any constraints are violated, and remove them from the actions that are allowed
  for (airplaneAction act : actions)
  {
    // Construct the followup state
    airtimeState new_state(nodeID, nodeID.t);
    this->UndoAction(new_state, act);

    // Check to see if it violates any hard constraint. If it does not, push it back.
    if (!ViolatesConstraint(nodeID, new_state))
    {
      //if (ticket_authority->CanObtainTicket(new_state)) {
        neighbors.push_back(new_state);
      //}
    }
  }
}

void AirplaneConstrainedEnvironment::ApplyAction(airtimeState &s, airplaneAction a) const
{
	// Apply the action on the hidden AE
	ae->ApplyAction(s, a);
    
    // Calculate how long it will take to move 3 meters (or the diagonal thereof) at speed
    static double physicalMaxSpeed(32.0); //MPS
    static double physicalMinSpeed(17.0); //MPS
    static double speedRange(physicalMaxSpeed-physicalMinSpeed);
    double factor(ae->gridSize/(physicalMinSpeed+double(s.speed-ae->minSpeed)*speedRange/double(ae->numSpeeds-ae->minSpeed)));
    
    // Compute time increase based on speed...
    // if speed is more than cruise speed, we arrive sooner, later if speed is less.
	s.t += (a.turn == kWait ? 1 : (s.heading%2?M_SQRT2:1.0))*factor;
}
void AirplaneConstrainedEnvironment::UndoAction(airtimeState &s, airplaneAction a) const
{
	// Undo the action on the hidden AW
	ae->UndoAction(s, a);

    static double speedRange(ae->maxSpeed-ae->minSpeed);
    double factor(ae->gridSize/(ae->minSpeed+double(s.speed-1)*speedRange/double(ae->numSpeeds-1)));

	s.t -= (a.turn == kWait ? 1 : (abs(a.turn)%2?M_SQRT2:1.0)*factor);
}
airplaneAction AirplaneConstrainedEnvironment::GetAction(const airtimeState &node1, const airtimeState &node2) const 
{
	// Get the action that lead from node1 to node2 on the hidden AE
	return ae->GetAction(node1, node2);
}
void AirplaneConstrainedEnvironment::GetNextState(const airtimeState &currents, airplaneAction dir, airtimeState &news) const
{
	// Get the next location from the owned AE and then increase the time by 1
	ae->GetNextState(currents, dir, news);

    static double speedRange(ae->maxSpeed-ae->minSpeed);
    double factor(ae->gridSize/(ae->minSpeed+double(currents.speed-1)*speedRange/double(ae->numSpeeds-1)));

	news.t = currents.t + (dir.turn == kWait ? 1 : (currents.heading%2?M_SQRT2:1.0))*factor;
}
// Invert action defined in the header


/// OCCUPANCY
/// Defined in the header file


/// HEURISTICS

/** Heuristic value between two arbitrary nodes. **/
double AirplaneConstrainedEnvironment::HCost(const airtimeState &node1, const airtimeState &node2) const
{
	//double res1 = this->ae->HCost(node1, node2);
	//double res2 = (node2.t>node1.t)?(node2.t-node1.t):0;
	//return std::max<int>(res1, res2);
    return this->ae->HCost(node1, node2);
}


// No single cost H-Cost
// G-Cost defined in the header
double AirplaneConstrainedEnvironment::GetPathLength(const std::vector<airtimeState> &n) const
{
	// Get the length of the path in the hidden airplane envirionment. This takes linear
	// time, which could be fixed by writing a type-conversion.
	// TODO: Write a conversion
	std::vector<airplaneState> n_air;
	for (airtimeState x : n)
		n_air.push_back(x);
	return ae->GetPathLength(n_air);
}

bool AirplaneConstrainedEnvironment::collisionCheck(airtimeState const& s1, airtimeState const& d1, float radius1, airtimeState const& s2, airtimeState const& d2,float radius2){
  Vector3D A(s1);
  Vector3D B(s2);
  Vector3D VA(d1);
  VA-=A;
  VA.Normalize();
  Vector3D VB(d2);
  VB-=B;
  VB.Normalize();
  return collisionImminent(A,VA,radius1,s1.t,d1.t,B,VB,radius2,s2.t,d2.t);
}

/// GOAL TESTING

bool AirplaneConstrainedEnvironment::GoalTest(const airtimeState &node, const airtimeState &goal) const
{
	// Notice that we have achieved the goal even if we're late - we might want to check this in
	// the future.
	return (this->ae->GoalTest(node, goal) && node.t >= goal.t);
}


/// HASHING
//TODO: Finish the airplane state hashing
uint64_t AirplaneConstrainedEnvironment::GetStateHash(const airtimeState &node) const
{
	uint64_t h = 0;
	
        // Assume x,y discretization of 3 meters
	h |= node.x;
	h = h << 16;
	h |= node.y;
	h = h << 10;
        // Assume height discretization of 25 meters
	h |= node.height & (0x400-1); // 10 bits
	h = h << 4;
        // Speed increments are in 1 m/sec
	h |= node.speed & (0xF); // 5 bits
	h = h << 3;
        // Heading increments are in 45 degrees
	h |= node.heading & (0x8-1); // 3 bits
	h = h << 1;
	h |= node.landed;
	h = h << 12;
        // Time is continuous (NOTE: this hashing is worthless; should mult by 1000 or something)
	h |= unsigned(node.t*16) & (0x1000-1);
        //std::cout << "Hash: "<<node << std::hex << h << std::dec << "\n";
	
	return h;
}

void AirplaneConstrainedEnvironment::GetStateFromHash(uint64_t hash, airtimeState &s) const
{
    s.t=((hash)&(0x1000-1))/16.0; // Can't represent a float like this...
    s.landed=(hash>>12)&0x1;
    s.heading=(hash>>13)&((0x7));
    s.speed=(hash>>16)&(0xf);
    s.height=(hash>>20)&(0x400-1);
    s.y=(hash>>30)&(0x10000-1);
    s.x=(hash>>46)&(0x10000-1);
}


uint64_t AirplaneConstrainedEnvironment::GetActionHash(airplaneAction act) const
{
	uint64_t h = 0;
	
	h |= act.turn;
	h = h << 8;
	h |= act.speed;
	h = h << 8;
	h |= act.height;
	h = h << 8;
	h |= act.takeoff;
	
	return h;
}

template<>
void Collision<airtimeState>::OpenGLDraw(MapInterface*) const 
{
        static float halfWidth(DrawableConstraint::width/2.0);
        static float halfLength(DrawableConstraint::length/2.0);
	glLineWidth(2.0); // Make it wide

	// Normalize coordinates between (-1, 1)
	GLfloat x_start = (start_state.x-halfWidth)/halfWidth;
	GLfloat y_start = (start_state.y-halfLength)/halfLength;
	GLfloat z_start = -start_state.height/float(DrawableConstraint::width);

	GLfloat x_end = (end_state.x-halfWidth)/halfWidth;
	GLfloat y_end = (end_state.y-halfLength)/halfLength;
	GLfloat z_end = -end_state.height/float(DrawableConstraint::width);


	GLfloat min_x, min_y, min_z, max_x, max_y, max_z;

        min_x = min(x_start-.5/halfWidth,x_end-.5/halfWidth);
        max_x = max(x_start+.5/halfWidth,x_end+.5/halfWidth);
        min_y = min(y_start-.5/halfLength,y_end-.5/halfLength);
        max_y = max(y_start+.5/halfLength,y_end+.5/halfLength);
        min_z = min(z_start-.5/halfLength,z_end-.5/halfLength);
        max_z = max(z_start+.5/halfLength,z_end+.5/halfLength);

	glDisable(GL_LIGHTING);
	glPushMatrix();

	// Draw the lower loop
	glBegin(GL_LINE_LOOP);
	glVertex3f(min_x, min_y, min_z);
	glVertex3f(max_x, min_y, min_z);
	glVertex3f(max_x, max_y, min_z);
	glVertex3f(min_x, max_y, min_z);
	glEnd();

	// Draw the upper loop
	glBegin(GL_LINE_LOOP);
	glVertex3f(min_x, min_y, max_z);
	glVertex3f(max_x, min_y, max_z);
	glVertex3f(max_x, max_y, max_z);
	glVertex3f(min_x, max_y, max_z);
	glEnd();

	// Draw the edge line segments
	glBegin(GL_LINES);

	glVertex3f(min_x, min_y, min_z);
	glVertex3f(min_x, min_y, max_z);

	glVertex3f(min_x, max_y, min_z);
	glVertex3f(min_x, max_y, max_z);

	glVertex3f(max_x, min_y, min_z);
	glVertex3f(max_x, min_y, max_z);

	glVertex3f(max_x, max_y, min_z);
	glVertex3f(max_x, max_y, max_z);

	glEnd();

	glPopMatrix();
}

void SoftConstraint<airtimeState>::OpenGLDraw(MapInterface*) const 
{
        static float halfWidth(DrawableConstraint::width/2.0);
        static float halfLength(DrawableConstraint::length/2.0);

	// Normalize coordinates between (-1, 1)
	GLfloat x((center.x-halfWidth)/halfWidth);
	GLfloat y((center.y-halfLength)/halfLength);
	GLfloat z(-center.height/float(DrawableConstraint::width));

	//glDisable(GL_LIGHTING);

	glLineWidth(1.0); // Make it wide
        //glEnable(GL_BLEND);
        //glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glPushMatrix();
        glColor3f(1, 0, 0);
        glTranslatef(x, y, z);
        glutWireSphere((radius/DrawableConstraint::width)*2.0, 20,20);
        glPopMatrix();
        glDisable(GL_BLEND);
}

/// DRAWING

void AirplaneConstrainedEnvironment::OpenGLDraw() const
{
	
	// Draw the base environment
	this->ae->OpenGLDraw();
	
	// Draw the constraints on the map
	for (int i = 0; i < constraints.size(); i++)
	{
			// Right now we don't draw constraints
	}

	// Draw all of the static constraints
	for (int i = 0; i < static_constraints.size(); i++)
	{
		glColor3f(1, 0, 0); // Make it red
		glLineWidth(2.0);
		static_constraints.at(i).OpenGLDraw(0);	
	}

        // Soft constraints
	for (int i = 0; i < sconstraints.size(); i++)
	{
		sconstraints.at(i).OpenGLDraw(0);	
	}

	// Draw the restricted airspace
        /*if(ticket_authority)
	for (int i = 0; i < ticket_authority->GetRestrictedAirspace().size(); i++)
	{
		glColor3f(0, 0, 1); // Make it blue
		glLineWidth(2.0);
		ticket_authority->GetRestrictedAirspace().at(i).governed_area.OpenGLDraw();	
	}*/
	
}

void AirplaneConstrainedEnvironment::OpenGLDraw(const airtimeState& l) const
{
	this->ae->OpenGLDraw(l);
}

void AirplaneConstrainedEnvironment::OpenGLDraw(const airtimeState& x, const airplaneAction& dir) const
{
	this->ae->OpenGLDraw(x, dir);
}

void AirplaneConstrainedEnvironment::OpenGLDraw(const airtimeState &oldState, const airtimeState &newState, float perc) const
{
	this->ae->OpenGLDraw(oldState, newState, perc);
}

void AirplaneConstrainedEnvironment::GLDrawLine(const airtimeState &x, const airtimeState &y) const
{
	this->ae->SetColor(0,1,0);
	this->ae->GLDrawLine(x, y);
}

void AirplaneConstrainedEnvironment::GLDrawPath(const std::vector<airtimeState> &p,const std::vector<airtimeState> &wpts) const
{
	std::vector<airplaneState> w_air;
	for (airtimeState x : wpts)
		w_air.push_back(x);
	std::vector<airplaneState> p_air;
	for (airtimeState x : p)
		p_air.push_back(x);
	this->ae->GLDrawPath(p_air,w_air);
}


