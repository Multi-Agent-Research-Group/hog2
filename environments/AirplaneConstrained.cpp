//
//  AirplaneConstrained.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneConstrained.h"

/////////////////////////////// PUBLIC ////////////////////////////////////////////////////


/** Operator for equivalent airtime states */
bool operator==(const airtimeState &l1, const airtimeState &l2)
{
	return (l1.t == l2.t) && (l1.l == l2.l);
}


/// CONSTRUCTORS
/** Construct a new airplane constrained environemnt with a random ground map */
AirplaneConstrainedEnvironment::AirplaneConstrainedEnvironment() {
	this->ae = new AirplaneEnvironment();
}
/** Construct an Airplane Constrained Environment given a pointer to an Airplane Environment */
AirplaneConstrainedEnvironment::AirplaneConstrainedEnvironment(AirplaneEnvironment *aire)
{
	this->ae = aire;
}

/// CONSTRAINT MANAGEMENT
void AirplaneConstrainedEnvironment::AddConstraint(airConstraint c)
{
	constraints.push_back(c);
}
void AirplaneConstrainedEnvironment::AddConstraint(airtimeState loc)
{
	airConstraint c;
	c.loc = loc;
	c.dir = airplaneAction(0,0,0);
	constraints.push_back(c);
}
void AirplaneConstrainedEnvironment::AddConstraint(airtimeState loc, airplaneAction dir)
{
	airConstraint c;
	c.loc = loc;
	c.dir = dir;
	constraints.push_back(c);
}
void AirplaneConstrainedEnvironment::ClearConstraints()
{
	constraints.resize(0);
}


/// STATE MANAGEMENT


void AirplaneConstrainedEnvironment::GetActions(const airtimeState &nodeID, std::vector<airplaneAction> &actions) const
{
	// Get the action information from the hidden AE
	this->ae->GetActions(nodeID.l, actions);
}
void AirplaneConstrainedEnvironment::GetSuccessors(const airtimeState &nodeID, std::vector<airtimeState> &neighbors) const
{
	// Create a location to hold the states
	std::vector<airplaneState> n;

	// Get the successors from the hidden AE
	this->ae->GetSuccessors(nodeID.l, n);

	// Check to see if any constraints are violated, and remove them from the actions that are allowed
	for (unsigned int x = 0; x < n.size(); x++)
	{
		if (!ViolatesConstraint(nodeID.l, n[x], nodeID.t))
		{
			airtimeState newLoc;
			newLoc.l = n[x];
			newLoc.t = nodeID.t+1;
			neighbors.push_back(newLoc);
		}
	}
}
void AirplaneConstrainedEnvironment::ApplyAction(airtimeState &s, airplaneAction a) const
{
	// Apply the action on the hidden AE
	ae->ApplyAction(s.l, a);
	s.t+=1;
}
void AirplaneConstrainedEnvironment::UndoAction(airtimeState &s, airplaneAction a) const
{
	// Undo the action on the hidden AW
	ae->UndoAction(s.l, a);
	s.t-=1;
}
airplaneAction AirplaneConstrainedEnvironment::GetAction(const airtimeState &node1, const airtimeState &node2) const 
{
	// Get the action that lead from node1.l to node2.l on the hidden AE
	return ae->GetAction(node1.l, node2.l);
}
void AirplaneConstrainedEnvironment::GetNextState(const airtimeState &currents, airplaneAction dir, airtimeState &news) const
{
	// Get the next location from the owned AE and then increase the time by 1
	ae->GetNextState(currents.l, dir, news.l);
	news.t = currents.t + 1;
}
// Invert action defined in the header


/// OCCUPANCY
/// Defined in the header file


/// HEURISTICS

/** Heuristic value between two arbitrary nodes. **/
double AirplaneConstrainedEnvironment::HCost(const airtimeState &node1, const airtimeState &node2) const
{
	double res1 = this->ae->HCost(node1.l, node2.l);
	double res2 = (node2.t>node1.t)?(node2.t-node1.t):0;
	return max(res1, res2);
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
		n_air.push_back(x.l);
	return ae->GetPathLength(n_air);
}



/// GOAL TESTING

bool AirplaneConstrainedEnvironment::GoalTest(const airtimeState &node, const airtimeState &goal) const
{
	// Notice that we have achieved the goal even if we're late - we might want to check this in
	// the future.
	return (node.l == goal.l && node.t >= goal.t);
}


/// HASHING
//TODO: Finish the airplane state hashing
uint64_t AirplaneConstrainedEnvironment::GetStateHash(const airtimeState &node) const
{
	uint64_t h = 0;
	
	h |= node.l.x;
	h = h << 8;
	h |= node.l.y;
	h = h << 8;
	h |= node.l.height;
	h = h << 8;
	h |= node.l.speed;
	h = h << 8;
	h |= node.l.heading;
	h = h << 16;
	h |= node.t;
	
	return h;
}

uint64_t AirplaneConstrainedEnvironment::GetActionHash(airplaneAction act) const
{
	uint64_t h = 0;
	
	h |= act.turn;
	h = h << 8;
	h |= act.speed;
	h = h << 8;
	h |= act.height;
	
	return h;
}


/// DRAWING

void AirplaneConstrainedEnvironment::OpenGLDraw() const
{

	// Draw the base environment
	this->ae->OpenGLDraw();
	
	// Draw the constraints on the map
	for (airConstraint constraint : constraints)
	{
		GLfloat x = (constraint.loc.l.x-40.0)/40.0;
		GLfloat y = (constraint.loc.l.y-40.0)/40.0;
		GLfloat z = -constraint.loc.l.height/80.0;

		glEnable(GL_LIGHTING);
		glColor4f(1.0,0.0,0.0,0.5);
		DrawSphere(x, y, z, 0.01);
		
	}
}

void AirplaneConstrainedEnvironment::OpenGLDraw(const airtimeState& l) const
{
	this->ae->OpenGLDraw(l.l);
}

void AirplaneConstrainedEnvironment::OpenGLDraw(const airtimeState& x, const airplaneAction& dir) const
{
	this->ae->OpenGLDraw(x.l, dir);
}

void AirplaneConstrainedEnvironment::GLDrawLine(const airtimeState &x, const airtimeState &y) const
{
	this->ae->SetColor(0,1,0);
	this->ae->GLDrawLine(x.l, y.l);
}

void AirplaneConstrainedEnvironment::GLDrawPath(const std::vector<airtimeState> &p) const
{
	std::vector<airplaneState> p_air;
	for (airtimeState x : p)
		p_air.push_back(x.l);
	this->ae->GLDrawPath(p_air);
}


/////////////////////////////// PRIVATE ////////////////////////////////////////////////////



bool AirplaneConstrainedEnvironment::ViolatesConstraint(const airplaneState &from, const airplaneState &to, int time) const
{
	for (airConstraint constraint : constraints)
	{
		// Check to see if the time, height, x and y are the same. Notice that heading has no influence on the 
		// constraints
		if (time+1 == constraint.loc.t && 
		    ( (to.x == constraint.loc.l.x) && (to.y == constraint.loc.l.y) && (to.height == constraint.loc.l.height) ) )
			return true;
	}
	return false;
}


