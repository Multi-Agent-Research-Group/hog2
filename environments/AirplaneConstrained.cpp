//
//  AirplaneConstrained.cpp
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#include "AirplaneConstrained.h"
#include <iostream>
 
/////////////////////////////// PUBLIC ////////////////////////////////////////////////////


/** Operator for equivalent airtime states */
bool operator==(const airtimeState &l1, const airtimeState &l2)
{
	return (fequal(l1.t,l2.t)) && (l1.l == l2.l);
}


/// CONSTRUCTORS
/** Construct a new airplane constrained environemnt with a random ground map */
/*AirplaneConstrainedEnvironment::AirplaneConstrainedEnvironment() {
	this->ae = new AirplaneEnvironment();
	ClearConstraints();
}*/
/** Construct an Airplane Constrained Environment given a pointer to an Airplane Environment */
AirplaneConstrainedEnvironment::AirplaneConstrainedEnvironment(AirplaneEnvironment *aire)
: ae(aire)
{
	ClearConstraints();

	//Add back the air-strip constraints
	for (landingStrip st : ae->GetLandingStrips())
	{
		airplaneState mi(min(st.x1, st.x2), min(st.y1, st.y2), 0, 0,0);
		airtimeState o(mi, 0);
		airplaneState ma(max(st.x1, st.x2), max(st.y1, st.y2), 10, 0,0);
		airtimeState f(ma, std::numeric_limits<float>::max());
		airConstraint c(o, f);
		c.strip=true;
		static_constraints.push_back(c);
	}
}

/// CONSTRAINT MANAGEMENT

// Add a constraint of any type
void AirplaneConstrainedEnvironment::AddConstraint(airConstraint c)
{
	constraints.push_back(c);
}

// Add a point constraint
void AirplaneConstrainedEnvironment::AddPointConstraint(const airtimeState &loc)
{
	airConstraint x(loc);
	constraints.push_back(x);
}

void AirplaneConstrainedEnvironment::AddBoxConstraint(const airtimeState &loc1, const airtimeState &loc2)
{
	airConstraint x(loc1, loc2);
	constraints.push_back(x);
}

void AirplaneConstrainedEnvironment::ClearConstraints()
{
	constraints.resize(0);
}


// Add a constraint of any type
void AirplaneConstrainedEnvironment::AddStaticConstraint(airConstraint c)
{
	static_constraints.push_back(c);
}

// Add a point constraint
void AirplaneConstrainedEnvironment::AddStaticPointConstraint(const airtimeState &loc)
{
	airConstraint x(loc);
	static_constraints.push_back(x);
}

void AirplaneConstrainedEnvironment::AddStaticBoxConstraint(const airtimeState &loc1, const airtimeState &loc2)
{
	airConstraint x(loc1, loc2);
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
	this->ae->GetActions(nodeID.l, actions);
}
void AirplaneConstrainedEnvironment::GetSuccessors(const airtimeState &nodeID, std::vector<airtimeState> &neighbors) const
{
	// Create a location to hold the states
	std::vector<airplaneAction> actions;

	// Get the successors from the hidden AE
	this->ae->GetActions(nodeID.l, actions);

	// Check to see if any constraints are violated, and remove them from the actions that are allowed
	for (airplaneAction act : actions)
	{
		// Construct the followup state
		airtimeState new_state(nodeID.l, nodeID.t);
		this->ApplyAction(new_state, act);

		// Check to see if it violates any constraint. If it does not, push it back.
		if (!ViolatesConstraint(nodeID, new_state))
        {
        //std::cout << "Adding state\n";
			neighbors.push_back(new_state);
        }
        //        else
        //                std::cout << "VIOLATION: " << act << "," << new_state << "vs" << nodeID << "\n";
	}
}
void AirplaneConstrainedEnvironment::ApplyAction(airtimeState &s, airplaneAction a) const
{
	// Apply the action on the hidden AE
	ae->ApplyAction(s.l, a);
        // Calculate how long it will take to move 3 meters (or the diagonal thereof) at speed
        static double speedRange(ae->maxSpeed-ae->minSpeed);
        double factor(ae->gridSize/(ae->minSpeed+double(s.l.speed-1)*speedRange/double(ae->numSpeeds-1)));
        // Compute time increase based on speed...
        // if speed is more than cruise speed, we arrive sooner, later if speed is less.
	s.t += (abs(a.turn)%2?M_SQRT2:1.0)*factor;
}
void AirplaneConstrainedEnvironment::UndoAction(airtimeState &s, airplaneAction a) const
{
	// Undo the action on the hidden AW
	ae->UndoAction(s.l, a);
        static double speedRange(ae->maxSpeed-ae->minSpeed);
        double factor(ae->gridSize/(ae->minSpeed+double(s.l.speed-1)*speedRange/double(ae->numSpeeds-1)));
	s.t -= (abs(a.turn)%2?M_SQRT2:1.0)*factor;
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
        static double speedRange(ae->maxSpeed-ae->minSpeed);
        double factor(ae->gridSize/(ae->minSpeed+double(currents.l.speed-1)*speedRange/double(ae->numSpeeds-1)));
	news.t = currents.t + (abs(dir.turn)%2?M_SQRT2:1.0)*factor;
}
// Invert action defined in the header


/// OCCUPANCY
/// Defined in the header file


/// HEURISTICS

/** Heuristic value between two arbitrary nodes. **/
double AirplaneConstrainedEnvironment::HCost(const airtimeState &node1, const airtimeState &node2) const
{
	//double res1 = this->ae->HCost(node1.l, node2.l);
	//double res2 = (node2.t>node1.t)?(node2.t-node1.t):0;
	//return max(res1, res2);
        return this->ae->HCost(node1.l, node2.l);
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
	return (this->ae->GoalTest(node.l, goal.l) && node.t >= goal.t);
}


/// HASHING
//TODO: Finish the airplane state hashing
uint64_t AirplaneConstrainedEnvironment::GetStateHash(const airtimeState &node) const
{
	uint64_t h = 0;
	
        // Assume x,y discretization of 3 meters
	h |= node.l.x;
	h = h << 16;
	h |= node.l.y;
	h = h << 10;
        // Assume height discretization of 25 meters
	h |= node.l.height & (0x400-1); // 10 bits
	h = h << 5;
        // Speed increments are in 1 m/sec
	h |= node.l.speed & (0x20-1); // 5 bits
	h = h << 3;
        // Heading increments are in 45 degrees
	h |= node.l.heading & (0x8-1); // 3 bits
	h = h << 12;
        // Time is continuous
	h |= unsigned(node.t) & (0x1000-1);
        //std::cout << "Hash: "<<node << std::hex << h << std::dec << "\n";
	
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
	for (int i = 0; i < constraints.size(); i++)
	{
		if (constraints.at(i).strip)
		{
			this->SetColor(1,0,0);
			constraints.at(i).OpenGLDraw();	
		}
		
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

void AirplaneConstrainedEnvironment::OpenGLDraw(const airtimeState &oldState, const airtimeState &newState, float perc) const
{
	this->ae->OpenGLDraw(oldState.l, newState.l, perc);
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

bool airConstraint::ConflictsWith(const airtimeState &state) const
{
//std::cout << "VERTEX"<<*this << "ConflictsWith" << state << "...\n";
	if (state.t >= start_state.t - 0.00001 && state.t <= end_state.t + 0.00001)
        {
                // Each constraint defines an x, y, z box with corners on the state locations. We 
                // need to check if they overlap at any point. We do this using the separating 
                // axis theorem. Note that all of our boxes are non-rotated, and thus, should never have
                // to worry about non-axis aligned planes of separation. Thus, we check to see if there is 
                // a separating plane on the X axis, Y axis and Z axis, and if no such plane exists, then
                // the boxes must intersect.
                
                // Generate a well formed set of boxes for the constraint box
                int c_minx = min(start_state.l.x, end_state.l.x);
                int c_maxx = max(start_state.l.x, end_state.l.x);

                int c_miny = min(start_state.l.y, end_state.l.y);
                int c_maxy = max(start_state.l.y, end_state.l.y);

                int c_minz = min(start_state.l.height, end_state.l.height);
                int c_maxz = max(start_state.l.height, end_state.l.height);

                
                if (state.l.x >= c_minx && state.l.x <= c_maxx && // Check if overlapping on the X axis
                    state.l.y >= c_miny && state.l.y <= c_maxy && // Check if overlapping on the Y axis
                    state.l.height >= c_minz && state.l.height <= c_maxz // Check if overlapping on the Z axis
                    )
                {
                        // If we overlap on all three axis, then there must be a common point, and thus
                        // we can return that the constraint was violated
                        return true;
                }
        }
return false;
}

bool airConstraint::ConflictsWith(const airtimeState &from, const airtimeState &to) const
{
//std::cout << "EDGE" <<*this << " ConflictsWith from" << from << "to"<<to<< "...\n";
        if (max(start_state.t, from.t) <= min(end_state.t, to.t) + 0.00001) 
        {
                // Each constraint defines an x, y, z box with corners on the state locations. We 
                // need to check if they overlap at any point. We do this using the separating 
                // axis theorem. Note that all of our boxes are non-rotated, and thus, should never have
                // to worry about non-axis aligned planes of separation. Thus, we check to see if there is 
                // a separating plane on the X axis, Y axis and Z axis, and if no such plane exists, then
                // the boxes must intersect.
                
                // Note that these comparisons go center to center, that is, the constraints evaluate unit-wise
                // and no unit is alowed inside another unit's box. The boxes may be allowed to overlap however
                
                // Generate a well formed set of boxes for the action box
                int a_minx = min(from.l.x, to.l.x);
                int a_maxx = max(from.l.x, to.l.x);

                int a_miny = min(from.l.y, to.l.y);
                int a_maxy = max(from.l.y, to.l.y);

                int a_minz = min(from.l.height, to.l.height);
                int a_maxz = max(from.l.height, to.l.height);
                
                // Generate a well formed set of boxes for the constraint box
                int c_minx = min(start_state.l.x, end_state.l.x);
                int c_maxx = max(start_state.l.x, end_state.l.x);

                int c_miny = min(start_state.l.y, end_state.l.y);
                int c_maxy = max(start_state.l.y, end_state.l.y);

                int c_minz = min(start_state.l.height, end_state.l.height);
                int c_maxz = max(start_state.l.height, end_state.l.height);

                
                if (max(c_minx, a_minx) <= min(c_maxx, a_maxx) && // Check if overlapping on the X axis
                    max(c_miny, a_miny) <= min(c_maxy, a_maxy) && // Check if overlapping on the Y axis
                    max(c_minz, a_minz) <= min(c_maxz, a_maxz)    // Check if overlapping on the Z axis
                    )
                {
                        // If we overlap on all three axis, then there must be a common point, and thus
                        // we can return that the constraint was violated
                        return true;
                }
        }
	return false;
}

bool airConstraint::ConflictsWith(const airConstraint &x) const
{
	return ConflictsWith(x.start_state, x.end_state);
}


void airConstraint::OpenGLDraw() const 
{
	glColor3f(1, 0, 0); // Make it red

	// Normalize coordinates between (-1, 1)
	GLfloat x_start = (start_state.l.x-40.0)/40.0;
	GLfloat y_start = (start_state.l.y-40.0)/40.0;
	GLfloat z_start = -start_state.l.height/80.0;

	GLfloat x_end = (end_state.l.x-40.0)/40.0;
	GLfloat y_end = (end_state.l.y-40.0)/40.0;
	GLfloat z_end = -end_state.l.height/80.0;


	GLfloat min_x, min_y, min_z, max_x, max_y, max_z;

	if (strip)
	{
		min_x = min(x_start-.5/40.0,x_end-.5/40.0);
		max_x = max(x_start+.5/40.0,x_end+.5/40.0);
		min_y = min(y_start-.5/40.0,y_end-.5/40.0);
		max_y = max(y_start+.5/40.0,y_end+.5/40.0);
		min_z = min(z_start,z_end);
		max_z = max(z_start+.5/40.0,z_end+.5/40.0);
	} else {
		min_x = min(x_start-.5/40.0,x_end-.5/40.0);
		max_x = max(x_start+.5/40.0,x_end+.5/40.0);
		min_y = min(y_start-.5/40.0,y_end-.5/40.0);
		max_y = max(y_start+.5/40.0,y_end+.5/40.0);
		min_z = min(z_start-.5/40.0,z_end-.5/40.0);
		max_z = max(z_start+.5/40.0,z_end+.5/40.0);
	}
	



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

bool AirplaneConstrainedEnvironment::ViolatesConstraint(const airplaneState &from, const airplaneState &to, int time) const
{
	// Generate the two timestates that define the action box
	airplaneAction act = ae->GetAction(from, to);
	airtimeState fromtimestate(from, time);
	airtimeState totimestate(from, time);
	this->ApplyAction(totimestate, act);
        return ViolatesConstraint(fromtimestate,totimestate);

/*
	// Generate a well formed set of boxes for the action box
	// which we will need later to compare
	int a_minx = min(fromtimestate.l.x, totimestate.l.x);
	int a_maxx = max(fromtimestate.l.x, totimestate.l.x);

	int a_miny = min(fromtimestate.l.y, totimestate.l.y);
	int a_maxy = max(fromtimestate.l.y, totimestate.l.y);

	int a_minz = min(fromtimestate.l.height, totimestate.l.height);
	int a_maxz = max(fromtimestate.l.height, totimestate.l.height);

	// Allocate some temp variables
	int c_minx, c_maxx, c_miny, c_maxy, c_minz, c_maxz;
	
	//Check if the action box violates any of the constraints that are in the constraints list
	for (airConstraint c : constraints)
	{
		// Check if the range of the constraint overlaps in time
		if (max(c.start_state.t, fromtimestate.t) <= min(c.end_state.t, totimestate.t) + 0.00001) 
		{
			// Each constraint defines an x, y, z box with corners on the state locations. We 
			// need to check if they overlap at any point. We do this using the separating 
			// axis theorem. Note that all of our boxes are non-rotated, and thus, should never have
			// to worry about non-axis aligned planes of separation. Thus, we check to see if there is 
			// a separating plane on the X axis, Y axis and Z axis, and if no such plane exists, then
			// the boxes must intersect.
			
			// Generate a well formed set of boxes for the constraint box
			c_minx = min(c.start_state.l.x, c.end_state.l.x);
			c_maxx = max(c.start_state.l.x, c.end_state.l.x);

			c_miny = min(c.start_state.l.y, c.end_state.l.y);
			c_maxy = max(c.start_state.l.y, c.end_state.l.y);

			c_minz = min(c.start_state.l.height, c.end_state.l.height);
			c_maxz = max(c.start_state.l.height, c.end_state.l.height);

			
			if (max(c_minx, a_minx) <= min(c_maxx, a_maxx) && // Check if overlapping on the X axis
				max(c_miny, a_miny) <= min(c_maxy, a_maxy) && // Check if overlapping on the Y axis
				max(c_minz, a_minz) <= min(c_maxz, a_maxz)    // Check if overlapping on the Z axis
				)
			{
				// If we overlap on all three axis, then there must be a common point, and thus
				// we can return that the constraint was violated
				return true;
			}
		}
	}

	// If no constraint is violated, return false
	return false;
*/
}

// Basically the same code as above, but overloaded so the first section is not necessary
bool AirplaneConstrainedEnvironment::ViolatesConstraint(const airtimeState &from, const airtimeState &to) const
{
	// Generate a well formed set of boxes for the action box
	// which we will need later to compare
	int a_minx = min(from.l.x, to.l.x);
	int a_maxx = max(from.l.x, to.l.x);

	int a_miny = min(from.l.y, to.l.y);
	int a_maxy = max(from.l.y, to.l.y);

	int a_minz = min(from.l.height, to.l.height);
	int a_maxz = max(from.l.height, to.l.height);

	// Allocate some temp variables
	int c_minx, c_maxx, c_miny, c_maxy, c_minz, c_maxz;

	//Check if the action box violates any of the constraints that are in the constraints list
	for (airConstraint c : constraints)
	{
		// Check if the range of the constraint overlaps in time
		if (max(c.start_state.t, from.t) <= min(c.end_state.t, to.t) + 0.00001) 
		{
			/*
	      	// Vertex collision
	        if(c.end_state.l.x == to.l.x && c.end_state.l.y == to.l.y && c.end_state.l.height == to.l.height) 
	        	return true;

	        // Edge collision
	        if((c.end_state.l.x == from.l.x && c.end_state.l.y == from.l.y && c.end_state.l.height == from.l.height) &&
	           (c.start_state.l.x == to.l.x && c.start_state.l.y == to.l.y && c.start_state.l.height == to.l.height)) 
	        	return true;
	        */
	       // Generate a well formed set of boxes for the constraint box
			c_minx = min(c.start_state.l.x, c.end_state.l.x);
			c_maxx = max(c.start_state.l.x, c.end_state.l.x);

			c_miny = min(c.start_state.l.y, c.end_state.l.y);
			c_maxy = max(c.start_state.l.y, c.end_state.l.y);

			c_minz = min(c.start_state.l.height, c.end_state.l.height);
			c_maxz = max(c.start_state.l.height, c.end_state.l.height);

			
			if (max(c_minx, a_minx) <= min(c_maxx, a_maxx) && // Check if overlapping on the X axis
				max(c_miny, a_miny) <= min(c_maxy, a_maxy) && // Check if overlapping on the Y axis
				max(c_minz, a_minz) <= min(c_maxz, a_maxz)    // Check if overlapping on the Z axis
				)
			{
				// If we overlap on all three axis, then there must be a common point, and thus
				// we can return that the constraint was violated
				return true;
			}

		}
	}

	//Check if the action box violates any of the constraints that are in the static constraints list
	for (airConstraint c : static_constraints)
	{
		// Check if the range of the constraint overlaps in time
		if (max(c.start_state.t, from.t) <= min(c.end_state.t, to.t) + 0.00001) 
		{
			/*
	      	// Vertex collision
	        if(c.end_state.l.x == to.l.x && c.end_state.l.y == to.l.y && c.end_state.l.height == to.l.height) 
	        	return true;

	        // Edge collision
	        if((c.end_state.l.x == from.l.x && c.end_state.l.y == from.l.y && c.end_state.l.height == from.l.height) &&
	           (c.start_state.l.x == to.l.x && c.start_state.l.y == to.l.y && c.start_state.l.height == to.l.height)) 
	        	return true;
	        */
	       // Generate a well formed set of boxes for the constraint box
			c_minx = min(c.start_state.l.x, c.end_state.l.x);
			c_maxx = max(c.start_state.l.x, c.end_state.l.x);

			c_miny = min(c.start_state.l.y, c.end_state.l.y);
			c_maxy = max(c.start_state.l.y, c.end_state.l.y);

			c_minz = min(c.start_state.l.height, c.end_state.l.height);
			c_maxz = max(c.start_state.l.height, c.end_state.l.height);

			
			if (max(c_minx, a_minx) <= min(c_maxx, a_maxx) && // Check if overlapping on the X axis
				max(c_miny, a_miny) <= min(c_maxy, a_maxy) && // Check if overlapping on the Y axis
				max(c_minz, a_minz) <= min(c_maxz, a_maxz)    // Check if overlapping on the Z axis
				)
			{
				// If we overlap on all three axis, then there must be a common point, and thus
				// we can return that the constraint was violated
				return true;
			}

		}
	}

	// If no constraint is violated, return false
	return false;
}


