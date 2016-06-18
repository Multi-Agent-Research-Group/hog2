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

// Add a constraint of any type
void AirplaneConstrainedEnvironment::AddConstraint(airConstraint &c)
{
	constraints.push_back(c);
}

// Add a point constraint
void AirplaneConstrainedEnvironment::AddPointConstraint(airtimeState &loc)
{
	pointConstraint l(loc);
	constraints.push_back(l);
}

// Add a sphere constraint
void AirplaneConstrainedEnvironment::AddSphereConstraint(airtimeState &loc, float rad)
{
	sphereConstraint s(loc, rad);
	constraints.push_back(s);
}

// Add a cylinder constraint
void AirplaneConstrainedEnvironment::AddCylConstraint(airtimeState &loc1, airtimeState &loc2, float rad)
{
	cylConstraint c(loc1, loc2, rad);
	constraints.push_back(c);
}

// Add a arc constraint
void AirplaneConstrainedEnvironment::AddArcConstraint(airtimeState &loc1, airtimeState &loc2, float rad)
{
	arcConstraint c(loc1, loc2, rad);
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
	
        // Assume x,y discretization of 3 meters
	h |= unsigned(round(node.l.x/3.0)) & (0x2000-1);
	h = h << 17;
	h |= unsigned(round(node.l.y/3.0)) & (0x2000-1);
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
        // Time increments are in 5 second intervals
	h |= node.t & (0x1000-1);
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
	// TODO: IMPLEMENT THIS FOR EACH CONSTRAINT
	for (int i = 0; i < constraints.size(); i++)
	{
		constraints.at(i).OpenGLDraw();	
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
	airtimeState fromtimestate(from, time);
	airtimeState totimestate(to, time);
	airplaneAction act = this->GetAction(fromtimestate, totimestate);

	for (int i = 0; i < constraints.size(); i++)
	{
		// Check to see if the time, height, x and y are the same. Notice that heading has no influence on the 
		// constraints
		if (constraints.at(i).ViolatesConstraint(fromtimestate, totimestate) || constraints.at(i).ViolatesEdgeConstraint(fromtimestate, totimestate, act))
			return true;
	}
	return false;
}

////////////////////// CONSTRAINT CHECKING /////////////////////////////////////////////////

/// Point constraints
bool pointConstraint::ViolatesConstraint(airtimeState &startingLoc, airtimeState &endLoc) const {
	if (startingLoc.t == this-> loc.t && 
		sqrt(pow((startingLoc.l.x - this->loc.l.x), 2.0) + pow((startingLoc.l.y - this->loc.l.y), 2.0) + pow((startingLoc.l.height - this->loc.l.height), 2.0)) 
			< airConstraint::POINT_DISTANCE_MARGIN
		)
		return true;
	return false;
}

bool pointConstraint::ViolatesEdgeConstraint(airtimeState &startingLoc, airtimeState &endLoc, airplaneAction &action) const {
	if (startingLoc.t <= this->loc.t && startingLoc.t + 1 >= this->loc.t)
	{
		// Compute the vector from start to end
		double AB_x = 0 , AB_y = 0, AB_z = 0, AB_squared = 0;
		AB_x = endLoc.l.x - startingLoc.l.x;
		AB_y = endLoc.l.y - startingLoc.l.y;
		AB_z = endLoc.l.height - startingLoc.l.height;
		AB_squared = pow(AB_x, 2.0) + pow(AB_y, 2.0) + pow(AB_z, 2.0);

		// If A and B are the same point, then we check if the constraint of this point
		// violates the single point structure
		if (AB_squared == 0) 
		{
			return sqrt(pow((startingLoc.l.x - this->loc.l.x), 2.0) + pow((startingLoc.l.y - this->loc.l.y), 2.0) + pow((startingLoc.l.height - this->loc.l.height), 2.0)) < airConstraint::POINT_DISTANCE_MARGIN;
		}

		// Compute the vector from A to the constraint point 
		double AP_x = 0 , AP_y = 0, AP_z = 0;
		AP_x = this->loc.l.x - startingLoc.l.x;
		AP_y = this->loc.l.y - startingLoc.l.y;
		AP_z = this->loc.l.height - startingLoc.l.height;

		// We then project this vector onto the other vector
		double t = (AP_x * AB_x + AP_y * AB_y + AP_z * AB_z)/AB_squared;

		// If it's on the line (that is, between 0 and 1, we know that we only have to
		// look at things projected between the two points, so we just compare against
		// then endpoints)
		if (t < 0.0) 
		{
			return sqrt(pow((startingLoc.l.x - this->loc.l.x), 2.0) + pow((startingLoc.l.y - this->loc.l.y), 2.0) + pow((startingLoc.l.height - this->loc.l.height), 2.0)) < airConstraint::POINT_DISTANCE_MARGIN;
		} 
		else if (t > 1.0)
		{
			return sqrt(pow((endLoc.l.x - this->loc.l.x), 2.0) + pow((endLoc.l.y - this->loc.l.y), 2.0) + pow((endLoc.l.height - this->loc.l.height), 2.0)) < airConstraint::POINT_DISTANCE_MARGIN;
		}

		// Otherwise we compare against the constrained point
		float C_x = startingLoc.l.x + t * AB_x;
		float C_y = startingLoc.l.y + t * AB_y;
		float C_z = startingLoc.l.height + t * AB_z;

		return sqrt(pow((C_x - this->loc.l.x), 2.0) + pow((C_y - this->loc.l.y), 2.0) + pow((C_z - this->loc.l.height), 2.0)) < airConstraint::POINT_DISTANCE_MARGIN;
	}

	return false;
}

/// Sphere constraints
bool sphereConstraint::ViolatesConstraint(airtimeState &startingLoc, airtimeState &endLoc) const {
	if (startingLoc.t == this-> loc.t && 
		sqrt(pow((startingLoc.l.x - this->loc.l.x), 2.0) + pow((startingLoc.l.y - this->loc.l.y), 2.0) + pow((startingLoc.l.height - this->loc.l.height), 2.0)) < this->radius
		)
		return true;
	return false;
}

// The only difference in this method is that the radius is no longer constant
bool sphereConstraint::ViolatesEdgeConstraint(airtimeState &startingLoc, airtimeState &endLoc, airplaneAction &action) const {
	if (startingLoc.t <= this->loc.t && startingLoc.t + 1 >= this->loc.t)
	{
		// Compute the vector from start to end
		double AB_x = 0 , AB_y = 0, AB_z = 0, AB_squared = 0;
		AB_x = endLoc.l.x - startingLoc.l.x;
		AB_y = endLoc.l.y - startingLoc.l.y;
		AB_z = endLoc.l.height - startingLoc.l.height;
		AB_squared = pow(AB_x, 2.0) + pow(AB_y, 2.0) + pow(AB_z, 2.0);

		// If A and B are the same point, then we check if the constraint of this point
		// violates the single point structure
		if (AB_squared == 0) 
		{
			return sqrt(pow((startingLoc.l.x - this->loc.l.x), 2.0) + pow((startingLoc.l.y - this->loc.l.y), 2.0) + pow((startingLoc.l.height - this->loc.l.height), 2.0)) < this->radius;
		}

		// Compute the vector from A to the constraint point 
		double AP_x = 0 , AP_y = 0, AP_z = 0;
		AP_x = this->loc.l.x - startingLoc.l.x;
		AP_y = this->loc.l.y - startingLoc.l.y;
		AP_z = this->loc.l.height - startingLoc.l.height;

		// We then project this vector onto the other vector
		double t = (AP_x * AB_x + AP_y * AB_y + AP_z * AB_z)/AB_squared;

		// If it's on the line (that is, between 0 and 1, we know that we only have to
		// look at things projected between the two points, so we just compare against
		// then endpoints)
		if (t < 0.0) 
		{
			return sqrt(pow((startingLoc.l.x - this->loc.l.x), 2.0) + pow((startingLoc.l.y - this->loc.l.y), 2.0) + pow((startingLoc.l.height - this->loc.l.height), 2.0)) < this->radius;
		} 
		else if (t > 1.0)
		{
			return sqrt(pow((endLoc.l.x - this->loc.l.x), 2.0) + pow((endLoc.l.y - this->loc.l.y), 2.0) + pow((endLoc.l.height - this->loc.l.height), 2.0)) < this->radius;
		}

		// Otherwise we compare against the constrained point
		float C_x = startingLoc.l.x + t * AB_x;
		float C_y = startingLoc.l.y + t * AB_y;
		float C_z = startingLoc.l.height + t * AB_z;

		return sqrt(pow((C_x - this->loc.l.x), 2.0) + pow((C_y - this->loc.l.y), 2.0) + pow((C_z - this->loc.l.height), 2.0)) < this->radius;
	}

	return false;
}

/// Cylinder constraint

// We project the airtime state onto the cylinder's central arc, and check against the radius
bool cylConstraint::ViolatesConstraint(airtimeState &startingLoc, airtimeState &endLoc) const {

	// Check if the times are equal
	if (startingLoc.t >= this->loc1.t && this->loc2.t >= startingLoc.t)
	{
		// Compute the vector from start to end
		double AB_x = 0 , AB_y = 0, AB_z = 0, AB_squared = 0;
		AB_x = this->loc2.l.x - this->loc1.l.x;
		AB_y = this->loc2.l.y - this->loc1.l.y;
		AB_z = this->loc2.l.height - this->loc1.l.height;
		AB_squared = pow(AB_x, 2.0) + pow(AB_y, 2.0) + pow(AB_z, 2.0);

		// If A and B are the same point, then we check if the constraint of this point
		// violates the single point structure
		if (AB_squared == 0) 
		{
			return sqrt(pow((startingLoc.l.x - this->loc1.l.x), 2.0) + pow((startingLoc.l.y - this->loc1.l.y), 2.0) + pow((startingLoc.l.height - this->loc1.l.height), 2.0)) < this->radius;
		}

		// Compute the vector from the first location in the constraint to the test-point
		double AP_x = 0 , AP_y = 0, AP_z = 0;
		AP_x = startingLoc.l.x - this->loc1.l.x;
		AP_y = startingLoc.l.y - this->loc1.l.y;
		AP_z = startingLoc.l.height - this->loc1.l.height;

		// We then project this vector onto the other vector
		double t = (AP_x * AB_x + AP_y * AB_y + AP_z * AB_z)/AB_squared;

		// If it's on the line (that is, between 0 and 1, we know that we only have to
		// look at things projected between the two points, so we just compare against
		// then endpoints)
		if (t < 0.0) 
		{
			return sqrt(pow((startingLoc.l.x - this->loc1.l.x), 2.0) + pow((startingLoc.l.y - this->loc1.l.y), 2.0) + pow((startingLoc.l.height - this->loc1.l.height), 2.0)) < this->radius;
		} 
		else if (t > 1.0)
		{
			return sqrt(pow((startingLoc.l.x - this->loc2.l.x), 2.0) + pow((startingLoc.l.y - this->loc2.l.y), 2.0) + pow((startingLoc.l.height - this->loc2.l.height), 2.0)) < this->radius;
		}

		// Otherwise we compare against the constrained point
		float C_x = this->loc1.l.x + t * AB_x;
		float C_y = this->loc1.l.y + t * AB_y;
		float C_z = this->loc1.l.height + t * AB_z;

		return sqrt(pow((C_x - startingLoc.l.x), 2.0) + pow((C_y - startingLoc.l.y), 2.0) + pow((C_z - startingLoc.l.height), 2.0)) < this->radius;
	}

	return false;
}

bool cylConstraint::ViolatesEdgeConstraint(airtimeState &startingLoc, airtimeState &endLoc,  airplaneAction &action) const {
	if ((startingLoc.t >= this->loc1.t && startingLoc.t <= this->loc2.t) || (startingLoc.t + 1 >= this->loc1.t && startingLoc.t + 1 <= this->loc2.t ) || (startingLoc.t <= this->loc1.t && startingLoc.t + 1 >= this->loc2.t))
	{
		// Compute the vector of the constraint
		double U_x = 0 , U_y = 0, U_z = 0, U_squared = 0;
		U_x = this->loc2.l.x - this->loc1.l.x;
		U_y = this->loc2.l.y - this->loc1.l.y;
		U_z = this->loc2.l.height - this->loc1.l.height;
		U_squared = pow(U_x, 2.0) + pow(U_y, 2.0) + pow(U_z, 2.0);
	    
		// Compute the vector of the action
		double V_x = 0 , V_y = 0, V_z = 0, V_squared = 0;
		V_x = endLoc.l.x - startingLoc.l.x;
		V_y = endLoc.l.y - startingLoc.l.y;
		V_z = endLoc.l.height - startingLoc.l.height;
		V_squared = pow(V_x, 2.0) + pow(V_y, 2.0) + pow(V_z, 2.0);

		// Compute the vector between the first two elements of each segment
		double W_x = 0 , W_y = 0, W_z = 0;
		W_x = this->loc1.l.x - startingLoc.l.x;
		W_y = this->loc1.l.y - startingLoc.l.y;
		W_z = this->loc1.l.height - startingLoc.l.height;

	    // Compute a number of key dot products
	    float    a = U_squared;         // always >= 0
	    float    b = U_x * V_x + U_y * V_y + U_z * V_z;
	    float    c = V_squared;         // always >= 0
	    float    d = U_x * W_x + U_y * W_y + U_z * W_z;
	    float    e = V_x * W_x + V_y * W_y + V_z * W_z;

	    // Create some values that we use to do math
	    float    D = a*c - b*b;        // always >= 0
	    float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	    float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

	    // compute the line parameters of the two closest points
	    if (D < 0.001) { // the lines are almost parallel
	        sN = 0.0;         // force using point P0 on segment S1
	        sD = 1.0;         // to prevent possible division by 0.0 later
	        tN = e;
	        tD = c;
	    }
	    else {                 // get the closest points on the infinite lines
	        sN = (b*e - c*d);
	        tN = (a*e - b*d);
	        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
	            sN = 0.0;
	            tN = e;
	            tD = c;
	        }
	        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
	            sN = sD;
	            tN = e + b;
	            tD = c;
	        }
	    }

	    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
	        tN = 0.0;
	        // recompute sc for this edge
	        if (-d < 0.0)
	            sN = 0.0;
	        else if (-d > a)
	            sN = sD;
	        else {
	            sN = -d;
	            sD = a;
	        }
	    }
	    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
	        tN = tD;
	        // recompute sc for this edge
	        if ((-d + b) < 0.0)
	            sN = 0;
	        else if ((-d + b) > a)
	            sN = sD;
	        else {
	            sN = (-d +  b);
	            sD = a;
	        }
	    }
	    // finally do the division to get sc and tc
	    sc = (abs(sN) < 0.001 ? 0.0 : sN / sD);
	    tc = (abs(tN) < 0.001 ? 0.0 : tN / tD);

	    // get the difference of the two closest points
	    float dP_x = W_x + (sc * U_x) - (tc * V_x);  // =  S1(sc) - S2(tc)
	    float dP_y = W_y + (sc * U_y) - (tc * V_y); 
	    float dP_z = W_z + (sc * U_z) - (tc * V_z); 

	    return pow(dP_x, 2.0) + pow(dP_y, 2.0) + pow(dP_z, 2.0) <= this->radius;   // return the closest distance
	}

	return false;
}


/// Arc Constraint - just discretizes an arc with cylinders
// We project the airtime state onto the cylinder's central arc, and check against the radius

arcConstraint::arcConstraint(airtimeState l1, airtimeState l2, float r) 
{
	//TODO: Discretize the arc action, and deal with it all.
	//right now it just uses a single cylinder
	
	this->radius = r;
	cylConstraint c* = new cylConstraint(l1, l2, r);
	ics.push_back(c);
	
}

bool arcConstraint::ViolatesConstraint(airtimeState &loc, airtimeState &endLoc) const
{
	for (cylConstraint c : this->ics)
	{
		if (c.ViolatesConstraint(loc, endLoc))
			return true;
	}
	return false;
}
bool arcConstraint::ViolatesEdgeConstraint(airtimeState &startingLoc, airtimeState &endLoc, airplaneAction &action) const
{
	for (cylConstraint c : this->ics)
	{
		if (c.ViolatesEdgeConstraint(startingLoc, endLoc, action))
			return true;
	}
	return false;
}