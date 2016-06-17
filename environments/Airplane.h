//
//  Airplane.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef Airplane_h
#define Airplane_h

#include <vector>
#include <cassert>
#include <cmath>
#include "SearchEnvironment.h"

// turn type:
// 45, 90, 0, shift
// speed:
// faster, slower
// height:
// up / down

const uint8_t k45 = 1;
const uint8_t k90 = 2;
const uint8_t kShift = 3;

struct airplaneAction {
public:
	airplaneAction(int8_t t=0, int8_t s=0, int8_t h=0)
	:turn(t), speed(s), height(h) {}
	int8_t turn;
	int8_t speed;
	int8_t height;
};

// state
struct airplaneState {
public:
	airplaneState() { x = y = heading = 0; speed = 1; height = 20; }
	uint8_t x;
	uint8_t y;
	uint8_t height;
	uint8_t speed : 2;
	uint8_t heading : 6;
};

/** Output the information in an airplane state */
static std::ostream& operator <<(std::ostream & out, const airplaneState &loc)
{
	out << "(x:" << unsigned(loc.x) << ", y:" << unsigned(loc.y) << ", h:" << unsigned(loc.height) << ": s:" << unsigned(loc.speed) <<
											    ", hdg: " << unsigned(loc.heading) << ")";
	return out;
}

bool operator==(const airplaneState &s1, const airplaneState &s2);

//class GoalTester {
//public:
//	virtual ~GoalTester() {}
//	virtual bool goalTest(const airplaneState &i1) const = 0;
//};

class AirplaneEnvironment : public SearchEnvironment<airplaneState, airplaneAction>
{
public:
	AirplaneEnvironment();
	virtual void GetSuccessors(const airplaneState &nodeID, std::vector<airplaneState> &neighbors) const;
	virtual void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual void ApplyAction(airplaneState &s, airplaneAction dir) const;
	virtual void UndoAction(airplaneState &s, airplaneAction dir) const;
	virtual void GetNextState(const airplaneState &currents, airplaneAction dir, airplaneState &news) const;
	
	virtual OccupancyInterface<airplaneState,airplaneAction> *GetOccupancyInfo() { return 0; }
	virtual bool InvertAction(airplaneAction &a) const { return false; }
	
	virtual double HCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double HCost(const airplaneState &)  const { assert(false); return 0; }
	virtual double GCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double GCost(const airplaneState &node1, const airplaneAction &act) const;

	virtual double GetPathLength(const std::vector<airplaneState> &n) const;

//	void SetGoalTest(GoalTester *t) {test = t;}
    virtual airplaneAction GetAction(const airplaneState &node1, const airplaneState &node2) const;
	virtual bool GoalTest(const airplaneState &node, const airplaneState &goal) const;
	virtual bool GoalTest(const airplaneState &) const { assert(false); return false; }
	virtual uint64_t GetStateHash(const airplaneState &node) const;
	virtual uint64_t GetActionHash(airplaneAction act) const;
	
	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const airplaneState &l) const;
	virtual void OpenGLDraw(const airplaneState& oldState, const airplaneState &newState, float perc) const;
	virtual void OpenGLDraw(const airplaneState &, const airplaneAction &) const;
	void GLDrawLine(const airplaneState &a, const airplaneState &b) const;
	void GLDrawPath(const std::vector<airplaneState> &p) const;
	
	recVec GetCoordinate(int x, int y, int z) const;


	std::vector<uint8_t> getGround();
	std::vector<recVec> getGroundNormals();
	std::vector<airplaneAction> getInternalActions();


protected:
	void SetGround(int x, int y, uint8_t val);
	uint8_t GetGround(int x, int y) const;
	bool Valid(int x, int y);
	recVec &GetNormal(int x, int y);
	recVec GetNormal(int x, int y) const;
	void RecurseGround(int x1, int y1, int x2, int y2);
	const int width = 80; 
	const int length = 80;
	const int height = 20;
	std::vector<uint8_t> ground;
	std::vector<recVec> groundNormals;
	void DoNormal(recVec pa, recVec pb) const;
	mutable std::vector<airplaneAction> internalActions;
};

#endif /* Airplane_h */
