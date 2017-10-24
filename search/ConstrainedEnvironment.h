//
//  AirplaneConstrained.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__ConstrainedEnvironment__
#define __hog2_glut__ConstrainedEnvironment__

#include <vector>
#include <algorithm>
#include "MapInterface.h"
#include "SearchEnvironment.h"
#include "PositionalUtils.h"
#include "VelocityObstacle.h"

/*
struct Action{
  public:
    Action(){}
    virtual bool operator==(Action const&)const=0; // Equivalency check
    virtual bool operator!=(Action const& other)const{return ! this->operator==(other);} // Unequivalency check
    virtual void OpenGLDraw() const {} // Draw
};

class State{
  public:
    State(){}
    virtual bool operator==(State const&)const=0; // Equivalency check
    virtual bool operator!=(State const& other)const{return ! this->operator==(other);} // Unequivalency check
    virtual bool operator+=(Action const&)const=0; // Perform action
    virtual bool operator-=(Action const&)const=0; // Undo action
    virtual uint64_t hash()const=0; //Unique hash function
    virtual void OpenGLDraw() const {} // Draw
};
*/

class DrawableConstraint{
  public:
    virtual void OpenGLDraw(MapInterface*) const=0;
    static int width;
    static int length;
    static int height;
};

// Represents a radial constraint where cost increases as we approach the center
template<typename State>
class SoftConstraint : public DrawableConstraint {
  public:
    SoftConstraint() {}
    SoftConstraint(State const& c, double r) : center(c),radius(r),logr(log(r)){}

    virtual double cost(State const& other, double scale) const{
      double d(Util::distance(center.x,center.y,center.height,other.x,other.y,other.height)/scale);
      return std::max(0.0,logr/d-logr/radius);
    }
    virtual void OpenGLDraw(MapInterface*) const {}

    State center;
    double radius;
    double logr;
};

template<typename State>
class Constraint : public DrawableConstraint{
  public:
    Constraint() {}
    Constraint(State const& start) : start_state(start), end_state(start) {}
    Constraint(State const& start, State const& end) : start_state(start), end_state(end) {}

    State start() const {return start_state;}
    State end() const {return end_state;}

    virtual double ConflictsWith(State const& s) const=0;
    virtual double ConflictsWith(State const& from, State const& to) const=0;
    virtual double ConflictsWith(Constraint const& x) const {return ConflictsWith(x.start_state, x.end_state);}
    virtual void OpenGLDraw(MapInterface*) const {}

    State start_state;
    State end_state;
};

template<typename State>
class Identical : public Constraint<State> {
  public:
    Identical():Constraint<State>(){}
    Identical(State const& start, State const& end):Constraint<State>(start,end) {}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    virtual double ConflictsWith(State const& from, State const& to) const {return (from == this->start_state && to == this->end_state)?from.t:0;}
};

template<typename State>
class Collision : public Constraint<State> {
  public:
    Collision(double radius=.25):Constraint<State>(),agentRadius(radius){}
    Collision(State const& start, State const& end,double radius=.25):Constraint<State>(start,end),agentRadius(radius){}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    virtual double ConflictsWith(State const& from, State const& to) const {return collisionCheck3D(from,to,this->start_state,this->end_state,agentRadius);}
    virtual double ConflictsWith(Collision<State> const& x) const {return collisionCheck3D(this->start_state,this->end_state,x.start_state,x.end_state,agentRadius,x.agentRadius);}
    virtual void OpenGLDraw(MapInterface*) const {}
    double agentRadius;
};

template<typename State>
class ConflictDetector{
  public:
    ConflictDetector(){}
    virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const=0;
    // Returns a newly allocated pointer to a constraint representing the conflict or zero
    // The caller is responsible for managing the memory
    virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const=0;
};

template<typename State>
class CollisionDetector : public ConflictDetector<State> {
  public:
    CollisionDetector(double radius):ConflictDetector<State>(),agentRadius(radius){}
    inline virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const{
      return collisionCheck3D(A1,A2,B1,B2);
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
       return new Collision<State>(B1,B2,agentRadius);
    }
    double agentRadius;
};

// Require LOS between agent A and at least one other agent in the set
template<typename State>
class AnyLOS: public ConflictDetector<State> {
  public:
    AnyLOS(unsigned agentA, std::vector<unsigned> const& a, double dist):ConflictDetector<State>(),mainAgent(agentA),agentNumbers(a),maxDistanceSq(dist*dist),finalResult(true),agent(-1){} // Squared distance
    // Assume a violation until we find one in the set that is within line of sight (if none are found, a violation has occurred)
    inline virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const{
      if(finalResult){
        if(agentA==mainAgent && std::find(agentNumbers.begin(),agentNumbers.end(),agentB)!=agentNumbers.end() && !(finalResult=(fleq(maxDistanceSq,distanceSquared(A1,A2,B1,B2))))){
          agent=agentB;
          agent1={A1,A2}; // Main agent
          agent2={B1,B2};
        }else if(agentB==mainAgent && std::find(agentNumbers.begin(),agentNumbers.end(),agentA)!=agentNumbers.end() && !(finalResult=(fleq(maxDistanceSq,distanceSquared(A1,A2,B1,B2))))){
          agent=agentA;
          agent1={B1,B2}; // Main agent
          agent2={A1,A2};
        }
      }
      return finalResult;
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
      return new Identical<State>(A1,A2);
    }
    unsigned mainAgent;
    std::vector<unsigned> agentNumbers; // List of agent numbers that must maintain tether
    double maxDistanceSq;
    mutable bool finalResult;
    mutable signed agent;
    mutable std::pair<State,State> agent1;
    mutable std::pair<State,State> agent2;
    
};

// Require LOS between all agents
template<typename State>
class AllLOS : public ConflictDetector<State> {
  public:
    AllLOS(std::vector<unsigned> const& agents, double dist):ConflictDetector<State>(),agentNumbers(agents),maxDistanceSq(dist*dist){} // Squared distance
    inline virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const{
      return std::find(agentNumbers.begin(),agentNumbers.end(),agentA)!=agentNumbers.end() &&
        std::find(agentNumbers.begin(),agentNumbers.end(),agentB)!=agentNumbers.end() &&
        fleq(maxDistanceSq,distanceSquared(A1,A2,B1,B2));
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
      return new Identical<State>(A1,A2);
    }
    std::vector<unsigned> agentNumbers; // List of agent numbers that must maintain tether
    double maxDistanceSq;
};

template<typename State, typename Action>
class ConstrainedEnvironment : public SearchEnvironment<State, Action> {
  public:
    /** Add a constraint to the model */
    virtual void AddConstraint(Constraint<State>* c){constraints.emplace_back(c);}
    /** Clear the constraints */
    virtual void ClearConstraints(){constraints.resize(0);}
    /** Get the possible actions from a state */
    virtual void GetActions(const State &nodeID, std::vector<Action> &actions) const = 0;
    virtual void GetReverseActions(const State &nodeID, std::vector<Action> &actions) const = 0;
    /** Get the successor states not violating constraints */
    virtual void GetSuccessors(const State &nodeID, std::vector<State> &neighbors) const = 0;
    /** Checks to see if any constraint is violated, returning the time of violation, 0 otherwise */
    virtual inline double ViolatesConstraint(const State &from, const State &to) const {
      //Check if the action violates any of the constraints that are in the constraints list
      for (auto const& c : constraints){
        double vtime(c->ConflictsWith(from,to));
        if(vtime)return vtime;
      }
      return 0;
    }
    virtual void GLDrawLine(const State &x, const State &y) const{}
    virtual void GLDrawPath(const std::vector<State> &p, const std::vector<State> &waypoints) const{
      if(p.size()<2) return;
      //TODO Draw waypoints as cubes.
      for(auto a(p.begin()+1); a!=p.end(); ++a){ GLDrawLine(*(a-1),*a); }
    }
    virtual State const& getGoal()const{}
    virtual void SetIgnoreTime(bool i){}
    virtual bool GetIgnoreTime()const{return false;}
    virtual void SetIgnoreHeading(bool i){}
    virtual bool GetIgnoreHeading()const{return false;}
    virtual bool collisionCheck(const State &s1, const State &d1, float r1, const State &s2, const State &d2, float r2)=0;

    std::vector<Constraint<State>*> constraints;
};

// We initialize these here, but they can be changed at run-time
// These are only necessary for displaying constraints in OpenGL
/*template<typename State>
  int Constraint<State>::width=80;
  template<typename State>
  int Constraint<State>::length=80;
template<typename State>
int Constraint<State>::height=20;
*/

#endif /* defined(__hog2_glut__ConstrainedEnvironment__) */
