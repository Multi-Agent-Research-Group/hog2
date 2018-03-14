//
//  AirplaneConstrained.h
//  hog2 glut
//
//  Created by David Chan on 6/8/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//
//  Modified by Thayne Walker 2017.
//

#ifndef __hog2_glut__ConstrainedEnvironment__
#define __hog2_glut__ConstrainedEnvironment__

#include <vector>
#include <set>
#include <map>
#include <memory>
#include <typeinfo>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include "MapInterface.h"
#include "SearchEnvironment.h"
#include "PositionalUtils.h"
#include "VelocityObstacle.h"
#include "BroadPhase.h"

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

    inline virtual double cost(State const& other, double scale) const{
      double d(Util::distance(center.x,center.y,center.height,other.x,other.y,other.height)/scale);
      return std::max(0.0,logr/d-logr/radius);
    }
    virtual void OpenGLDraw(MapInterface*) const {}

    State center;
    double radius;
    double logr;
};

template<typename BB>
class Constraint : public DrawableConstraint, public BB{
  public:
    Constraint() {}
    //Constraint(State const& start): BB(start,start){}
    Constraint(typename BB::State const& start, typename BB::State const& end, unsigned agent):BB(start,end,agent) {}
    Constraint(BB const& v):BB(v) {}

    //typename BB::State start() const {return start;}
    //typename BB::State end() const {return end;}

    virtual double ConflictsWith(typename BB::State const& s) const=0;
    virtual double ConflictsWith(typename BB::State const& from, typename BB::State const& to) const=0;
    virtual double ConflictsWith(Constraint const& x) const {return ConflictsWith(x.start, x.end);}
    virtual void OpenGLDraw(MapInterface*) const {}
    virtual void print(std::ostream& os)const{
      os << "{" << this->start << "-->" << this->end << "}";
    }
};

template<typename BB>
class Identical : public Constraint<BB> {
  public:
    Identical():Constraint<BB>(){}
    Identical(typename BB::State const& start, typename BB::State const& end, unsigned agent):Constraint<BB>(start,end,agent) {}
    Identical(BB const& v):Constraint<BB>(v){}
    virtual double ConflictsWith(typename BB::State const& s) const {return 0;} // Vertex collisions are ignored
    virtual double ConflictsWith(typename BB::State const& from, typename BB::State const& to) const {return (from.sameLoc(this->start) && to.sameLoc(this->end))?from.t:0;}
};

template<typename BB>
class Collision : public Constraint<BB> {
  public:
    Collision(double radius=.25):Constraint<BB>(),agentRadius(radius){}
    Collision(BB const& v,double radius=.25):Constraint<BB>(v),agentRadius(radius){}
    Collision(typename BB::State const& start, typename BB::State const& end, unsigned agent, double radius=.25):Constraint<BB>(start,end,agent),agentRadius(radius){}
    virtual double ConflictsWith(typename BB::State const& s) const {return 0;} // Vertex collisions are ignored
    virtual double ConflictsWith(typename BB::State const& from, typename BB::State const& to) const {return collisionCheck3D(from,to,this->start,this->end,agentRadius);}
    virtual double ConflictsWith(Collision<BB> const& x) const {return collisionCheck3D(this->start,this->end,x.start,x.end,agentRadius,x.agentRadius);}
    virtual void OpenGLDraw(MapInterface*) const {}
    double agentRadius;
};

template<typename BB>
class ConflictDetector{
  public:
    ConflictDetector(){}
    virtual bool HasConflict(typename BB::State const& A1, typename BB::State const& A2, typename BB::State const& B1, typename BB::State const& B2)const=0;
    // Returns a newly allocated pointer to a constraint representing the conflict or zero
    // The caller is responsible for managing the memory
    virtual Constraint<BB>* GetConstraint(typename BB::State const& A1, typename BB::State const& A2, typename BB::State const& B1, typename BB::State const& B2)const=0;
    virtual void OpenGLDraw()const{}
};

template<typename BB>
class CollisionDetector : public ConflictDetector<BB> {
  public:
    CollisionDetector(double radius):ConflictDetector<BB>(),agentRadius(radius){}
    inline virtual bool HasConflict(typename BB::State const& A1, typename BB::State const& A2, typename BB::State const& B1, typename BB::State const& B2)const{
      return collisionCheck3D(A1,A2,B1,B2,agentRadius);
    }
    inline virtual Constraint<BB>* GetConstraint(typename BB::State const& A1, typename BB::State const& A2, typename BB::State const& B1, typename BB::State const& B2)const{
       return new Collision<BB>(B1,B2,agentRadius);
    }
    double agentRadius;
};

template<typename BB>
class GroupConflictDetector{
  public:
    GroupConflictDetector(unsigned mainAgent, std::vector<unsigned> const& a):agent1(mainAgent),agentNumbers(a),agent2(-1){}
    inline virtual bool HasConflict(std::vector<std::pair<typename BB::State,typename BB::State>> const& states, std::unordered_map<unsigned,unsigned> const& translation, std::vector<std::vector<typename BB::State>> const& staticpaths, std::vector<unsigned> const& staticAgents)const=0;
    virtual bool HasConflict(std::vector<std::vector<typename BB::State>> const& solution)const=0;
    virtual bool HasConflict(std::vector<std::vector<typename BB::State>> const& solution, std::vector<Constraint<BB> const*>& c, std::pair<unsigned,unsigned>& conflict)const=0;
    virtual bool ShouldMerge(std::vector<std::vector<typename BB::State>> const& solution, std::set<unsigned>& toMerge)const=0;
    // Returns a newly allocated pointer to a constraint representing the conflict or zero
    // The caller is responsible for managing the memory
    //virtual Constraint<BB>* GetConstraint(typename BB::State const& A1, typename BB::State const& A2, typename BB::State const& B1, typename BB::State const& B2)const=0;
    virtual void OpenGLDraw(std::vector<std::pair<typename BB::State,typename BB::State>> const& states, double time, MapInterface* map)const=0;
    virtual GroupConflictDetector<BB>* clone()=0;
    virtual bool operator==(GroupConflictDetector<BB>* other)const=0;
    mutable unsigned agent1;
    std::vector<unsigned> agentNumbers; // List of agent numbers that must maintain tether
    mutable signed agent2;
    mutable std::map<unsigned,std::unordered_map<unsigned,std::vector<unsigned>>> staticConn;
};


template<typename BB, typename Action>
class ConstrainedEnvironment : public SearchEnvironment<typename BB::State, Action> {
  public:
    ConstrainedEnvironment(unsigned a):agent(a){}
    /** Add a constraint to the model */
    inline virtual void AddConstraint(Constraint<BB> const* c){constraints->insert(c);}
    inline virtual void AddConstraints(std::vector<std::unique_ptr<Constraint<BB> const>> const& cs){for(auto const& c:cs)constraints->insert(c.get());}
    /** Clear the constraints */
    virtual void ClearConstraints(){constraints->clear();}
    /** Get the possible actions from a state */
    virtual void GetActions(const typename BB::State &nodeID, std::vector<Action> &actions) const = 0;
    virtual void GetReverseActions(const typename BB::State &nodeID, std::vector<Action> &actions) const = 0;
    /** Get the successor states not violating constraints */
    virtual void GetSuccessors(const typename BB::State &nodeID, std::vector<typename BB::State> &neighbors) const = 0;
    /** Checks to see if any constraint is violated, returning the time of violation, 0 otherwise */
    virtual inline double ViolatesConstraint(typename BB::State const& from, typename BB::State const& to)const{
      //Check if the action violates any of the constraints that are in the constraints list
      return constraints->hasConflict(BB(from,to,agent));
    }
    virtual double GetPathLength(std::vector<BB> const&)const=0;
    virtual void GLDrawLine(const typename BB::State &x, const typename BB::State &y) const{}
    virtual void GLDrawPath(const std::vector<typename BB::State> &p, const std::vector<typename BB::State> &waypoints) const{
      if(p.size()<2) return;
      //TODO Draw waypoints as cubes.
      for(auto a(p.begin()+1); a!=p.end(); ++a){ GLDrawLine(*(a-1),*a); }
    }
    virtual void SetIgnoreTime(bool i){}
    virtual bool GetIgnoreTime()const{return false;}
    virtual void SetIgnoreHeading(bool i){}
    virtual bool GetIgnoreHeading()const{return false;}
    virtual bool collisionCheck(const BB &s1, float r1, const BB &s2, float r2)=0;

    typename BB::State theGoal;
    unsigned agent;
    BroadPhase<BB>* constraints;
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
