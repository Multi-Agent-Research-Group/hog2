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

template<typename State>
class Constraint : public DrawableConstraint{
  public:
    Constraint() {}
    Constraint(State const& start) : start_state(start), end_state(start) {}
    Constraint(State const& start, State const& end, bool neg=true) : start_state(start), end_state(end), negative(neg) {}

    State start() const {return start_state;}
    State end() const {return end_state;}

    virtual double ConflictsWith(State const& s) const=0;
    virtual double ConflictsWith(State const& from, State const& to) const=0;
    virtual double ConflictsWith(Constraint const& x) const {return ConflictsWith(x.start_state, x.end_state);}
    virtual void OpenGLDraw(MapInterface*) const {}
    virtual void print(std::ostream& os)const{
      os << "{" << start() << "-->" << end() << "}";
    }

    State start_state;
    State end_state;
    bool negative;
};

namespace std {
   template<typename State>
   struct less<Constraint<State> const*> {
     bool operator()(Constraint<State> const* a, Constraint<State> const* b)const{return a->start_state==b->start_state?a->end_state<b->end_state:a->start_state<b->start_state;}
   };
}

template<typename State>
class Box : public Constraint<State> {
  public:
    Box():Constraint<State>(){}
    Box(State const& start, State const& end, bool neg=true):Constraint<State>(start,end,neg) {}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    // Check whether the opposing action has an edge or bounding box conflict
    virtual double ConflictsWith(State const& from, State const& to) const {
      // Check time...
      if(this->start_state.t > to.t ||
          this->end_state.t < from.t){
        return 0;
      }
      // Check edge collisions...
      if(this->start_state.sameLoc(to) && this->end_state.sameLoc(from)){
        return std::max(this->start_state.t, from.t);
      }
      // Check bounding box...
      return (std::min(this->start_state.x,this->end_state.x) > std::max(to.x,from.x) ||
          std::max(this->start_state.x,this->end_state.x) < std::min(to.x,from.x) ||
          std::min(this->start_state.y,this->end_state.y) > std::max(to.y,from.y) ||
          std::max(this->start_state.y,this->end_state.y) < std::min(to.y,from.y))? 0 :
        std::max(this->start_state.t, from.t);

    }
};

template<typename State>
class Identical : public Constraint<State> {
  public:
    Identical():Constraint<State>(){}
    Identical(State const& s, bool neg=true):Constraint<State>(s,s,neg) {}
    Identical(State const& start, State const& end, bool neg=true):Constraint<State>(start,end,neg) {}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    // Check whether the action has the exact same time and to/from
    virtual double ConflictsWith(State const& from, State const& to) const {return (from==this->start_state && to==this->end_state)?from.t:0;}
};

template<typename State>
class TimeRange : public Constraint<State> {
  public:
    TimeRange():Constraint<State>(){}
    // Pass in: start,end with start/end coords and start/end times of collision...
    // The times do NOT correspond to the actual action times, but the time interval to avoid
    TimeRange(State const& start, State const& end, bool neg=true):Constraint<State>(start,end,neg){}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    // Check whether the opposing action has a conflict with this one
    virtual double ConflictsWith(State const& from, State const& to) const {return from.sameLoc(this->start_state) && to.sameLoc(this->end_state) && from.t >= this->start_state.t && to.t <= this->end_state.t; }
    virtual void OpenGLDraw(MapInterface*) const {}
};

template<typename State>
class Collision : public Constraint<State> {
  public:
    Collision(double radius=.25):Constraint<State>(),agentRadius(radius){}
    Collision(State const& start, State const& end,double radius=.25, bool neg=true):Constraint<State>(start,end,neg),agentRadius(radius){}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    // Check whether the opposing action has a conflict with this one
    virtual double ConflictsWith(State const& from, State const& to) const {return collisionCheck3D(from,to,this->start_state,this->end_state,agentRadius);}
    virtual double ConflictsWith(Collision<State> const& x) const {return collisionCheck3D(this->start_state,this->end_state,x.start_state,x.end_state,agentRadius,x.agentRadius);}
    virtual void OpenGLDraw(MapInterface*) const {}
    double agentRadius;
};

template<typename State>
class ConflictDetector{
  public:
    ConflictDetector(){}
    virtual bool HasConflict(State const& A1, State const& A2, State const& B1, State const& B2)const=0;
    // Returns a newly allocated pointer to a constraint representing the conflict or zero
    // The caller is responsible for managing the memory
    virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const=0;
    virtual void OpenGLDraw()const{}
};

template<typename State>
class CollisionDetector : public ConflictDetector<State> {
  public:
    CollisionDetector(double radius):ConflictDetector<State>(),agentRadius(radius){}
    inline virtual bool HasConflict(State const& A1, State const& A2, State const& B1, State const& B2)const{
      return collisionCheck3D(A1,A2,B1,B2,agentRadius);
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
       return new Collision<State>(B1,B2,agentRadius);
    }
    double agentRadius;
};

template<typename State>
class GroupConflictDetector{
  public:
    GroupConflictDetector(unsigned mainAgent, std::vector<unsigned> const& a):agent1(mainAgent),agentNumbers(a),agent2(-1){}
    inline virtual bool HasConflict(std::vector<std::pair<State,State>> const& states, std::unordered_map<unsigned,unsigned> const& translation, std::vector<std::vector<State>> const& staticpaths, std::vector<unsigned> const& staticAgents)const=0;
    virtual bool HasConflict(std::vector<std::vector<State>> const& solution)const=0;
    virtual bool HasConflict(std::vector<std::vector<State>> const& solution, std::vector<Constraint<State> const*>& c, std::pair<unsigned,unsigned>& conflict)const=0;
    virtual bool ShouldMerge(std::vector<std::vector<State>> const& solution, std::set<unsigned>& toMerge)const=0;
    // Returns a newly allocated pointer to a constraint representing the conflict or zero
    // The caller is responsible for managing the memory
    //virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const=0;
    virtual void OpenGLDraw(std::vector<std::pair<State,State>> const& states, double time, MapInterface* map)const=0;
    virtual GroupConflictDetector<State>* clone()=0;
    virtual bool operator==(GroupConflictDetector<State>* other)const=0;
    mutable unsigned agent1;
    std::vector<unsigned> agentNumbers; // List of agent numbers that must maintain tether
    mutable signed agent2;
    mutable std::map<unsigned,std::unordered_map<unsigned,std::vector<unsigned>>> staticConn;
};


template<typename State, typename Action>
class ConstrainedEnvironment : public SearchEnvironment<State, Action> {
  public:
    /** Add a constraint to the model */
    virtual void AddConstraint(Constraint<State> const* c){constraints.insert(c);}
    virtual void AddPositiveConstraint(Constraint<State> const* c){pconstraints.insert(c);}
    virtual void AddConstraints(std::vector<std::unique_ptr<Constraint<State> const>> const& cs){for(auto const& c:cs)constraints.insert(c.get());}
    /** Clear the constraints */
    virtual void ClearConstraints(){constraints.clear();pconstraints.clear();}
    /** Get the possible actions from a state */
    virtual void GetActions(const State &nodeID, std::vector<Action> &actions) const = 0;
    virtual void GetReverseActions(const State &nodeID, std::vector<Action> &actions) const = 0;
    /** Get the successor states not violating constraints */
    virtual void GetSuccessors(const State &nodeID, std::vector<State> &neighbors) const = 0;
    /** Checks to see if any constraint is violated, returning the time of violation, 0 otherwise */
    virtual inline double ViolatesConstraint(const State &from, const State &to) const {
      //Check if the action violates any of the constraints that are in the constraints list
      Identical<State> start(from);
      Identical<State> end(to);
      auto s(constraints.lower_bound((Constraint<State> const*)&start));
      auto const& e(constraints.upper_bound((Constraint<State> const*)&end));
      for(;s!=e;++s){
        double vtime((*s)->ConflictsWith(from,to));
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
    virtual void SetIgnoreTime(bool i){}
    virtual bool GetIgnoreTime()const{return false;}
    virtual void SetIgnoreHeading(bool i){}
    virtual bool GetIgnoreHeading()const{return false;}
    virtual bool collisionCheck(const State &s1, const State &d1, float r1, const State &s2, const State &d2, float r2)=0;

    State theGoal;
    std::set<Constraint<State> const*, std::less<Constraint<State> const*>> constraints;
    std::set<Constraint<State> const*, std::less<Constraint<State> const*>> pconstraints;
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
