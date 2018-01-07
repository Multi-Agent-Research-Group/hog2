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
    Constraint(State const& start, State const& end) : start_state(start), end_state(end) {}

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
};

template<typename State>
class Identical : public Constraint<State> {
  public:
    Identical():Constraint<State>(){}
    Identical(State const& start, State const& end):Constraint<State>(start,end) {}
    virtual double ConflictsWith(State const& s) const {return 0;} // Vertex collisions are ignored
    virtual double ConflictsWith(State const& from, State const& to) const {return (from.sameLoc(this->start_state) && to.sameLoc(this->end_state))?from.t:0;}
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
class GroupConstraint : public Constraint<State> {
  public:
    GroupConstraint(unsigned agent, std::vector<std::pair<State,State>> const& others):Constraint<State>(),agent1(agent),constraints(others){}
    virtual double ConflictsWith(State const& from, State const& to) const=0;
    virtual void OpenGLDraw(MapInterface*) const {}
    virtual void print(std::ostream& os)const{
      os << "{" << this->agent1 << " (";
      for(auto const& a: constraints)
        os << a.first << "," << a.second << " ";
      os << ")}";
    }

    unsigned agent1;
    mutable std::vector<std::pair<State,State>> constraints;
};

template<typename State>
class ChainLOS;

template<typename State>
class ChainLOSConstraint : public GroupConstraint<State> {
  public:
    ChainLOSConstraint(unsigned agent, std::vector<std::pair<State,State>> const& others, ChainLOS<State> const* gdetector)
    :GroupConstraint<State>(agent,others),detector(gdetector){}

    virtual double ConflictsWith(State const& from, State const& to) const{
      // Check time overlap
      unsigned ctime(from.t);
      for(auto const& s:this->constraints){
        if(from.t > s.second.t || to.t < s.first.t){
          //std::cout << "OK-time:" << from << to << " wrt " << s.first << s.second << "\n";
          return 0; // No overlap so no collision
        }
        ctime=std::min(ctime,s.first.t);
      }
      ctime=std::max(ctime,from.t); // Time of collision cannot be before from.t

      //std::cout << "Check " << from << to << " versus :\n";
      //for(auto const& s:this->constraints){
        //std::cout << s.first << s.second << "\n";
      //}
      this->constraints.emplace_back(from,to);
      if(detector->HasConflict(this->constraints)){
        this->constraints.pop_back();
        //std::cout << "BAD:" << from << to << "\n";
        return ctime;
      }
      this->constraints.pop_back();
      //std::cout << "OK:" << from << to << "\n";
      return 0; // No collision
    }
    virtual double ConflictsWith(State const& s) const {return ConflictsWith(s,s);} // Vertex collisions are ignored
    virtual void OpenGLDraw(MapInterface* map) const {
          GLdouble xx, yy, zz, rad;
          glColor3f(1,0,0);
          glLineWidth(10.0f);
      for(auto const& s:this->constraints){
          glBegin(GL_LINES);
          map->GetOpenGLCoord(s.first.x, s.first.y, s.first.z, xx, yy, zz, rad);
          glVertex3f(xx, yy, zz-rad/2);
          map->GetOpenGLCoord(s.second.x, s.second.y, s.second.z, xx, yy, zz, rad);
          glVertex3f(xx, yy, zz-rad/2);
          glEnd();
      }
    }
    ChainLOS<State> const* detector;
};

template<typename State>
class AnyLOS;

template<typename State>
class AnyLOSConstraint : public GroupConstraint<State> {
  public:
    AnyLOSConstraint(unsigned agent, std::vector<std::pair<State,State>> const& others, AnyLOS<State> const* gdetector)
    :GroupConstraint<State>(agent,others),detector(gdetector){}

    virtual double ConflictsWith(State const& from, State const& to) const{
      unsigned ctime(from.t);
      bool found(false);
      for(auto const& s:this->constraints){
        // Check time overlap
        if(from.t > s.second.t || to.t < s.first.t)
          return 0; // No overlap so no collision
        if(!detector->env->LineOfSight(std::make_pair(from,to),s)){
          ctime=std::min(ctime,s.first.t);
        }else{found=true;}
      }
      return found?0:ctime;
    }
    virtual double ConflictsWith(State const& s) const {return ConflictsWith(s,s);} // Vertex collisions are ignored
    virtual void OpenGLDraw(MapInterface*) const {}
    AnyLOS<State> const* detector;
};

template<typename State>
class AnyMinDist;

template<typename State>
class AnyMinDistConstraint : public GroupConstraint<State> {
  public:
    AnyMinDistConstraint(unsigned agent, std::vector<std::pair<State,State>> const& others, AnyMinDist<State> const* gdetector)
    :GroupConstraint<State>(agent,others),detector(gdetector){}

    virtual double ConflictsWith(State const& from, State const& to) const{
      unsigned ctime(from.t);
      bool found(false);
      for(auto const& s:this->constraints){
        // Check time overlap
        if(from.t > s.second.t || to.t < s.first.t)
          return 0; // No overlap so no collision
        if(fleq(detector->maxDistanceSq,distanceSquared(to,s.second))){
          ctime=std::min(ctime,s.first.t);
        }else{found=true;}
      }
      return found?0:ctime;
    }
    virtual double ConflictsWith(State const& s) const {return ConflictsWith(s,s);} // Vertex collisions are ignored
    virtual void OpenGLDraw(MapInterface*) const {}
    AnyMinDist<State> const* detector;
};


template<typename State>
class ConflictDetector{
  public:
    ConflictDetector(){}
    virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const=0;
    // Returns a newly allocated pointer to a constraint representing the conflict or zero
    // The caller is responsible for managing the memory
    virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const=0;
    virtual void OpenGLDraw()const{}
};

template<typename State>
class CollisionDetector : public ConflictDetector<State> {
  public:
    CollisionDetector(double radius):ConflictDetector<State>(),agentRadius(radius){}
    inline virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const{
      return collisionCheck3D(A1,A2,B1,B2,agentRadius);
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
       return new Collision<State>(B1,B2,agentRadius);
    }
    double agentRadius;
};

// Require LOS between all agents
template<typename State>
class AllLOS : public ConflictDetector<State> {
  public:
    AllLOS(std::vector<unsigned> const& agents, ObjectiveEnvironment<State> const*const e):ConflictDetector<State>(),agentNumbers(agents),env(e){}
    inline virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const{
      return std::find(agentNumbers.begin(),agentNumbers.end(),agentA)!=agentNumbers.end() &&
        std::find(agentNumbers.begin(),agentNumbers.end(),agentB)!=agentNumbers.end() &&
        env->LineOfSight(std::make_pair(A1,A2),std::make_pair(B1,B2));
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
      return new Identical<State>(A1,A2);
    }
    virtual void OpenGLDraw()const{}
    std::vector<unsigned> agentNumbers; // List of agent numbers that must maintain tether
    ObjectiveEnvironment<State> const*const env;
};

// Require MinDist between all agents
template<typename State>
class AllMinDist : public ConflictDetector<State> {
  public:
    AllMinDist(std::vector<unsigned> const& agents, double dist):ConflictDetector<State>(),agentNumbers(agents),maxDistanceSq(dist*dist){} // Squared distance
    inline virtual bool HasConflict(unsigned agentA, State const& A1, State const& A2, unsigned agentB, State const& B1, State const& B2)const{
      return std::find(agentNumbers.begin(),agentNumbers.end(),agentA)!=agentNumbers.end() &&
        std::find(agentNumbers.begin(),agentNumbers.end(),agentB)!=agentNumbers.end() &&
        fleq(maxDistanceSq,distanceSquared(A1,A2,B1,B2));
    }
    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
      return new Identical<State>(A1,A2);
    }
    virtual void OpenGLDraw()const{}
    std::vector<unsigned> agentNumbers; // List of agent numbers that must maintain tether
    double maxDistanceSq;
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

// Require all agents have LOS to a unique agent
template<typename State>
class ChainLOS: public GroupConflictDetector<State> {
  public:
    ChainLOS(std::vector<unsigned> const& a, ObjectiveEnvironment<State> const*const e):GroupConflictDetector<State>(a[0],a),env(e){}


    // DFS to accumulate visited agents
    inline void connCheck(unsigned agent, std::unordered_map<unsigned,std::vector<unsigned>>& conn, std::unordered_set<unsigned>& agents)const{
      if(agents.find(agent)!=agents.end()) return;
      agents.insert(agent);
      for(auto& a:conn[agent]){
        connCheck(a,conn,agents);
      }
    }

    inline virtual bool HasConflict(std::vector<std::pair<State,State>> const& states)const{
      std::unordered_map<unsigned,std::vector<unsigned>> conn(states.size()); // Connectivity graph
      // Compute adjacency
      //std::cout << "checking actions:\n";
      for(int i(0); i<states.size(); ++i){
        for(int j(i+1); j<states.size(); ++j){
          if(env->LineOfSight(states[i],states[j])){
            conn[i].push_back(j);
            conn[j].push_back(i);
            //std::cout << "Actions have LOS: " << states[i].first << states[i].second << " - " << states[j].first << states[j].second << "\n";
          }
          //else
            //std::cout << "Actions NO LOS: " << states[i].first << states[i].second << " - " << states[j].first << states[j].second << "\n";
        }
      }

      // Now search the graph for connectivity to all agents
      std::unordered_set<unsigned> agents;
      connCheck(0,conn,agents);
      return agents.size()!=this->agentNumbers.size();
    }


    inline virtual bool HasConflict(std::vector<std::pair<State,State>> const& states, std::unordered_map<unsigned,unsigned> const& agentTranslation, std::vector<std::vector<State>> const& staticpaths, std::vector<unsigned> const& staticagents)const{
      // Compute adjacency for static paths at all times
      // (this only needs to happen once since static paths don't change)
      std::vector<unsigned> staticAgents(staticagents);
      for(auto a(staticAgents.begin()); a!=staticAgents.end(); /**/){
        if(std::find(this->agentNumbers.begin(),this->agentNumbers.end(),*a)==this->agentNumbers.end()){
          a=staticAgents.erase(a);
        }else{
          ++a;
        }
      }
      std::unordered_map<unsigned,unsigned> translation(agentTranslation);
      for(auto a(translation.begin()); a!=translation.end(); /**/){
        if(std::find(this->agentNumbers.begin(),this->agentNumbers.end(),a->first)==this->agentNumbers.end()){
          a=translation.erase(a);
        }else{
          ++a;
        }
      }
      if(this->staticConn.empty()&&staticAgents.size()>1){
        unsigned t(0);
        std::vector<unsigned> indices(staticAgents.size());
        this->staticConn[t]=std::unordered_map<unsigned,std::vector<unsigned>>();
        bool allDone(false);
        while(!allDone){
          unsigned newt(0xfffff);
          // Create adjacency lists for each time
          for(int i(0); i<staticAgents.size(); ++i){
            for(int j(i+1); j<staticAgents.size(); ++j){
              if(env->LineOfSight(std::make_pair(staticpaths[staticAgents[i]][indices[i]],staticpaths[staticAgents[i]][indices[i]+1]),
                  std::make_pair(staticpaths[staticAgents[j]][indices[j]],staticpaths[staticAgents[j]][indices[j]+1]))){
                this->staticConn[t][staticAgents[i]].push_back(staticAgents[j]);
                this->staticConn[t][staticAgents[j]].push_back(staticAgents[i]);
              }
            }
            if(indices[i]!=staticpaths[staticAgents[i]].size()-2 && staticpaths[staticAgents[i]][indices[i]+1].t<newt){
              newt=staticpaths[staticAgents[i]][indices[i]+1].t;
            }
          }
          t=newt;
          // Increment to next time step, check for finish criteria
          allDone=true;
          for(int i(0); i<staticAgents.size(); ++i){
            if(indices[i]!=staticpaths[staticAgents[i]].size()-2){
              allDone=false;
              if(staticpaths[staticAgents[i]][indices[i]+1].t==t){
                indices[i]++;
              }
            }
          }
        }
      }

      std::unordered_map<unsigned,std::vector<unsigned>>* statics(nullptr);
      unsigned maxTime(INT32_MAX);
      unsigned minTime(0);
      for(auto const& s:states){
        minTime=std::max(minTime,s.first.t);
        maxTime=std::min(maxTime,s.second.t);
      }
      auto const& list(this->staticConn.find(minTime));
      if(list!=this->staticConn.end()){
        statics=&list->second;
      }else{
        unsigned diff(INT32_MAX);
        std::unordered_map<unsigned,std::vector<unsigned>>* last(nullptr);
        for(auto& s:this->staticConn){
          if(s.first>=minTime){
            statics=&s.second;
            break;
          }
          last=&s.second;
        }
        if(!statics){
          statics=last;
        }
      }
      std::unordered_map<unsigned,std::vector<unsigned>> conn(this->agentNumbers.size()); // Connectivity graph
      // Compute adjacency for all non-static agents vs. all agents
      for(auto i(translation.begin()); i!=translation.end(); ++i){
        State const& mainState(states[i->second].first);
        State const& succState(states[i->second].second);
        // Check vs. agents in the translation map (agents being actively planned)
        for(auto j(i); ++j!=translation.end(); /*++j*/){
          if(env->LineOfSight(std::make_pair(mainState,succState),states[j->second])){
            conn[i->first].push_back(j->first);
            conn[j->first].push_back(i->first);
          }else{
            //std::cout << "NO LOS between agents " << i->first << " and " << j->first << "@" << minTime << "\n";
          }
        }
        // Check vs. static agents (agents not being actively planned)
        for(auto const& a:staticAgents){
          unsigned diff(INT32_MAX);
          // Fetch the states which have time overlap
          auto s1(std::lower_bound(staticpaths[a].begin(),staticpaths[a].end(), staticpaths[a][0]/*dummy!*/,[&](State const& lhs, State const& ){return lhs.t<minTime;}));
          auto s2(s1+1 != staticpaths[a].end()?s1+1:s1);
          if(s2 == staticpaths[a].end() || env->LineOfSight(std::make_pair(mainState,succState),std::make_pair(*s1,*s2))){
            conn[i->first].push_back(a);
            conn[a].push_back(i->first);
          }else{
            //std::cout << "NO LOS to static agent " << a << " vs " << i->first << "@" << *s1 << "-->" << *s2 << "\n";
          }
        }
        if(conn[i->first].size()==0){
          //std::cout << "Nothing in LOS for agent: " << i->first << " at " << minTime << "\n";
          return true; // Nothing is in LOS
        }
      }
      if(statics){
        for(auto const& a:staticAgents){
          if(conn.find(a)!=conn.end()){
            conn[a].insert(conn[a].begin(),(*statics)[a].begin(),(*statics)[a].end());
          }else{
            conn[a]=(*statics)[a];
          }
        }
      }

      // Now search the graph for connectivity to all agents
      std::unordered_set<unsigned> agents;
      connCheck(translation.begin()->first,conn,agents);
      // there may be some staticAgents unrelated to this constraint...

      if(agents.size()!=this->agentNumbers.size()){
        //std::cout << "NO Conn: ";
        //for(auto const& s:states){
          //std::cout << s.first << " --> " << s.second << ", ";
        //}
        //std::cout <<"\n";
        return true;
      }else{
        return false;
      }
    }


    inline virtual bool ShouldMerge(std::vector<std::vector<State>> const& solution, std::set<unsigned>& toMerge)const{
      std::vector<unsigned> indices(this->agentNumbers.size()); // Defaults to zero
      unsigned index(0);

      unsigned minTime(solution[this->agent1][index].t);
      // Move other agents forward in time if necessary (we don't assume all paths start at t=0)
      for(int i(0); i<this->agentNumbers.size(); ++i){
        while(indices[i]<solution[this->agentNumbers[i]].size()-1 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
      }
      this->agent2=-1;
      while(index<solution[this->agent1].size()-1){
        double closest(999999999);
        double dist(999999999);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          dist=distanceSquared(solution[this->agent1][index],solution[this->agent1][index+1],solution[this->agentNumbers[i]][indices[i]],solution[this->agentNumbers[i]][indices[i]+1]);
          if(dist<closest){
            closest=dist;
            this->agent2=this->agentNumbers[i];
          }
        }
        toMerge.insert(this->agent2);
        minTime=solution[this->agent1][++index].t;
        // Move other agents forward in time if necessary
        for(int i(0); i<this->agentNumbers.size(); ++i){
          while(indices[i]<solution[this->agentNumbers[i]].size()-2 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
        }
      }
      toMerge.insert(this->agent1);
      return toMerge.size()>1;
    }

    inline virtual bool HasConflict(std::vector<std::vector<State>> const& solution)const{
      std::vector<Constraint<State> const*> c;
      std::pair<unsigned,unsigned> conflict;
      return HasConflict(solution,c,conflict);
    }

    inline virtual bool HasConflict(std::vector<std::vector<State>> const& solution, std::vector<Constraint<State> const*>& c, std::pair<unsigned,unsigned>& conflict)const{
      // Step through time, checking for group-wise conflicts
      std::vector<unsigned> indices(this->agentNumbers.size()); // Defaults to zero
      bool foundConflict(false);
      unsigned minTime(0);
      double closest(999999999);
      this->agent2=-1;
      unsigned i1(0);
      unsigned i2(0);
      while(!foundConflict){
        std::unordered_map<unsigned,std::vector<unsigned>> conn;
        double dist(999999999);
        bool violation(true);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          for(int j(i+1); j<this->agentNumbers.size(); ++j){
            if(env->LineOfSight(std::make_pair(solution[this->agentNumbers[i]][indices[i]],solution[this->agentNumbers[i]][indices[i]+1]),
                std::make_pair(solution[this->agentNumbers[j]][indices[j]],solution[this->agentNumbers[j]][indices[j]+1]))){
              //std::cout << "LOS: " << solution[this->agentNumbers[i]][indices[i]] << solution[this->agentNumbers[i]][indices[i]+1] << " - " << solution[this->agentNumbers[j]][indices[j]] << solution[this->agentNumbers[j]][indices[j]+1] << "\n";
              conn[i].push_back(j);
              conn[j].push_back(i);
            }
            //else
              //std::cout << "NO LOS: " << solution[this->agentNumbers[i]][indices[i]] << solution[this->agentNumbers[i]][indices[i]+1] << " - " << solution[this->agentNumbers[j]][indices[j]] << solution[this->agentNumbers[j]][indices[j]+1] << "\n";
          }
        }
        std::unordered_set<unsigned> agents;
        connCheck(0,conn,agents);
        if(agents.size()!=this->agentNumbers.size()){
          //Pick a random agent from the set
          //auto a(agents.begin());
          //std::advance(a,rand()%agents.size());
          //this->agent1=*a;
          /*
          std::vector<unsigned> disconnected;
          for(int i(0); i<this->agentNumbers.size(); ++i){
            if(agents.find(this->agentNumbers[i])==agents.end()){
              disconnected.push_back(this->agentNumbers[i]);
            }
          }
          //this->agent2=disconnected[rand()%disconnected.size()];
          std::cout << "agents ";
          for(auto const& a:agents){
            std::cout << a << ",";
          }
          std::cout << " are disconnected from ";
          for(auto const& a:disconnected){
            std::cout << a << ",";
          }
          std::cout << "at t=" << minTime << "\n";
          */
          conflict.first++;
          foundConflict=true;
          break;
        }
        // Increment to next time step, check for finish criteria
        minTime=+0xfffff;
        bool done(true);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          if(indices[i]<solution[this->agentNumbers[i]].size()-2)
            minTime=std::min(solution[this->agentNumbers[i]][indices[i]+1].t,minTime);
          if(indices[i]!=solution[this->agentNumbers[i]].size()-2){
            done=false;
          }
        }
        if(done)break;
        // Move other agents forward in time if necessary
        for(int i(0); i<this->agentNumbers.size(); ++i){
          while(indices[i]<solution[this->agentNumbers[i]].size()-2 && solution[this->agentNumbers[i]][indices[i]+1].t==minTime){indices[i]++;}
        }
      }
      if(foundConflict){
        c.resize(0);
        for(auto const& a:this->agentNumbers){
          std::vector<std::pair<State,State>> v(this->agentNumbers.size()-1);
          int i(0);
          for(auto const& b:this->agentNumbers){
             if(a==b) continue;
             v[i]=std::make_pair(solution[b][indices[i]],solution[b][indices[i]+1]);
             i++;
          }
          c.push_back(new ChainLOSConstraint<State>(a,v,this));
        }
      }
      return foundConflict;
    }

    virtual void OpenGLDraw(std::vector<std::pair<State,State>> const& states, double time, MapInterface* map)const{
      std::vector<TemporalVector3D> s;
      for(auto const& a:states){
        TemporalVector3D s1(a.first);
        TemporalVector3D vec = TemporalVector3D(a.second)-s1; // Vector pointing from a to b
        vec.Normalize();
        s1+=vec*(time-s1.t);
        s.push_back(s1);
      }
      for(int i(0); i<s.size(); ++i){
        for(int j(i+1); j<s.size(); ++j){
          if(!map->LineOfSight(s[i],s[j])){
            continue;
          }

          glEnable(GL_BLEND);
          GLdouble xx1, yy1, zz1, rad, xx2, yy2, zz2;
          map->GetOpenGLCoord(s[i].x, s[i].y, s[i].z, xx1, yy1, zz1, rad);
          map->GetOpenGLCoord(s[j].x, s[j].y, s[j].z, xx2, yy2, zz2, rad);
          glBegin(GL_LINES);
          glVertex3f(xx1, yy1, zz1-rad/2);
          glVertex3f(xx2, yy2, zz2-rad/2);
          glEnd();
          glDisable(GL_BLEND);
        }
      }
    }
    ObjectiveEnvironment<State> const*const env=nullptr;
    virtual GroupConflictDetector<State>* clone(){return new ChainLOS(*this);}
    virtual bool operator==(GroupConflictDetector<State>* other)const{return typeid(*this)==typeid(*other) && this->agentNumbers==other->agentNumbers;}
};

// Require LOS between agent A and at least one other agent in the set
template<typename State>
class AnyLOS: public GroupConflictDetector<State> {
  public:
    AnyLOS(unsigned agentA, std::vector<unsigned> const& a, ObjectiveEnvironment<State> const*const e):GroupConflictDetector<State>(agentA,a),env(e){}


    inline virtual bool HasConflict(std::vector<std::pair<State,State>> const& states, std::unordered_map<unsigned,unsigned> const& translation, std::vector<std::vector<State>> const& staticpaths, std::vector<unsigned> const& staticAgents)const{
      State const& mainState(states[translation.find(this->agent1)->second].first);
      State const& succState(states[translation.find(this->agent1)->second].second);
      for(auto const& i:this->agentNumbers){
        // Check vs. agents in the translation map (agents being actively planned)
        auto val(translation.find(i)); // Translate global to local
        if(val!=translation.end()){
          if(env->LineOfSight(std::make_pair(mainState,succState),states[val->second])){
            return false;
          }
        }else{
          // Check vs. static agents (agents not being actively planned)
          auto s1(staticpaths[i].begin());
          for(auto s2(staticpaths[i].begin()+1); s2!=staticpaths[i].end(); ++s2,++s1){
            if(s2->t<mainState.t)continue;
            if(s1->t>succState.t)break;
            if(env->LineOfSight(std::make_pair(mainState,succState),std::make_pair(*s1,*s2))){
              return false;
            }
          }
        }
      }
      return true;
    }

    inline virtual bool ShouldMerge(std::vector<std::vector<State>> const& solution, std::set<unsigned>& toMerge)const{
      std::vector<unsigned> indices(this->agentNumbers.size()); // Defaults to zero
      unsigned index(0);

      unsigned minTime(solution[this->agent1][index].t);
      // Move other agents forward in time if necessary (we don't assume all paths start at t=0)
      for(int i(0); i<this->agentNumbers.size(); ++i){
        while(indices[i]<solution[this->agentNumbers[i]].size()-1 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
      }
      this->agent2=-1;
      while(index<solution[this->agent1].size()-1){
        double closest(999999999);
        double dist(999999999);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          dist=distanceSquared(solution[this->agent1][index],solution[this->agent1][index+1],solution[this->agentNumbers[i]][indices[i]],solution[this->agentNumbers[i]][indices[i]+1]);
          if(dist<closest){
            closest=dist;
            this->agent2=this->agentNumbers[i];
          }
        }
        toMerge.insert(this->agent2);
        minTime=solution[this->agent1][++index].t;
        // Move other agents forward in time if necessary
        for(int i(0); i<this->agentNumbers.size(); ++i){
          while(indices[i]<solution[this->agentNumbers[i]].size()-2 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
        }
      }
      toMerge.insert(this->agent1);
      return toMerge.size()>1;
    }

    inline virtual bool HasConflict(std::vector<std::vector<State>> const& solution)const{
      std::vector<Constraint<State> const*> c;
      std::pair<unsigned,unsigned> conflict;
      return HasConflict(solution,c,conflict);
    }

    inline virtual bool HasConflict(std::vector<std::vector<State>> const& solution, std::vector<Constraint<State> const*>& c, std::pair<unsigned,unsigned>& conflict)const{
      // Step through time, checking for group-wise conflicts
      std::vector<unsigned> indices(this->agentNumbers.size()); // Defaults to zero
      unsigned index(0);
      bool foundConflict(false);
      unsigned minTime(solution[this->agent1][index].t);
      // Move other agents forward in time if necessary (we don't assume all paths start at t=0)
      for(int i(0); i<this->agentNumbers.size(); ++i){
        while(indices[i]<solution[this->agentNumbers[i]].size()-1 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
      }
      double closest(999999999);
      this->agent2=-1;
      unsigned i1(index);
      unsigned i2(index);
      while(index<solution[this->agent1].size()-2){
        double dist(999999999);
        bool violation(true);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          if(!env->LineOfSight(std::make_pair(solution[this->agent1][index],solution[this->agent1][index+1]),
              std::make_pair(solution[this->agentNumbers[i]][indices[i]],solution[this->agentNumbers[i]][indices[i]+1]))){
            //dist=distanceSquared(solution[this->agent1][index],solution[this->agentNumbers[i]][indices[i]]);
            //if(dist<closest){
            if(this->agent2==-1||rand()%2){
              closest=dist;
              this->agent2=this->agentNumbers[i];
              i1=index;
              i2=indices[i];
            }
          }else{
            violation=false;
            break;
          }
        }
        if(violation){ // No agents are in LOS of the main agent
          conflict.first++;
          foundConflict=true;
          break;
        }
        minTime=solution[this->agent1][++index].t;
        // Move other agents forward in time if necessary
        for(int i(0); i<this->agentNumbers.size(); ++i){
          while(indices[i]<solution[this->agentNumbers[i]].size()-2 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
        }
      }
      if(foundConflict){
        // Create constraint for main agent
        {
          std::vector<std::pair<State,State>> v(this->agentNumbers.size());
          int i(0);
          for(auto const& a:this->agentNumbers){
            v[i++]=std::make_pair(solution[a][indices[a]],solution[a][indices[a]+1]);
            c.push_back(new AnyLOSConstraint<State>(this->agent1,v,this));
          }
        }
        // Create constraints for other agents
        for(auto const& a:this->agentNumbers){
          c.push_back(new AnyLOSConstraint<State>(a,std::vector<std::pair<State,State>>(1,std::make_pair(solution[this->agent1][index],solution[this->agent1][index+1])),this));
        }
      }
      return foundConflict;
    }

    inline virtual Constraint<State>* GetConstraint(State const& A1, State const& A2, State const& B1, State const& B2)const{
      return new Identical<State>(A1,A2);
    }
    virtual void OpenGLDraw(std::vector<std::pair<State,State>> const& states, double time, MapInterface* map)const{
      // agent1 is the first entry in "states"
      std::vector<TemporalVector3D> s;
      for(auto const& a:states){
        TemporalVector3D s1(a.first);
        TemporalVector3D vec = TemporalVector3D(a.second)-s1; // Vector pointing from a to b
        vec.Normalize();
        s1+=vec*(time-s1.t);
        s.push_back(s1);
      }
      for(int i(1); i<s.size(); ++i){
        if(!map->LineOfSight(s[0],s[i])){
          //glColor4f(0,0,0,0);
          continue;
        }//else{
          //glColor4f(1,0,0,0);
        //}
        GLdouble xx1, yy1, zz1, rad, xx2, yy2, zz2;
        map->GetOpenGLCoord(s[0].x, s[0].y, s[0].z, xx1, yy1, zz1, rad);
        map->GetOpenGLCoord(s[i].x, s[i].y, s[i].z, xx2, yy2, zz2, rad);
        glBegin(GL_LINES);
        glVertex3f(xx1, yy1, zz1-rad/2);
        glVertex3f(xx2, yy2, zz2-rad/2);
        glEnd();

      }
    }
    ObjectiveEnvironment<State> const*const env=nullptr;
    virtual GroupConflictDetector<State>* clone(){return new AnyLOS(*this);}
    virtual bool operator==(GroupConflictDetector<State>* other)const{return typeid(*this)==typeid(*other) && this->agent1==other->agent1 && this->agentNumbers==other->agentNumbers;}
};

// Require MinDist between agent A and at least one other agent in the set
template<typename State>
class AnyMinDist: public GroupConflictDetector<State> {
  public:
    AnyMinDist(unsigned agentA, std::vector<unsigned> const& a, double dist):GroupConflictDetector<State>(agentA,a),maxDistanceSq(dist*dist){} // Squared distance

    inline virtual bool HasConflict(std::vector<std::pair<State,State>> const& states, std::unordered_map<unsigned,unsigned> const& translation, std::vector<std::vector<State>> const& staticpaths, std::vector<unsigned> const& staticAgents)const{
      auto const& a1(translation.find(this->agent1));

      State const* mainState(nullptr);
      State const* succState(nullptr);
      unsigned minTime(states[0].first.t);
      unsigned maxTime(states[0].second.t);
      for(auto s(states.begin()+1); s!=states.end(); ++s){
        minTime = std::min(s->first.t,minTime);
        maxTime = std::max(s->second.t,maxTime);
      }
      if(a1==translation.end()){
        bool found(false);
        for(auto s(staticpaths[this->agent1].begin()); s!=staticpaths[this->agent1].end()-1; ++s){
          if(minTime<=s->t && maxTime>=s->t){
            mainState = &*s;
            succState = &*(s+1);
            found=true;
            break;
          }
        }
        if(!found){
          mainState=&*(staticpaths[this->agent1].rbegin()+1);
          succState=&*(staticpaths[this->agent1].rbegin());
        }
      }else{
        mainState=&states[a1->second].first;
        succState=&states[a1->second].second;
      }

      for(auto const& i:this->agentNumbers){
        auto const& trans(translation.find(i));
        if(trans!=translation.end()){
          if(fgreater(this->maxDistanceSq,distanceSquared(*mainState,*succState,states[trans->second].first,states[trans->second].second))){
            return false;
          }
        }else{
          for(auto const& s:staticpaths[i]){
            if(mainState->t<=s.t && succState->t>=s.t &&
                (fgreater(this->maxDistanceSq,distanceSquared(*mainState,s))||
                 fgreater(this->maxDistanceSq,distanceSquared(*succState,s)))){
              return false;
            }
          }
        }
      }
      for(auto const& a:staticAgents){
        if(std::find(this->agentNumbers.begin(),this->agentNumbers.end(),a)!=this->agentNumbers.end()){
          for(auto const& s:staticpaths[a]){
            if(mainState->t<=s.t && succState->t>=s.t &&
                (fgreater(this->maxDistanceSq,distanceSquared(*mainState,s))||
                 fgreater(this->maxDistanceSq,distanceSquared(*succState,s)))){
              return false;
            }
          }
        }
      }
      return true;
    }

    inline virtual bool ShouldMerge(std::vector<std::vector<State>> const& solution, std::set<unsigned>& toMerge)const{
      std::vector<unsigned> indices(this->agentNumbers.size()); // Defaults to zero
      unsigned index(0);

      unsigned minTime(solution[this->agent1][index].t);
      // Move other agents forward in time if necessary (we don't assume all paths start at t=0)
      for(int i(0); i<this->agentNumbers.size(); ++i){
        while(indices[i]<solution[this->agentNumbers[i]].size()-1 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
      }
      this->agent2=-1;
      while(index<solution[this->agent1].size()-1){
        double closest(999999999);
        double dist(999999999);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          dist=distanceSquared(solution[this->agent1][index],solution[this->agent1][index+1],solution[this->agentNumbers[i]][indices[i]],solution[this->agentNumbers[i]][indices[i]+1]);
          if(dist<closest){
            closest=dist;
            this->agent2=this->agentNumbers[i];
          }
        }
        toMerge.insert(this->agent2);
        minTime=solution[this->agent1][++index].t;
        // Move other agents forward in time if necessary
        for(int i(0); i<this->agentNumbers.size(); ++i){
          while(indices[i]<solution[this->agentNumbers[i]].size()-2 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
        }
      }
      toMerge.insert(this->agent1);
      return toMerge.size()>1;
    }

    inline virtual bool HasConflict(std::vector<std::vector<State>> const& solution)const{
      std::vector<Constraint<State> const*> c;
      std::pair<unsigned,unsigned> conflict;
      return HasConflict(solution,c,conflict);
    }

    inline virtual bool HasConflict(std::vector<std::vector<State>> const& solution, std::vector<Constraint<State> const*>& c, std::pair<unsigned,unsigned>& conflict)const{
      // Step through time, checking for group-wise conflicts
      std::vector<unsigned> indices(this->agentNumbers.size()); // Defaults to zero
      unsigned index(0);
      bool foundConflict(false);
      unsigned minTime(solution[this->agent1][index].t);
      // Move other agents forward in time if necessary (we don't assume all paths start at t=0)
      for(int i(0); i<this->agentNumbers.size(); ++i){
        while(indices[i]<solution[this->agentNumbers[i]].size()-1 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
      }
      double closest(999999999);
      this->agent2=-1;
      unsigned i1(index);
      unsigned i2(index);
      while(index<solution[this->agent1].size()-1){
        double dist(999999999);
        bool violation(true);
        for(int i(0); i<this->agentNumbers.size(); ++i){
          if(fleq(maxDistanceSq,dist=distanceSquared(solution[this->agent1][index],solution[this->agent1][index+1],solution[this->agentNumbers[i]][indices[i]],solution[this->agentNumbers[i]][indices[i]+1]))){
            if(dist<closest){
              closest=dist;
              this->agent2=this->agentNumbers[i];
              i1=index;
              i2=indices[i];
            }
          }else{
            violation=false;
            break;
          }
        }
        if(violation){ // No agents are in MinDist of the main agent
          conflict.first++;
          foundConflict=true;
          break;
        }
        minTime=solution[this->agent1][++index].t;
        // Move other agents forward in time if necessary
        for(int i(0); i<this->agentNumbers.size(); ++i){
          while(indices[i]<solution[this->agentNumbers[i]].size()-2 && solution[this->agentNumbers[i]][indices[i]].t<minTime){indices[i]++;}
        }
      }
      if(foundConflict){
        // Create constraint for main agent
        {
          std::vector<std::pair<State,State>> v(this->agentNumbers.size());
          int i(0);
          for(auto const& a:this->agentNumbers){
            v[i++]=std::make_pair(solution[a][indices[a]],solution[a][indices[a]+1]);
            c.push_back(new AnyMinDistConstraint<State>(this->agent1,v,this));
          }
        }
        // Create constraints for other agents
        for(auto const& a:this->agentNumbers){
          c.push_back(new AnyMinDistConstraint<State>(a,std::vector<std::pair<State,State>>(1,std::make_pair(solution[this->agent1][index],solution[this->agent1][index+1])),this));
        }
      }
      return foundConflict;
    }

    virtual void OpenGLDraw(std::vector<std::pair<State,State>> const& states, double time, MapInterface* map)const{
      // agent1 is the first entry in "states"
      std::vector<TemporalVector3D> s;
      for(auto const& a:states){
        TemporalVector3D s1(a.first);
        TemporalVector3D vec = TemporalVector3D(a.second)-s1; // Vector pointing from a to b
        vec.Normalize();
        s1+=vec*(time-s1.t);
        s.push_back(s1);
      }
      for(int i(1); i<s.size(); ++i){
        double distSq((s[0].x-s[i].x)*(s[0].x-s[i].x)+(s[0].y-s[i].y)*(s[0].y-s[i].y)+(s[0].z-s[i].z)*(s[0].z-s[i].z));
        if(distSq>maxDistanceSq){
          //glColor4f(0,0,0,0);
          continue;
        }//else{
          //glColor4f(1,0,0,0);
        //}
        GLdouble xx1, yy1, zz1, rad, xx2, yy2, zz2;
        map->GetOpenGLCoord(s[0].x, s[0].y, s[0].z, xx1, yy1, zz1, rad);
        map->GetOpenGLCoord(s[i].x, s[i].y, s[i].z, xx2, yy2, zz2, rad);
        glBegin(GL_LINES);
        glVertex3f(xx1, yy1, zz1-rad/2);
        glVertex3f(xx2, yy2, zz2-rad/2);
        glEnd();

      }
    }
    double maxDistanceSq;
    virtual GroupConflictDetector<State>* clone(){return new AnyMinDist(*this);}
    virtual bool operator==(GroupConflictDetector<State>* other)const{return typeid(*this)==typeid(*other) && this->agent1==other->agent1 && this->agentNumbers==other->agentNumbers;}
};

template<typename State, typename Action>
class ConstrainedEnvironment : public SearchEnvironment<State, Action> {
  public:
    /** Add a constraint to the model */
    virtual void AddConstraint(Constraint<State> const* c){constraints.emplace_back(c);}
    virtual void AddConstraints(std::vector<std::unique_ptr<Constraint<State> const>> const& cs){for(auto const& c:cs)constraints.push_back(c.get());}
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
    virtual void SetIgnoreTime(bool i){}
    virtual bool GetIgnoreTime()const{return false;}
    virtual void SetIgnoreHeading(bool i){}
    virtual bool GetIgnoreHeading()const{return false;}
    virtual bool collisionCheck(const State &s1, const State &d1, float r1, const State &s2, const State &d2, float r2)=0;

    State theGoal;
    std::vector<Constraint<State> const*> constraints;
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
