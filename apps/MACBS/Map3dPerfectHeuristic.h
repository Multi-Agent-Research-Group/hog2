#include "Grid3DConstrainedEnvironment.h"
#include "Heuristic.h"
#include "PEAStar.h"
#include "TemplateAStar.h"

// Computes a perfect heuristic given a goal state.
// Note that this only works assuming that agents are
// holonomic (otherwise a reverse search becomes imperative)

template<typename state, typename action>
class Map3dPerfectHeuristic: public Heuristic<state> {

  public:
    // constructor
    Map3dPerfectHeuristic( Map3D *_m, Grid3DConstrainedEnvironment* _e): m(_m),e(_e),depth(e->GetMapEnv()->agentType==Map3D::air?m->GetMapDepth():1),elapsed(0.0),loaded(false),list(m->GetMapHeight()*m->GetMapWidth()*depth){}

    double HCost( const state &s1, const state &s2 ) const {
      if(!loaded){
        goal=s2;
        Timer tmr;
        tmr.StartTimer();
        bool originalIgnoreTime(e->GetIgnoreTime());
        bool originalIgnoreHeading(e->GetIgnoreHeading());
        e->SetIgnoreTime(true); // Otherwise the search would never terminate
        e->SetIgnoreHeading(true);  // Don't care about alternate paths to this state
        //PEAStar<state,action,Grid3DConstrainedEnvironment> astar;
        TemplateAStar<state,action,Grid3DConstrainedEnvironment> astar;
        //std::cout << "Loading heuristic\n";
        //astar.SetVerbose(false);
        astar.SetHeuristic(new ZeroHeuristic<state>);
        astar.SetStopAfterGoal(false); // Search the entire space
        // Now perform a search to get all costs
        // NOTE: This should be a reverse-search, but our agents are holonomic
        // so the costs forward are the same as the costs backward
        std::vector<state> path;
        astar.GetPath(e,goal,s1,path);
        for(int w(0); w<m->GetMapWidth(); ++w){
          for(int h(0); h<m->GetMapHeight(); ++h){
            for(int d(0); d<depth; ++d){
              uint64_t h1(e->GetStateHash({w,h,d}));
              uint64_t id1;
              if(kClosedList==astar.GetOpenList()->Lookup(h1,id1))
                list[w*m->GetMapHeight()*depth+h*depth+d]=astar.GetOpenList()->Lookat(id1).g;
              else
                list[w*m->GetMapHeight()*depth+h*depth+d]=9999999;
            }
          }
        }
        e->SetIgnoreTime(originalIgnoreTime);
        e->SetIgnoreHeading(originalIgnoreHeading);
        loaded=true;
        elapsed+=tmr.EndTimer();
      }
      assert(s2.sameLoc(goal));
      return list[s1.x*m->GetMapHeight()*depth+s1.y*depth+s1.z];
    }
    mutable double elapsed;

  private:
    Map3D *m;
    Grid3DConstrainedEnvironment *e;
    unsigned depth;
    mutable bool loaded;
    mutable std::vector<float> list;
    mutable state goal;

};

