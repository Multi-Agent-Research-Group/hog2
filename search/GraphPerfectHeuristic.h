/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include "DigraphEnvironment.h"
#include "Heuristic.h"
#include "TemplateAStar.h"

// Computes a perfect heuristic given a goal state.
// Note that this only works assuming that agents are
// holonomic (otherwise a reverse search becomes imperative)
#define INF 9999999

class GraphPerfectHeuristic: public Heuristic<node_t> {

  public:
    // constructor
    GraphPerfectHeuristic( MapInterface *_m, DigraphEnvironment* _e):e(_e),elapsed(0.0),loaded(false),list(e->nodes.size()){}
    GraphPerfectHeuristic( MapInterface *_m, ConstrainedEnvironment<node_t,int>* _e):e((DigraphEnvironment*)_e),elapsed(0.0),loaded(false),list(e->nodes.size()){}

    double HCost( const node_t &s1, const node_t &s2 ) const {
      if(!loaded){
        goal=s2;
        if (e->adj[goal.id].size()==1)
        {
          for (auto const &n : e->nodes)
          {
            if (n.x == goal.x && n.y == goal.y && goal.id != n.id)
            {
              goal.id = n.id;
              break;
            }
          }
        }
        Timer tmr;
        tmr.StartTimer();
        TemplateAStar<node_t,int,DigraphEnvironment> astar;
        bool originalIgnoreTime(e->GetIgnoreTime());
        e->SetIgnoreTime(true); // Otherwise the search would never terminate
        //std::cout << "Loading heuristic\n";
        //astar.SetVerbose(true);
        std::unique_ptr<ZeroHeuristic<node_t>> h(new ZeroHeuristic<node_t>);
        astar.SetHeuristic(h.get());
        astar.SetStopAfterGoal(false); // Search the entire space
        // Now perform a search to get all costs
        // NOTE: This should be a reverse-search, but our agents are holonomic
        // so the costs forward are the same as the costs backward
        std::vector<node_t> path;
        astar.GetPath(e,goal,s1,path);
        goal=s2; // override
        for(auto const& n:e->nodes){
          if(list.size()-1<n.id){
            list.resize(n.id+1);
          }
          if(n.id<list.size()){
            uint64_t h1(e->GetStateHash(n));
            uint64_t id1;
            if(n.id==goal.id)
              list[n.id]=0;
            else if(kClosedList==astar.GetOpenList()->Lookup(h1,id1))
              list[n.id]=astar.GetOpenList()->Lookat(id1).g;
            else
              list[n.id]=INF;
          }
        }
        // Nodes with duplicate location can replace INF hueristics
        // This only happens with directional graphs
        for (auto const &n : e->nodes)
        {
          if (list[n.id] == INF)
          {
            for (auto const &a : e->adj[n.id])
            {
              for (auto const &b : e->adj[a])
              {
                if (e->nodes[b].x == n.x && e->nodes[b].y == n.y && b!=n.id)
                {
                  list[n.id] = list[b];
                  break;
                }
              }
            }
          }
        }
        e->SetIgnoreTime(originalIgnoreTime);
        loaded=true;
        elapsed+=tmr.EndTimer();
      }
      assert(s2.sameLoc(goal));
      return list[s1.id];
    }
    mutable double elapsed;

  private:
    DigraphEnvironment* e;
    mutable bool loaded;
    mutable std::vector<float> list;
    mutable node_t goal;

};

