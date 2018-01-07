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
#include "Map2DConstrainedEnvironment.h"
#include "Heuristic.h"
#include "TemporalAStar.h"

// this is a heuristic that uses the differential heuristic
// with 100 canonical states and maximizes with octile distance
// heuristic.
#define INF 9999999

class Map2DDiffHeuristic: public Heuristic<xytLoc> {

  public:
    // constructor
    Map2DDiffHeuristic( Map *_m, Map2DConstrainedEnvironment* _e, unsigned int number_canonical_states ): m(_m),e(_e){
      //number_canonical_states=1; //for debug
      list.resize(number_canonical_states);
      std::string mapname(m->GetMapName());
      bool originalIgnoreTime(e->GetIgnoreTime());
      bool originalIgnoreHeading(e->GetIgnoreHeading());
      e->SetIgnoreTime(true); // Otherwise the search would never terminate
      e->SetIgnoreHeading(true);  // Don't care about alternate paths to this state
      std::set<xytLoc> points; // Make sure all are unique
      for(int i(0); i<number_canonical_states; ++i){
        list[i].resize(m->GetMapWidth()*m->GetMapHeight());
        std::stringstream fname;
        fname << mapname << "_" << i << "_" << (e->GetConnectedness()%2?e->GetConnectedness()-1:e->GetConnectedness());
        FILE* fp(0);
        if((fp=fopen(fname.str().c_str(), "rb"))==NULL) {
          std::cout << "Cannot open file: " << fname.str() << "... Generating from scratch.\n";
          TemporalAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
          astar.SetVerbose(true);
          astar.SetHeuristic(new ZeroHeuristic<xytLoc>);
          astar.SetStopAfterGoal(false); // Search the entire space
          xytLoc s;
          xytLoc g(99,99); // Goal doesn't really matter - ignored
          std::vector<xytLoc> path;
          if(points.size()==0){
            do{
              s={rand()%m->GetMapWidth(),rand()%m->GetMapHeight()};
            }while(!m->IsTraversable(s.x,s.y));
            astar.InitializeSearch(e,s,g,path,0);
          }else{
            s=*points.begin();
            astar.InitializeSearch(e,s,g,path,0);
            auto a(points.cbegin());
            ++a;
            for(; a!=points.end(); ++a){
              astar.AddAdditionalStartState(*a);
            }
          }
          // Get the furthest point away
          while(!astar.DoSingleSearchStep(path,0)){}
          points.insert(path.back());
          s=path.back();

          // Now perform a search to get all costs
          // NOTE: This should be a reverse-search, but our agents are holonomic
          // so the costs forward are the same as the costs backward
          path.clear();
          astar.GetPath(e,s,g,path,0);
          astar.GetOpenList()->SaveToFile(fname.str().c_str());
          for(int w(0); w<m->GetMapWidth(); ++w){
            for(int h(0); h<m->GetMapHeight(); ++h){
              uint64_t h1(e->GetStateHash({w,h}));
              uint64_t id1;
              if(kClosedList==astar.GetOpenList()->Lookup(h1,id1))
                list[i][w*m->GetMapHeight()+h]=astar.GetOpenList()->Lookat(id1).g;
              else
                list[i][w*m->GetMapHeight()+h]=INF;
            }
          }
          fp=fopen(fname.str().c_str(), "wb");
          fwrite(&list[i][0],list[i].size(),sizeof(float),fp);
          fclose(fp);
        }else{
          fread(&list[i][0],list[i].size(),sizeof(float),fp);
          fclose(fp);
          //std::cout << "Reading from file: " << fname.str() << "...\n";
          points.insert({i,0}); // add placeholder
        }
      }
      e->SetIgnoreTime(originalIgnoreTime);
      e->SetIgnoreHeading(originalIgnoreHeading);
    }

    double HCost( const xytLoc &s1, const xytLoc &s2 ) const {
      double hcost(e->HCost( s1, s2 ));
      for(auto const& v:list){
        hcost=std::max(hcost,fabs(v[s1.x*m->GetMapHeight()+s1.y]-v[s2.x*m->GetMapHeight()+s2.y]));
      }
      return hcost;
    }

    xytLoc findFurthest(){return {0,0};}

  private:
    Map *m;
    Map2DConstrainedEnvironment *e;
    //std::vector<AStarOpenClosed<xytLoc, TemporalAStarCompare<xytLoc>>> list;
    std::vector<std::vector<float>> list;

};

