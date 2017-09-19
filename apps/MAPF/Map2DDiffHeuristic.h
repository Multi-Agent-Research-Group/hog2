#include "Map2DConstrainedEnvironment.h"
#include "Heuristic.h"
#include "TemporalAStar.h"

// this is a heuristic that uses the differential heuristic
// with 100 canonical states and maximizes with octile distance
// heuristic.

class Map2DDiffHeuristic: public Heuristic<xytLoc> {

  public:
    // constructor
    Map2DDiffHeuristic( Map *_m, Map2DConstrainedEnvironment* _e, unsigned int number_canonical_states ): m(_m),e(_e){
      list.resize(number_canonical_states);
      std::string mapname(m->GetMapName());
      for(int i(0); i<number_canonical_states; ++i){
        std::stringstream fname(mapname);
        fname << "_" << i << "_" << (e->GetConnectedness()%2?e->GetConnectedness()-1:e->GetConnectedness()) << "\n";
        FILE* fp(0);
        if((fp=fopen(fname.str().c_str(), "rb"))==NULL) {
          std::cout << "Cannot open file: " << fname.str() << "... Generating from scratch.\n";
          std::set<xytLoc> points;
          while(points.size()<number_canonical_states){
            xytLoc s;
            do{
              s={rand()%m->GetMapWidth(),rand()%m->GetMapHeight()};
            }while(!m->IsTraversable(s.x,s.y));
            TemporalAStar<xytLoc,tDirection,Map2DConstrainedEnvironment> astar;
          }
        }else{
          fclose(fp);
          list[i].LoadFromFile(fname.str().c_str());
        }
      }
    }

    double HCost( const xytLoc &s1, const xytLoc &s2 ) const {
      double hcost(e->HCost( s1, s2 ));
      uint64_t h1(e->GetStateHash(s1));
      uint64_t h2(e->GetStateHash(s2));
      for(auto const& v:list){
        uint64_t id1;
        v.Lookup(h1,id1);
        uint64_t id2;
        v.Lookup(h2,id2);
        hcost=std::max(hcost,fabs(v.Lookat(id1).g-v.Lookat(id2).g));
      }
      return hcost;
    }

    xytLoc findFurthest(){return {0,0};}

  private:
    Map *m;
    Map2DConstrainedEnvironment *e;
    std::vector<AStarOpenClosed<xytLoc, TemporalAStarCompare<xytLoc>>> list;

};

