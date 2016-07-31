//==============================================================================
// AirplanePerimeterDBBuilder
//==============================================================================

#ifndef __AirplanePerimeterDBBuilder__
#define __AirplanePerimeterDBBuilder__

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <limits>
#include <iostream>
#include <string.h>

template <typename state, typename action, typename environment, unsigned xySize=2, unsigned zSize=2, unsigned numHeadings=8, unsigned numSpeeds=5>
class AirplanePerimeterDBBuilder
{
  public:

    AirplanePerimeterDBBuilder() : expansions(0), openSize(sizeof(list)/sizeof(float)), loaded(false){
      //uint32_t big(std::numeric_limits<uint32_t>::max());
      memset(list,127,openSize*sizeof(float));
    }

    bool loadGCosts(environment &e, state &start, std::string const& fname){
      FILE *fp(0);
      // Attempt to read the heuristic from the file.
      // On failure, build it from scratch
      bool error(false);
      if((fp=fopen(fname.c_str(), "rb"))==NULL) {
        std::cout << "Cannot open file: " << fname << "\n";
        error = true;
      }

      if(!error){
        if(fread(list, sizeof(float), openSize, fp) != openSize) {
          if(feof(fp))
            std::cout << "Premature end of file.\n";
          else
            std::cout << "File read error: " << fname << "\n";
          error = true;
        } else {
          loaded = true;
          std::cout << "Successfully loaded heuristic from: " << fname << "\n";
        }
        fclose(fp);
        return loaded;
      }

      std::cout << "Generating AirplanePerimeter Heuristic\n";
      std::queue<std::pair<float,state> > q;
      for(int hdg(0); hdg<numHeadings; ++hdg){
        for(int spd(1); spd<=numSpeeds; ++spd){
          //std::cout << "H " << hdg << " S " << spd << "\n";
          start.heading=hdg;
          start.speed=spd;
          q.push(std::make_pair(0.0,start));
          unsigned count(0);
          while(!q.empty() && count < (xySize*2+1+xySize*2+1+zSize*2+1+numSpeeds+numHeadings)){ 
            auto const& entry(q.front());
            state s(entry.second);
            float gcost(entry.first);
            q.pop();

            // Compute rank
            unsigned x(s.x-start.x+xySize);
            unsigned y(s.y-start.y+xySize);
            unsigned z(s.height-start.height+zSize);
            //std::cout << "X" << x << " Y " << y << " Z " << z << std::endl;
            if(x<0||y<0||z<0||x>xySize*2||y>xySize*2||z>zSize*2 || gcost >= list[x][y][z][s.speed-1][s.heading][start.speed-1][start.heading]){continue;}// {std::cout << "Skipping " << s << "X" << x << " Y " << y << " Z " << z << std::endl; continue;}
            //std::cout << "solution at " << s <<  " to " << start << " is "  << "X" << x << " Y " << y << " Z " << z << " "<< gcost << "\n";
            // Update cost
            if(list[x][y][z][s.speed-1][s.heading][start.speed-1][start.heading] == std::numeric_limits<float>::max()) { ++count; }
            list[x][y][z][s.speed-1][s.heading][start.speed-1][start.heading]=gcost;

            std::vector<action> actions;
            e.GetReverseActions(s,actions);
            //std::cout << "Num Actions " << actions.size() << "\n";
            expansions++;
            for(typename std::vector<action>::const_iterator a(actions.begin());
                a!=actions.end(); ++a){
              //std::cout << "   Applying action " << *a << "\n";
              state s2 = s;
              e.UndoAction(s2, *a);
              //if(s2.x==40&&s.x==41&&s2.y==40&&s.y==41&&s.height==10&&s2.height==10&&s.speed==3&&s2.speed==3&&s.heading==0&&s2.heading==0)
                //std::cout <<"gc"<< s2 << s << gcost <<" "<<e.GCost(s2,s)<<"\n";
              q.push(std::make_pair(gcost+e.GCost(s2,s),s2));
            }
          }
        }
      }

      loaded = true;
      std::cout << "DONE" << std::endl;

      fp = fopen(fname.c_str(),"w");
      fwrite(list,sizeof(float),openSize,fp);
      fclose(fp);
      std::cout << "Wrote to " << fname << "\n";
      return true;
    }

    float GCost(state const& s, state const& g,bool verbose=false) const {
            unsigned x(s.x-g.x+xySize);
            unsigned y(s.y-g.y+xySize);
            unsigned z(s.height-g.height+zSize);
            //if(verbose)std::cout << "Fetching " << s << g << "X " << x << " Y " << y << " Z " << z << " s" << (s.speed-1) << " h" << (unsigned)s.heading << " s" << (g.speed-1) << " h" << (unsigned)g.heading << " : "<< list[x][y][z][s.speed-1][s.heading][g.speed-1][g.heading] << std::endl;
      return list[x][y][z][s.speed-1][s.heading][g.speed-1][g.heading];
    }
  
    // Returns the total nodes expanded by the last GetPath call.
    uint64_t GetNodesExpanded() const { return expansions;}

  private:

    uint64_t expansions;
    float list[xySize*2+1][xySize*2+1][zSize*2+1][numSpeeds][numHeadings][numSpeeds][numHeadings];
    size_t openSize;
    bool loaded;
};

# endif
