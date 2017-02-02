//==============================================================================
// AirplanePerimeterBuilder
//==============================================================================

#ifndef __AirplanePerimeterBuilder__
#define __AirplanePerimeterBuilder__

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <limits>
#include <iostream>
#include <string.h>

template <typename state, typename action, typename environment, unsigned xySize=3, unsigned zSize=3, unsigned numHeadings=8, unsigned numSpeeds=5>
class AirplanePerimeterBuilder
{
private:
  const int sz(16);
  const int wd(sz*2+1);

  uint64_t toIndex(airplaneState const& s, airplaneState const& start){
    return (s.x-start.x+sz)*wd*wd*5*8+
      (s.y-start.y+sz)*wd*5*8+
      (s.height-start.height+sz)*5*8+
      (s.speed-1)*8+
      s.heading;
  }
  float get(float const* const list,int x, int y, int z, int s, int h){
    return list[x*wd*wd*5*8+ y*wd*5*8+ z*5*8+ s*8+ h];
  }
  float get(float const* const list, airplaneState const& s, airplaneState const& start){
    int x(s.x-start.x+sz);
    int y(s.y-start.y+sz);
    int z(s.height-start.height+sz);

    if(x>=0||y>=0||z>=0||x<wd||y<wd||z<wd)
      return  list[toIndex(s,start)];
    return std::numeric_limits<float>::max();
  }
  airplaneState fromIndex(uint64_t index, airplaneState const& start){
    unsigned h(index%8);
    unsigned sp(((index-h)/8)%5);
    unsigned z(((index-sp*8-h)/(8*5))%wd);
    unsigned y(((index-z*8*5-sp*8-h)/(wd*8*5))%wd);
    unsigned x(((index-y*wd*8*5-z*8*5-sp*8-h)/(wd*wd*8*5))%wd);
    return airplaneState(x+start.x-sz,y+start.y-sz,
      z+start.height-sz,sp+1,h,false,AirplaneType::PLANE);
  }

  void set(float* list,int x, int y, int z, int s, int h, float val){
    list[x*wd*wd*5*8+
      y*wd*5*8+
      z*5*8+
      s*8+
      h] = val;
  }
  bool set(float* list, airplaneState const& s, airplaneState const& start, float val){
    int x(s.x-start.x+sz);
    int y(s.y-start.y+sz);
    int z(s.height-start.height+sz);

    if(x<0||y<0||z<0||x>wd-1||y>wd-1||z>wd-1||fgeq(val,get(list,s,start)))
      return false;
    set(list,x,y,z,s.speed-1,s.heading,val);
    return true;
  }


  void learnSpace(int dimensions, SearchEnvironment<state, action> env, std::vector<state> startStates, int depthLimit){
  struct Container{
    int depth;
    state s;
  };
    FunctionApproximator *cl = new LinearRegression(dimensions,1,.01);
    std::vector data(dimensions);
    for(state start: startStates){
      AStarOpenClosed<Container> q;
      q.AddOpenNode(start,env.GetStateHash(start),0,0,0);
      while(!q.OpenSize()){
        uint64_t nodeid(q.Close());
        double G(q.Lookup(nodeid).g);
        Container node(q.Lookup(nodeid).data);
        env.GetDimensions(node.s,start,data);
        std::vector<state> succ;
        env.GetSuccessors(node.s,succ);
        for(state s: succ){
          uint64_t theID;
          switch(q.Lookup(env.GetStateHash(s),theID)):
            case kClosedList:
              break;
            case kOpenList:
              if()
              break:
            case kNotFound:
            default:
            break;
        }
      }
    }
  }

  void testAdmissibility(){
    std::cout << "testAdmissibility";
    { 
      float* list = new float[wd*wd*wd*5*8];
      AirplaneEnvironment e;
      e.loadPerimeterDB();
      //std::queue<std::pair<double,airplaneState> > q;
      std::queue<uint64_t> q;
      airplaneState start(40,40,10,1,0,false,AirplaneType::PLANE);
      airplaneState goal(43,41,14,3,7,false,AirplaneType::PLANE);
      //std::cout << float(e.HCost(goal,start)) << "...\n";
      StraightLineHeuristic<airplaneState> z;
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&e,goal,start,sol);
      //std::cout << "Actual " <<  e.GetPathLength(sol) << "\n";
      //for(auto &a : sol)
        //std::cout << "  " << a<<"\n";
      //exit(0);
      //x:43, y:41, h:14, s:3, hdg: 7
      //(x:40, y:44, h:11, s:1, hdg: 7, l: 0, type: PLANE)
      //x:38, y:44, h:12, s:4, hdg: 2,
      //FAIL (x:41, y:44, h:14, s:1, hdg: 5, l: 0, type: PLANE)(x:40, y:40, h:10, s:1, hdg: 0, l: 0, type: PLANE)0.00346985 0.00333137
      unsigned grandtotal(0);
      for(int hdg(0); hdg<8; ++hdg){
        unsigned total(0);
        std::cout << "HDG " << hdg << std::endl;
        for(int spd(1); spd<=5; ++spd){
          std::cout << "SPD " << spd << std::endl;
          // Initialize all to INF
          memset(list,127,wd*wd*wd*5*8*sizeof(float));
          start.heading=hdg;
          start.speed=spd;
          uint64_t sindex(toIndex(start,start));
          q.push(sindex);
          set(list,start,start,0.0f);
          unsigned count(0);
          while(!q.empty()){
            uint64_t index(q.front());
            airplaneState s(fromIndex(index,start));
            //std::cout << s << index<<"\n";
            float gcost(list[index]);
            //if(index != toIndex(s,start)){std::cout << "index error\n";exit(1);}
            if(!count)gcost=0.0;
            q.pop();
            //std::cout << "qs " << q.size() << "\n";
            float hc(e.HCost(s,start));

            //std::cout << "Compare "<<s << hc << " " << gcost << "\n";
            //else std::cout << gcost<<"\n";
            if(fgreater(hc,gcost)){
              std::cout << "FAIL " << s << start << hc << " " <<gcost<< std::endl;
              return 1;
            }

            std::vector<airplaneAction> actions;
            e.GetReverseActions(s,actions);
            for(typename std::vector<airplaneAction>::const_iterator a(actions.begin());
                a!=actions.end(); ++a){
              airplaneState s2 = s;
              e.UndoAction(s2, *a);
              uint64_t ix(toIndex(s2,start));
              if(ix<wd*wd*wd*5*8&&fless(gcost+e.GCost(s2,s), get(list,s2,start))){
                q.push(ix);
                set(list,s2,start,gcost+e.GCost(s2,s));
                count++;
              }
            }
          }
          std::cout << "Count " << count << "\n";
          total += count;
        }
        std::cout << "Total " << total << "\n";
        grandtotal += total;
      }
      std::cout << "Grand Total " << grandtotal << "\n";
    }
  }

  public:

    AirplanePerimeterBuilder() : expansions(0), openSize(sizeof(list)/sizeof(float)), loaded(false){
      //uint32_t big(std::numeric_limits<uint32_t>::max());
      memset(list,127,openSize*sizeof(float));
    }

    bool loadGCosts(environment &e, state &start, std::string const& fname){
      e.setGoal(start);
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
          while(!q.empty()){// && count < (xySize*2+1+xySize*2+1+zSize*2+1+numSpeeds+numHeadings)){ 
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
            if(s.speed==5){
              int x = 0;
            }
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

      /*loaded = true;
      for(int hdg1(0); hdg1<numHeadings; ++hdg1){
        for(int spd1(1); spd1<=numSpeeds; ++spd1){
          for(int hdg(0); hdg<numHeadings; ++hdg){
            for(int spd(1); spd<=numSpeeds; ++spd){
              for(int x(0); x<=xySize*2; ++x){
                for(int y(0); y<=xySize*2; ++y){
                  for(int z(0); z<=zSize*2; ++z){
                    // fix any lingering values to zero
                    if(list[x][y][z][spd1][hdg1][spd][hdg] > 1000000.0)
                      list[x][y][z][spd1][hdg1][spd][hdg] = 0.0;
                  }
                }
              }
            }
          }
        }
      }*/

      std::cout << "DONE" << std::endl;

      fp = fopen(fname.c_str(),"w");
      fwrite(list,sizeof(float),openSize,fp);
      fclose(fp);
      std::cout << "Wrote to " << fname << "\n";
      return true;
    }
    // TODO - refactor copy/pasted code
    bool loadReverseGCosts(environment &e, state &start, std::string const& fname){
      e.setGoal(start);
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
            e.GetActions(s,actions);
            //std::cout << "Num Actions " << actions.size() << "\n";
            expansions++;
            for(typename std::vector<action>::const_iterator a(actions.begin());
                a!=actions.end(); ++a){
              //std::cout << "   Applying action " << *a << "\n";
              state s2 = s;
              e.ApplyAction(s2, *a);
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
