#ifndef __hog2_glut__TestEnvironment__
#define __hog2_glut__TestEnvironment__

#include "Timer.h"
#include "Airplane.h" // Includes Airplane
//#include "AirplaneOld.h"
#include "AirplaneSimple.h"
#include "AirplaneHighway.h"
#include "AirplaneHighway4Cardinal.h"
#include "AirplaneCardinal.h"
#include "AirplaneGridCardinal.h"
#include "AirplaneGridOctile.h"
#include "AirplaneGrid3DCardinal.h"
#include "AirplaneGrid3DOctile.h"
#include "AirplaneMultiAgent.h"
#include "TemplateIntervalTree.h"
#include <iostream>
#include <algorithm>
#include <queue>
#include "TemplateAStar.h"
#include "Heuristic.h"
#include <stdlib.h>
#include <set>

void renderScene(){}
static const int sz(5);
static const int wd(sz*2+1);

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

void testReverseSuccessors(AirplaneEnvironment& env, airplaneState const& s, airplaneState const& g, airplaneState o) {
  //std::cout << "original " << o << "\n";
  std::vector<airplaneState> ancestors;
  env.GetReverseSuccessors(o,ancestors);
  for (auto &a : ancestors)
  {
    //std::cout << " Ancestor " << a << "\n";
    // All ancestors should have a successor that is equal to the original state
    std::vector<airplaneState> successors;
    env.GetSuccessors(a,successors);
    bool found(false);
    for(auto &c: successors){
      //std::cout << "        " << c << "\n";
      if(c==o){found=true; break;}
    }
    assert(found && "Did not create a correct ancestor");
  }
}

struct IntervalData{
  IntervalData(uint64_t h, uint8_t a):hash(h),agent(a){}
  uint64_t hash;
  uint8_t agent;
  bool operator==(IntervalData const& other)const{return other.hash==hash && other.agent==agent;}
};
  std::ostream& operator<<(std::ostream& ss, IntervalData v) {ss << "<H:"<<v.hash<<" A:"<<unsigned(v.agent)<<">"; return ss;}

void testIntervalTree(){
  std::cout << "testIntervalTree\n";
  TemplateIntervalTree<IntervalData,float> i;
  IntervalData d1(1234,0);
  IntervalData d2(1234,1);
  std::cout << "contains " << i.size() << "\n";
  i.insert(1.0,1.1,d1); std::cout << "insert\n";
  std::cout << "contains " << i.size() << "\n";
  auto o(i.findOverlapping(0.9,1.01));
  std::cout << "found " << o.size() << "\n";
  std::cout << o[0] << "\n";
  o = i.findOverlapping(0.9,0.99);
  std::cout << "found " << o.size() << "\n";
  i.insert(1.0,1.1,d2); std::cout << "insert\n";
  std::cout << "contains " << i.size() << "\n";
  o = i.findOverlapping(0.9,1.01);
  std::cout << "found " << o.size() << "\n";
  std::cout << o[0] << "\n";
  std::cout << o[1] << "\n";
  i.remove(1.0,1.1,d2); std::cout << "remove\n";
  std::cout << "contains " << i.size() << "\n";
  o = i.findOverlapping(0.9,1.01);
  std::cout << "found " << o.size() << "\n";
  std::cout << o[0] << "\n";
  i.insert(2.0,2.1,d2); std::cout << "insert\n";
  std::cout << "contains " << i.size() << "\n";
  o = i.findOverlapping(0.9,1.01);
  std::cout << "found " << o.size() << "\n";
  std::cout << o[0] << "\n";
  o = i.findOverlapping(1.9,2.01);
  std::cout << "found " << o.size() << "\n";
  std::cout << o[0] << "\n";
  std::vector<TemplateInterval<IntervalData,float> > data;
  for(float v(0.0); v<28; v+=1.0){
    data.push_back(TemplateInterval<IntervalData,float>(v,v+.1,d1));
  }
  i=TemplateIntervalTree<IntervalData,float>(data);
  std::cout << "SIZE " << i.size() << "("<<data.size()<<")" << " DEPTH "<< i.depth() << "\n";
}

void testHash(){
  std::cout << "testHash\n";
  AirplaneEnvironment* env(new AirplaneEnvironment());
  airplaneState start(40,40,10,1,0,false,AirplaneType::PLANE);
  uint64_t hash(env->GetStateHash(start));
  airplaneState after;
  env->GetStateFromHash(hash,after);
  std::cout << start << "\n";
  std::cout << after << "\n";
  AirplaneConstrainedEnvironment* env2(new AirplaneConstrainedEnvironment(env));
  airtimeState s(start,1.234);
  uint64_t h(env2->GetStateHash(s));
  airtimeState a;
  env2->GetStateFromHash(h,a);
  std::cout << s << "\n";
  std::cout << a << "\n";
  env=new AirplaneGridCardinalEnvironment();
  uint64_t hash1(env->GetStateHash(start));
  start.height=start.heading=start.landed=1;
  uint64_t hash2(env->GetStateHash(start));
  assert(hash1==hash2&&"Hash of same x,y,s in grid is same");
  env=new AirplaneGridOctileEnvironment();
  hash1=env->GetStateHash(start);
  start.height=start.heading=start.landed=3;
  hash2=env->GetStateHash(start);
  assert(hash1==hash2&&"Hash of same x,y,s in grid is same");
}

void testIndexConv(){
std::cout << "testIndexConv\n";
airplaneState start(40,40,10,1,0,false,AirplaneType::PLANE);
airplaneState goal(43,41,14,3,7,false,AirplaneType::PLANE);
uint64_t ix(toIndex(goal,start));
std::cout << goal << "\n";
std::cout << ix << "\n";
std::cout << fromIndex(ix,start) << "\n";

}

void testPathUniqueness(){
  std::cout << "testPathUniqueness\n";
  srand(123456);
  std::vector<AirplaneEnvironment*> envs;
  std::vector<AirplaneEnvironment*> renvs;
  std::vector<TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment>*> ra;

  airplaneState g(40,40,10,3,0,false,AirplaneType::PLANE);
  //airplaneState s(40,40,10,3,0,false,AirplaneType::PLANE);
  AirplaneEnvironment* ae(new AirplaneEnvironment());
  ae->setGoal(g);
  ae->setStart(g);
  ae->loadPerimeterDB();

  AirplaneEnvironment* a8e(new AirplaneHighwayEnvironment());
  a8e->setGoal(g);
  a8e->setStart(g);
  a8e->loadPerimeterDB();

  AirplaneEnvironment* ase(new AirplaneSimpleEnvironment());
  ase->setGoal(g);
  ase->setStart(g);
  ase->loadPerimeterDB();

  AirplaneEnvironment* a4e(new AirplaneCardinalEnvironment());
  a4e->setGoal(g);
  a4e->setStart(g);
  a4e->loadPerimeterDB();

  AirplaneEnvironment* a4he(new AirplaneHighway4CardinalEnvironment());
  a4he->setGoal(g);
  a4he->setStart(g);
  a4he->loadPerimeterDB();

  AirplaneEnvironment* age(new AirplaneGridCardinalEnvironment());
  AirplaneEnvironment* aoe(new AirplaneGridOctileEnvironment());
  AirplaneEnvironment* ag3e(new AirplaneGrid3DCardinalEnvironment());
  AirplaneEnvironment* ao3e(new AirplaneGrid3DOctileEnvironment());
  //age->loadPerimeterDB();

  envs.push_back(aoe);
  envs.push_back(age);
  envs.push_back(ag3e);
  envs.push_back(ao3e);
  envs.push_back(ae);
  envs.push_back(a4e);
  envs.push_back(a8e);
  envs.push_back(a4he);
  envs.push_back(ase);
  /*
  */

  for(int k(0); k<envs.size(); ++k){
    AirplaneEnvironment* e(envs[k]);
    TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
    std::cout << "\n";
    std::cout << e->name() << "\n";
    double total(0.0);
    double numOpt(0.0);
    double incl(0.0);
    //for(int i(0); i<1; ++i){
    for(int i(0); i<1000; ++i){
      std::set<uint64_t> hashes;
      airplaneState start(rand() % 20+5, rand() % 20+5, rand() % 10+5, rand() % 5 + 1, rand() % 4*2, false);
      airplaneState goal(rand() % 20+5, rand() % 20+5, rand() % 10+5, rand() % 5 + 1, rand() % 4*2, false);
      //std::cout << i << "\n";
      //std::cout << "\n"<<start << "\n" << goal << "\n";
      //std::cout << "=====================" << std::endl;
      //std::cout << "\n";
      if(e->GoalTest(start,goal))continue;
      std::vector<airplaneState> soln;
      e->setStart(start); // This is needed for reverse actions
      e->setGoal(goal);
      double hc1(e->HCost(start,goal));
      astar.GetPath(e,start,goal,soln);
      double cStar(e->GetPathLength(soln));
      for(auto const& a: soln){
        hashes.insert(e->GetStateHash(a));
        //std::cout << a << "\n";
      }
      incl += 1.0;
      numOpt += soln.size();
      //std::cout << soln.size() << "/";

      //std::cout << "cstar " << cStar << "\n";
      while(true){
        astar.DoSingleSearchStep(soln);
        uint64_t key(astar.openClosedList.Peek());
        double f(astar.openClosedList.Lookup(key).g+astar.openClosedList.Lookup(key).h);
        //std::cout << "-"<<std::endl;
        //std::cout << f << std::endl;
        if(fgreater(f,cStar)){
          break;
        }
      }

      //if(i%100==0)std::cout << i << "\n";
      std::queue<std::pair<airplaneState,double> > q;
      q.push(std::make_pair(goal,0.0));
      while(q.size()){
        //std::cout << "_" << q.size() << std::endl;
        std::pair<airplaneState,double> s(q.front());
        q.pop();
        std::vector<airplaneState> ancestors;
        e->GetReverseSuccessors(s.first,ancestors);
        //std::cout << s.first << "-----------\n";
        for(auto const& a: ancestors){
          //std::cout << " ."<< std::endl;
          uint64_t key(e->GetStateHash(a));
          double g1(e->GCost(a,s.first)+s.second);
          double g2;
          //std::cout <<"a"<< a << "-->g" << s.first <<"="<<g1<<"\n";
          if(astar.GetClosedListGCost(a,g2)){
            //std::cout << "   g"<<s.first << "-->a" << a <<"="<<g2<<"\n";
            if(fequal(g1+g2,cStar)&&hashes.find(key)==hashes.end()){
              //std::cout << " push "<< a<< std::endl;
              hashes.insert(key);
              q.push(std::make_pair(a,g1));
            }
          }
          //else
            //std::cout << "a not in \n";
        }
      }
      //for(auto const& a: hashes){
        //std::cout << e->GetState(a) << "\n";
      //}
      total+=hashes.size();
      //std::cout << hashes.size() << "\n";
    }
    std::cout << total/incl << "/" << numOpt/incl << "=" << (total/numOpt) << "\n";
  }
}

bool testAdmissibility(){
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
return 0;
  {
    AirplanePerimeterDBBuilder<airplaneState,airplaneAction,AirplaneEnvironment> builder;
    airplaneState goal(10,10,10,3,0,false,AirplaneType::PLANE);
    AirplaneEnvironment env;
    //env.loadPerimeterDB();
    airplaneState target(10,10,10,1,0,false,AirplaneType::PLANE);
    builder.loadGCosts(env, target, "airplanePerimeter.dat");
    double best=0;
    // Find largest cost. What are it's characteristics?
    for(int hh(0); hh<8; ++hh){
      goal.heading = hh;
      std::cout <<"H"<<hh<<" ";
      for(int x(-2); x<3; ++x){
        for(int y(-2); y<3; ++y){
          for(int z(-2); z<3; ++z){
            if(abs(x) != 2 && abs(y) !=2 && abs(z) !=2) {std::cout << "nmid\n"; continue;}
            double besth=10000;
            for(int h(0); h<8; ++h){
              airplaneState s(goal.x+x,goal.y+y,goal.height+z,3,h);
              uint8_t travel(s.headingTo(goal));
              double gc(builder.GCost(s,goal));
              best=(gc<1000&&best<gc)?gc:best;
              if(fleq(gc,besth)){std::cout << "new besth "; besth=gc; if(((travel-h+8)%8)>1){std::cout<<"badh? ";}}
              std::cout <<x<<","<<y<<","<<z<<","<<h<<"("<<(unsigned)s.headingTo(goal)<<")="<<gc<<"\n";
            }
            bool chrash(true);
            for(int h(-1); h<2; ++h){
              airplaneState s(goal.x+x,goal.y+y,goal.height+z,3,0);
              s.heading = (s.headingTo(goal)+h+8)%8;
              double gc(builder.GCost(s,goal));
              if(fequal(gc,besth)){chrash=false;}
            }
            if(chrash)std::cout << "CRASH!\n";
            std::cout << "\n";
          }
        }
      }
      std::cout << "best " << best << "\n";
    }
  }
}

/*bool compareHeuristicSpeed(){
  srand(123456);
  std::cout << "Heuristic relative speed\n";
  AirplaneEnvironment aenv;
  //AirplaneOldEnvironment oenv;
  aenv.loadPerimeterDB();
  //oenv.loadPerimeterDB();

  Timer timer;
  double h1(0.0);
  double h2(0.0);
  for(int i(0); i<10000000; ++i){
    airplaneState s(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
    airplaneState g(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
    if(s==g)continue;

    timer.StartTimer();
    aenv.HCost(s,g);
    h1+=timer.EndTimer();
    timer.StartTimer();
    aenv.OldHCost(s,g);
    h2+=timer.EndTimer();
  }
  std::cout << "Old Speed: " << h2 << std::endl;
  std::cout << "New Speed: " << h1 << std::endl;
  return 0;
}*/

bool testMultiAgent(){
  std::cout << "reverse\n";
  {
    airplaneState start(50,45,16,3,4,false,AirplaneType::PLANE);
    airplaneState goal(55,55,16,3,4,false,AirplaneType::PLANE);
    AirplaneEnvironment a;
    std::cout << "H " << a.HCost(start,goal);
    a.loadPerimeterDB();
    a.setGoal(goal);
    TemplateAStar<airplaneState,airplaneAction,AirplaneEnvironment> astar;
    std::vector<airplaneState> sol;
    astar.GetPath(&a,start,goal,sol);
    std::cout << "expansions: " << astar.GetNodesExpanded() << "\n";
    astar.SetSuccessorFunc(&AirplaneEnvironment::GetReverseSuccessors);
    a.setSearchType(SearchType::REVERSE);
    std::vector<airplaneState> solr;
    astar.GetPath(&a,goal,start,solr);
    std::cout << "expansions: " << astar.GetNodesExpanded() << "\n";
    solr.begin()->landed=true;
    std::reverse(solr.begin(),solr.end());
    std::cout << "pathlen " << sol.size() << " " << solr.size() << "\n";
    std::cout << "paths\n";
    for(int i(0); i<sol.size(); ++i)
      std::cout << sol[i] << solr[i] << "==?"<< (sol[i]==solr[i]) << "\n";
    return true;
  }
  std::cout << "multi\n";
  {
    std::cout << "Highway Environment\n";
    AirplaneHighwayEnvironment env;
    env.loadPerimeterDB();
    AirplaneConstrainedEnvironment cenv(&env);
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> mae(&cenv);
    TemplateAStar<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState, AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentAction, AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> > astar;
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState start;
    start.push_back(airtimeState(airplaneState(50,45,16,3,4,false,AirplaneType::PLANE),0));
    start.push_back(airtimeState(airplaneState(55,50,16,3,6,false,AirplaneType::PLANE),0));
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState goal;
    goal.push_back(airtimeState(airplaneState(50,55,16,3,4,false,AirplaneType::PLANE),0));
    goal.push_back(airtimeState(airplaneState(45,50,16,3,6,false,AirplaneType::PLANE),0));
    mae.setGoal(goal);

    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentAction> act;
    mae.GetActions(start,act);
    //std::cout << act.size() << " actions\n";
    //for(auto const& ma : act){
    //std::cout << "\n";
    //for(auto const& a : ma){
    //std::cout << a << "\n";
    //}
    //}

    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState> st;
    mae.GetSuccessors(start,st);
    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState> sol;
    astar.GetPath(&mae,start,goal,sol);

    std::cout <<  "solution\n";
    std::cout << astar.GetNodesExpanded() << " expansions\n";
    int j(0);
    for(auto const& ma : sol){
      std::cout << "\n";
      int i(0);
      for(auto const& a : ma){
        std::cout << a ;
        if(j>0) std::cout << mae.getEnv()->GCost(sol[j-1][i],a);
        ++i;
        std::cout <<"\n";
      }
      j++;
    }
    std::cout << "H " << mae.HCost(start,goal);
    std::cout << "C " << mae.GetPathLength(sol);
  }
  {
    std::cout << "Simple Environment\n";
    AirplaneSimpleEnvironment env;
    env.loadPerimeterDB();
    AirplaneConstrainedEnvironment cenv(&env);
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> mae(&cenv);
    TemplateAStar<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState, AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentAction, AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> > astar;
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState start;
    start.push_back(airtimeState(airplaneState(50,45,16,3,4,false,AirplaneType::PLANE),0));
    start.push_back(airtimeState(airplaneState(55,50,16,3,6,false,AirplaneType::PLANE),0));
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState goal;
    goal.push_back(airtimeState(airplaneState(50,55,16,3,4,false,AirplaneType::PLANE),0));
    goal.push_back(airtimeState(airplaneState(45,50,16,3,6,false,AirplaneType::PLANE),0));
    mae.setGoal(goal);

    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentAction> act;
    mae.GetActions(start,act);
    //std::cout << act.size() << " actions\n";
    //for(auto const& ma : act){
    //std::cout << "\n";
    //for(auto const& a : ma){
    //std::cout << a << "\n";
    //}
    //}

    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState> st;
    mae.GetSuccessors(start,st);
    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState> sol;
    astar.GetPath(&mae,start,goal,sol);

    std::cout <<  "solution\n";
    std::cout << astar.GetNodesExpanded() << "expansions \n";
    int j(0);
    for(auto const& ma : sol){
      std::cout << "\n";
      int i(0);
      for(auto const& a : ma){
        std::cout << a ;
        if(j>0) std::cout << mae.getEnv()->GCost(sol[j-1][i],a);
        ++i;
        std::cout <<"\n";
      }
      j++;
    }
    std::cout << "H " << mae.HCost(start,goal);
    std::cout << "C " << mae.GetPathLength(sol);
  }
  {
    std::cout << "Complex Environment\n";
    AirplaneEnvironment env;
    env.loadPerimeterDB();
    AirplaneConstrainedEnvironment cenv(&env);
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>  mae(&cenv);
    TemplateAStar<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState, AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentAction, AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> > astar;
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState start;
    start.push_back(airtimeState(airplaneState(50,45,16,3,4,false,AirplaneType::PLANE),0));
    start.push_back(airtimeState(airplaneState(55,50,16,3,6,false,AirplaneType::PLANE),0));
    AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState goal;
    goal.push_back(airtimeState(airplaneState(50,55,16,3,4,false,AirplaneType::PLANE),0));
    goal.push_back(airtimeState(airplaneState(45,50,16,3,6,false,AirplaneType::PLANE),0));
    mae.setGoal(goal);

    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentAction> act;
    mae.GetActions(start,act);
    //std::cout << act.size() << " actions\n";
    //for(auto const& ma : act){
    //std::cout << "\n";
    //for(auto const& a : ma){
    //std::cout << a << "\n";
    //}
    //}

    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState> st;
    mae.GetSuccessors(start,st);
    std::vector<AirplaneMultiAgentEnvironment<airtimeState,airplaneAction,AirplaneConstrainedEnvironment>::MultiAgentState> sol;
    astar.GetPath(&mae,start,goal,sol);

    std::cout <<  "solution\n";
    std::cout << astar.GetNodesExpanded() << "expansions \n";
    int j(0);
    for(auto const& ma : sol){
      std::cout << "\n";
      int i(0);
      for(auto const& a : ma){
        std::cout << a ;
        if(j>0) std::cout << mae.getEnv()->GCost(sol[j-1][i],a);
        ++i;
        std::cout <<"\n";
      }
      j++;
    }
    std::cout << "H " << mae.HCost(start,goal);
    std::cout << "C " << mae.GetPathLength(sol);
  }
  return true;
}
bool testLoadPerimeterHeuristic(){
  std::cout << "testLoadPerimeterHeuristic";
  {
    AirplanePerimeterDBBuilder<airplaneState,airplaneAction,AirplaneEnvironment> builder;
    AirplaneEnvironment env;
    env.loadPerimeterDB();
    airplaneState target(10,10,10,1,0,false,AirplaneType::PLANE);
    builder.loadGCosts(env, target, "airplanePerimeter.dat");

    airplaneState s;
    s.x = 40;
    s.y = 40;
    s.height = 14;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);
    ZeroHeuristic<airplaneState> z;

    //PLANE
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(s,g))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(s,g) << "\n";}
      assert(fequal(gcost,builder.GCost(s,g))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      //std::cout << "expanded " << astar.GetNodesExpanded() << "\n";
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    //QUAD
    target = airplaneState(10,10,10,1,0,false,AirplaneType::QUAD);
    AirplanePerimeterDBBuilder<airplaneState,airplaneAction,AirplaneEnvironment> qbuilder;
    qbuilder.loadGCosts(env, target, "quadairplanePerimeter.dat");
    s.type=AirplaneType::QUAD;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,qbuilder.GCost(s,g))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << qbuilder.GCost(s,g) << "\n";}
      assert(fequal(gcost,qbuilder.GCost(s,g))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,qbuilder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << qbuilder.GCost(g,s) << "\n";}
      assert(fequal(gcost,qbuilder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      //std::cout << "expanded " << astar.GetNodesExpanded() << "\n";
      if(!fequal(gcost,qbuilder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << qbuilder.GCost(g,s) << "\n";}
      assert(fequal(gcost,qbuilder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

  }
  {
  // Simple Env.
    AirplanePerimeterDBBuilder<airplaneState,airplaneAction,AirplaneEnvironment> builder;
    AirplaneSimpleEnvironment env;
    env.loadPerimeterDB();
    airplaneState target(10,10,10,1,0);
    builder.loadGCosts(env, target, "airplaneSimplePerimeter.dat");

    airplaneState s;
    s.x = 40;
    s.y = 40;
    s.height = 14;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);
    ZeroHeuristic<airplaneState> z;

    // PLANE
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(s,g))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(s,g) << "\n";}
      assert(fequal(gcost,builder.GCost(s,g))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      //std::cout << "expanded " << astar.GetNodesExpanded() << "\n";
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    // QUAD
    target.type = AirplaneType::QUAD;
    s.type = AirplaneType::QUAD;
    builder.loadGCosts(env, target, "airplaneSimplePerimeter.dat");
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(s,g))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(s,g) << "\n";}
      assert(fequal(gcost,builder.GCost(s,g))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol); // Invert start and goal
      double gcost(env.GetPathLength(sol));
      //std::cout << "expanded " << astar.GetNodesExpanded() << "\n";
      if(!fequal(gcost,builder.GCost(g,s))){
        std:: cout << s << " " << g << "\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " P " << builder.GCost(g,s) << "\n";}
      assert(fequal(gcost,builder.GCost(g,s))&& "Costs of the perimeter and real are not equal");
      std::cout << "." << std::flush;
    }

  }

  std::cout << "PASSED\n";
}

bool testHCost(){
  std::cout << "testHCost()";
  {
    std::cout << " Simple Environment Plane: ";
    AirplaneSimpleEnvironment env;
    airplaneState s;
    s.x = 50;
    s.y = 50;
    s.height = 16;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      //if(gcost!=env.GCost(s,g))
        //std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " H " << env.HCost(s,g) << "\n";
        assert(fequal(gcost,env.HCost(s,g))||fgreater(gcost,env.HCost(s,g)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> z;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      astar.SetWeight(.5);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }

    {
      airplaneState s(6,9,8,3,2,false);
      airplaneState g(9,12,6,3,2,false);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));

    }
    for(int i(0); i<1000; ++i){
      airplaneState s(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      airplaneState g(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      if(s==g)continue;
      {
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g,s)))
        {
          std::cout << "\n";
          std::cout << "  " << *(sol.begin()) <<"\n";
          for(auto a(sol.begin()+1); a!=sol.end(); ++a)
            std::cout << "  " << *a << " " <<env.GCost(*(a-1),*a)<<"\n";
          std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
        }
        assert(fgeq(gcost,env.HCost(g,s)));
      }
      std::cout << "." << std::flush;
    }

  }
  {
    std::cout << " Simple Environment Quad: ";
    AirplaneSimpleEnvironment env;
    airplaneState s;
    s.x = 50;
    s.y = 50;
    s.height = 16;
    s.heading = 0;
    s.speed = 3;
    s.type = AirplaneType::QUAD;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      //if(gcost!=env.GCost(s,g))
        //std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " H " << env.HCost(s,g) << "\n";
        assert(fequal(gcost,env.HCost(s,g))||fgreater(gcost,env.HCost(s,g)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> z;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      astar.SetWeight(.5);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }

    {
      airplaneState s(6,9,8,3,2,false);
      airplaneState g(9,12,6,3,2,false);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fequal(gcost,env.HCost(g,s))||fgreater(gcost,env.HCost(g,s)));

    }
    for(int i(0); i<1000; ++i){
      airplaneState s(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      airplaneState g(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      if(s==g)continue;
      {
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g,s)))
        {
          std::cout << "\n";
          std::cout << "  " << *(sol.begin()) <<"\n";
          for(auto a(sol.begin()+1); a!=sol.end(); ++a)
            std::cout << "  " << *a << " " <<env.GCost(*(a-1),*a)<<"\n";
          std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
        }
        assert(fgeq(gcost,env.HCost(g,s)));
      }
      std::cout << "." << std::flush;
    }

  }
  {
    std::cout << " Normal Environment Plane: ";
    AirplaneEnvironment env;
    airplaneState s;
    s.x = 50;
    s.y = 50;
    s.height = 16;
    s.heading = 0;
    s.speed = 3;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,env.HCost(s,g))){
        std::cout << "s"<<s<<"-->g"<<g<<"\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " H " << env.HCost(s,g) << "\n";}
        assert(fequal(gcost,env.HCost(s,g)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> z;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fgeq(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      for(auto &a2: as){
        airplaneState g2(g);
        env.ApplyAction(g2,a2);
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        astar.SetHeuristic(&z);
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g2,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g2,s)))
        {
          std::cout << "\n";
          for(auto &a : sol)
            std::cout << "  " << a<<"\n";
          std::cout << g2 << s << " G " << gcost << " H " << env.HCost(g2,s) << "\n";
        }
        assert(fgeq(gcost,env.HCost(g2,s)));
        std::cout << "." << std::flush;
      }
    }

  }
  {
    std::cout << " Normal Environment Quad: ";
    AirplaneEnvironment env;
    airplaneState s;
    s.x = 50;
    s.y = 50;
    s.height = 16;
    s.heading = 0;
    s.speed = 3;
    s.type = AirplaneType::QUAD;

    std::vector<airplaneAction> as;
    env.GetActions(s,as);

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,s,g,sol);
      double gcost(env.GetPathLength(sol));
      if(!fequal(gcost,env.HCost(s,g))){
        std::cout << "s"<<s<<"-->g"<<g<<"\n";
        std::cout << env.GCost(s,g) << "!=?" << " G " << gcost << " H " << env.HCost(s,g) << "\n";}
        assert(fequal(gcost,env.HCost(s,g)));
      std::cout << "." << std::flush;
    }

    StraightLineHeuristic<airplaneState> z;
    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      astar.SetHeuristic(&z);
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      if(fless(gcost,env.HCost(g,s)))
      {
        std::cout << "\n";
        for(auto &a : sol)
          std::cout << "  " << a<<"\n";
        std::cout << g << s << " G " << gcost << " H " << env.HCost(g,s) << "\n";
      }
      assert(fgeq(gcost,env.HCost(g,s)));
      std::cout << "." << std::flush;
    }

    for(auto &a: as){
      airplaneState g(s);
      env.ApplyAction(g,a);
      for(auto &a2: as){
        airplaneState g2(g);
        env.ApplyAction(g2,a2);
        TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
        astar.SetHeuristic(&z);
        std::vector<airplaneState> sol;
        astar.GetPath(&env,g2,s,sol);
        double gcost(env.GetPathLength(sol));
        if(fless(gcost,env.HCost(g2,s)))
        {
          std::cout << "\n";
          for(auto &a : sol)
            std::cout << "  " << a<<"\n";
          std::cout << g2 << s << " G " << gcost << " H " << env.HCost(g2,s) << "\n";
        }
        assert(fgeq(gcost,env.HCost(g2,s)));
        std::cout << "." << std::flush;
      }
    }

  }
  std::cout << "PASSED\n";
}

bool testGrid3DOctileActions(){
  std::cout << "grid 3D cardinal:";
  AirplaneGrid3DOctileEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.speed = 3;
  s.heading = 0;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  assert(26==actions.size() && "Wrong number of actions generated");
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  assert(26==ractions.size() && "Wrong number of reverse actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    std::cout << s1 << "-->" << s << "\n" << a << " " << a2 << "\n";
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

bool testGrid3DCardinalActions(){
  std::cout << "grid 3D cardinal:";
  AirplaneGrid3DCardinalEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.speed = 3;
  s.heading = 0;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  assert(6==actions.size() && "Wrong number of actions generated");
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  assert(6==ractions.size() && "Wrong number of reverse actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    std::cout << s1 << "-->" << s << "\n" << a << " " << a2 << "\n";
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

bool testGridOctileActions(){
  std::cout << "grid cardinal:";
  AirplaneGridOctileEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.speed = 3;
  s.heading = 0;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  assert(8==actions.size() && "Wrong number of actions generated");
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  assert(8==ractions.size() && "Wrong number of reverse actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    std::cout << s1 << "-->" << s << "\n" << a << " " << a2 << "\n";
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

bool testGridCardinalActions(){
  std::cout << "grid cardinal:";
  AirplaneGridCardinalEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.speed = 3;
  s.heading = 0;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  assert(4==actions.size() && "Wrong number of actions generated");
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  assert(4==ractions.size() && "Wrong number of reverse actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    std::cout << s1 << "-->" << s << "\n" << a << " " << a2 << "\n";
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

bool testCardinalActions(){
  std::cout << "cardinal:";
  AirplaneCardinalEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.speed = 3;
  s.heading = 0;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  assert(27==actions.size() && "Wrong number of actions generated");
  std::vector<airplaneAction> ractions;
  s.heading=1;
  env.setStart(s);
  env.GetReverseActions(s,ractions);
  assert(18==ractions.size() && "Wrong number of start reverse actions generated");
  s.heading=0;
  env.setStart(g);
  env.GetReverseActions(s,ractions);
  assert(27==ractions.size() && "Wrong number of reverse actions generated");
  env.setGoal(g);
  env.GetReverseActions(g,ractions);
  assert(63==ractions.size() && "Wrong number of goal reverse actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

bool testCardinalEnvHCost(){
    srand(123456);
    std::cout << " Cardinal Environment HCost: ";
    AirplaneEnvironment aenv;
    aenv.loadPerimeterDB();
    AirplaneCardinalEnvironment env;
    env.loadPerimeterDB();

    TemplateAStar<airplaneState, airplaneAction, AirplaneCardinalEnvironment> astar;
    airplaneState s(5,8,5,3,4);
    airplaneState s1(5,9,6,3,3);
    airplaneState s2(6,10,7,3,3);
    airplaneState g(7,11,8,4,4);
    //TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
    std::vector<airplaneState> sol;
    astar.GetPath(&env,s,g,sol);
    std::cout << s <<"\n" << s1 << env.GCost(s,s1) << "\n" << s2 << env.GCost(s1,s2) << "\n" << g << env.GCost(s2,g) << "\n";
    double gcost(env.GetPathLength(sol));
    std::cout << "HCOST\n";
    double hcost(env.HCost(s,g));
    double ahcost(aenv.HCost(s,g));
    std::cout << "\n";
    std::cout << "  " << *(sol.begin()) <<"\n";
    for(auto a(sol.begin()+1); a!=sol.end(); ++a)
      std::cout << "  " << *a << " " <<env.GCost(*(a-1),*a)<<"\n";
    std::cout << s << g << " G " << gcost << " H " << hcost << "\n";
    return false;

    for(int i(0); i<1000; ++i){
      airplaneState s(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      airplaneState g(rand() % 8+5, rand() % 8+5, rand() % 5+5, rand() % 5 + 1, rand() % 4*2, false);
      if(s==g)continue;
      TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
      std::vector<airplaneState> sol;
      astar.GetPath(&env,g,s,sol);
      double gcost(env.GetPathLength(sol));
      double hcost(env.HCost(g,s));
      if(fless(gcost,hcost))
      {
        double hc(env.HCost(g,s));
        std::cout << "\n";
        std::cout << "  " << *(sol.begin()) <<"\n";
        for(auto a(sol.begin()+1); a!=sol.end(); ++a)
          std::cout << "  " << *a << " " <<env.GCost(*(a-1),*a)<<"\n";
        std::cout << g << s << " G " << gcost << " H " << hcost << "\n";
      }
      assert(fgeq(gcost,hcost));
      std::cout << "." << std::flush;
    }
  return true;
}


bool testCardinalHighwayActions(){
  std::cout << " highway 4 cardinal:";
  AirplaneHighway4CardinalEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.heading = 0;
  s.speed = 3;
  airplaneState g(50,40,18,0,3);
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,16,3,0);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  // 1. do nothing
  // 2. slow down
  // 3. speed up
  // 4. up and turn right
  // 5. down and turn left
  // 6. up and turn right + slow down
  // 7. up and turn right + speed up
  // 8. down and turn left + slow down
  // 9. down and turn left + speed up
  assert(9==ractions.size() && "Wrong number of reverse actions generated");
  assert(9==actions.size() && "Wrong number of actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    airplaneState s2=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  testReverseSuccessors(env,s,g,o);
  /*o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);*/
  std::cout << "PASSED\n";
}

bool testSimpleActions(){
  std::cout << "testActions()";
  std::cout << " simple:";
  AirplaneSimpleEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.heading = 0;
  s.speed = 1;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

void testNormalActions(){
  std::cout << " normal:";
  AirplaneEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.heading = 0;
  s.speed = 3;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  std::vector<airplaneAction> actions;
  env.GetActions(s,actions);
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(s,ractions);
  // 1-7 turns (also includes no turn)
  // 8-14 increase speed x turns
  // 15-21 decrease speed x turns
  // 22-28 increase height x turns
  // 29-35 decrease height x turns
  // 36-42 increase speed, increase height x turns
  // 43-49 decrease speed, decrease height x turns
  // 50-56 increase speed, decrease height x turns
  // 57-63 decrease speed, increase height x turns
  assert(actions.size()==63 && "Number of actions differs from expected");
  assert(ractions.size()==63 && "Number of reverse actions differs from expected");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  env.setStart(s);
  env.setGoal(g);
  airplaneState o(56,44,11,3,0);
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}
void testHighway8Actions(){
  std::cout << " highway:";
  AirplaneHighwayEnvironment env;
  airplaneAction a(0,0,0);
  airplaneState s;
  s.x = 50;
  s.y = 50;
  s.height = 16;
  s.heading = 0;
  s.speed = 3;
  airplaneState g(50,40,18,3,0);
  env.setGoal(g);
  env.setStart(s);
  airplaneState o(56,44,16,3,0);
  std::vector<airplaneAction> actions;
  env.GetActions(o,actions);
  std::vector<airplaneAction> ractions;
  env.GetReverseActions(o,ractions);
  // 1. do nothing
  // 2. slow down
  // 3. speed up
  // 4. up and turn right
  // 5. down and turn left
  // 6. up and turn right + slow down
  // 7. up and turn right + speed up
  // 8. down and turn left + slow down
  // 9. down and turn left + speed up
  assert(9==ractions.size() && "Wrong number of reverse actions generated");
  assert(9==actions.size() && "Wrong number of actions generated");
  for (auto &a : actions)
  {
    airplaneState s1=s;
    env.ApplyAction(s,a);
    airplaneAction a2(env.GetAction(s1,s));
    env.UndoAction(s,a);
    //std::cout << a << s << s1 << "\n";
    assert(s==s1 && "Action not reversed properly");
    assert(a==a2 && "Action not inferred properly");
    std::cout << ".";
    s=s1; // Reset
  }
  testReverseSuccessors(env,s,g,o);
  o=s;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  o=g;
  o.x+=1;
  testReverseSuccessors(env,s,g,o);
  std::cout << "PASSED\n";
}

bool testCardinalHeadingTo(){
  std::cout << "testCardinalHeadingTo()";
  airplaneState s1(10,10,0,0,0);
  airplaneState s2(10,9,0,0,0);
  assert(s1.cardinalHeadingTo(s2)==0); // South
  std::cout << ".";

  s2.x=11;
  assert(s1.cardinalHeadingTo(s2)==2); // SE
  std::cout << ".";

  s2.y=10;
  assert(s1.cardinalHeadingTo(s2)==2); // East
  std::cout << ".";

  s2.y=11;
  assert(s1.cardinalHeadingTo(s2)==4); // NE
  std::cout << ".";

  s2.x=10;
  assert(s1.cardinalHeadingTo(s2)==4); // North
  std::cout << ".";

  s2.x=9;
  assert(s1.cardinalHeadingTo(s2)==6); // NW
  std::cout << ".";

  s2.y=10;
  assert(s1.cardinalHeadingTo(s2)==6); // West
  std::cout << ".";

  s2.y=9;
  assert(s1.cardinalHeadingTo(s2)==6); // SW
  std::cout << ".";

  std::cout << "PASSED\n";
}

bool testHeadingTo(){
  std::cout << "testHeadingTo()";
  airplaneState s1(10,10,0,0,0);
  airplaneState s2(10,9,0,0,0);
  assert(s1.headingTo(s2)==0); // Straight down
  std::cout << ".";

  s2.x=11;
  assert(s1.headingTo(s2)==1); // Straight down
  std::cout << ".";

  s2.y=10;
  assert(s1.headingTo(s2)==2); // Straight down
  std::cout << ".";

  s2.y=11;
  assert(s1.headingTo(s2)==3); // Straight down
  std::cout << ".";

  s2.x=10;
  assert(s1.headingTo(s2)==4); // Straight down
  std::cout << ".";

  s2.x=9;
  assert(s1.headingTo(s2)==5); // Straight down
  std::cout << ".";

  s2.y=10;
  assert(s1.headingTo(s2)==6); // Straight down
  std::cout << ".";

  s2.y=9;
  assert(s1.headingTo(s2)==7); // Straight down
  std::cout << ".";

  std::cout << "PASSED\n";
}

bool testConstraints() {
	std::cout << "testConstraints()";
	AirplaneConstrainedEnvironment ace(new AirplaneEnvironment());

	airplaneState O(0,0,0,0,0);
	airtimeState OT(O, 1.0f);
	Constraint<airtimeState> CO(OT);


	// Check self-conflict - you should always conflict with yourself
	{
		airplaneState s1(10,10,10,0,0);
		airtimeState st1(s1, 1.0f);
		Constraint<airtimeState> c(st1);

		// Should always conflict with yourself
		assert(c.ConflictsWith(c));
		assert(c.ConflictsWith(st1));
		assert(c.ConflictsWith(st1, st1));

		// Should not conflict with this
		assert(!c.ConflictsWith(CO));

		std::cout << ".";
	}

	// Don't conflict if times do not overlap
	{
		airplaneState s1(10,10,10,0,0);
		airtimeState st1(s1, 1.0f);
		airtimeState st2(s1, 2.0f);
		Constraint<airtimeState> c(st1);

		assert(!c.ConflictsWith(st2));

		// Should not conflict with this
		assert(!c.ConflictsWith(CO));

		std::cout << ".";
	}

	// another case..
	{
		airplaneState s1(32,40,14,1,6);
		airplaneState s2(31,41,14,2,6);
		airtimeState st1(s1, 0.0f);
		airtimeState st2(s1, 0.204465f);
		Constraint<airtimeState> c(st1);

		assert(!c.ConflictsWith(st2));

		std::cout << ".";
	}

	// Check simple straight conflict
	{
		// Check over all speeds
		for (int j = 1; j < ace.GetSpeeds(); j++)
		{
			// Check over all headings
			for (int i = 0; i < 8; i++)
			{
				airplaneState s1(10,10,10,j,i);
				airtimeState st1(s1, 1.0f);
				airplaneAction a1(0, 0, 0);
				
				airtimeState st2(s1, 1.0f);
				ace.ApplyAction(st2, a1);
				
				Constraint<airtimeState> c1(st1, st2);

				
				// Should conflict at both endpoints
				assert(c1.ConflictsWith(st2));
				assert(c1.ConflictsWith(st1));

				// Should conflict with the motion
				assert(c1.ConflictsWith(st1, st2));

				// Should also conflict with something in the middle
				airtimeState st3(s1, 0.9f);
				ace.ApplyAction(st3, a1);
				assert(c1.ConflictsWith(st3));

				assert(!c1.ConflictsWith(CO));

				std::cout << ".";
			}
		}
	}

	// Check simple turn conflict
	{
		// Check all turns
		for (int k = -3; k <= 3; k++)
		{
			// Check over all speeds
			for (int j = 1; j < ace.GetSpeeds(); j++)
			{
				// Check over all headings
				for (int i = 0; i < 8; i++)
				{
					airplaneState s1(10,10,10,j,i);
					airtimeState st1(s1, 1.0f);

					airplaneAction a1(k, 0, 0);
					
					airtimeState st2(s1, 1.0f);
					ace.ApplyAction(st2, a1);
					
					Constraint<airtimeState> c1(st1, st2);

					
					// Should conflict at both endpoints
					assert(c1.ConflictsWith(st2));
					assert(c1.ConflictsWith(st1));

					// Should conflict with the motion
					assert(c1.ConflictsWith(st1, st2));

					// Should also conflict with something in the middle
					airtimeState st3(s1, 0.9f);
					ace.ApplyAction(st3, a1);
					assert(c1.ConflictsWith(st3));

					assert(!c1.ConflictsWith(CO));

					std::cout << ".";
				}
			}
		}
	}

	// Check height change conflict
	{
		// Check all turns
		for (int k = -3; k <= 3; k++)
		{
			// Check over all speeds
			for (int j = 1; j < ace.GetSpeeds(); j++)
			{
				// Check over all headings
				for (int i = 0; i < 8; i++)
				{
					airplaneState s1(10,10,10,j,i);
					airtimeState st1(s1, 1.0f);

					airplaneAction a1(k, 0, 1);
					
					airtimeState st2(s1, 1.0f);
					ace.ApplyAction(st2, a1);
					
					Constraint<airtimeState> c1(st1, st2);

					
					// Should conflict at both endpoints
					assert(c1.ConflictsWith(st2));
					assert(c1.ConflictsWith(st1));

					// Should conflict with the motion
					assert(c1.ConflictsWith(st1, st2));

					// Should also conflict with something in the middle
					airtimeState st3(s1, 0.9f);
					ace.ApplyAction(st3, a1);
					assert(c1.ConflictsWith(st3));

					assert(!c1.ConflictsWith(CO));

					std::cout << ".";
				}
			}
		}
	}

	std::cout << "PASSED\n";
}

bool TestQuadcopterActions() {
  std::cout << "TestQuadcopterActions()";
  AirplaneEnvironment ae;

  airplaneState x(40,40,8,1,0,false,AirplaneType::QUAD);
  std::vector<airplaneAction> actions;
  ae.GetActions(x, actions);

  bool ns = false;

  for (auto a : actions) {
    airplaneState y(40,40,8,1,0,false,AirplaneType::QUAD);
    ae.ApplyAction(y, a);
    std::vector<airplaneAction> backacts;
    ae.GetActions(y, backacts);
    //std::cout << "Moved to: " << y << "\n";
    airplaneState z = y;
    ae.UndoAction(z,a);
    assert(x==z&&"Broken actions in Quadricopter....");
    ae.GetReverseActions(y, backacts);
    bool valid(false);
    for (auto b : backacts) {
      airplaneState g = y;
      ae.UndoAction(g, b);
      //std::cout << "   Moved reverse to: " << g << " via: " << b << "\n";
      if (x.x == g.x && x.y == g.y && x.height == g.height){
        valid = true;
        break;
      }
    }
    if (!valid) {
      assert(!"Broken back actions in Quadricopter....");
    }
    std::cout <<".";

    if (y.height == 9 && y.x == 40 && y.y == 40)
      ns = true;

  }

  if (!ns) {
    assert(!"Error going up.");
  }

  std::cout << "PASSED\n";


}

void testPEAStar()
{
if(false){
  airplaneState s(10, 10, 16, 3, 0, false);
  airplaneState g(40, 20, 11, 3, 0, false);
  AirplaneGridCardinalEnvironment env;
  env.setGoal(g);
  env.loadPerimeterDB();
  TemplateAStar<airplaneState, airplaneAction, AirplaneGridCardinalEnvironment> astar;
  std::vector<airplaneState> sol;
  astar.GetPath(&env,g,s,sol);
  std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << "\n";
  //for(airplaneState const&a: sol) std::cout << a << "\n";
  astar.SetDoPartialExpansion(true);
  astar.GetPath(&env,g,s,sol);
  std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << "\n";
  //for(airplaneState const&a: sol) std::cout << a << "\n";
}
{
  airplaneState s(10, 10, 16, 3, 0, false);
  airplaneState g(40, 20, 11, 3, 0, false);
  AirplaneEnvironment env;
  env.setGoal(g);
  env.loadPerimeterDB();
  TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
  std::vector<airplaneState> sol;
  astar.GetPath(&env,g,s,sol);
  std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << "\n";
  //for(airplaneState const&a: sol) std::cout << a << "\n";
  astar.SetDoPartialExpansion(true);
  astar.GetPath(&env,g,s,sol);
  std::cout << "Regular A* expansions: " << astar.GetNodesExpanded() << " unique:" << astar.GetUniqueNodesExpanded() << " generations: " << astar.GetNodesTouched() << " mem: " << astar.GetMemoryUsage() << " path len: " << sol.size() << "\n";
  //for(airplaneState const&a: sol) std::cout << a << "\n";
}
}

#endif
