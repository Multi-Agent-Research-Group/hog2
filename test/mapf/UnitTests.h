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
#ifndef mapf_UnitTests_h_
#define mapf_UnitTests_h_

#include "Timer.h"
#include <gtest/gtest.h>
#include "Grid3DEnvironment.h"
#include "Grid3DConstrainedEnvironment.h"
#include "MultiAgentEnvironment.h"
//#include "Grid3DEnvironment.h"
//#include "Grid3DConstrainedEnvironment.h"
//#include "Map3d.h"
#include "TemplateAStar.h"
#include "TemporalAStar.h"
#include "dtedreader.h"
#include "ICTSAlgorithm.h"
#include "SAP.h"
#include "ITree.h"
#include "Pairwise.h"

extern double agentRadius;
/*
TEST(ICTSAlgorithm, SimpleTest){
  agentRadius=.25;
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv,0);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);
  env.WaitTime(10);

  std::vector<EnvironmentContainer<xyztAABB,t3DDirection>*> envs(4);
  for(auto& e:envs){
    e=new EnvironmentContainer<xyztAABB,t3DDirection>();
    e->environment=&env;
  }
  Solution<xyztAABB> solution;
  MultiAgentState<xyztLoc> start = {{0,0},{7,7},{0,7},{7,0}};
  MultiAgentState<xyztLoc> goal = {{7,7},{0,0},{7,0},{0,7}};
  ICTSAlgorithm<xyztAABB,t3DDirection> ia;
  ia.verify=true;
  ia.verbose=false;
  std::string hint;
  ia.GetSolution(envs,start,goal,solution,hint);
  for(auto const& ss: solution)
    for(auto const& s:ss)
      std::cout << s << "\n";
  ASSERT_TRUE(validateBBSolution(solution));
}

TEST(ICTSAlgorithm, Funky){
  agentRadius=.25;
  Map3D map(8,8,8);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);

  std::vector<EnvironmentContainer<xyztLoc,t3DDirection>*> envs(2);
  for(auto& e:envs){
    e=new EnvironmentContainer<xyztLoc,t3DDirection>();
    e->environment=&env;
  }
  Solution<xyztLoc> solution;
  MultiAgentState<xyztLoc> start = {{3,6},{4,5}};
  MultiAgentState<xyztLoc> goal = {{5,6},{5,7}};
  ICTSAlgorithm<xyztLoc,t3DDirection> ia;
  ia.verify=true;
  ia.verbose=true;
  std::string hint;
  ia.GetSolution(envs,start,goal,solution,hint);
  for(auto const& ss: solution)
    for(auto const& s:ss)
      std::cout << s << "\n";
  ASSERT_TRUE(validateSolution(solution));
}

TEST(MAAStar, search){
  Map3D map(64,64,1);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv,0);
  env.SetIgnoreTime(true);
  env.SetIgnoreHeading(true);
  std::vector<Grid3DConstrainedEnvironment const*> envs ={&env,&env};
  MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment> mae(envs);
  TemplateAStar<MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState, MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentAction, MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment> > astar;
  MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState start;
  start.emplace_back(xyztLoc(50,45,0),xyztLoc(50,45,0));
  start.emplace_back(xyztLoc(55,50,0),xyztLoc(55,50,0));
  MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState goal;
  goal.emplace_back(xyztLoc(50,55,0),xyztLoc(50,55,0));
  goal.emplace_back(xyztLoc(45,50,0),xyztLoc(54,50,0));
  std::vector<MultiAgentEnvironment<std::pair<xyztLoc,xyztLoc>,t3DDirection,Grid3DConstrainedEnvironment>::MultiAgentState> path;
  astar.GetPath(&mae,start,goal,path);
  
  for(auto const& s:path){
    std::cout << s << "\n";
  }
}
*/

void verifySorting(SAP<xyztAABB> const& sap){
  // Verify that sorting property is still good
  auto mm(sap.sorted.begin());
  for(auto nn(mm+1); nn!=sap.sorted.end(); ++nn){
    if(!(*mm->key<*nn->key || *mm->key==*nn->key)){
      std::cout << "!" << *mm->key << " > "<< *nn->key << "\n";
    }else{
      //std::cout << "sorted<" << mm->lowerBound[0].value << "," << mm->lowerBound[1].cvalue << "," << mm->lowerBound[2].cvalue << "> > "<<nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">\n";
    }
    //ASSERT_LE(*mm,*nn);
    ASSERT_TRUE(*mm->key<*nn->key || *mm->key==*nn->key);
    ++mm;
  }
}

TEST(broadphase, comparison){
  std::vector<std::vector<xyztLoc>> waypoints;
  unsigned nagents(4);
  //unsigned tnum(2);
  unsigned type(9);

  for(unsigned tnum(0); tnum<1;++tnum){
  // Load problems from file
  {
    std::stringstream filename;
    //filename << "../../test/environments/instances/64x64/" << nagents << "/" << tnum << ".csv";
    filename << "ctext.csv";
    std::ifstream ss(filename.str());
    int x,y;
    float t(0.0);
    std::string line;
    while(std::getline(ss, line)){
      std::vector<xyztLoc> wpts;
      std::istringstream is(line);
      std::string field;
      while(is >> field){
        size_t a(std::count(field.begin(), field.end(), ','));
        if(a==1){
          sscanf(field.c_str(),"%d,%d", &x,&y);
        }else if(a==2){
          sscanf(field.c_str(),"%d,%d,%f", &x,&y,&t);
        }else{
          assert(!"Invalid value inside problem file");
        }
        wpts.emplace_back(x,y,t);
      }
      waypoints.push_back(wpts);
    }
  }

  Map3D map(64,64,1);
  Grid3DEnvironment menv(&map);
  menv.SetOneConnected();
  menv.SetGround();
  Grid3DConstrainedEnvironment env(&menv,0);
  env.WaitTime(10);
  env.constraints=new SAP<Constraint<xyztAABB>>(0u);
  // Find paths
  TemporalAStar<xyztAABB,Grid3DConstrainedEnvironment> astar;
  std::vector<std::vector<xyztAABB>> p;
  std::vector<std::vector<xyztAABB>*> p2;
  for(int i(0); i<nagents; ++i){
    astar.SetAgent(i);
    std::vector<xyztAABB> path;
    env.setGoal(waypoints[i][1]);
    astar.GetPath(&env,waypoints[i][0],waypoints[i][1],path,1000);
    p.push_back(path);
    std::cout << "AGENT " << i <<":\n";
    for(auto const& v:path){
        std::cout << v.start<< v.end<< "\n";
    }
  }
  for(int i(0); i<nagents; ++i){
    p2.push_back(&p[i]);
  }

  float radius(.25);

  //std::cout << "AABBs\n";
  //for(auto foo:temp){
    //for(auto nn(foo.begin()); nn!=foo.end(); ++nn){
      //std::cout << "AABB:" << nn->lowerBound[0].value << "," << nn->lowerBound[1].cvalue << "," << nn->lowerBound[2].cvalue << ">,<"<<nn->upperBound[0].value << "," << nn->upperBound[1].cvalue << "," << nn->upperBound[2].cvalue << ">\n";
    //}
    //std::cout << "\n";
  //}
    Timer tmr0;
    Timer tmr1;
    unsigned count0(0);
    tmr0.StartTimer();
    //for(auto const& s:sorted)
    //std::cout << "<"<<s.lowerBound[0].value<<"~"<<s.upperBound[0].value<<","<<s.lowerBound[1].cvalue<<"~"<<s.upperBound[1].cvalue<<","<<s.lowerBound[2].cvalue<<"~"<<s.upperBound[2].cvalue<<">\n";
    Pairwise<xyztAABB> pw(&p2,1);
    SAP<xyztAABB> sap(&p2,false);
    verifySorting(sap);
    std::vector<std::pair<xyztAABB const*,xyztAABB const*>> pairs;
    std::vector<std::pair<xyztAABB const*,xyztAABB const*>> c;
    sap.getAllPairs(pairs);
    std::cout << pairs.size() << " pairs\n";
    for(auto pp(pairs.begin()); pp!=pairs.end(); ++pp){
      if(collisionCheck3D(pp->first->start,pp->first->end,pp->second->start,pp->second->end,radius)){
        //std::cout << *pp->first << *pp->second << "\n";
        c.push_back(*pp);
        ++count0;
      }
    }
    tmr0.EndTimer();
    
    tmr1.StartTimer();
    unsigned count1(0);
    std::vector<std::pair<xyztAABB const*,xyztAABB const*>> pairs2;
    std::vector<std::pair<xyztAABB const*,xyztAABB const*>> c2;
    pw.getAllPairs(pairs2);
    std::cout << pairs2.size() << " pairs2\n";
    for(auto pp(pairs2.begin()); pp!=pairs2.end(); ++pp){
      if(collisionCheck3D(pp->first->start,pp->first->end,pp->second->start,pp->second->end,radius)){
        std::cout << *pp->first<< *pp->second<< "\n";
        c2.push_back(*pp);
        ++count1;
      }
    }
    tmr1.EndTimer();

    std::cout << "SAP\n";
    std::sort(c.begin(),c.end(),[](std::pair<xyztAABB const*,xyztAABB const*> const& a, std::pair<xyztAABB const*,xyztAABB const*> const& b) -> bool
        {
        return a.first->agent==b.first->agent?
               a.second->agent==b.second->agent?
               a.first->start.t==b.first->start.t?
               a.second->start.t==b.second->start.t?
               a.first->end.t==b.first->end.t?
               a.second->end.t<b.second->end.t:
               a.first->end.t<b.first->end.t:
               a.second->start.t<b.second->start.t:
               a.first->start.t<b.first->start.t:
               a.second->agent<b.second->agent:
               a.first->agent<b.first->agent;
        }
        );
    for(auto const& a:c){
      std::cout << a.first->start<<"->"<<a.first->end<<":"<<a.first->agent<<","<<a.second->start<<"->"<<a.second->end<<":"<<a.second->agent << "\n";
    }
    std::cout << "PW\n";
    for(auto const& a:c2){
      std::cout << a.first->start<<"->"<<a.first->end<<":"<<a.first->agent<<","<<a.second->start<<"->"<<a.second->end<<":"<<a.second->agent << "\n";
    }
    std::cout << "DONE\n";
    for(auto const& a:c){
      bool found(false);
      for(auto const& b:c2){
        if((*a.first==*b.first && *a.second==*b.second)||(*a.first==*b.second && *a.second==*b.first)){
          found=true;
          break;
        }
      }
        if(!found){
          std::cout << *a.first<<*a.second << "Not in pw\n";
        }
    }
    for(auto const& b:c2){
      bool found(false);
      for(auto const& a:c){
        if((*a.first==*b.first && *a.second==*b.second)||(*a.first==*b.second && *a.second==*b.first)){
          found=true;
          break;
        }
      }
        if(!found){
          std::cout << *b.first<<*b.second << "Not in sap\n";
        }
    }
    std::cout << "Replace test (linear) on " << nagents << " with " << pairs.size() << " vs " << pairs2.size() << " and " << sap.comparisons << " vs " << pw.comparisons << " checks, " << count0 << " vs " << count1 << " collisions took " << "(" << tmr0.EndTimer() << ", " << tmr1.EndTimer() << ")\n";
  }
}

TEST(ITree, singleCollision){
  ITree<xyztAABB> tree(8);
  xyztAABB a(xyztLoc(0,0,0,0.0f),xyztLoc(1,1,0,1.4f),1);
  xyztAABB b(xyztLoc(1,1,0,2.0f),xyztLoc(2,2,0,3.4f),2);
  xyztAABB c(xyztLoc(0,0,0,0.0f),xyztLoc(1,1,0,1.4f),3);
  xyztAABB d(xyztLoc(0,1,0,1.0f),xyztLoc(1,1,0,2.0f),4);
  tree.insert(&a);
  ASSERT_EQ(1,tree.tree.size());
  ASSERT_EQ(1,tree.nodes.size());
  std::vector<xyztAABB const*> cc; 
  tree.getConflicts(&a,cc); // Same agent
  ASSERT_EQ(0,cc.size());
  tree.getConflicts(&b,cc); // No overlap
  ASSERT_EQ(0,cc.size());
  tree.getConflicts(&c,cc);
  ASSERT_EQ(1,cc.size());
  ASSERT_EQ((xyztAABB const*)&a,cc[0]);
  cc.clear();
  tree.getConflicts(&d,cc);
  ASSERT_EQ(1,cc.size());
  ASSERT_EQ((xyztAABB const*)&a,cc[0]);
  tree.clear();
  ASSERT_EQ(0,tree.tree.size());
  ASSERT_EQ(0,tree.nodes.size());
}


#endif
