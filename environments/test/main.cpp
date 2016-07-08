#include "TestEnvironment.h"
#include "../Airplane.h"
#include "../AirplaneSimple.h"

int main(){
/*
  airtimeState s1, g1;
  s1.x = 38;
  s1.y = 36;
  s1.height = 14;
  s1.heading = 2;
  s1.speed = 5;
  s1.t = 0;

  g1.x = 42;
  g1.y = 40;
  g1.height = 14;
  g1.heading = 6;
  g1.speed = 5;
  g1.t = 0;

  AirplaneEnvironment* ae(new AirplaneEnvironment());
  AirplaneSimpleEnvironment* ase(new AirplaneSimpleEnvironment());

  // This section is just here to test out different heuristics

  StraightLineHeuristic<airplaneState> sh;
  OctileDistanceHeuristic<airplaneState> oh;

  TemplateAStar<airplaneState, airplaneAction, AirplaneEnvironment> astar;
  std::vector<airplaneState> sol;
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Orig - hcost: " << ae->HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(2.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Origx2 - hcost: " << ae->HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetHeuristic(&sh);
  astar.SetWeight(1.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Line - hcost: " << sh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(2.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "linex2 - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;


  sol.clear();
  astar.SetHeuristic(&oh);
  astar.SetWeight(1.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Octile - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(2.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Octilex2 - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(1.0);
  astar.SetHeuristic(0);
  astar.GetPath(ase,s1,g1,sol);
  std::cout << "Orig simple - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(2.0);
  //astar.GetPath(ae,s1,g1,sol);
  //std::cout << "Origx2 simple - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(1.0);
  astar.SetHeuristic(&sh);
  //astar.GetPath(ase,s1,g1,sol);
  //std::cout << "Line simple - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(2.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Linex2 simple - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  ManhattanHeuristic<airplaneState> mh;
  sol.clear();
  astar.SetWeight(1.0);
  astar.SetHeuristic(&mh);
  //astar.GetPath(ase,s1,g1,sol);
  //std::cout << "Manhattan simple - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;

  sol.clear();
  astar.SetWeight(2.0);
  astar.GetPath(ae,s1,g1,sol);
  std::cout << "Manhattanx2 simple - hcost: " << oh.HCost(s1,g1) << " pathcost: " <<ae->GetPathLength(sol) <<" expansions: " << astar.GetNodesExpanded()<<"\n";;
*/
  testHCost();
  testGetAction();
  testHeadingTo();
  testConstraints();

  return 0;

}
