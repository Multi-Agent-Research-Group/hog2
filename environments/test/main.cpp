#include "TestEnvironment.h"
#include "../Airplane.h"
#include "../AirplaneSimple.h"

int main(){
  testGetAction();
  testHeadingTo();
  testConstraints();
  testHCost();

  airtimeState s1, g1;
  s1.x = 36;
  s1.y = 40;
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
  return 0;
}
