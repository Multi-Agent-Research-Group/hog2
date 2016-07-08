#include "TestEnvironment.h"
#include "../Airplane.h"
#include "../AirplaneSimple.h"

int main(){
/*

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
  testLoadPerimeterHeuristic();
  testHCost();
  testGetAction();
  testHeadingTo();
  testConstraints();

  return 0;

}
