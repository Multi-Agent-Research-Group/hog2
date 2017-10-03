#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "MultiObjectiveConstrainedEnvironment.h"
#include "Map2DConstrainedEnvironment.h"
#include "MOCBSUnits.h"
#include "NonUnitTimeCAT.h"
#include "NAMOAStar.h"
#include "RadialSafety2DObjectiveEnvironment.h"
#include "Terrain2DObjectiveEnvironment.h"

#include <sstream>

extern double agentRadius;
bool greedyCT = false; // use greedy heuristic at the high-level
bool ECBSheuristic = false; // use ECBS heuristic at low-level
bool randomalg = false; // Randomize tiebreaking
bool useCAT = false; // Use conflict avoidance table
bool verify = false;
bool mouseTracking;
unsigned killtime(300); // Kill after some number of seconds
unsigned killex(INT_MAX); // Kill after some number of expansions
bool disappearAtGoal(false);
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
int width = 8;
int length = 8;
int height = 0;
bool recording = false; // Record frames
bool verbose(false);
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double currStepRate(1.0/10000.0);
double frameIncrement = 1.0/10000.0;
std::vector<std::vector<xytLoc> > waypoints;
//std::vector<SoftConstraint<xytLoc> > sconstraints;

int cutoffs[10] = {0,9999,9999,9999,9999,9999,9999,9999,9999,9999}; // for each env
double weights[10] = {1,1,1,1,1,1,1,1,1,1}; // for each env
std::vector<MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>> environs;
int seed = clock();
int num_agents = 5;
int minsubgoals(1);
int maxsubgoals(1);
bool use_wait = false;
bool nobypass = true; // Bypass doesn't make sense for AnyAngle paths (probably)
bool fullSet(false);
bool blind(false);
bool ignoreHeading(false);

bool paused = false;
#define NUM_OBJECTIVES 3
typedef LexicographicTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES> LexiComparator;
typedef LexicographicGoalTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES> LexiGoalComparator;
typedef DominanceTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES> DominanceComparator;

// Lexicographic comparators
typedef CBSUnit<xytLoc,tDirection,
MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,
LexicographicTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,
NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,
NUM_OBJECTIVES,
NAMOAStar<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,NUM_OBJECTIVES>> LexObjectiveUnit;
typedef CBSGroup<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,LexicographicTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,NUM_OBJECTIVES,NAMOAStar<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,NUM_OBJECTIVES>> LexObjectiveGroup;

// Goal-based lexicographic comparators
typedef CBSUnit<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,LexicographicGoalTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,NUM_OBJECTIVES,NAMOAStar<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,NUM_OBJECTIVES>> LexGoalObjectiveUnit;
typedef CBSGroup<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,LexicographicGoalTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,NUM_OBJECTIVES,NAMOAStar<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,NUM_OBJECTIVES>> LexGoalObjectiveGroup;

// Dominance comparators
typedef CBSUnit<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,DominanceTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,NUM_OBJECTIVES,NAMOAStar<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,NUM_OBJECTIVES,AStarOpenClosed<xytLoc,DominanceTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NAMOAOpenClosedData<xytLoc,NUM_OBJECTIVES>>>> DominanceObjectiveUnit;
typedef CBSGroup<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,DominanceTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,NUM_OBJECTIVES,NAMOAStar<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>,NUM_OBJECTIVES,AStarOpenClosed<xytLoc,DominanceTieBreaking<xytLoc,tDirection,NUM_OBJECTIVES>,NAMOAOpenClosedData<xytLoc,NUM_OBJECTIVES>>>> DominanceObjectiveGroup;

MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection> *ace = 0;
UnitSimulation<xytLoc, tDirection, MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>> *sim = 0;
RadialSafety2DObjectiveEnvironment* safetyEnv = 0;
Terrain2DObjectiveEnvironment* terrainEnv = 0;
LexObjectiveGroup* lgroup = 0;
LexGoalObjectiveGroup* ggroup = 0;
DominanceObjectiveGroup* dgroup = 0;

bool gui=true;
int animate(0);
void InitHeadless();

int main(int argc, char* argv[])
{
  if (argc > 1) {
    num_agents = atoi(argv[1]);
  }


  InstallHandlers();
  ProcessCommandLineArgs(argc, argv);


  if(gui)
  {
    RunHOGGUI(argc, argv);
  }
  else
  {
    InitHeadless();
    if(lgroup){
      while (true)
      {
        lgroup->ExpandOneCBSNode();
      }
      /*if(verbose)*/for(int i(0);i<lgroup->GetNumMembers();++i){
        std::cout << "final path for agent " << i << ":\n";
        for(auto const& n: lgroup->tree.back().paths[i])
          std::cout << n << "\n";
      }
    } else if(ggroup){
      while (true)
      {
        ggroup->ExpandOneCBSNode();
      }
      /*if(verbose)*/for(int i(0);i<ggroup->GetNumMembers();++i){
        std::cout << "final path for agent " << i << ":\n";
        for(auto const& n: ggroup->tree.back().paths[i])
          std::cout << n << "\n";
      }
    }
    if(dgroup){
      while (true)
      {
        dgroup->ExpandOneCBSNode();
      }
      /*if(verbose)*/for(int i(0);i<dgroup->GetNumMembers();++i){
        std::cout << "final path for agent " << i << ":\n";
        for(auto const& n: dgroup->tree.back().paths[i])
          std::cout << n << "\n";
      }
    }
  }
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	SetNumPorts(id, 1);
	
//	unitSims.resize(id+1);
//	unitSims[id] = new DirectionSimulation(new Directional2DEnvironment(map, kVehicle));
//	unitSims[id]->SetStepType(kRealTime);
//	unitSims[id]->GetStats()->EnablePrintOutput(true);
//	unitSims[id]->GetStats()->AddIncludeFilter("gCost");
//	unitSims[id]->GetStats()->AddIncludeFilter("nodesExpanded");
//	dp = new DirectionalPlanner(quad);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Speed Up Simulation", "Speed Up simulation execution.", kNoModifier, '=');
	InstallKeyboardHandler(MyDisplayHandler, "Slow Down Simulation", "Slow Down simulation execution.", kNoModifier, '-');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Toggle recording.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');

	InstallCommandLineHandler(MyCLHandler, "-uwait", "-uwait", "Choose if the wait action is used.");
	InstallCommandLineHandler(MyCLHandler, "-dimensions", "-dimensions width,length,height", "Set the length,width and height of the environment (max 65K,65K,1024).");
	InstallCommandLineHandler(MyCLHandler, "-nagents", "-nagents <number>", "Select the number of agents.");
	InstallCommandLineHandler(MyCLHandler, "-nsubgoals", "-nsubgoals <number>,<number>", "Select the min,max number of subgoals per agent.");
	InstallCommandLineHandler(MyCLHandler, "-seed", "-seed <number>", "Seed for random number generator (defaults to clock)");
	InstallCommandLineHandler(MyCLHandler, "-type", "-type <number>", "Objective type: l=LEXICOGRAPHIC, lg=GOAL_LEXICOGRAPHIC, w=WEIGHTED_SUM, wg=GOAL_WEIGHTED_SUM, p=PURE, pg=GOAL_PURE");
	InstallCommandLineHandler(MyCLHandler, "-nobypass", "-nobypass", "Turn off bypass option");
	InstallCommandLineHandler(MyCLHandler, "-fullSet", "-fullSet", "Return full pareto-optimal set of paths");
	InstallCommandLineHandler(MyCLHandler, "-blind", "-blind", "Blind search option");
	InstallCommandLineHandler(MyCLHandler, "-ignoreHeading", "-ignoreHeading", "Ignore heading dimension");
        InstallCommandLineHandler(MyCLHandler, "-disappear", "-disappear", "Disappear at goal");
	InstallCommandLineHandler(MyCLHandler, "-record", "-record", "Record frames");
	InstallCommandLineHandler(MyCLHandler, "-radius", "-radius", "agent radius (in grid units)");
	InstallCommandLineHandler(MyCLHandler, "-cutoffs", "-cutoffs <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Number of conflicts to tolerate before switching to less constrained layer of environment. Environments are ordered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
	InstallCommandLineHandler(MyCLHandler, "-weights", "-weights <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Weight to apply to the low-level search for each environment entered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
	InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
	InstallCommandLineHandler(MyCLHandler, "-dangerfile", "-dangerfile", "Load danger information from file");
	InstallCommandLineHandler(MyCLHandler, "-terrainfile", "-terrainfile", "Load terrain information from file");
	InstallCommandLineHandler(MyCLHandler, "-goals", "-goals", "Goals for Goal-Based pathfinding");
	InstallCommandLineHandler(MyCLHandler, "-coeffs", "-coeffs", "Weights/Coefficients for dominance factor scaling");
	InstallCommandLineHandler(MyCLHandler, "-constraints", "-constraints", "Load constraints from file");
	InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime", "Kill after this many seconds");
	InstallCommandLineHandler(MyCLHandler, "-killex", "-killex", "Kill after this many expansions");
	InstallCommandLineHandler(MyCLHandler, "-animate", "-animate", "Animate CBS search");
	InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
	InstallCommandLineHandler(MyCLHandler, "-verbose", "-verbose", "Turn on verbose output");
	InstallCommandLineHandler(MyCLHandler, "-cat", "-cat", "Use Conflict Avoidance Table (CAT)");
	InstallCommandLineHandler(MyCLHandler, "-verify", "-verify", "Verify results");
	InstallCommandLineHandler(MyCLHandler, "-random", "-random", "Randomize conflict resolution order");
	InstallCommandLineHandler(MyCLHandler, "-greedyCT", "-greedyCT", "Greedy sort high-level search by number of conflicts (GCBS)");
	InstallCommandLineHandler(MyCLHandler, "-ECBSheuristic", "-ECBSheuristic", "Use heuristic in low-level search");

    InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		glClearColor(0.6, 0.8, 1.0, 1.0);
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		InitSim();
		CreateSimulation(windowID);
	}
}

// Heuristics
class StraightLineHeuristic : public Heuristic<xytLoc> {
  public:
  double HCost(const xytLoc &a,const xytLoc &b) const {
        return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
  }
};

void InitHeadless(){
  //std::cout << "Setting seed " << seed << "\n";
  srand(seed);
  srandom(seed);
  Map* map(new Map(width,length));
  //StraightLineHeuristic* sh(new StraightLineHeuristic());
  Map2DConstrainedEnvironment* w4 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w4->GetEnv()->SetFourConnected();
  Map2DConstrainedEnvironment* w5 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w5->GetEnv()->SetFiveConnected();
  Map2DConstrainedEnvironment* w8 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w8->GetEnv()->SetEightConnected();
  Map2DConstrainedEnvironment* w9 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w9->GetEnv()->SetNineConnected();
  Map2DConstrainedEnvironment* w24 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w24->GetEnv()->SetTwentyFourConnected();
  Map2DConstrainedEnvironment* w25 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w25->GetEnv()->SetTwentyFiveConnected();
  Map2DConstrainedEnvironment* w48 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w48->GetEnv()->SetFortyEightConnected();
  Map2DConstrainedEnvironment* w49 = new Map2DConstrainedEnvironment(new MapEnvironment(map)); w49->GetEnv()->SetFortyNineConnected();

  // Necessary for lexicographic mode
  if(cost<NUM_OBJECTIVES>::compareType==cost<NUM_OBJECTIVES>::LEXICOGRAPHIC||cost<NUM_OBJECTIVES>::compareType==cost<NUM_OBJECTIVES>::GOAL_LEXICOGRAPHIC){
    w4->SetIgnoreTime(true);
    w5->SetIgnoreTime(true);
    w8->SetIgnoreTime(true);
    w9->SetIgnoreTime(true);
    w24->SetIgnoreTime(true);
    w25->SetIgnoreTime(true);
    w48->SetIgnoreTime(true);
    w49->SetIgnoreTime(true);
  }

  if(ignoreHeading){
    w4->SetIgnoreHeading(true);
    w5->SetIgnoreHeading(true);
    w8->SetIgnoreHeading(true);
    w9->SetIgnoreHeading(true);
    w24->SetIgnoreHeading(true);
    w25->SetIgnoreHeading(true);
    w48->SetIgnoreHeading(true);
    w49->SetIgnoreHeading(true);
  }

  std::vector<ObjectiveEnvironment<xytLoc>*> objectives5; // In priority order...
  objectives5.push_back(safetyEnv);
  objectives5.push_back(terrainEnv);
  objectives5.push_back(w5);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo4 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives5,w4);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo5 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives5,w5);
  std::vector<ObjectiveEnvironment<xytLoc>*> objectives9; // In priority order...
  objectives9.push_back(safetyEnv);
  objectives9.push_back(terrainEnv);
  objectives9.push_back(w9);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo8 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives9,w8);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo9 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives9,w9);
  std::vector<ObjectiveEnvironment<xytLoc>*> objectives25; // In priority order...
  objectives25.push_back(safetyEnv);
  objectives25.push_back(terrainEnv);
  objectives25.push_back(w25);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo24 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives25,w24);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo25 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives25,w25);
  std::vector<ObjectiveEnvironment<xytLoc>*> objectives49; // In priority order...
  objectives49.push_back(safetyEnv);
  objectives49.push_back(terrainEnv);
  objectives49.push_back(w49);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo48 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives49,w48);
  MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>* mo49 = new MultiObjectiveEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(objectives49,w49);
  // Cardinal Grid
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo4->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo4),0,cutoffs[0],weights[0]));
  if(verbose)std::cout << "Added " << mo4->name() << " @" << cutoffs[0] << " conflicts\n";
  // Cardinal Grid w/ Waiting
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo5->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo5),0,cutoffs[1],weights[1]));
  if(verbose)std::cout << "Added " << mo5->name() << " @" << cutoffs[1] << " conflicts\n";
  // Octile Grid
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo8->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo8),0,cutoffs[2],weights[2]));
  if(verbose)std::cout << "Added " << mo8->name() << " @" << cutoffs[2] << " conflicts\n";
  // Octile Grid w/ Waiting
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo9->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo9),0,cutoffs[3],weights[3]));
  if(verbose)std::cout << "Added " << mo9->name() << " @" << cutoffs[3] << " conflicts\n";
  // 24-connected Grid
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo24->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo24),0,cutoffs[4],weights[4]));
  if(verbose)std::cout << "Added " << mo24->name() << " @" << cutoffs[4] << " conflicts\n";
  // 24-connected Grid w/ Waiting
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo25->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo25),0,cutoffs[5],weights[5]));
  if(verbose)std::cout << "Added " << mo25->name() << " @" << cutoffs[5] << " conflicts\n";
  // 48-connected Grid
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo48->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo48),0,cutoffs[6],weights[6]));
  if(verbose)std::cout << "Added " << mo48->name() << " @" << cutoffs[6] << " conflicts\n";
  // 48-connected Grid w/ Waiting
  environs.push_back(MOEnvironmentContainer<xytLoc,tDirection,MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(mo49->name(),new MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>(mo49),0,cutoffs[7],weights[7]));
  if(verbose)std::cout << "Added " << mo49->name() << " @" << cutoffs[7] << " conflicts\n";

  //For use with NAMOAStar only...
  //for(auto& e:environs){
    //e.environment->SetIgnoreTime(true);
  //}

  ace=environs.rbegin()->environment;
  for(int i(0); i<10; ++i){
    if(cutoffs[i]==0)terrainEnv->SetConnectedness(environs[i].environment->GetEnv()->GetPhysicalEnv()->GetEnv()->GetConnectedness());
  }
  //ace->SetAgentRadius(agentRadius);

  LexObjectiveGroup::greedyCT=greedyCT;
  LexGoalObjectiveGroup::greedyCT=greedyCT;
  DominanceObjectiveGroup::greedyCT=greedyCT;
  LexiComparator::randomalg=LexiGoalComparator::randomalg=DominanceComparator::randomalg=randomalg;
  LexiComparator::useCAT=LexiGoalComparator::useCAT=DominanceComparator::useCAT=useCAT;
  LexiGoalComparator::randomalg=LexiGoalComparator::randomalg=DominanceComparator::randomalg=randomalg;
  LexiGoalComparator::useCAT=LexiGoalComparator::useCAT=DominanceComparator::useCAT=useCAT;
  DominanceComparator::randomalg=LexiGoalComparator::randomalg=DominanceComparator::randomalg=randomalg;
  DominanceComparator::useCAT=LexiGoalComparator::useCAT=DominanceComparator::useCAT=useCAT;
  if(cost<NUM_OBJECTIVES>::compareType==cost<NUM_OBJECTIVES>::LEXICOGRAPHIC){
    lgroup = new LexObjectiveGroup(environs, verbose);
    lgroup->disappearAtGoal=disappearAtGoal;
    lgroup->timer=new Timer();
    lgroup->seed=seed;
    lgroup->keeprunning=gui;
    lgroup->animate=animate;
    lgroup->killex=killex;
    lgroup->ECBSheuristic=ECBSheuristic;
    lgroup->nobypass=nobypass;
    lgroup->verify=verify;
    lgroup->astar.SetFullSet(fullSet);

    if(blind)
      lgroup->astar.SetHCostFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::NullVector);
    //lgroup->astar.SetSuccessorFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::GetAllSuccessors);
    //lgroup->astar2.SetSuccessorFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::GetAllSuccessors);
    if(gui){
      sim = new UnitSimulation<xytLoc, tDirection, MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(ace);
      sim->SetStepType(kLockStep);

      sim->AddUnitGroup(lgroup);
    }
  }else if(cost<NUM_OBJECTIVES>::compareType==cost<NUM_OBJECTIVES>::GOAL_LEXICOGRAPHIC){
    ggroup = new LexGoalObjectiveGroup(environs, verbose);
    ggroup->disappearAtGoal=disappearAtGoal;
    ggroup->timer=new Timer();
    ggroup->seed=seed;
    ggroup->keeprunning=gui;
    ggroup->animate=animate;
    ggroup->killex=killex;
    ggroup->ECBSheuristic=ECBSheuristic;
    ggroup->nobypass=nobypass;
    ggroup->verify=verify;
    ggroup->astar.SetFullSet(fullSet);

    if(blind)
      ggroup->astar.SetHCostFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::NullVector);
    //ggroup->astar.SetSuccessorFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::GetAllSuccessors);
    //ggroup->astar2.SetSuccessorFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::GetAllSuccessors);
    if(gui){
      sim = new UnitSimulation<xytLoc, tDirection, MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(ace);
      sim->SetStepType(kLockStep);

      sim->AddUnitGroup(ggroup);
    }
  }else{
    dgroup = new DominanceObjectiveGroup(environs, verbose);
    dgroup->disappearAtGoal=disappearAtGoal;
    dgroup->timer=new Timer();
    dgroup->seed=seed;
    dgroup->keeprunning=gui;
    dgroup->animate=animate;
    dgroup->killex=killex;
    dgroup->ECBSheuristic=ECBSheuristic;
    dgroup->nobypass=nobypass;
    dgroup->verify=verify;
    dgroup->astar.SetFullSet(fullSet);

    if(blind)
      dgroup->astar.SetHCostFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::NullVector);
    //dgroup->astar.SetSuccessorFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::GetAllSuccessors);
    //dgroup->astar2.SetSuccessorFunc(&MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>::GetAllSuccessors);
    if(gui){
      sim = new UnitSimulation<xytLoc, tDirection, MultiObjectiveConstrainedEnvironment<Map2DConstrainedEnvironment,xytLoc,tDirection>>(ace);
      sim->SetStepType(kLockStep);

      sim->AddUnitGroup(dgroup);
    }
  }


  if(verbose)std::cout << "Adding " << num_agents << "agents." << std::endl;
  if(!gui){
    Timer::Timeout func1(std::bind(&LexObjectiveGroup::processSolution, lgroup, std::placeholders::_1));
    if(lgroup)lgroup->timer->StartTimeout(std::chrono::seconds(killtime),func1);
    Timer::Timeout func2(std::bind(&LexGoalObjectiveGroup::processSolution, ggroup, std::placeholders::_1));
    if(ggroup)ggroup->timer->StartTimeout(std::chrono::seconds(killtime),func2);
    Timer::Timeout func3(std::bind(&DominanceObjectiveGroup::processSolution, dgroup, std::placeholders::_1));
    if(dgroup)dgroup->timer->StartTimeout(std::chrono::seconds(killtime),func3);
  }

  for (int i = 0; i < num_agents; i++) {
    if(waypoints.size()<num_agents){
      // Adding random waypoints
      std::vector<xytLoc> s;
      unsigned r(maxsubgoals-minsubgoals);
      int numsubgoals(minsubgoals+1);
      if(r>0){
        numsubgoals = rand()%(maxsubgoals-minsubgoals)+minsubgoals+1;
      }
      if(verbose)std::cout << "Agent " << i << " add " << numsubgoals << " subgoals\n";

      for(int n(0); n<numsubgoals; ++n){
        bool conflict(true);
        while(conflict){
          conflict=false;
          xyLoc rs1(rand() % width, rand() % length);
          if(!ace->GetMap()->IsTraversable(rs1.x,rs1.y)){conflict=true;continue;}
          xytLoc start(rs1, 0);
          for (int j = 0; j < waypoints.size(); j++)
          {
            if(i==j){continue;}
            if(waypoints[j].size()>n)
            {
              xytLoc a(waypoints[j][n]);
              // Make sure that no subgoals at similar times have a conflict
              Constraint<xytLoc> x_c(a,a);
              if(x_c.ConflictsWith(start,start)){conflict=true;break;}
              if(a==start){conflict=true;break;}
            }
            /*xytLoc a(start,1.0);
            xytLoc b(a);
            b.x++;
            if(conflict=ace->ViolatesConstraint(a,b)){break;}*/
          }
          if(!conflict) s.push_back(start);
        }
      }
      waypoints.push_back(s);
    }
    for(auto w(waypoints.begin()+1); w!=waypoints.end();/*++w*/){
      if(*(w-1) == *w)
        waypoints.erase(w);
      else
        ++w;
    }

    if(verbose){
      std::cout << "Set unit " << i << " subgoals: ";
      for(auto &a: waypoints[i])
        std::cout << a << " ";
      std::cout << std::endl;
    }
    float softEff(.9);
    LexObjectiveUnit* unit = new LexObjectiveUnit(waypoints[i],softEff);
    unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
    if(lgroup) lgroup->AddUnit(unit); // Add to the group
    else if(ggroup) ggroup->AddUnit(unit); // Add to the group
    else if(dgroup) dgroup->AddUnit(unit); // Add to the group
    if(verbose)std::cout << "initial path for agent " << i << ":\n";
    if(lgroup)if(verbose)for(auto const& n: lgroup->tree[0].paths[i]) std::cout << n << "\n";
    else if(ggroup)if(verbose)for(auto const& n: ggroup->tree[0].paths[i]) std::cout << n << "\n";
    else if(dgroup)if(verbose)for(auto const& n: dgroup->tree[0].paths[i]) std::cout << n << "\n";
    if(gui){sim->AddUnit(unit);} // Add to the group
  }
  //assert(false && "Exit early");
}

void InitSim(){
  InitHeadless();
}

void MyComputationHandler()
{
  while (true)
  {
    sim->StepTime(currStepRate);
  }
}

//std::vector<tDirection> acts;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
  if (ace){
    if(lgroup)for(auto u : lgroup->GetMembers()){
      if(lgroup->donePlanning()){currStepRate=stepsPerFrame;}
      glLineWidth(4.0);
      GLfloat r, g, b;
      u->GetColor(r, g, b);
      ace->SetColor(r,g,b);
      
      ace->GLDrawPath(((LexObjectiveUnit const*)u)->GetPath(),((LexObjectiveUnit const*)u)->GetWaypoints());
    }else if(ggroup)for(auto u : ggroup->GetMembers()){
      if(ggroup->donePlanning()){currStepRate=stepsPerFrame;}
      glLineWidth(4.0);
      GLfloat r, g, b;
      u->GetColor(r, g, b);
      ace->SetColor(r,g,b);
      
      ace->GLDrawPath(((LexObjectiveUnit const*)u)->GetPath(),((LexObjectiveUnit const*)u)->GetWaypoints());
    }else if(dgroup)for(auto u : dgroup->GetMembers()){
      if(dgroup->donePlanning()){currStepRate=stepsPerFrame;}
      glLineWidth(4.0);
      GLfloat r, g, b;
      u->GetColor(r, g, b);
      ace->SetColor(r,g,b);
      
      ace->GLDrawPath(((LexObjectiveUnit const*)u)->GetPath(),((LexObjectiveUnit const*)u)->GetWaypoints());
    }
  }

  //static double ptime[500];
  //memset(ptime,0,500*sizeof(double));
  if (sim)
    sim->OpenGLDraw();
  if (!paused) {
    sim->StepTime(currStepRate);

    /*std::cout << "Printing locations at time: " << sim->GetSimulationTime() << std::endl;
      for (int x = 0; x < group->GetNumMembers(); x ++) {
      LexObjectiveUnit *c = (LexObjectiveUnit*)group->GetMember(x);
      xytLoc cur;
      c->GetLocation(cur);
    //if(!fequal(ptime[x],sim->GetSimulationTime())
    std::cout << "\t" << x << ":" << cur << std::endl;
    }*/
  }


  if (recording)
  {
    static int index = 0;
    char fname[255];
    sprintf(fname, "movies/cbs-%05d", index);
    SaveScreenshot(windowID, fname);
    printf("Saving '%s'\n", fname);
    index++;
  }
}

int MyCLHandler(char *argument[], int maxNumArgs)
{

	if(strcmp(argument[0], "-ECBSheuristic") == 0)
	{
                ECBSheuristic = true;
		return 1;
	}
	if(strcmp(argument[0], "-greedyCT") == 0)
	{
                greedyCT = true;
		return 1;
	}
	if(strcmp(argument[0], "-verify") == 0)
	{
                verify = true;
		return 1;
	}
	if(strcmp(argument[0], "-cat") == 0)
	{
                useCAT = true;
		return 1;
	}
	if(strcmp(argument[0], "-random") == 0)
	{
                randomalg = true;
		return 1;
	}
	if(strcmp(argument[0], "-killex") == 0)
	{
                killex = atoi(argument[1]);
		return 2;
	}
	if(strcmp(argument[0], "-killtime") == 0)
	{
                killtime = atoi(argument[1]);
		return 2;
	}
	if(strcmp(argument[0], "-animate") == 0)
	{
		animate=atoi(argument[1]);
		return 2;
	}
	if(strcmp(argument[0], "-nogui") == 0)
	{
		gui = false;
		return 1;
	}
	if(strcmp(argument[0], "-verbose") == 0)
	{
		verbose = true;
		return 1;
	}
	if(strcmp(argument[0], "-radius") == 0)
	{
		agentRadius = atof(argument[1]);
		return 2;
	}
	if(strcmp(argument[0], "-record") == 0)
	{
		recording = true;
		return 1;
	}
	if(strcmp(argument[0], "-disappear") == 0)
	{
		disappearAtGoal = true;
		return 1;
	}
	if(strcmp(argument[0], "-nobypass") == 0)
	{
		nobypass = true;
		return 1;
	}
	if(strcmp(argument[0], "-fullSet") == 0)
	{
		fullSet = true;
		return 1;
	}
	if(strcmp(argument[0], "-ignoreHeading") == 0)
	{
                std::cout << "Blind Search\n";
		ignoreHeading = true;
		return 1;
	}
	if(strcmp(argument[0], "-blind") == 0)
	{
                std::cout << "Blind Search\n";
		blind = true;
		return 1;
	}
	if(strcmp(argument[0], "-uwait") == 0)
	{
		use_wait = true;
		return 1;
	}
	if(strcmp(argument[0], "-probfile") == 0){
		std::cout << "Reading instance from file: \""<<argument[1]<<"\"\n";
		std::ifstream ss(argument[1]);
		int x,y;
                float t(0.0);
		std::string line;
		num_agents=0;
		while(std::getline(ss, line)){
			std::vector<xytLoc> wpts;
			std::istringstream is(line);
			std::string field;
			while(is >> field){
                                size_t n(std::count(field.begin(), field.end(), ','));
                                if(n==1){
                                  sscanf(field.c_str(),"%d,%d", &x,&y);
                                }else if(n==2){
				  sscanf(field.c_str(),"%d,%d,%f", &x,&y,&t);
                                }else{
                                  assert(!"Invalid value inside problem file");
                                }
				wpts.emplace_back(x,y,t);
			}
			waypoints.push_back(wpts);
			num_agents++;
		}
		return 2;
	}
	if(strcmp(argument[0], "-dangerfile") == 0){
		std::cout << "Reading danger costs from file: \""<<argument[1]<<"\"\n";
		std::ifstream ss(argument[1]);
                std::vector<xytLoc> centers;
                std::vector<double> radii;
		int x,y;
                double r(1.0);
		std::string line;
		while(std::getline(ss, line)){
			std::istringstream is(line);
			std::string field;
			while(is >> field){
                                size_t n(std::count(field.begin(), field.end(), ','));
                                if(n==2){
				  sscanf(field.c_str(),"%d,%d,%lf", &x,&y,&r);
                                }else{
                                  assert(!"Invalid value inside problem file");
                                }
                                std::cout << "Danger zone: " << x<<","<<y<<" r="<<r<<"\n";
				centers.emplace_back(x,y,0);
                                radii.push_back(r);
			}
		}
                safetyEnv=new RadialSafety2DObjectiveEnvironment(centers,radii,width,length);
		return 2;
	}
	if(strcmp(argument[0], "-terrainfile") == 0){
		std::cout << "Reading elevation data from file: \""<<argument[1]<<"\"\n";
                terrainEnv=new Terrain2DObjectiveEnvironment(width,length,argument[1],100.0); // Scale between 0 and 10 for elevation
		return 2;
	}
	if(strcmp(argument[0], "-constraints") == 0){
		std::cout << "Reading constraints from file: \""<<argument[1]<<"\"\n";
		std::ifstream ss(argument[1]);
		int x,y,r;
		std::string line;
		while(std::getline(ss, line)){
			std::vector<xytLoc> wpts;
			std::istringstream is(line);
			std::string field;
			while(is >> field){
				sscanf(field.c_str(),"%d,%d,%d", &x,&y,&r);
				//sconstraints.push_back(SoftConstraint<xytLoc>(xytLoc(x,y,0),r));
			}
		}
		return 2;
	}
	if(strcmp(argument[0], "-cutoffs") == 0)
        {
          std::string str = argument[1];

          std::stringstream ss(str);

          int i;
          int index(0);

          while (ss >> i)
          {
            cutoffs[index++] = i;

            if (ss.peek() == ',')
              ss.ignore();
          }
          return 2;
        }
	if(strcmp(argument[0], "-weights") == 0)
        {
          std::string str = argument[1];

          std::stringstream ss(str);

          double i;
          int index(0);

          while (ss >> i)
          {
            weights[index++] = i;

            if (ss.peek() == ',')
              ss.ignore();
          }
          return 2;
        }
	if(strcmp(argument[0], "-seed") == 0)
	{
		seed = atoi(argument[1]);	
		return 2;
	}
	if(strcmp(argument[0], "-type") == 0)
	{
                if(!strncasecmp(argument[1],"l",1)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::LEXICOGRAPHIC;
                else if(!strncasecmp(argument[1],"lg",2)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::GOAL_LEXICOGRAPHIC;
                else if(!strncasecmp(argument[1],"w",1)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::WEIGHTED_SUM;
                else if(!strncasecmp(argument[1],"wg",2)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::GOAL_WEIGHTED_SUM;
                else if(!strncasecmp(argument[1],"p",1)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::PURE;
                else if(!strncasecmp(argument[1],"pg",2)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::GOAL_PURE;
                else if(!strncasecmp(argument[1],"g",1)) cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::GOAL_PURE;
                else cost<NUM_OBJECTIVES>::compareType=cost<NUM_OBJECTIVES>::PURE;
                std::cout << "ctype " << cost<NUM_OBJECTIVES>::compareType <<"\n";
		return 2;
	}
	if(strcmp(argument[0], "-nsubgoals") == 0)
        {
          std::string str = argument[1];

          std::stringstream ss(str);

          int i;
          ss >> i;
          minsubgoals = i;
          if (ss.peek() == ',')
            ss.ignore();
          ss >> i;
          maxsubgoals = i;
          return 2;
        }
	if(strcmp(argument[0], "-coeffs") == 0)
        {
          std::string str(argument[1]);
          std::stringstream ss(str);

          double i;
          int index(0);

          while (ss >> i && index<NUM_OBJECTIVES)
          {
            cost<NUM_OBJECTIVES>::weights[index++] = i;

            if (ss.peek() == ',')
              ss.ignore();
          }
          return 2;
        }
	if(strcmp(argument[0], "-goals") == 0)
        {
          std::string str(argument[1]);
          std::stringstream ss(str);

          double i;
          int index(0);

          while (ss >> i && index<NUM_OBJECTIVES)
          {
            cost<NUM_OBJECTIVES>::goals[index++] = i;

            if (ss.peek() == ',')
              ss.ignore();
          }
          return 2;
        }
	if(strcmp(argument[0], "-dimensions") == 0)
        {
          std::string str = argument[1];

          std::stringstream ss(str);

          int i;
          ss >> i;
          width = i;
          if (ss.peek() == ',')
            ss.ignore();
          ss >> i;
          length = i;
          if (ss.peek() == ',')
            ss.ignore();
          ss >> i;
          height = i;
          return 2;
        }
	if(strcmp(argument[0], "-nagents") == 0)
	{
		num_agents = atoi(argument[1]);	
		return 2;
	}
	return 1; //ignore typos
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	xyLoc b;
	switch (key)
	{
		case 'r': recording = !recording; break;
		case '[': recording = true; break;
		case ']': recording = false; break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case '-': 
                        if(stepsPerFrame>0)stepsPerFrame-=frameIncrement;
                        break;
		case '=': 
                        stepsPerFrame+=frameIncrement;
                        break;
		case 'p': 
			paused = !paused;
			break;//unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
//			if (unitSims[windowID]->GetPaused())
//			{
//				unitSims[windowID]->SetPaused(false);
//				unitSims[windowID]->StepTime(1.0/30.0);
//				unitSims[windowID]->SetPaused(true);
//			}
			break;
		case 'd':
			
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long windowID, tKeyboardModifier mod, char)
{
	
}

void MyPathfindingKeyHandler(unsigned long , tKeyboardModifier , char)
{
//	// attmpt to learn!
//	Map m(100, 100);
//	Directional2DEnvironment d(&m);
//	//Directional2DEnvironment(Map *m, model envType = kVehicle, heuristicType heuristic = kExtendedPerimeterHeuristic);
//	xySpeedHeading l1(50, 50), l2(50, 50);
//	__gnu_cxx::hash_map<uint64_t, xySpeedHeading, Hash64 > stateTable;
//	
//	std::vector<xySpeedHeading> path;
//	TemplateAStar2<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> alg;
//	alg.SetStopAfterGoal(false);
//	alg.InitializeSearch(&d, l1, l1, path);
//	for (int x = 0; x < 2000; x++)
//		alg.DoSingleSearchStep(path);
//	int count = alg.GetNumItems();
//	LinearRegression lr(37, 1, 1/37.0); // 10 x, 10 y, dist, heading offset [16]
//	std::vector<double> inputs;
//	std::vector<double> output(1);
//	for (unsigned int x = 0; x < count; x++)
//	{
//		// note that the start state is always at rest;
//		// we actually want the goal state at rest?
//		// or generate everything by backtracking through the parents of each state
//		const AStarOpenClosedData<xySpeedHeading> val = GetItem(x);
//		inputs[0] = sqrt((val.data.x-l1.x)*(val.data.x-l1.x)+(val.data.y-l1.)*(val.data.y-l1.y));
//		// fill in values
//		if (fabs(val.data.x-l1.x) >= 10)
//			inputs[10] = 1;
//		else inputs[1+fabs(val.data.x-l1.x)] = 1;
//		if (fabs(val.data.y-l1.y) >= 10)
//			inputs[20] = 1;
//		else inputs[11+fabs(val.data.y-l1.y)] = 1;
//		// this is wrong -- I need the possibility of flipping 15/1 is only 2 apart
//		intputs[30+((int)(fabs(l1.rotation-val.data.rotation)))%16] = 1;
//		output[0] = val.g;
//		lr.train(inputs, output);
//		// get data and learn to predict the g-cost
//		//val.data.
//		//val.g;
//	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	return false;
	mouseTracking = false;
	if (button == kRightButton)
	{
		switch (mType)
		{
			case kMouseDown:
				//unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px1, py1);
				//printf("Mouse down at (%d, %d)\n", px1, py1);
				break;
			case kMouseDrag:
				mouseTracking = true;
				//unitSims[windowID]->GetEnvironment()->GetMap()->GetPointFromCoordinate(loc, px2, py2);
				//printf("Mouse tracking at (%d, %d)\n", px2, py2);
				break;
			case kMouseUp:
			{
//				if ((px1 == -1) || (px2 == -1))
//					break;
//				xySpeedHeading l1, l2;
//				l1.x = px1;
//				l1.y = py1;
//				l2.x = px2;
//				l2.y = py2;
//				DirPatrolUnit *ru1 = new DirPatrolUnit(l1, dp);
//				ru1->SetNumPatrols(1);
//				ru1->AddPatrolLocation(l2);
//				ru1->AddPatrolLocation(l1);
//				ru1->SetSpeed(2);
//				unitSims[windowID]->AddUnit(ru1);
			}
			break;
		}
		return true;
	}
	return false;
}
