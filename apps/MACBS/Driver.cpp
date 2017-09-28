#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "Map2DConstrainedEnvironment.h"
#include "MACBSUnits.h"
#include "NonUnitTimeCAT.h"
#include "Map2DDiffHeuristic.h"
#include "ICTSAlgorithm.h"

#include <sstream>

extern double agentRadius;
bool greedyCT = false; // use greedy heuristic at the high-level
bool ECBSheuristic = false; // use ECBS heuristic at low-level
bool randomalg = false; // Randomize tiebreaking
bool useCAT = false; // Use conflict avoidance table
bool verify = false;
bool mouseTracking;
unsigned killtime(3600); // Kill after some number of seconds
unsigned killex(INT_MAX); // Kill after some number of expansions
bool disappearAtGoal(false);
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
int width = 64;
int length = 64;
int height = 0;
bool recording = false; // Record frames
bool verbose(false);
bool quiet(false);
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double frameIncrement = 1.0/10000.0;
std::string mapfile;
unsigned mergeThreshold(5);
std::vector<std::vector<xytLoc> > waypoints;
//std::vector<SoftConstraint<xytLoc> > sconstraints;
#define NUMBER_CANONICAL_STATES 10
Heuristic<xytLoc>* h4(0);
Heuristic<xytLoc>* h8(0);
Heuristic<xytLoc>* h24(0);
Heuristic<xytLoc>* h48(0);
Map* map(0);
MapEnvironment* w4(0);
MapEnvironment* w5(0);
MapEnvironment* w8(0);
MapEnvironment* w9(0);
MapEnvironment* w24(0);
MapEnvironment* w25(0);
MapEnvironment* w48(0);
MapEnvironment* w49(0);
Map2DConstrainedEnvironment* e4(0);
Map2DConstrainedEnvironment* e5(0);
Map2DConstrainedEnvironment* e8(0);
Map2DConstrainedEnvironment* e9(0);
Map2DConstrainedEnvironment* e24(0);
Map2DConstrainedEnvironment* e25(0);
Map2DConstrainedEnvironment* e48(0);
Map2DConstrainedEnvironment* e49(0);

int cutoffs[10] = {0,9999,9999,9999,9999,9999,9999,9999,9999,9999}; // for each env
  double weights[10] = {1,1,1,1,1,1,1,1,1,1}; // for each env
  std::vector<std::vector<EnvironmentContainer<xytLoc,tDirection>>> environs;
  int seed = clock();
  int num_agents = 0;
  int minsubgoals(1);
  int maxsubgoals(1);
  bool use_wait = false;
  bool nobypass = false;

  bool paused = false;

  ConstrainedEnvironment<xytLoc,tDirection> *ace = 0;
  UnitSimulation<xytLoc, tDirection, ConstrainedEnvironment<xytLoc, tDirection>> *sim = 0;
  typedef CBSGroup<xytLoc,tDirection,TieBreaking<xytLoc,tDirection>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>,ICTSAlgorithm<xytLoc,tDirection>> MACBSGroup;
  typedef CBSUnit<xytLoc,tDirection,TieBreaking<xytLoc,tDirection>,NonUnitTimeCAT<xytLoc,tDirection,HASH_INTERVAL_HUNDREDTHS>> MACBSUnit;

  MACBSGroup* group = 0;

  bool gui=true;
  int animate(0);
  void InitHeadless();

  int main(int argc, char* argv[])
  {
  InstallHandlers();
  ProcessCommandLineArgs(argc, argv);
  
  if(gui)
  {
    RunHOGGUI(0, 0);
  }
  else
  {
    InitHeadless();
    while (true)
    {
      group->ExpandOneCBSNode();
    }
    if(verbose)for(int i(0);i<group->GetNumMembers();++i){
      std::cout << "final path for agent " << i << ":\n";
      for(auto const& n: group->tree.back().paths[i])
        std::cout << n << "\n";
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
	InstallCommandLineHandler(MyCLHandler, "-nobypass", "-nobypass", "Turn off bypass option");
        InstallCommandLineHandler(MyCLHandler, "-record", "-record", "Record frames");
	InstallCommandLineHandler(MyCLHandler, "-cutoffs", "-cutoffs <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Number of conflicts to tolerate before switching to less constrained layer of environment. Environments are ordered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
	InstallCommandLineHandler(MyCLHandler, "-weights", "-weights <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Weight to apply to the low-level search for each environment entered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
	InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
	InstallCommandLineHandler(MyCLHandler, "-constraints", "-constraints", "Load constraints from file");
	InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime", "Kill after this many seconds");
	InstallCommandLineHandler(MyCLHandler, "-killex", "-killex", "Kill after this many expansions");
	InstallCommandLineHandler(MyCLHandler, "-mapfile", "-mapfile", "Map file to use");
	InstallCommandLineHandler(MyCLHandler, "-scenfile", "-scenfile", "Scenario file to use");
	InstallCommandLineHandler(MyCLHandler, "-mergeThreshold", "-mergeThreshold", "Number of conflicts to tolerate between meta-agents before merging");
	InstallCommandLineHandler(MyCLHandler, "-radius", "-radius", "Radius in units of agent");
	InstallCommandLineHandler(MyCLHandler, "-disappear", "-disappear", "Agents disappear at goal");
	InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
	InstallCommandLineHandler(MyCLHandler, "-verbose", "-verbose", "Turn on verbose output");
	InstallCommandLineHandler(MyCLHandler, "-quiet", "-quiet", "Extreme minimal output");
	InstallCommandLineHandler(MyCLHandler, "-cat", "-cat", "Use Conflict Avoidance Table (CAT)");
	InstallCommandLineHandler(MyCLHandler, "-animate", "-animate", "Animate CBS search");
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


void InitHeadless(){
  //std::cout << "Setting seed " << seed << "\n";
  // Use a map file if provided
  // Cardinal Grid
  for(int i(0);i<num_agents;++i){
    std::vector<EnvironmentContainer<xytLoc,tDirection>> envs;
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w4->name(),e4,h4,cutoffs[0],weights[0]));
    if(verbose)std::cout << "Added " << w4->name() << " @" << cutoffs[0] << " conflicts\n";
    // Cardinal Grid w/ Waiting
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w5->name(),e5,h4,cutoffs[1],weights[1]));
    if(verbose)std::cout << "Added " << w5->name() << " @" << cutoffs[1] << " conflicts\n";
    // Octile Grid
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w8->name(),e8,h8,cutoffs[2],weights[2]));
    if(verbose)std::cout << "Added " << w8->name() << " @" << cutoffs[2] << " conflicts\n";
    // Octile Grid w/ Waiting
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w9->name(),e9,h8,cutoffs[3],weights[3]));
    if(verbose)std::cout << "Added " << w9->name() << " @" << cutoffs[3] << " conflicts\n";
    // 24-connected Grid
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w24->name(),e24,h24,cutoffs[4],weights[4]));
    if(verbose)std::cout << "Added " << w24->name() << " @" << cutoffs[4] << " conflicts\n";
    // 24-connected Grid w/ Waiting
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w25->name(),e25,h24,cutoffs[5],weights[5]));
    if(verbose)std::cout << "Added " << w25->name() << " @" << cutoffs[5] << " conflicts\n";
    // 48-connected Grid
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w48->name(),e48,h48,cutoffs[6],weights[6]));
    if(verbose)std::cout << "Added " << w48->name() << " @" << cutoffs[6] << " conflicts\n";
    // 48-connected Grid w/ Waiting
    envs.push_back(EnvironmentContainer<xytLoc,tDirection>(w49->name(),e49,h48,cutoffs[7],weights[7]));
    if(verbose)std::cout << "Added " << w49->name() << " @" << cutoffs[7] << " conflicts\n";
    environs.push_back(envs);
  }

  /*for(auto& e:environs){
    for(auto& s:sconstraints){
      e.environment->AddSoftConstraint(s);
    }
  }*/

  ace=environs.rbegin()->rbegin()->environment;

  group = new MACBSGroup(environs,verbose); // Changed to 10,000 expansions from number of conflicts in the tree
  MACBSGroup::greedyCT=greedyCT;
  group->disappearAtGoal=disappearAtGoal;
  group->timer=new Timer();
  group->seed=seed;
  group->keeprunning=gui;
  group->animate=animate;
  group->killex=killex;
  group->mergeThreshold=mergeThreshold;
  group->ECBSheuristic=ECBSheuristic;
  group->nobypass=nobypass;
  group->verify=verify;
  group->quiet=quiet;
  TieBreaking<xytLoc,tDirection>::randomalg=randomalg;
  TieBreaking<xytLoc,tDirection>::useCAT=useCAT;
  if(gui){
    sim = new UnitSimulation<xytLoc, tDirection, ConstrainedEnvironment<xytLoc, tDirection>>(ace);
    sim->SetStepType(kLockStep);
    sim->SetLogStats(false);

    sim->AddUnitGroup(group);
  }


  if(verbose)std::cout << "Adding " << num_agents << "agents." << std::endl;

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
          xyLoc rs1(rand() % 64, rand() % 64);
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

    if(!quiet){
      std::cout << "Set unit " << i << " subgoals: ";
      for(auto &a: waypoints[i])
        std::cout << a << " ";
      std::cout << std::endl;
    }
    float softEff(.9);
    MACBSUnit* unit = new MACBSUnit(waypoints[i],softEff);
    unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
    group->AddUnit(unit); // Add to the group
    if(verbose)std::cout << "initial path for agent " << i << ":\n";
    if(verbose)for(auto const& n: group->tree[0].paths[i])
      std::cout << n << "\n";
    if(gui){sim->AddUnit(unit);} // Add to the group
  }
  if(!gui){
    Timer::Timeout func(std::bind(&MACBSGroup::processSolution, group, std::placeholders::_1));
    group->timer->StartTimeout(std::chrono::seconds(killtime),func);
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
		sim->StepTime(stepsPerFrame);
	}
}

//std::vector<tDirection> acts;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{

	if (ace){
        for(auto u : group->GetMembers()){
            glLineWidth(2.0);
            GLfloat r, g, b;
            u->GetColor(r, g, b);
            ace->SetColor(r,g,b);
            ace->GLDrawPath(((MACBSUnit const*)u)->GetPath(),((MACBSUnit const*)u)->GetWaypoints());
        }
    }

  //static double ptime[500];
  //memset(ptime,0,500*sizeof(double));
        if (sim){
          sim->OpenGLDraw();
          if (!paused) {
            sim->StepTime(stepsPerFrame);

            /*std::cout << "Printing locations at time: " << sim->GetSimulationTime() << std::endl;
              for (int x = 0; x < group->GetNumMembers(); x ++) {
              MACBSUnit* c = (MACBSUnit*)group->GetMember(x);
              xytLoc cur;
              c->GetLocation(cur);
            //if(!fequal(ptime[x],sim->GetSimulationTime())
            std::cout << "\t" << x << ":" << cur << std::endl;
            }*/
          }
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
	if(strcmp(argument[0], "-scenfile") == 0)
        {
          std::string pathprefix("../../"); // Because I always run from the build directory...
          if(num_agents==0){
            std::cout<<"-nagents must be specified before -scenfile\n";
            exit(1);
          }
          ScenarioLoader sl(argument[1]);
          if(sl.GetNumExperiments()==0)
          {
            std::cout<<"No experiments in this scenario file or invalid file.\n";
            exit(1);
          }
          mapfile=sl.GetNthExperiment(0).GetMapName();
          mapfile.insert(0,pathprefix); // Add prefix
          map=new Map(mapfile.c_str());
          w4 = new MapEnvironment(map); w4->SetFourConnected();
          w5 = new MapEnvironment(map); w5->SetFiveConnected();
          w8 = new MapEnvironment(map); w8->SetEightConnected();
          w9 = new MapEnvironment(map); w9->SetNineConnected();
          w24 = new MapEnvironment(map); w24->SetTwentyFourConnected();
          w25 = new MapEnvironment(map); w25->SetTwentyFiveConnected();
          w48 = new MapEnvironment(map); w48->SetFortyEightConnected();
          w49 = new MapEnvironment(map); w49->SetFortyNineConnected();
          e4 = new Map2DConstrainedEnvironment(w4);
          e5 = new Map2DConstrainedEnvironment(w5);
          e8 = new Map2DConstrainedEnvironment(w8);
          e9 = new Map2DConstrainedEnvironment(w9);
          e24 = new Map2DConstrainedEnvironment(w24);
          e25 = new Map2DConstrainedEnvironment(w25);
          e48 = new Map2DConstrainedEnvironment(w48);
          e49 = new Map2DConstrainedEnvironment(w49);
        
          for(int i(0); i<num_agents; ++i){
            std::vector<xytLoc> wpts;
            Experiment e(sl.GetRandomExperiment());
            wpts.emplace_back(e.GetStartX(),e.GetStartY());
            wpts.emplace_back(e.GetGoalX(),e.GetGoalY());
            waypoints.push_back(wpts);
          }
          h4=new Map2DDiffHeuristic(map,e4,NUMBER_CANONICAL_STATES);
          h8=new Map2DDiffHeuristic(map,e8,NUMBER_CANONICAL_STATES);
          h24=new Map2DDiffHeuristic(map,e24,NUMBER_CANONICAL_STATES);
          h48=new Map2DDiffHeuristic(map,e48,NUMBER_CANONICAL_STATES);
          
          return 2;
        }
	if(strcmp(argument[0], "-mapfile") == 0)
        {
          mapfile=argument[1];
          std::string pathprefix("../../"); // Because I always run from the build directory...
          mapfile.insert(0,pathprefix); // Add prefix
          map=new Map(mapfile.c_str());
          w4 = new MapEnvironment(map); w4->SetFourConnected();
          w5 = new MapEnvironment(map); w5->SetFiveConnected();
          w8 = new MapEnvironment(map); w8->SetEightConnected();
          w9 = new MapEnvironment(map); w9->SetNineConnected();
          w24 = new MapEnvironment(map); w24->SetTwentyFourConnected();
          e4 = new Map2DConstrainedEnvironment(w4);
          e5 = new Map2DConstrainedEnvironment(w5);
          e8 = new Map2DConstrainedEnvironment(w8);
          e9 = new Map2DConstrainedEnvironment(w9);
          e24 = new Map2DConstrainedEnvironment(w24);
          e25 = new Map2DConstrainedEnvironment(w25);
          e48 = new Map2DConstrainedEnvironment(w48);
          e49 = new Map2DConstrainedEnvironment(w49);
          w25 = new MapEnvironment(map); w25->SetTwentyFiveConnected();
          w48 = new MapEnvironment(map); w48->SetFortyEightConnected();
          w49 = new MapEnvironment(map); w49->SetFortyNineConnected();
          return 2;
        }
	if(strcmp(argument[0], "-mergeThreshold") == 0)
        {
                mergeThreshold = atoi(argument[1]);
		return 2;
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
	if(strcmp(argument[0], "-quiet") == 0)
	{
		quiet = true;
		return 1;
	}
	if(strcmp(argument[0], "-verbose") == 0)
	{
		verbose = true;
		return 1;
	}
	if(strcmp(argument[0], "-disappear") == 0)
	{
		disappearAtGoal = true;
		return 1;
	}
	if(strcmp(argument[0], "-radius") == 0)
	{
		agentRadius=atof(argument[1]);
		return 2;
	}
	if(strcmp(argument[0], "-nobypass") == 0)
	{
		nobypass = true;
		return 1;
	}
        if(strcmp(argument[0], "-record") == 0)
        {
                recording = true;
                return 1;
        }
	if(strcmp(argument[0], "-uwait") == 0)
	{
		use_wait = true;
		return 1;
	}
	if(strcmp(argument[0], "-probfile") == 0){
		//std::cout << "Reading instance from file: \""<<argument[1]<<"\"\n";
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
                srand(seed);
                srandom(seed);
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
          map = new Map(width,length);
          w4 = new MapEnvironment(map); w4->SetFourConnected();
          w5 = new MapEnvironment(map); w5->SetFiveConnected();
          w8 = new MapEnvironment(map); w8->SetEightConnected();
          w9 = new MapEnvironment(map); w9->SetNineConnected();
          w24 = new MapEnvironment(map); w24->SetTwentyFourConnected();
          w25 = new MapEnvironment(map); w25->SetTwentyFiveConnected();
          w48 = new MapEnvironment(map); w48->SetFortyEightConnected();
          w49 = new MapEnvironment(map); w49->SetFortyNineConnected();
          e4 = new Map2DConstrainedEnvironment(w4);
          e5 = new Map2DConstrainedEnvironment(w5);
          e8 = new Map2DConstrainedEnvironment(w8);
          e9 = new Map2DConstrainedEnvironment(w9);
          e24 = new Map2DConstrainedEnvironment(w24);
          e25 = new Map2DConstrainedEnvironment(w25);
          e48 = new Map2DConstrainedEnvironment(w48);
          e49 = new Map2DConstrainedEnvironment(w49);
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
