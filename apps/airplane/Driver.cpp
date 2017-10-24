#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "ScenarioLoader.h"
#include "AirplaneGridCardinal.h"
#include "AirplaneGridOctile.h"
#include "AirplaneGrid3DCardinal.h"
#include "AirplaneGrid3DOctile.h"
#include "AirplaneSimple.h"
#include "AirplaneHighway.h"
#include "AirplaneHighway4.h"
#include "AirplaneCardinal.h"
#include "AirplaneHighway4Cardinal.h"
#include "AirplaneConstrained.h"
#include "CBSUnits.h"
#include "NonUnitTimeCAT.h"

#include <sstream>

bool greedyCT = false; // use greedy heuristic at the high-level
bool ECBSheuristic = false; // use ECBS heuristic at low-level
bool randomalg = false; // Randomize tiebreaking
bool useCAT = false; // Use conflict avoidance table
bool mouseTracking;
unsigned killtime(3600); // Kill after some number of seconds
unsigned killex(INT_MAX); // Kill after some number of expansions
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
int width = 80;
int length = 80;
int height = 80;
bool recording = false; // Record frames
bool verbose(false);
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double frameIncrement = 1.0/10000.0;
std::vector<airtimeState> thePath;
std::vector<std::vector<airtimeState> > waypoints;
std::vector<SoftConstraint<airtimeState> > sconstraints;

  int cutoffs[10] = {0,9999,9999,9999,9999,9999,9999,9999,9999,9999}; // for each env
  double weights[10] = {1,1,1,1,1,1,1,1,1,1}; // for each env
  std::vector<std::vector<EnvironmentContainer<airtimeState,airplaneAction>>> environs;
  int seed = clock();
  int num_airplanes = 5;
  int minsubgoals(1);
  int maxsubgoals(1);
  bool use_rairspace = false;
  bool use_wait = false;
  bool nobypass = false;

  bool paused = false;

  ConstrainedEnvironment<airtimeState,airplaneAction> *ace = 0;
  UnitSimulation<airtimeState, airplaneAction, ConstrainedEnvironment<airtimeState,airplaneAction>> *sim = 0;
  typedef CBSGroup<airtimeState,airplaneAction,RandomTieBreaking<airtimeState,airplaneAction>,NonUnitTimeCAT<airtimeState,airplaneAction,AIR_HASH_INTERVAL_HUNDREDTHS>> AirplaneGroup;
  typedef CBSUnit<airtimeState,airplaneAction,RandomTieBreaking<airtimeState,airplaneAction>,NonUnitTimeCAT<airtimeState,airplaneAction,AIR_HASH_INTERVAL_HUNDREDTHS>> AirplaneUnit;
  AirplaneGroup* group = 0;

  bool gui=true;
  void InitHeadless();

  int main(int argc, char* argv[])
  {
    if (argc > 1) {
      num_airplanes = atoi(argv[1]);
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
    while (true)
    {
      group->ExpandOneCBSNode();
    }
    for(int i(0);i<group->GetNumMembers();++i){
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

	InstallCommandLineHandler(MyCLHandler, "-rairspace", "-rairspace", "Choose if the restricted airspace is used.");
	InstallCommandLineHandler(MyCLHandler, "-uwait", "-uwait", "Choose if the wait action is used.");
	InstallCommandLineHandler(MyCLHandler, "-dimensions", "-dimensions width,length,height", "Set the length,width and height of the environment (max 65K,65K,1024).");
	InstallCommandLineHandler(MyCLHandler, "-nairplanes", "-nairplanes <number>", "Select the number of airplanes.");
	InstallCommandLineHandler(MyCLHandler, "-nsubgoals", "-nsubgoals <number>,<number>", "Select the min,max number of subgoals per agent.");
	InstallCommandLineHandler(MyCLHandler, "-seed", "-seed <number>", "Seed for random number generator (defaults to clock)");
	InstallCommandLineHandler(MyCLHandler, "-nobypass", "-nobypass", "Turn off bypass option");
	InstallCommandLineHandler(MyCLHandler, "-record", "-record", "Record frames");
	InstallCommandLineHandler(MyCLHandler, "-pause", "-pause", "Record frames");
	InstallCommandLineHandler(MyCLHandler, "-cutoffs", "-cutoffs <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Number of conflicts to tolerate before switching to less constrained layer of environment. Environments are ordered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
	InstallCommandLineHandler(MyCLHandler, "-weights", "-weights <n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>,<n>", "Weight to apply to the low-level search for each environment entered as: CardinalGrid,OctileGrid,Cardinal3D,Octile3D,H4,H8,Simple,Cardinal,Octile,48Highway");
	InstallCommandLineHandler(MyCLHandler, "-probfile", "-probfile", "Load MAPF instance from file");
	InstallCommandLineHandler(MyCLHandler, "-constraints", "-constraints", "Load constraints from file");
	InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime", "Kill after this many seconds");
	InstallCommandLineHandler(MyCLHandler, "-killex", "-killex", "Kill after this many expansions");
	InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
	InstallCommandLineHandler(MyCLHandler, "-verbose", "-verbose", "Turn on verbose output");
	InstallCommandLineHandler(MyCLHandler, "-cat", "-cat", "Use Conflict Avoidance Table (CAT)");
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
  std::cout << "Setting seed " << seed << "\n";
  srand(seed);
  srandom(seed);
  AirplaneEnvironment* agce = new AirplaneGridCardinalEnvironment(width,length,height);
  AirplaneEnvironment* agoe = new AirplaneGridOctileEnvironment(width,length,height);
  AirplaneEnvironment* a3ce = new AirplaneGrid3DCardinalEnvironment(width,length,height);
  AirplaneEnvironment* a3oe = new AirplaneGrid3DOctileEnvironment(width,length,height);
  AirplaneEnvironment* ae = new AirplaneEnvironment(width,length,height);
  ae->loadPerimeterDB();
  AirplaneEnvironment* ase = new AirplaneSimpleEnvironment(width,length,height);
  ase->loadPerimeterDB();
  AirplaneEnvironment* ahe = new AirplaneHighwayEnvironment(width,length,height);
  ahe->loadPerimeterDB();
  AirplaneEnvironment* ah4e = new AirplaneHighway4Environment(width,length,height);
  ah4e->loadPerimeterDB();
  AirplaneEnvironment* ah4c = new AirplaneHighway4CardinalEnvironment(width,length,height);
  ah4c->loadPerimeterDB();
  AirplaneEnvironment* ac = new AirplaneCardinalEnvironment(width,length,height);
  ac->loadPerimeterDB();
  for(int i(0); i<num_airplanes; ++i){
    std::vector<EnvironmentContainer<airtimeState,airplaneAction>> ev;
    // Cardinal Grid
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(agce->name(),new AirplaneConstrainedEnvironment(agce,width,length,height),0,cutoffs[0],weights[0]));
    // Octile Grid
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(agoe->name(),new AirplaneConstrainedEnvironment(agoe,width,length,height),0,cutoffs[1],weights[1]));
    // Cardinal 3D Grid
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(a3ce->name(),new AirplaneConstrainedEnvironment(a3ce,width,length,height),0,cutoffs[2],weights[2]));
    // Octile 3D Grid
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(a3oe->name(),new AirplaneConstrainedEnvironment(a3oe,width,length,height),0,cutoffs[3],weights[3]));
    // Highway 4 Airplane
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(ah4c->name(),new AirplaneConstrainedEnvironment(ah4c,width,length,height),0,cutoffs[4],weights[4]));
    // Highway 8 Airplane
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(ahe->name(),new AirplaneConstrainedEnvironment(ahe,width,length,height),0,cutoffs[5],weights[5]));
    // Simple Airplane
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(ase->name(),new AirplaneConstrainedEnvironment(ase,width,length,height),0,cutoffs[6],weights[6]));
    // Cardinal Airplane
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(ac->name(),new AirplaneConstrainedEnvironment(ac,width,length,height),0,cutoffs[7],weights[7]));
    // Octile Airplane
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(ae->name(),new AirplaneConstrainedEnvironment(ae,width,length,height),0,cutoffs[8],weights[8]));
    // Highway 4/8 Airplane
    ev.push_back(EnvironmentContainer<airtimeState,airplaneAction>(ah4e->name(),new AirplaneConstrainedEnvironment(ah4e,width,length,height),0,cutoffs[9],weights[9]));

    for(auto& e:ev){
      for(auto& s:sconstraints){
        ((AirplaneConstrainedEnvironment*)e.environment)->AddSoftConstraint(s);
      }
    }
    environs.push_back(ev);
  }

  ace=environs[0].rbegin()->environment;

  group = new AirplaneGroup(environs,verbose); // Changed to 10,000 expansions from number of conflicts in the tree
  AirplaneGroup::greedyCT=greedyCT;
  group->timer=new Timer();
  group->seed=seed;
  group->keeprunning=gui;
  group->killex=killex;
  group->ECBSheuristic=ECBSheuristic;
  group->nobypass=nobypass;
  RandomTieBreaking<airtimeState,airplaneAction>::randomalg=randomalg;
  RandomTieBreaking<airtimeState,airplaneAction>::useCAT=useCAT;
  if(gui){
    sim = new UnitSimulation<airtimeState, airplaneAction, ConstrainedEnvironment<airtimeState,airplaneAction>>(ace);
    sim->SetStepType(kLockStep);

    sim->AddUnitGroup(group);
  }


  // Updated so we're always testing the landing conditions
  // and forcing the airplane environment to be such that
  // we are inducing high conflict areas.
  std::cout << "Adding " << num_airplanes << "planes." << std::endl;

  for (int i = 0; i < num_airplanes; i++) {
    if(waypoints.size()<num_airplanes){
      // Adding random waypoints
      std::vector<airtimeState> s;
      unsigned r(maxsubgoals-minsubgoals);
      int numsubgoals(minsubgoals+1);
      if(r>0){
        numsubgoals = rand()%(maxsubgoals-minsubgoals)+minsubgoals+1;
      }
      std::cout << "Agent " << i << " add " << numsubgoals << " subgoals\n";
      for(int n(0); n<numsubgoals; ++n){
        bool conflict(true);
        while(conflict){
          conflict=false;
          airplaneState rs1(rand() % 70 + 5, rand() % 70 + 5, rand() % 15 + 2, rand() % 3 + 1, rand() % 8, false);
          airtimeState start(rs1, 0);
          for (int j = 0; j < waypoints.size(); j++)
          {
            if(i==j){continue;}
            if(waypoints[j].size()<n)
            {
              airtimeState a(waypoints[j][n]);
              // Make sure that no subgoals at similar times have a conflict
              Collision<airtimeState> x_c(a,a,.25);
              if(x_c.ConflictsWith(start,start)){conflict=true;break;}
            }
            airtimeState a(start,1);
            airtimeState b(a);
            b.x++;
            if(conflict=ace->ViolatesConstraint(a,b)){break;}
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

    std::cout << "Set unit " << i << " subgoals: ";
    for(auto &a: waypoints[i])
      std::cout << a << " ";
    std::cout << std::endl;
    float softEff(.9);
    AirplaneUnit* unit = new AirplaneUnit(waypoints[i],softEff);
    unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
    group->AddUnit(unit); // Add to the group
    std::cout << "initial path for agent " << i << ":\n";
    for(auto const& n: group->tree[0].paths[i])
      std::cout << n << "\n";
    if(gui){sim->AddUnit(unit);} // Add to the group
  }
  if(!gui){
    Timer::Timeout func(std::bind(&AirplaneGroup::processSolution, group, std::placeholders::_1));
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

//std::vector<airplaneAction> acts;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{

	if (ace){
        for(auto u : group->GetMembers()){
            glLineWidth(2.0);
            ace->GLDrawPath(((AirplaneUnit const*)u)->GetPath(),((AirplaneUnit const*)u)->GetWaypoints());
        }
    }

  //static double ptime[500];
  //memset(ptime,0,500*sizeof(double));
	if (sim)
		sim->OpenGLDraw();
	if (!paused) {
		sim->StepTime(stepsPerFrame);
		
		/*std::cout << "Printing locations at time: " << sim->GetSimulationTime() << std::endl;
		for (int x = 0; x < group->GetNumMembers(); x ++) {
			AirplaneUnit *c = (AirplaneUnit*)group->GetMember(x);
			airtimeState cur;
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
	if(strcmp(argument[0], "-rairspace") == 0)
	{
		use_rairspace = true;
		return 1;
	}
	if(strcmp(argument[0], "-nobypass") == 0)
	{
		nobypass = true;
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
		int x,y,z,s,h;
                float t;
		std::string line;
		num_airplanes=0;
		while(std::getline(ss, line)){
			std::vector<airtimeState> wpts;
			std::istringstream is(line);
			std::string field;
			while(is >> field){
				sscanf(field.c_str(),"%d,%d,%d,%d,%d,%f", &x,&y,&z,&s,&h,&t);
				wpts.push_back(airtimeState(x,y,z,s,h,false,t));
			}
			waypoints.push_back(wpts);
			num_airplanes++;
		}
		return 2;
	}
	if(strcmp(argument[0], "-constraints") == 0){
		std::cout << "Reading constraints from file: \""<<argument[1]<<"\"\n";
		std::ifstream ss(argument[1]);
		int x,y,z,r;
		std::string line;
		while(std::getline(ss, line)){
			std::vector<airtimeState> wpts;
			std::istringstream is(line);
			std::string field;
			while(is >> field){
				sscanf(field.c_str(),"%d,%d,%d,%d", &x,&y,&z,&r);
				sconstraints.push_back(SoftConstraint<airtimeState>(airtimeState(x,y,z,0,0,0),r));
			}
		}
		return 2;
	}
	if(strcmp(argument[0], "-pause") == 0)
	{
		paused = true;
		return 1;
	}
	if(strcmp(argument[0], "-record") == 0)
	{
		recording = true;
		return 1;
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
          return 2;
        }
	if(strcmp(argument[0], "-nairplanes") == 0)
	{
		num_airplanes = atoi(argument[1]);	
		return 2;
	}
	return 1; //ignore typos
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	airplaneState b;
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
