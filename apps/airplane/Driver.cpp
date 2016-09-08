#include "Common.h"
#include "Driver.h"
//#include "PRAStar.h"
//#include "SearchUnit.h"
#include "UnitSimulation.h"
//#include "Directional2DEnvironment.h"
//#include "RandomUnit.h"
//#include "AStar.h"
//#include "GenericSearchUnit.h"
//#include "GenericPatrolUnit.h"
//#include "TemplateAStar2.h"
//#include "MapSectorAbstraction.h"
//#include "DirectionalPlanner.h"
#include "ScenarioLoader.h"
#include "AirplaneSimple.h"
#include "AirplaneHighway.h"
#include "AirplaneHighway4.h"
#include "AirplaneConstrained.h"
#include "AirplaneCBSUnits.h"

#include <sstream>

bool highsort = false;
bool heuristic = false;
bool randomalg = false;
bool mouseTracking;
unsigned killtime(500);
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
bool recording = false;
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double frameIncrement = 1.0/10000.0;
std::vector<airtimeState> thePath;

int cutoffs[4] = {0,3,6,9}; // for each env
  std::vector<EnvironmentContainer> environs;
  int seed = clock();
  int num_airplanes = 5;
  bool use_rairspace = false;
  bool use_wait = false;
  bool nobypass = false;

  bool paused = false;

  AirplaneConstrainedEnvironment *ace = 0;
  UnitSimulation<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *sim = 0;
  AirCBSGroup* group = 0;

  bool gui=true;
  void InitHeadless();

  int main(int argc, char* argv[])
  {

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
      group->ExpandOneCBSNode(gui);
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
	InstallKeyboardHandler(MyDisplayHandler, "Recird", "Toggle recording.", kNoModifier, 'r');
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
	InstallCommandLineHandler(MyCLHandler, "-nairplanes", "-nairplanes <number>", "Select the number of airplanes.");
	InstallCommandLineHandler(MyCLHandler, "-seed", "-seed <number>", "Seed for random number generator (defaults to clock)");
	InstallCommandLineHandler(MyCLHandler, "-nobypass", "-nobypass", "Turn off bypass option");
	InstallCommandLineHandler(MyCLHandler, "-cutoffs", "-cutoffs <number>,<number>,<number,...", "Number of conflicts to tolerate before switching to less constrained layer of environment");
	InstallCommandLineHandler(MyCLHandler, "-killtime", "-killtime", "Kill after this many seconds");
	InstallCommandLineHandler(MyCLHandler, "-nogui", "-nogui", "Turn off gui");
	InstallCommandLineHandler(MyCLHandler, "-random", "-random", "Randomize conflict resolution order");
	InstallCommandLineHandler(MyCLHandler, "-highsort", "-highsort", "Use sort high-level search by number of conflicts");
	InstallCommandLineHandler(MyCLHandler, "-heuristic", "-heuristic", "Use heuristic in low-level search");

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
  AirplaneEnvironment* ae = new AirplaneEnvironment();
  ae->loadPerimeterDB();
  AirplaneEnvironment* ase = new AirplaneSimpleEnvironment();
  ase->loadPerimeterDB();
  AirplaneEnvironment* ahe = new AirplaneHighwayEnvironment();
  ahe->loadPerimeterDB();
  AirplaneEnvironment* ah4e = new AirplaneHighway4Environment();
  ah4e->loadPerimeterDB();
  environs.push_back(EnvironmentContainer(ahe->name(),new AirplaneConstrainedEnvironment(ahe),0,cutoffs[0],1));
  environs.push_back(EnvironmentContainer(ah4e->name(),new AirplaneConstrainedEnvironment(ah4e),0,cutoffs[1],1));
  environs.push_back(EnvironmentContainer(ase->name(),new AirplaneConstrainedEnvironment(ase),0,cutoffs[2],1));
  environs.push_back(EnvironmentContainer(ae->name(),new AirplaneConstrainedEnvironment(ae),0,cutoffs[3],1));

    group = new AirCBSGroup(environs,use_rairspace, use_wait, nobypass); // Changed to 10,000 expansions from number of conflicts in the tree

  // Updated so we're always testing the landing conditions
  // and forcing the airplane environment to be such that
  // we are inducing high clonflict areas.
  std::cout << "Adding " << num_airplanes << "planes." << std::endl;
  std::vector<airtimeState> starts;
  std::vector<airtimeState> goals;
  for (int i = 0; i < num_airplanes; i++) {


    // 0% are landed at the beginning
    if (false) {
      airplaneState land(18, 23, 0, 0, 0, true);
      airtimeState start(land, 0);

      airplaneState rs(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
      airtimeState goal(rs, 0);

      AirCBSUnit* unit = new AirCBSUnit(start, goal);
      unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
      group->AddUnit(unit); // Add to the group
      //std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;

    } else {
      bool conflict(true);
      while(conflict){
        conflict=false;
        airplaneState rs1(rand() % 40 + 5, rand() % 40 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
        airtimeState start(rs1, 0);
        for(auto a: starts)
        {
          // Make sure that no start points have a conflict
          airConstraint x_c(a);
          if(x_c.ConflictsWith(start)){conflict=true;break;}
        }
        if(!conflict)
          starts.push_back(start);
      }

      // 0% total are landing intially
      if (false) {
        // Replan the node to a landing location
        /*airplaneState land(18, 23, 0, 0, 0, true);
          airtimeState goal(land, 0);

          AirCBSUnit* unit = new AirCBSUnit(start, goal);
          unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
          group->AddUnit(unit); // Add to the group
          std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;*/

      } else {
        conflict=true;
        while(conflict){
          conflict=false;
          airplaneState rs1(rand() % 40 + 5, rand() % 40 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
          airtimeState goal(rs1, 0);
          for(auto a: goals)
          {
            // Make sure that no start points have a conflict
            airConstraint x_c(a);
            if(x_c.ConflictsWith(goal)){conflict=true;break;}
          }
          if(!conflict)
            goals.push_back(goal);
        }

        AirCBSUnit* unit = new AirCBSUnit(*starts.rbegin(), *goals.rbegin());
        unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
        group->AddUnit(unit); // Add to the group
        //std::cout << "Set unit " << i << " directive from " << *starts.rbegin() << " to " << *goals.rbegin() << std::endl;
      }

    }
  }

  //std::cout << "cmdflags:" << use_rairspace << use_wait << std::endl;

  // Add a Quadcopter
  /*auto x = 40;
    auto y = 40;
    airplaneState ss(x, y, 7, 1, 7, false, AirplaneType::QUAD);
    airtimeState start(ss, 0);
    airplaneState gs(x, y, 18, 5, 2, false, AirplaneType::QUAD);
    airtimeState goal(gs, 0);
    AirCBSUnit* unit = new AirCBSUnit(start, goal);
    unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
    std::cout << "Set quadcopter directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;
    group->AddUnit(unit); // Add to the group
    sim->AddUnit(unit); // Add the unit to the simulation
   */


}

void InitSim(){
  std::cout << "Setting seed " << seed << "\n";
  std::cout << "Setting nobypass " << nobypass << "\n";
  srand(seed);
  srandom(seed);
  AirplaneEnvironment* ae = new AirplaneEnvironment();
  ae->loadPerimeterDB();
  AirplaneEnvironment* ase = new AirplaneSimpleEnvironment();
  ase->loadPerimeterDB();
  AirplaneEnvironment* ahe = new AirplaneHighwayEnvironment();
  ahe->loadPerimeterDB();
  AirplaneEnvironment* ah4e = new AirplaneHighway4Environment();
  ah4e->loadPerimeterDB();
  
  environs.push_back(EnvironmentContainer(ahe->name(),new AirplaneConstrainedEnvironment(ahe),0,cutoffs[0],1));
  environs.push_back(EnvironmentContainer(ah4e->name(),new AirplaneConstrainedEnvironment(ah4e),0,cutoffs[1],1));
  environs.push_back(EnvironmentContainer(ase->name(),new AirplaneConstrainedEnvironment(ase),0,cutoffs[2],1));
  environs.push_back(EnvironmentContainer(ae->name(),new AirplaneConstrainedEnvironment(ae),0,cutoffs[3],1));

  ace=environs.rbegin()->environment;

  group = new AirCBSGroup(environs,use_rairspace, use_wait, nobypass); // Changed to 10,000 expansions from number of conflicts in the tree


  sim = new UnitSimulation<airtimeState, airplaneAction, AirplaneConstrainedEnvironment>(ace);
  sim->SetStepType(kLockStep);

  sim->AddUnitGroup(group);

  // Updated so we're always testing the landing conditions
  // and forcing the airplane environment to be such that
  // we are inducing high clonflict areas.
  std::cout << "Adding " << num_airplanes << "planes." << std::endl;
  std::vector<airtimeState> starts;
  std::vector<airtimeState> goals;
  for (int i = 0; i < num_airplanes; i++) {

    // 0% are landed at the beginning
    if (false) {
      airplaneState land(18, 23, 0, 0, 0, true);
      airtimeState start(land, 0);

      airplaneState rs(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
      airtimeState goal(rs, 0);

      AirCBSUnit* unit = new AirCBSUnit(start, goal);
      unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
      group->AddUnit(unit); // Add to the group
      //std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;

    } else {
      bool conflict(true);
      while(conflict){
        conflict=false;
        airplaneState rs1(rand() % 40 + 5, rand() % 40 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
        airtimeState start(rs1, 0);
        for(auto a: starts)
        {
          // Make sure that no start points have a conflict
          airConstraint x_c(a);
          if(x_c.ConflictsWith(start)){conflict=true;break;std::cout<<a<<" conflictswith "<<start<<"\n";}
        }
        if(!conflict){std::cout<<"added start: " << start<<"\n";
          starts.push_back(start);}
      }

      // 0% total are landing intially
      if (false) {
        // Replan the node to a landing location
        /*airplaneState land(18, 23, 0, 0, 0, true);
          airtimeState goal(land, 0);

          AirCBSUnit* unit = new AirCBSUnit(start, goal);
          unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
          group->AddUnit(unit); // Add to the group
          std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;*/

      } else {
        conflict=true;
        while(conflict){
          conflict=false;
          airplaneState rs1(rand() % 40 + 5, rand() % 40 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
          airtimeState goal(rs1, 0);
          for(auto a: goals)
          {
            // Make sure that no start points have a conflict
            airConstraint x_c(a);
            if(x_c.ConflictsWith(goal)){conflict=true;break;std::cout<<a<<" conflictswith "<<goal<<"\n";}
          }
          if(!conflict){std::cout<<"added goal: " << goal<<"\n";
            goals.push_back(goal);}
        }

        AirCBSUnit* unit = new AirCBSUnit(*starts.rbegin(), *goals.rbegin());
        unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
        group->AddUnit(unit); // Add to the group
        sim->AddUnit(unit); // Add to the group
        std::cout << "Set unit " << i << " directive from " << *starts.rbegin() << " to " << *goals.rbegin() << std::endl;
      }

    }
  }
std::cout << "Done adding all units.\n";

  //std::cout << "cmdflags:" << use_rairspace << use_wait << std::endl;

  // Add a Quadcopter
  /*auto x = 40;
    auto y = 40;
    airplaneState ss(x, y, 7, 1, 7, false, AirplaneType::QUAD);
    airtimeState start(ss, 0);
    airplaneState gs(x, y, 18, 5, 2, false, AirplaneType::QUAD);
    airtimeState goal(gs, 0);
    AirCBSUnit* unit = new AirCBSUnit(start, goal);
    unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
    std::cout << "Set quadcopter directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;
    group->AddUnit(unit); // Add to the group
    sim->AddUnit(unit); // Add the unit to the simulation
   */


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
            ace->GLDrawPath(((AirCBSUnit const*)u)->GetPath());
        }
    }

	if (sim)
		sim->OpenGLDraw();
	if (!paused) {
		sim->StepTime(stepsPerFrame);
		/*
		std::cout << "Printing locations at time: " << sim->GetSimulationTime() << std::endl;
		for (int x = 0; x < group->GetNumMembers(); x ++) {
			AirCBSUnit *c = (AirCBSUnit*)group->GetMember(x);
			airtimeState cur;
			c->GetLocation(cur);
			std::cout << "\t" << x << ":" << cur << std::endl;
		}*/
	}


	/*
	if (recording)
	{
		static int index = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/anim-%d%d%d", index/100, (index/10)%10, index%10);
		SaveScreenshot(windowID, fname);
		printf("Saving '%s'\n", fname);
		index++;
	}*/
}

int MyCLHandler(char *argument[], int maxNumArgs)
{

	if(strcmp(argument[0], "-heuristic") == 0)
	{
                heuristic = true;
		return 1;
	}
	if(strcmp(argument[0], "-highsort") == 0)
	{
                highsort = true;
		return 1;
	}
	if(strcmp(argument[0], "-random") == 0)
	{
                randomalg = true;
		return 1;
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
	if(strcmp(argument[0], "-seed") == 0)
	{
		seed = atoi(argument[1]);	
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
