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
//#include "TemplateAStar.h"
//#include "MapSectorAbstraction.h"
//#include "DirectionalPlanner.h"
#include "ScenarioLoader.h"
#include "AirplaneSimple.h"
#include "AirplaneHighway.h"
#include "AirplaneConstrained.h"
#include "AirplaneCBSUnits.h"

#include <thread>

bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;
int mapSize = 128;
bool recording = false;
double simTime = 0;
double stepsPerFrame = 1.0/100.0;
double frameIncrement = 1.0/10000.0;
std::vector<airtimeState> thePath;

int num_airplanes = 5;
bool use_rairspace = false;
bool use_wait = false;

bool paused = false;



//DirectionalPlanner *dp = 0;
//MapSectorAbstraction *quad = 0;
//std::vector<DirectionSimulation *> unitSims;
//typedef GenericSearchUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> DirSearchUnit;
//typedef RandomUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> RandomDirUnit;
//typedef GenericPatrolUnit<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> DirPatrolUnit;

//void RunBatchTest(char *file, model mm, heuristicType heuristic, int length);

/*
AirplaneEnvironment ae;
airplaneState s;
airplaneState s2;
airplaneState s3;
airplaneState s4;
airplaneState tmp;
 */


int main(int argc, char* argv[])
{
	if (argc > 1) {
		num_airplanes = atoi(argv[1]);
	}

	InstallHandlers();
	RunHOGGUI(argc, argv);
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

AirplaneConstrainedEnvironment *ace = 0;
AirplaneConstrainedEnvironment *aces = 0;
UnitSimulation<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *sim = 0;
AirCBSGroup* group = 0;
AirCBSUnit* u1 = 0;
AirCBSUnit* u2 = 0;
AirCBSUnit* u3 = 0;
AirCBSUnit* u4 = 0;
AirCBSUnit* u5 = 0;
AirCBSUnit* u6 = 0;
airtimeState s1, s2, s3, s4, s5, s6, g1, g2, g3, g4, g5, g6;


void InitSim(){
    AirplaneEnvironment* ae = new AirplaneEnvironment();
    ae->loadPerimeterDB();
    AirplaneEnvironment* ase = new AirplaneSimpleEnvironment();
    AirplaneEnvironment* ahe = new AirplaneHighwayEnvironment();
    ase->loadPerimeterDB();
    ahe->loadPerimeterDB();
	ace = new AirplaneConstrainedEnvironment(ae);
	aces = new AirplaneConstrainedEnvironment(ase);
	//aces = new AirplaneConstrainedEnvironment(ahe);



	sim = new UnitSimulation<airtimeState, airplaneAction, AirplaneConstrainedEnvironment>(ace);
	sim->SetStepType(kLockStep);
	//group = new AirCBSGroup(ace,ace,4);
    // TODO: Have it use the simple environment and switch to the complex one
    //       after too many conflicts
	group = new AirCBSGroup(ace,ace,10000, use_rairspace, use_wait); // Changed to 10,000 expansions from number of conflicts in the tree
	
	sim->AddUnitGroup(group);

	
	// Updated so we're always testing the landing conditions
	// and forcing the airplane environment to be such that
	// we are inducing high clonflict areas.
	std::cout << "Adding " << num_airplanes << "planes." << std::endl;
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
			sim->AddUnit(unit); // Add the unit to the simulation
			std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;

		} else {
			airplaneState rs1(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
			airtimeState start(rs1, 0);

			// 0% total are landing intially
			if (true) {
				// Replan the node to a landing location
				airplaneState land(18, 23, 0, 0, 0, true);
				airtimeState goal(land, 0);

				AirCBSUnit* unit = new AirCBSUnit(start, goal);
				unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
				group->AddUnit(unit); // Add to the group
				sim->AddUnit(unit); // Add the unit to the simulation
				std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;

			} else {
				// Replan the node to a random location
				airplaneState rs(rand() % 70 + 5, rand() % 70 + 5, rand() % 7 + 11, rand() % 3 + 1, rand() % 8, false);
				airtimeState goal(rs, 0);
				
				AirCBSUnit* unit = new AirCBSUnit(start, goal);
				unit->SetColor(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0); // Each unit gets a random color
				group->AddUnit(unit); // Add to the group
				sim->AddUnit(unit); // Add the unit to the simulation
				std::cout << "Set unit " << i << " directive from " << start << " to " << goal << " rough heading: " << (unsigned)start.headingTo(goal) << std::endl;
			}

		}
			



	}

	std::cout << "cmdflags:" << use_rairspace << use_wait << std::endl;

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

	if(strcmp(argument[0], "-rairspace") == 0)
	{
		use_rairspace = true;
		return 1;
	}
	if(strcmp(argument[0], "-uwait") == 0)
	{
		use_wait = true;
		return 1;
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
//	TemplateAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> alg;
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
