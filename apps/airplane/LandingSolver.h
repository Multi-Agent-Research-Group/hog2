// We need to solve the landing problem here
// Steps - generate the optimal paths for each agent, and then start a random query processArgs

#ifndef __hog2_glut__LandingSolver__
#define __hog2_glut__LandingSolver__

#include "AirplaneCBSUnits.h" // Pull the imports from AirplaneCBSUnits header
#include "AirStates.h"

struct AirLandingPlan {
	AirLandingPlan(){}
	std::vector< std::vector<airtimeState> > paths;
	std::vector< airtimeState > goals;
	std::vector<unsigned> waits;
};

class AirLandingGroup : public UnitGroup<airtimeState, airplaneAction, AirplaneConstrainedEnvironment>
{
public:
	AirLandingGroup(std::vector<EnvironmentContainer> const&);
	bool MakeMove(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, 
				  SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si, airplaneAction& a);
	void UpdateLocation(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u, AirplaneConstrainedEnvironment *e, 
						airtimeState &loc, bool success, SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *si);
	void AddUnit(Unit<airtimeState, airplaneAction, AirplaneConstrainedEnvironment> *u);
	
	void OpenGLDraw(const AirplaneConstrainedEnvironment *, const SimulationInfo<airtimeState,airplaneAction,AirplaneConstrainedEnvironment> *)  const;
	double getTime() {return time;}
	bool donePlanning() {return planFinished;}
	
    void DoPlanningStep(bool gui=true);

	std::vector<AirCBSTreeNode> tree;
    void processSolution(double);

private:    

    AirLandingPlan final_plan;

    unsigned HasConflict(std::vector<airtimeState> const& a, std::vector<airtimeState> const& b, int x, int y, airConflict &c1, airConflict &c2, bool update, bool verbose=false);
	unsigned FindFirstConflict(AirLandingPlan const& location, airConflict &c1, airConflict &c2);
	
	bool planFinished;

	/* Code for dealing with multiple environments */
	std::vector<EnvironmentContainer> environments;
	EnvironmentContainer* currentEnvironment;

	void SetEnvironment(unsigned);
   
	TemplateAStar<airtimeState, airplaneAction, AirplaneConstrainedEnvironment, AStarOpenClosed<airtimeState, RandomTieBreaking<airtimeState> > > astar;

	uint TOTAL_EXPANSIONS = 0;
    double time;

};

#endif 