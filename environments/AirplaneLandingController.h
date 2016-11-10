#ifndef A_Landing_h
#define A_Landing_h

#include "AirStates.h"
#include <unordered_map>

#if defined(__APPLE__)
	#include <OpenGL/gl.h>
#else
	#include <gl.h>
#endif

class AirplaneEnvironment;

class AirLandingStrip {
	
	private:
 		

	public:
	    //std::vector<agent> Stored_Agents;
 		airplaneState goal_state;
 		std::vector<airplaneState> landing_targets;
 		std::vector<airplaneState> launch_locations;
 		uint16_t x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_val = 0;

	 	// Borders on the world and then the launch_state, the landing state and the goal state
		AirLandingStrip(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2, airplaneState lu_s, airplaneState la_s, airplaneState gs) {
			x_min = std::min<int>(x1,x2);
			x_max = std::max<int>(x1,x2);
			y_min = std::min<int>(y1,y2);
			y_max = std::max<int>(y1,y2);

			goal_state = gs;
			landing_targets.push_back(la_s);
			launch_locations.push_back(lu_s);
		}
		void AddLandingLocation(airplaneState as) {landing_targets.push_back(as);}
		void AddLaunchLocation(airplaneState lu_s) {launch_locations.push_back(lu_s);}

		const std::vector<airplaneState>& GetLandingLocations() {return landing_targets;}
		const std::vector<airplaneState>& GetTakeoffLocations() {return launch_locations;}
		const airplaneState& GetGoal() {return goal_state;}		
		
		uint16_t getX_min() {return x_min;}
		uint16_t getX_max() {return x_max;}
		uint16_t getY_min() {return y_min;}
		uint16_t getY_max() {return y_max;}
		
		void glDraw() {
			float width = 80;
			float xval = (x_max-x_min)/2.0f + x_min;
		    glLineStipple(1, 0x00FF);
		    glLineWidth(100);
		    glEnable(GL_LINE_STIPPLE);
		    glDisable(GL_LIGHTING);
		    glBegin(GL_LINES);
		      glVertex3f((xval-width/2.0)/(width/2.0),(y_min-width/2.0)/(width/2.0) ,-4.0*z_val/(255.0*80));
		      glVertex3f((xval-width/2.0)/(width/2.0),(y_max-width/2.0)/(width/2.0) ,-4.0*z_val/(255.0*80));
		    glEnd();
		    glEnable(GL_LIGHTING);
		    glLineStipple(0, 0xFFFF);
		    glLineWidth(1);
		}
 };

class AirLandingController {
	private:
		std::vector<AirLandingStrip*> owned_ls;
		std::unordered_map<airplaneState, std::vector<AirLandingStrip*>> states_ls;
		std::unordered_map<airplaneState, std::vector<AirLandingStrip*>> states_ts;
	public:
		AirLandingController() {}
		bool GetLandingStates(const airplaneState &current_state, std::vector<airplaneState> &goal_state);
		bool GetTakeoffStates(const airplaneState &current_state, std::vector<airplaneState> &launch_states);
	
		void AddLandingStrip(AirplaneEnvironment* parent_environment, AirLandingStrip *s);
		void RemoveLandingStrip(AirLandingStrip *s);

		// Ticket handler - we can control landing based on a ticket system with the avilability of a landing location

		void GLDrawLandingStrips(); // Draw the alnding strips with openGL. 
		bool GLIsLandingStrip(uint16_t x, uint16_t y); // Determine if the land below should be gray
};


#endif /* Landing_Controller */