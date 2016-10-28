
#include "AirplaneLandingController.h"

// Add the feasible landing actions from a given location to the actions list
bool AirLandingController::GetLandingStates(airplaneState &current_state, std::vector<airplaneState> &goal_states) {
	if (states_ls[current_state].size() == 0)
		return false;
	else for (auto airstrip : states_ls[current_state]) goal_states.push_back(airstrip->goal_state);
	return true;
}

bool AirLandingController::GetTakeoffStates(airplaneState &current_state, std::vector<airplaneState> &launch_states) {
	if (states_ts[current_state].size() == 0)
		return false;
	else for (auto airstrip : states_ts[current_state]) for (s : airstrip->GetTakeoffLocations()) goal_states.push_back(s);
	return true;
}


// NOTE THAT NONE OF THIS IS SAFE TO CHANGES IN THE LANDING STRIP WHILE IT ACTUALLY IS 
// ADDED TO THE CODE

// Add a landing strip to the controller
void AirLandingController::AddLandingStrip(AirplaneEnvironment* parent_environment, AirLandingStrip* s) {
	// Add it to the controller
	this->owned_ls.push_back(s);
	for (auto landing_location : s->GetLandingLocations()) {
		this->states_ls[landing_location].push_back(s);
	}
	this->states_ts[s->goal_state].push_back(s);

	// Reset the parent environment normals to take into account this landing strip
	s->z_val = parent_enviornment->RedrawLSNormals(s->getX_min(), s->getX_max(), s->getY_min(), s->getY_max());
}

// Remove a landing strip from the controller
void AirLandingController::RemoveLandingStrip(AirLandingStrip* s) {
	this->owned_ls.erase(std::remove(this->owned_ls.begin(), this->owned_ls.end(), s), this->owned_ls.end());
	for (auto landing_location : s->GetLandingLocations()) {
		this->states_ls[landing_location].erase(std::remove(this->states_ls[landing_location].begin(),
															this->states_ls[landing_location].end(), s),
															this->owned_ls.end());
	}
}

// Draw the landing strip
void AirLandingController::GLDrawLandingStrips() {
	for (auto airstrip : owned_ls) airstrip.glDraw();
}

bool AirLandingController::GLIsLandingStrip(uint16_t x, uint16_t y) {
	for (auto ls : this->owned_ls) {
		if (ls->getX_min =< x && ls->getX_max >= x && ls->getY_min <= y && ls->getY_max >= y)
			return true;
	}
	return false;
}