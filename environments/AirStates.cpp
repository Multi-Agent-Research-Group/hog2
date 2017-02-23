#include "AirStates.h"


//const double PlatformState::SPEEDS[]={0.0,0.1,0.14,0.18,0.22,0.26}; // KMPS
const double PlatformState::SPEEDS[]={0.0,0.05556*3.,0.07778*3.,0.10*3.,0.12222*3.,0.14444*3.}; // grid-units per second
const double PlatformState::SPEED_COST[]={0.0,1.15,1.05,1.0,1.10,1.20}; // cost factor by speed
const double PlatformState::TIMESTEP=1.0; // Seconds
const double PlatformState::angularPrecision=0.5; // Degrees
