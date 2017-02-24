#include "AirStates.h"


//const double PlatformState::SPEEDS[]={0.0,0.1,0.14,0.18,0.22,0.26}; // KMPS
const double PlatformState::SPEEDS[]={0.0,0.5556,0.7778,1.0,1.2222,1.4444}; // grid-units per second
// Assuming a cruise speed of 180 mps, our grid size is 180 meters, roughly 1/10 NMI.
// At cruise speed, we have a turn radius of about 7.5 grids (.75 NMI)
// Turn radius=knots/((rate of turn)*20*pi)
// R=(180mps=350kts)/(7.5 degrees/sec *20 pi)=0.7433NMI~=0.75 NMI=7.5 grids
// At minimum speed:
// R=(350*.5556)/(7.5*20*pi)=.413 NMI = ~4 grids.
// Thus, for our heuristics sake, we will not accomodate any goals less than 8 grid squares (.8NMI) away.
// If you're going to fly less than .8NMI (~1480 meters), you're out of luck.
const double PlatformState::SPEED_COST[]={0.0,1.15,1.05,1.0,1.10,1.20}; // cost factor by speed
const double PlatformState::TIMESTEP=1.0; // Seconds
const double PlatformState::angularPrecision=0.5; // Degrees
