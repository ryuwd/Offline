// A function to find the SimParticle that entered the volume
// (e.g. the gas in the straw) and is "responsible" for making a
// StepPointMC.  This aggregates showers inside a volume back to the
// particle that caused it.
//
// Andrei Gaponenko, 2014

#ifndef Mu2eUtilities_inc_particleEnteringG4Volume_hh
#define Mu2eUtilities_inc_particleEnteringG4Volume_hh

#include "Offline/MCDataProducts/inc/SimParticle.hh"
#include "Offline/MCDataProducts/inc/StepPointMC.hh"
#include "Offline/MCDataProducts/inc/StrawGasStep.hh"
#include "canvas/Persistency/Common/Ptr.h"

namespace mu2e{
class StepPointMC;
class StrawGasStep;
struct SimParticle;

  art::Ptr<mu2e::SimParticle> particleEnteringG4Volume(const StepPointMC& step);
  art::Ptr<mu2e::SimParticle> particleEnteringG4Volume(const StrawGasStep& step);
}

#endif /* Mu2eUtilities_inc_particleEnteringG4Volume_hh */
