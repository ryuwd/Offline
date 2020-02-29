// static member function to determine proper lifetime of particles
// from multi-staged simulations.  Requires the SimParticle instance
// as the first argument, and then a set of StepPointMC collections
// corresponding to the various stages of the simulation.
//
// Original authors: Kyle Knoepfel
//                   Andrei Gaponenko

#ifndef Mu2eUtilities_SimParticleGetTau_hh
#define Mu2eUtilities_SimParticleGetTau_hh

// C++ includes
#include <vector>                                               // for vector

// Mu2e includes
#include "GlobalConstantsService/inc/GlobalConstantsHandle.hh"  // for Globa...
#include "GlobalConstantsService/inc/PhysicsParams.hh"          // for Physi...
#include "MCDataProducts/inc/StepPointMC.hh"                    // for StepP...

namespace art {
template <typename T> class Ptr;
}  // namespace art

namespace mu2e {

  class SimParticle;

  class SimParticleGetTau {
  public:
    typedef std::vector<StepPointMCCollection> VspMC;

    static double calculate( const art::Ptr<SimParticle>& p, 
                             const VspMC& hitColls, 
                             const std::vector<int>& decayOffCodes = std::vector<int>(),
                             const PhysicsParams& gc = *GlobalConstantsHandle<PhysicsParams>());

    static double calculate( const StepPointMC& sp,
                             const VspMC& hitColls,
                             const std::vector<int>& decayOffCodes = std::vector<int>(),
                             const PhysicsParams& gc = *GlobalConstantsHandle<PhysicsParams>());
  private:
    static double getMultiStageTau( const art::Ptr<SimParticle>& sp,                                            
                                    const VspMC& hitColls,                                            
                                    const std::vector<int>& decayOffCodes,
                                    const PhysicsParams& gc );
  };
}

#endif/*Mu2eUtilities_SimParticleGetTau_hh*/
