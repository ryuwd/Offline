//
// Check self consistency of all SimParticleCollections in the event.
//
// $Id: SimParticleCheck00_module.cc,v 1.3 2013/10/21 20:44:04 genser Exp $
// $Author: genser $
// $Date: 2013/10/21 20:44:04 $
//
// Original author Rob Kutschke
//

// C++ includes.
#include <iostream>                                         // for operator<<
#include <string>                                           // for string
#include <set>                                              // for set, set<...
#include <algorithm>                                        // for max
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <utility>                                          // for pair
#include <vector>                                           // for vector

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "MCDataProducts/inc/ProcessCode.hh"                // for ProcessCode
#include "MCDataProducts/inc/SimParticle.hh"                // for SimParticle
#include "TH1.h"                                            // for TH1F
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "cetlib/map_vector.h"                              // for map_vecto...
#include "cetlib_except/exception.h"                        // for operator<<
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace fhicl {
class ParameterSet;
}  // namespace fhicl

using namespace std;

namespace mu2e {

  class SimParticleCheck00 : public art::EDAnalyzer {
  public:

    explicit SimParticleCheck00(fhicl::ParameterSet const& pset);

    virtual void beginJob( );
    virtual void endJob  ( );

    virtual void analyze ( const art::Event& event);


  private:

    int nSims;
    int nBornBeforeParent;
    set<ProcessCode> badCodes;

    TH1F* _hNCollections;
    TH1F* _hDeltaTAll;
    TH1F* _hDeltaTBad;

  };

  SimParticleCheck00::SimParticleCheck00(fhicl::ParameterSet const& pset) :
    art::EDAnalyzer(pset),

    nSims(0),
    nBornBeforeParent(0),
    badCodes(),

    // Histograms
    _hNCollections(0),
    _hDeltaTAll(0),
    _hDeltaTBad(0)
  {}

  void SimParticleCheck00::beginJob(){

    art::ServiceHandle<art::TFileService> tfs;

    _hNCollections = tfs->make<TH1F>( "hNCollections", "Number of SimParticle Collections",    20,      0.,    20. );
    _hDeltaTAll    = tfs->make<TH1F>( "hDeltaTAll",    "Delta Time (Particle-Parent); [ns]",  200,  -3000.,  3000. );
    _hDeltaTBad    = tfs->make<TH1F>( "hDeltaTBad",    "Delta Time (Particle-Parent); [ns]",  200,  -3000.,     0. );

  } // end beginJob

  void SimParticleCheck00::analyze(const art::Event& event) {
    typedef std::vector< art::Handle<SimParticleCollection> > HandleVector;
    HandleVector allSims;
    event.getManyByType(allSims);

    _hNCollections->Fill(allSims.size());

    // See if any children are created before the parent!
    for ( HandleVector::const_iterator i=allSims.begin(), e=allSims.end(); i != e; ++i ){
      const SimParticleCollection& sims(**i);
      nSims += sims.size();
      for ( SimParticleCollection::const_iterator j=sims.begin(), je=sims.end(); j != je; ++j ){
        const SimParticle& sim = j->second;
        if ( sim.isSecondary() ){
          double t  = sim.startGlobalTime();
          double t0 = sim.parent()->startGlobalTime();
          double dt = t-t0;
          _hDeltaTAll->Fill(dt);
          if ( t < t0 ){
            ++nBornBeforeParent;
            badCodes.insert(sim.creationCode());
            _hDeltaTBad->Fill(dt);
          }
        }
      }
    }

  } // end analyze

  void SimParticleCheck00::endJob(){

    cout << "\nNumber of SimParticles in this file:                      " << nSims             << endl;
    cout << "Number of particles born before their parents in this file: " << nBornBeforeParent << endl;
    if ( nBornBeforeParent == 0 ) return;

    double ratio = ( nSims > 0 ) ? double(nBornBeforeParent)/double(nSims) : 0.;
    cout << "Ratio:                                                      " << ratio << endl;

    cout << "\nNumber of bad process codes that give daughters born before parent: " << badCodes.size() << endl;

    for ( set<ProcessCode>::const_iterator i=badCodes.begin(), e=badCodes.end(); i!=e; ++i){
      cout << "ProcessCode for SimParticle born before parent: " << *i << endl;
    }
  }

}  // end namespace mu2e

DEFINE_ART_MODULE(mu2e::SimParticleCheck00);
