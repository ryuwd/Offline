//
// Plugin to read StepPoints in PS Vacuum and create ntuples
//
//  $Id: ReadPSVacuum_module.cc,v 1.2 2013/10/21 20:44:04 genser Exp $
//  $Author: genser $
//  $Date: 2013/10/21 20:44:04 $
//
// Original author Zhengyun You
//

#include <exception>                                 // for exception
#include <stddef.h>                                         // for size_t
#include <iostream>                                         // for operator<<
#include <string>                                           // for string
#include <algorithm>                                        // for max, copy
#include <memory>                                           // for unique_ptr
#include <set>                                              // for set
#include <typeinfo>                                         // for type_info
#include <vector>                                           // for vector

#include "CLHEP/Units/SystemOfUnits.h"                      // for keV
#include "TNtuple.h"                                        // for TNtuple
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "CLHEP/Vector/ThreeVector.h"                       // for Hep3Vector

#include "MCDataProducts/inc/SimParticle.hh"                // for SimPartic...
#include "MCDataProducts/inc/StepPointMC.hh"                // for StepPointMC
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Provenance/EventID.h"          // for EventID
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "cetlib/map_vector.h"                              // for map_vector
#include "fhiclcpp/coding.h"                                // for ps_sequen...
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace art {
class Run;
}  // namespace art

using namespace std;

using CLHEP::Hep3Vector;
using CLHEP::keV;

namespace mu2e {

  class ReadPSVacuum : public art::EDAnalyzer {
  public:

    typedef vector<int> Vint;
    typedef SimParticleCollection::key_type key_type;

    explicit ReadPSVacuum(fhicl::ParameterSet const& pset) :
      art::EDAnalyzer(pset),
      _psVacuumStepPoints(pset.get<string>("psVacuumStepPoints","PSVacuum")),
      _nAnalyzed(0),
      _maxPrint(pset.get<int>("maxPrint",0)),
      _ntpsVacuum(0),
      _generatorModuleLabel(pset.get<std::string>("generatorModuleLabel", "generate")),
      _g4ModuleLabel(pset.get<std::string>("g4ModuleLabel", "g4run"))
    {

      Vint const & pdg_ids = pset.get<Vint>("savePDG", Vint());
      if( pdg_ids.size()>0 ) {
        cout << "ReadPSVacuum: save following particle types in the ntuple: ";
        for( size_t i=0; i<pdg_ids.size(); ++i ) {
          pdg_save.insert(pdg_ids[i]);
          cout << pdg_ids[i] << ", ";
        }
        cout << endl;
      }

      nt = new float[1000];

    }

    virtual ~ReadPSVacuum() { }

    virtual void beginJob();
    virtual void beginRun(art::Run const&);

    void analyze(const art::Event& e);

  private:

    // Name of the VD and TVD StepPoint collections
    std::string  _psVacuumStepPoints;

    // Control printed output.
    int _nAnalyzed;
    int _maxPrint;

    TNtuple* _ntpsVacuum;

    float *nt; // Need this buffer to fill TTree ntpsVacuum

    // List of particles of interest for the particles ntuple
    set<int> pdg_save;

    // Label of the generator.
    std::string _generatorModuleLabel;

    // Module label of the g4 module that made the hits.
    std::string _g4ModuleLabel;

  };

  void ReadPSVacuum::beginJob(){

    // Get access to the TFile service.

    art::ServiceHandle<art::TFileService> tfs;

    _ntpsVacuum = tfs->make<TNtuple>( "ntpsVacuum", "PSVacuum ntuple",
                                "run:evt:trk:pdg:time:x:y:z:px:py:pz:"
                                "gtime");
  }

  void ReadPSVacuum::beginRun(art::Run const& run){

  }

  void ReadPSVacuum::analyze(const art::Event& event) {

    ++_nAnalyzed;

    // Ask the event to give us a "handle" to the requested hits.
    art::Handle<StepPointMCCollection> hits;
    event.getByLabel(_g4ModuleLabel,_psVacuumStepPoints,hits);

    art::Handle<SimParticleCollection> simParticles;
    event.getByLabel(_g4ModuleLabel, simParticles);
    bool haveSimPart = simParticles.isValid();
    if ( haveSimPart ) haveSimPart = !(simParticles->empty());

    // Loop over all VD hits.
    if( hits.isValid() ) for ( size_t i=0; i<hits->size(); ++i ){

      // Alias, used for readability.
      const StepPointMC& hit = (*hits)[i];

      // Get the hit information.
      const CLHEP::Hep3Vector& pos = hit.position();
      const CLHEP::Hep3Vector& mom = hit.momentum();

      // Get track info
      key_type trackId = hit.trackId();
      int pdgId = 0;
      if ( haveSimPart ){
        if( !simParticles->has(trackId) ) {
          pdgId = 0;
        } else {
          SimParticle const& sim = simParticles->at(trackId);
          pdgId = sim.pdgId();
        }
      }

      // Fill the ntuple.
      nt[0]  = event.id().run();
      nt[1]  = event.id().event();
      nt[2]  = trackId.asInt();
      nt[3]  = pdgId;
      nt[4]  = hit.time();
      nt[5]  = pos.x();
      nt[6]  = pos.y();
      nt[7]  = pos.z();
      nt[8]  = mom.x();
      nt[9]  = mom.y();
      nt[10] = mom.z();
      nt[11] = hit.properTime();

      _ntpsVacuum->Fill(nt);
      if ( _nAnalyzed < _maxPrint){
        cout << "VD hit: "
             << event.id().run()   << " | "
             << event.id().event() << " | "
             << hit.volumeId()     << " "
             << pdgId              << " | "
             << hit.time()         << " "
             << mom.mag()
             << endl;

      }

    } // end loop over hits.

  }

}  // end namespace mu2e

using mu2e::ReadPSVacuum;
DEFINE_ART_MODULE(ReadPSVacuum);
