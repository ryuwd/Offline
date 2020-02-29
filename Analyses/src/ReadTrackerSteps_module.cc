//
// example Plugin to read Tracker Step data and create ntuples
//
// Original author KLG
//

#include <exception>                                 // for exception
#include <stddef.h>                                         // for size_t
//#include <cmath>
#include <iostream>                                         // for operator<<
#include <string>                                           // for string
#include <iomanip>                                          // for operator<<
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

// CLHEP includes
#include "CLHEP/Units/SystemOfUnits.h"                      // for keV
#include "TrackerGeom/inc/Tracker.hh"                       // for Tracker
#include "TNtuple.h"                                        // for TNtuple
// Framework includes
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "messagefacility/MessageLogger/MessageLogger.h"    // for LogError
#include "CLHEP/Vector/ThreeVector.h"                       // for Hep3Vector

#include "GeometryService/inc/GeometryService.hh"           // for GeometryS...
#include "MCDataProducts/inc/SimParticle.hh"                // for SimPartic...
#include "MCDataProducts/inc/StepPointMC.hh"                // for StepPointMC
#include "TH1.h"                                            // for TH1F
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Provenance/EventID.h"          // for EventID
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "cetlib/map_vector.h"                              // for map_vector
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

using namespace std;

using CLHEP::Hep3Vector;
using CLHEP::keV;

namespace mu2e {

  class ReadTrackerSteps : public art::EDAnalyzer {
  public:

    explicit ReadTrackerSteps(fhicl::ParameterSet const& pset);
    virtual ~ReadTrackerSteps() {}

    virtual void beginJob();
    void analyze(const art::Event& e);

  private:

    // debug output llevel
    int _diagLevel;

   // Module label of the module that made the hits.
    std::string _hitMakerModuleLabel;

    // Control printed output.
    int _nAnalyzed;
    int _maxFullPrint;

    // Pointers to histograms, ntuples.

    TH1F*    _hNtset;
    TH1F*    _hNtsetH;

    TNtuple* _nttts;

    // Name of the TS StepPoint collection
    std::string  _tsStepPoints;

  };

  ReadTrackerSteps::ReadTrackerSteps(fhicl::ParameterSet const& pset) :
    art::EDAnalyzer(pset),
    // Run time parameters
    _diagLevel(pset.get<int>("diagLevel",0)),
    _hitMakerModuleLabel(pset.get<string>("hitMakerModuleLabel","g4run")),
    _nAnalyzed(0),
    _maxFullPrint(pset.get<int>("maxFullPrint",5)),
    _hNtset(0),
    _hNtsetH(0),
    _nttts(0),
    _tsStepPoints(pset.get<string>("tsStepPoints","trackerDS"))
  {}



  void ReadTrackerSteps::beginJob(){

    // Get access to the TFile service.

    art::ServiceHandle<art::TFileService> tfs;

    _hNtset  = tfs->make<TH1F>( "hNtset ",
                                 "Number/ID of the detector with step",
                                 50,  0., 50. );

    _hNtsetH = tfs->make<TH1F>( "hNtsetH",
                                 "Number of steps",
                                 50,  0., 50. );

    // Create an ntuple.
    _nttts = tfs->make<TNtuple>( "nttts",
                                  "Tracker steps ntuple",
                                  "evt:trk:sid:pdg:time:x:y:z:px:py:pz:"
                                  "g4bl_time");
  }

  void ReadTrackerSteps::analyze(const art::Event& event) {

    ++_nAnalyzed;

    if (_diagLevel>1 ) {
      cout << "ReadTrackerSteps::" << __func__
           << setw(4) << " called for event "
           << event.id().event()
           << " hitMakerModuleLabel "
           << _hitMakerModuleLabel
           << " tsStepPoints "
           << _tsStepPoints
           << endl;
    }

    art::ServiceHandle<GeometryService> geom;

    if (!geom->hasElement<Tracker>())
      {
        mf::LogError("Geom")
          << "Skipping ReadTrackerSteps::analyze due to lack of tracker\n";
        return;
      }

    // Access detector geometry information

    // Get a handle to the hits created by G4.
    art::Handle<StepPointMCCollection> hitsHandle;

    event.getByLabel(_hitMakerModuleLabel, _tsStepPoints, hitsHandle);

    if (!hitsHandle.isValid() ) {

      mf::LogError("Hits")
        << " Skipping ReadTrackerSteps::analyze due to problems with hits\n";
      return;
    }

    StepPointMCCollection const& hits = *hitsHandle;

    unsigned int const nhits = hits.size();

    if (_diagLevel>0 && nhits>0 ) {
      cout << "ReadTrackerSteps::" << __func__
           << " Number of TS hits: " <<setw(4)
           << nhits
           << endl;
    }

    art::Handle<SimParticleCollection> simParticles;
    event.getByLabel(_hitMakerModuleLabel, simParticles);
    bool haveSimPart = simParticles.isValid();
    if ( haveSimPart ) haveSimPart = !(simParticles->empty());

    // Fill histograms & ntuple

    // Fill histogram with number of hits per event.
    _hNtsetH->Fill(nhits);

    float nt[_nttts->GetNvar()];

    // Loop over all TS hits.
    for ( size_t i=0; i<nhits; ++i ){

      // Alias, used for readability.
      const StepPointMC& hit = (hits)[i];

      // Get the hit information.

      int did = hit.volumeId();

      // Fill histogram with the detector numbers
      _hNtset->Fill(did);

      const CLHEP::Hep3Vector& pos = hit.position();
      const CLHEP::Hep3Vector& mom = hit.momentum();

      // Get track info
      SimParticleCollection::key_type trackId = hit.trackId();
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
      nt[0]  = event.id().event();
      nt[1]  = trackId.asInt();
      nt[2]  = did;
      nt[3]  = pdgId;
      nt[4]  = hit.time();
      nt[5]  = pos.x();
      nt[6]  = pos.y();
      nt[7]  = pos.z();
      nt[8]  = mom.x();
      nt[9]  = mom.y();
      nt[10] = mom.z();
      nt[11] = hit.properTime();

      _nttts->Fill(nt);

      if ( _nAnalyzed < _maxFullPrint){
        cout <<  "ReadTrackerSteps::" << __func__
             << ": TS hit: "
             << setw(8) << event.id().event() << " | "
             << setw(4) << hit.volumeId()     << " | "
             << setw(6) << pdgId              << " | "
             << setw(8) << hit.time()         << " | "
             << setw(8) << mom.mag()          << " | "
             << pos
             << endl;

      }

    } // end loop over hits.

  }

}  // end namespace mu2e

using mu2e::ReadTrackerSteps;
DEFINE_ART_MODULE(ReadTrackerSteps);
