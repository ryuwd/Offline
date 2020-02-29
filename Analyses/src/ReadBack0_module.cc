//
// An EDAnalyzer module that serves as a first introduction to Mu2e software.
// Make a few histograms about tracker and calorimeter information found in the event.
//
// $Id: ReadBack0_module.cc,v 1.7 2013/10/21 20:44:04 genser Exp $
// $Author: genser $
// $Date: 2013/10/21 20:44:04 $
//
// Original author Rob Kutschke
//

#include <exception>                                 // for exception
#include <stddef.h>                                         // for size_t
// C++ includes.
#include <iostream>                                         // for std
#include <string>                                           // for allocator
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
// Mu2e includes.
#include "GeometryService/inc/GeomHandle.hh"                // for GeomHandle
#include "TrackerGeom/inc/Tracker.hh"                       // for Tracker
#include "TNtuple.h"                                        // for TNtuple
// Other includes.
#include "CLHEP/Units/SystemOfUnits.h"                      // for keV
#include "CLHEP/Vector/ThreeVector.h"                       // for Hep3Vector

#include "DataProducts/inc/StrawId.hh"                      // for StrawId
#include "MCDataProducts/inc/StepPointMC.hh"                // for StepPoint...
#include "TH1.h"                                            // for TH1F
#include "TrackerGeom/inc/Straw.hh"                         // for Straw
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

using namespace std;

namespace mu2e {

  class ReadBack0 : public art::EDAnalyzer {
  public:

    explicit ReadBack0(fhicl::ParameterSet const& pset);
    virtual ~ReadBack0() { }

    // The framework calls this at the start of the job.
    virtual void beginJob();

    // The framework calls this for each event.
    void analyze(const art::Event& e);

  private:

    // Module label of the g4 module that made the hits.
    std::string _g4ModuleLabel;

    // Name of the StepPointMC collection for the tracker.
    std::string _trackerStepPoints;

    // Cut on the minimum energy.
    double _minimumEnergy;

    // Pointers to histograms, ntuples.
    TH1F* _hStrawEDep;
    TH1F* _hNstraw;
    TH1F* _hCrystalEDep;
    TH1F* _hNcrystal;

    TNtuple* _ntup;

    // Partition the per event work into two steps.
    void doTracker(const art::Event& event);
    void doCalorimeter(const art::Event& event);

  };

  ReadBack0::ReadBack0(fhicl::ParameterSet const& pset) :
    art::EDAnalyzer(pset),

    // These will become run time parameters in a later example.
    _g4ModuleLabel("g4run"),
    _trackerStepPoints("tracker"),

    // Run time parameters
    _minimumEnergy(pset.get<double>("minimumEnergy")),

    // Histograms
    _hStrawEDep(0),
    _hNstraw(0),
    _hCrystalEDep(0),
    _hNcrystal(0),
    _ntup(0){
  }

  // At the start of the job, book histograms.
  void ReadBack0::beginJob(){

    // Get a handle to the TFile service.
    art::ServiceHandle<art::TFileService> tfs;

    // Create some histograms for the tracker.
    _hStrawEDep    = tfs->make<TH1F>( "hStrawEDep", "Energy Deposited in straw;(keV)", 100,  0.,   10. );
    _hNstraw       = tfs->make<TH1F>( "hNstraw",    "Tracker StepPointMC per Event",   100,  0.,  200. );

    // Create some histograms for the calorimeter.
    _hCrystalEDep  = tfs->make<TH1F>( "hCrystalEDep", "Total energy deposition in calorimeter;(MeV)", 1200, 0.,  1200. );
    _hNcrystal     = tfs->make<TH1F>( "hNcrystal",    "Number of hit crystals in calorimeter",          50, 0.,    50. );

    // Create an ntuple.
    _ntup = tfs->make<TNtuple>( "ntup", "Hit ntuple", "x:y:z:t:plane:panel:layer:edep");

  } // end beginJob

  // For each event, look at tracker hits and calorimeter hits.
  void ReadBack0::analyze(const art::Event& event) {

    doTracker(event);
    doCalorimeter(event);

  }

  void ReadBack0::doCalorimeter(const art::Event& event) {

  }

  void ReadBack0::doTracker(const art::Event& event){

    // Get a handle to the hits created by G4.
    art::Handle<StepPointMCCollection> hitsHandle;
    event.getByLabel("g4run","tracker",hitsHandle);
    StepPointMCCollection const& hits = *hitsHandle;

    // Get a handle to the Tracker geometry.
    GeomHandle<Tracker> tracker;

    // Fill histogram with number of hits per event.
    _hNstraw->Fill(hits.size());

    // ntuple buffer.
    float nt[_ntup->GetNvar()];

    // Loop over all hits.
    for ( size_t i=0; i<hits.size(); ++i ){

      // Alias, used for readability.
      const StepPointMC& hit = hits[i];

      // Skip hits with low pulse height.
      if ( hit.eDep() < _minimumEnergy ) continue;

      // Position of the hit.
      const CLHEP::Hep3Vector& pos = hit.position();

      // Information about the straw.
      const Straw& straw = tracker->getStraw( hit.strawId() );

      // Fill a histogram.
      _hStrawEDep->Fill(hit.eDep()/CLHEP::keV);

      // Fill the ntuple.
      nt[0]  = pos.x();
      nt[1]  = pos.y();
      nt[2]  = pos.z();
      nt[3]  = hit.time();
      nt[4]  = straw.id().getPlane();
      nt[5]  = straw.id().getPanel();
      nt[6]  = straw.id().getLayer();
      nt[7]  = hit.eDep()/CLHEP::keV;

      _ntup->Fill(nt);

    } // end loop over hits.

  } // end doTracker

}  // end namespace mu2e

// Part of the magic that makes this class a module.
// create an instance of the module.  It also registers
using mu2e::ReadBack0;
DEFINE_ART_MODULE(ReadBack0);
