//
//
// $Id: ReadMCTrajectories_module.cc,v 1.2 2014/01/18 04:04:50 kutschke Exp $
// $Author: kutschke $
// $Date: 2014/01/18 04:04:50 $
//
// Contact person Rob Kutschke
//

#include <exception>                                 // for exception
// C++ includes.
#include <iostream>                                         // for operator<<
#include <string>                                           // for string
#include <map>                                              // for map
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <utility>                                          // for pair
#include <vector>                                           // for vector

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "MCDataProducts/inc/MCTrajectoryCollection.hh"     // for MCTraject...
#include "TNtuple.h"                                        // for TNtuple
#include "MCDataProducts/inc/MCTrajectory.hh"               // for MCTrajectory
#include "MCDataProducts/inc/MCTrajectoryPoint.hh"          // for MCTraject...
#include "TH1.h"                                            // for TH1F
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Common/Ptr.h"                  // for operator<<
#include "canvas/Persistency/Provenance/EventID.h"          // for EventID
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

using namespace std;

namespace mu2e {

  class ReadMCTrajectories : public art::EDAnalyzer {
  public:

    explicit ReadMCTrajectories(fhicl::ParameterSet const& pset);

    virtual void beginJob();
    void analyze(const art::Event& e);

  private:

    art::InputTag trajectoriesTag_;

    int maxPrint_;
    int nPrint_;

    TH1F*    nTraj_;
    TH1F*    nPoints1_;
    TH1F*    nPoints2_;
    TNtuple* ntup_;

  };

  ReadMCTrajectories::ReadMCTrajectories(fhicl::ParameterSet const& pset) :
    art::EDAnalyzer(pset),
    trajectoriesTag_(pset.get<std::string>("trajectoriesTag")),
    maxPrint_(pset.get<int>("maxPrint",20)),
    nPrint_(0),
    nTraj_(nullptr),
    nPoints1_(nullptr),
    nPoints2_(nullptr),
    ntup_(nullptr){
  }

  void ReadMCTrajectories::beginJob(){

    art::ServiceHandle<art::TFileService> tfs;
    nTraj_   = tfs->make<TH1F>("nTraj",     "Number of Stored Trajectories per Event",  20, 0.,   20.);
    nPoints1_ = tfs->make<TH1F>("nPoints1", "Number of Points per Stored Trajectory",   20, 0.,   20.);
    nPoints2_ = tfs->make<TH1F>("nPoints2", "Number of Points per Stored Trajectory",  100, 0., 2000.);
    ntup_    = tfs->make<TNtuple>( "ntup",  "Hit ntuple", "evt:x:y:z:t");

  }


  void ReadMCTrajectories::analyze(const art::Event& event) {

    bool doPrint = ( ++nPrint_ <= maxPrint_ );

    auto trajectories = event.getValidHandle<MCTrajectoryCollection>(trajectoriesTag_);
    nTraj_->Fill(trajectories->size());

    float nt[5];

    for ( auto const& i : *trajectories ){
      MCTrajectory const& traj = i.second;
      if ( doPrint ) {
        cout << "Trajectory: "
             << event.id().event() << " "
             << i.first   << " "
             << traj.sim() << " "
             << traj.size()
             << endl;
      }
      nPoints1_->Fill( traj.size() );
      nPoints2_->Fill( traj.size() );
      nt[0] = event.id().event();
      for ( auto const& pos : traj.points() ){
        nt[1] = pos.x();
        nt[2] = pos.y();
        nt[3] = pos.z();
        nt[4] = pos.t();
        ntup_->Fill(nt);
      }
    }


  }

}  // end namespace mu2e

DEFINE_ART_MODULE(mu2e::ReadMCTrajectories);
