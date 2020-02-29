//
// Example of accessing fitted tracks via a KalRepPtrCollection.
//
// $Id: ReadPtr_module.cc,v 1.2 2014/04/18 20:17:03 kutschke Exp $
// $Author: kutschke $
// $Date: 2014/04/18 20:17:03 $
//
// Original author Rob Kutschke
//

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "BTrk/TrkBase/TrkErrCode.hh"                       // for TrkErrCode
#include "CLHEP/Vector/ThreeVector.h"                       // for Hep3Vector

#include "TH1.h"                                            // for TH1D
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr
#include "canvas/Persistency/Common/detail/aggregate.h"     // for CLHEP
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

// Need this for the BaBar headers.
using namespace CLHEP;

#include <exception>                                 // for exception
// C++ includes.
#include <iostream>                                         // for std
#include <string>                                           // for string
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <vector>                                           // for vector

#include "BTrk/KalmanTrack/KalRep.hh"                       // for KalRep
#include "BTrk/TrkBase/TrkParticle.hh"                      // for TrkParticle
// mu2e tracking
#include "RecoDataProducts/inc/TrkFitDirection.hh"          // for TrkFitDir...
#include "RecoDataProducts/inc/KalRepPtrCollection.hh"      // for KalRepPtr...

using namespace std;

namespace mu2e {

  class ReadPtr : public art::EDAnalyzer {
  public:

    explicit ReadPtr(fhicl::ParameterSet const& pset);

    void analyze( art::Event const& e) override;
    void beginJob() override;

  private:

    // Information about the data product that contains the fitted tracks.
    TrkParticle     _tpart;
    TrkFitDirection _fdir;
    std::string     _instanceName;
    art::InputTag   _inputTag;

    TH1D* _hNTracks;
    TH1D* _hFitStatus;
    TH1D* _hP;

  };
}   // end namespace mu2e

mu2e::ReadPtr::ReadPtr(fhicl::ParameterSet const& pset):
  EDAnalyzer(pset),
  _tpart((TrkParticle::type)(pset.get<int>("fitparticle"))),
  _fdir((TrkFitDirection::FitDirection)(pset.get<int>("fitdirection"))),
  _instanceName( _fdir.name() + _tpart.name()),
  _inputTag( pset.get<string>("inputModuleLabel"), _instanceName)
{}

void mu2e::ReadPtr::beginJob(){
  art::ServiceHandle<art::TFileService> tfs;
  _hNTracks   = tfs->make<TH1D>( "hNTracks",   "Number of tracks per Event",  5,  0.,   5.);
  _hFitStatus = tfs->make<TH1D>( "hFitStatus", "Fit Status",                  10, 0.,  10.);
  _hP         = tfs->make<TH1D>( "hP",         "Reconstructed Momentum",     30, 80., 110.);
}

void mu2e::ReadPtr::analyze(art::Event const& event) {

  auto trksHandle    = event.getValidHandle<KalRepPtrCollection>(_inputTag);
  auto const& tracks = *trksHandle;

  _hNTracks->Fill(tracks.size());

  for ( auto const& trk : tracks ){

    KalRep const& krep = *trk;

    int stat = krep.fitStatus().success();
    _hFitStatus->Fill(stat);

    if ( stat != 1 ) continue;

    double s0   = krep.startValidRange();
    _hP->Fill( krep.momentum(s0).mag() ) ;


  }

}


DEFINE_ART_MODULE(mu2e::ReadPtr);
