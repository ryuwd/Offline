//
// Example of how to read a KalRepCollection and create a matching KalRepPtrCollection.
// This can be used as a model for writing a module that merges two KalRepCollections.
//
// $Id: PtrTest_module.cc,v 1.2 2014/04/18 20:17:03 kutschke Exp $
// $Author: kutschke $
// $Date: 2014/04/18 20:17:03 $
//
// Original author Rob Kutschke
//

// Framework includes.
#include "art/Framework/Core/EDProducer.h"                  // for EDProducer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "GeneralUtilities/inc/OwningPointerCollection.hh"  // for OwningPoi...
#include "canvas/Persistency/Common/detail/aggregate.h"     // for CLHEP
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

// Need this for the BaBar headers.
using namespace CLHEP;

#include <exception>                                 // for exception
#include <stddef.h>                                         // for size_t
// C++ includes.
#include <iostream>                                         // for std
#include <string>                                           // for string
#include <algorithm>                                        // for max
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <utility>                                          // for move
#include <vector>                                           // for vector

#include "BTrk/TrkBase/TrkParticle.hh"                      // for TrkParticle
// mu2e tracking
#include "RecoDataProducts/inc/TrkFitDirection.hh"          // for TrkFitDir...
// This is fragile and needs to be last until CLHEP is
// properly qualified and included in the BaBar classes.
#include "RecoDataProducts/inc/KalRepCollection.hh"         // for KalRepCol...
#include "RecoDataProducts/inc/KalRepPtrCollection.hh"      // for KalRepPtr...

class KalRep;

using namespace std;

namespace mu2e {

  class PtrTest : public art::EDProducer {
  public:

    explicit PtrTest(fhicl::ParameterSet const& pset);
    void produce( art::Event& e) override;

  private:

    // Information about the data product that contains the fitted tracks.
    // The instance name is also used as the instance name of the output data product.
    TrkParticle     _tpart;
    TrkFitDirection _fdir;
    std::string     _instanceName;
    art::InputTag   _inputTag;

  };

}  // end namespace mu2e

mu2e::PtrTest::PtrTest(fhicl::ParameterSet const& pset):
  EDProducer{pset},
  _tpart((TrkParticle::type)(pset.get<int>("fitparticle"))),
  _fdir((TrkFitDirection::FitDirection)(pset.get<int>("fitdirection"))),
  _instanceName( _fdir.name() + _tpart.name()),
  _inputTag( pset.get<string>("inputModuleLabel"), _instanceName)
{
  produces<KalRepPtrCollection>(_instanceName);
}

void mu2e::PtrTest::produce(art::Event& event)
{

  // Access the input data product.
  auto trksHandle  = event.getValidHandle<KalRepCollection>(_inputTag);
  auto const& inputTracks = *trksHandle;

  // Create an empty output data product.
  unique_ptr<KalRepPtrCollection> outputTracks(new KalRepPtrCollection );
  outputTracks->reserve(inputTracks.size());

  // Populate the output data product.
  for ( size_t i=0; i<inputTracks.size(); ++i ){

    KalRep const* krep = inputTracks.get(i);
    if ( !krep ) continue;

    outputTracks->emplace_back(trksHandle,i);
  }

  // Add the output data product to the event.
  event.put(std::move(outputTracks),_instanceName);

}

DEFINE_ART_MODULE(mu2e::PtrTest);
