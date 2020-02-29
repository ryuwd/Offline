//
// Module used to develop, test and illustrate the use of some
// helper functions/classes to access provenance information about
// KalRepCollections.
//
// Original author Rob Kutschke
//

#include <exception>                                 // for exception
#include <iostream>                                         // for operator<<
#include <string>                                           // for string
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <vector>                                           // for vector

#include "Mu2eUtilities/inc/decodeTrackPatRecType.hh"       // for decodeTra...
#include "Mu2eUtilities/inc/KalRepCollectionInfo.hh"        // for KalRepCol...
#include "Mu2eUtilities/inc/TrackPatRecType.hh"             // for TrackPatR...
#include "RecoDataProducts/inc/KalRepPtrCollection.hh"      // for KalRepPtr...
#include "RecoDataProducts/inc/KalRepCollection.hh"         // for KalRepCol...
#include "RecoDataProducts/inc/TrkFitDirection.hh"          // for TrkFitDir...
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "GeneralUtilities/inc/EnumToStringSparse.hh"       // for operator<<
#include "GeneralUtilities/inc/OwningPointerCollection.hh"  // for OwningPoi...
#include "canvas/Persistency/Common/Ptr.h"                  // for operator<<
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

using namespace std;

namespace mu2e {

  class TestKalRepProvenanceHelpers : public art::EDAnalyzer {
  public:

    explicit TestKalRepProvenanceHelpers(fhicl::ParameterSet const& pset);
    void analyze(const art::Event& e) override;

  private:

    // Module label of the module that created a KalRepPtrCollection.
    art::InputTag _tracksTag;

  };
}  // end namespace mu2e

mu2e::TestKalRepProvenanceHelpers::TestKalRepProvenanceHelpers(fhicl::ParameterSet const& pset) :
  art::EDAnalyzer(pset),
  _tracksTag(pset.get<std::string>("tracksTag")){
  TrackPatRecType::printAll(cout);
}

void mu2e::TestKalRepProvenanceHelpers::analyze(const art::Event& event) {

  // Test the collection level helper.
  art::Handle<KalRepCollection> kalReps;
  if ( event.getByLabel(_tracksTag, kalReps) ) {
    KalRepCollectionInfo info( kalReps );
    cout << "KalReps: "
         << kalReps->size() << "  | "
         << info.patRecType() <<  " "
         << info.instanceName() <<  " "
         << info.direction().name() <<  " "
         << info.particleType() << " "
         << info.charge()
         << endl;
  }

  // Test the ptr level helpers.
  auto ptrs = event.getValidHandle<KalRepPtrCollection>(_tracksTag);
  cout << "KalRep ptrs: " << ptrs->size() << endl;
  for ( auto const& ptr : *ptrs ){
    KalRepCollectionInfo info( ptr, event);
    TrackPatRecType type = decodeTrackPatRecType( ptr, event);
    cout << "    : "
         << _tracksTag.label()       << " "
         << ptr                      << " "
         << info.patRecType()        << " "
         << type                     << " | "
         << info.instanceName()      << " "
         << info.direction().name()  << " "
         << info.particleType()      << " "
         << info.charge()
         << endl;
  }

} // end analyze

DEFINE_ART_MODULE(mu2e::TestKalRepProvenanceHelpers);
