// TrackSummaryDataAnalyzer: no MC dependencies.
//
// Andrei Gaponenko, 2014

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <vector>                                           // for vector
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "RecoDataProducts/inc/TrackSummary.hh"             // for TrackSumm...
#include "Mu2eUtilities/inc/TrackCuts.hh"                   // for TrackCuts
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace mu2e {

  class TrackSummaryDataAnalyzer : public art::EDAnalyzer {
  public:
    explicit TrackSummaryDataAnalyzer(fhicl::ParameterSet const& pset);
    void analyze(const art::Event& evt) override;
  private:
    TrackCuts an_;
    art::InputTag trackInput_;
  };

  //================================================================
  TrackSummaryDataAnalyzer::TrackSummaryDataAnalyzer(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , an_(pset.get<fhicl::ParameterSet>("cuts"), *art::ServiceHandle<art::TFileService>(), "")
    , trackInput_(pset.get<std::string>("trackInput"))
  {}

  //================================================================
  void TrackSummaryDataAnalyzer::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<TrackSummaryCollection>(trackInput_);
    for(unsigned i=0; i<ih->size(); ++i) {
      const auto& track = ih->at(i);
      const double weight = 1.; // no weights for real data
      if(an_.accepted(track, weight)) {
        // Fill more histograms for selected tracks here...
      }
    }
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::TrackSummaryDataAnalyzer);
