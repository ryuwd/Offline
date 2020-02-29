// TrackSummaryMCAnalyzer: start with tracks.
// Use MC weights and MC process info to evaluate
// backgrounds without double counting.
//
// Andrei Gaponenko, 2014

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <vector>                                           // for vector
#include <algorithm>                                        // for for_each
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

#include "cetlib_except/exception.h"                        // for operator<<
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Common/FindOneP.h"             // for FindOneP
#include "RecoDataProducts/inc/TrackSummary.hh"             // for TrackSumm...
#include "MCDataProducts/inc/SimParticle.hh"                // for SimPartic...
#include "MCDataProducts/inc/EventWeight.hh"                // for EventWeight
#include "Mu2eUtilities/inc/TrackCuts.hh"                   // for TrackCuts
#include "canvas/Persistency/Common/Assns.h"                // for Assns
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr
#include "canvas/Persistency/Provenance/ProductID.h"        // for ProductID
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/coding.h"                                // for ps_sequen...
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...
#include "MCDataProducts/inc/TrackSummaryTruthAssns.hh"     // for TrackSummaryMatchInfo

namespace mu2e {

  class TrackSummaryMCAnalyzer : public art::EDAnalyzer {
  public:
    explicit TrackSummaryMCAnalyzer(fhicl::ParameterSet const& pset);
    void analyze(const art::Event& evt) override;
  private:
    TrackCuts signalAnalysis_;
    TrackCuts otherAnalysis_;
    TrackCuts fakeAnalysis_;

    art::InputTag trackInput_;
    art::InputTag truthMapInput_;
    art::InputTag signalSimParticleCollection_;

    std::vector<art::InputTag> eventWeights_;
  };

  //================================================================
  TrackSummaryMCAnalyzer::TrackSummaryMCAnalyzer(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , signalAnalysis_(pset.get<fhicl::ParameterSet>("cuts"), *art::ServiceHandle<art::TFileService>(), "signal")
    , otherAnalysis_(pset.get<fhicl::ParameterSet>("cuts"), *art::ServiceHandle<art::TFileService>(), "other")
    , fakeAnalysis_(pset.get<fhicl::ParameterSet>("cuts"), *art::ServiceHandle<art::TFileService>(), "fake")
    , trackInput_(pset.get<std::string>("trackInput"))
    , truthMapInput_(pset.get<std::string>("truthMapInput"))
    , signalSimParticleCollection_(pset.get<std::string>("signalSimParticleCollection"))
    , eventWeights_(pset.get<std::vector<art::InputTag> >("eventWeights"))
  {}

  //================================================================
  void TrackSummaryMCAnalyzer::analyze(const art::Event& event) {
    // Can't use ValidHandle: redmine #6422
    // auto ih = event.getValidHandle<TrackSummaryCollection>(trackInput_);
    art::Handle<TrackSummaryCollection> ih;
    event.getByLabel(trackInput_, ih);

    art::Handle<SimParticleCollection> signalHandle;
    event.getByLabel(signalSimParticleCollection_, signalHandle);

    for(unsigned i=0; i<ih->size(); ++i) {
      const auto& track = ih->at(i);

      // To evaluate backgrounds without double counting we should
      // accept only tracks corresponding to the simulated process of
      // interest.

      art::FindOneP<SimParticle, mu2e::TrackSummaryMatchInfo> f1(ih, event, truthMapInput_);
      if(f1.size() == 1) {
        const art::Ptr<SimParticle>& sim = f1.at(i);

        if(sim.id() == signalHandle.id()) {

          // Use the specified weights.
          double weight = 1.;
          for (const auto& iwt : eventWeights_ ) {
            weight *= event.getValidHandle<EventWeight>(iwt)->weight();
          }

          if(signalAnalysis_.accepted(track, weight)) {
            // Fill more histograms for selected tracks...
          }

        }
        else {
          // Known MC source, different than the current process of interest
          otherAnalysis_.accepted(track, 1.);
        }
      }
      else {
        // Fake tracks. (Or no MC info.)
        fakeAnalysis_.accepted(track, 1.);
      }

    } // for(tracks)
  } // analyze()

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::TrackSummaryMCAnalyzer);
