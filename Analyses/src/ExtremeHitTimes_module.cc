// Distributions of the earliest and the latest hit times in an event.
// Useful in e.g. setting a time cut to pre-filter beam flash mixing inputs.
//
// Andrei Gaponenko, 2014

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <vector>                                           // for vector
#include <algorithm>                                        // for max, min
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "MCDataProducts/inc/StepPointMC.hh"                // for StepPoint...
#include "TH1.h"                                            // for TH1D, TH1
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/coding.h"                                // for ps_sequen...
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace mu2e {

  //================================================================
  class ExtremeHitTimes : public art::EDAnalyzer {
    typedef std::vector<art::InputTag> InputTags;
    InputTags inputs_;
    TH1 *hearly_;
    TH1 *hlate_;
    art::ServiceHandle<art::TFileService> tfs() { return art::ServiceHandle<art::TFileService>(); }
  public:
    explicit ExtremeHitTimes(const fhicl::ParameterSet& pset);
    virtual void analyze(const art::Event&) override;
  };

  //================================================================
  ExtremeHitTimes::ExtremeHitTimes(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , hearly_(tfs()->make<TH1D>("tearly", "Earliest hit time", 400, 0., 2000.))
    , hlate_(tfs()->make<TH1D>("tlate", "Latest hit time", 400, 0., 2000.))
  {
    typedef std::vector<std::string> VS;
    const VS instr(pset.get<VS>("inputs"));
    for(const auto& i : instr) {
      inputs_.emplace_back(i);
    }
  }

  //================================================================
  void ExtremeHitTimes::analyze(const art::Event& event) {
    double tearly = 0.;
    double tlate = 0.;
    bool haveData = false;
    for(const auto& tag : inputs_) {
      const auto& coll = event.getValidHandle<StepPointMCCollection>(tag);
      for(const auto& step: *coll) {
        if(haveData) {
          tearly = std::min(tearly, step.time());
          tlate = std::max(tlate, step.time());
        }
        else {
          tearly = tlate = step.time();
          haveData = true;
        }
      }
    }

    if(haveData) {
      hearly_->Fill(tearly);
      hlate_->Fill(tlate);
    }
  }

  //================================================================
} // namespace mu2e

DEFINE_ART_MODULE(mu2e::ExtremeHitTimes);
