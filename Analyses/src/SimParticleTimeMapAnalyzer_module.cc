// Histogram time offsets in a time map object.
//
// Andrei Gaponenko, 2016

#include <exception>                                 // for exception
#include <algorithm>                                        // for all_of
#include <map>                                              // for map
#include <memory>                                           // for allocator
#include <string>                                           // for string
#include <typeinfo>                                         // for type_info
#include <utility>                                          // for pair

#include "fhiclcpp/types/Atom.h"                            // for Atom
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "MCDataProducts/inc/SimParticleTimeMap.hh"         // for SimPartic...
#include "TH1.h"                                            // for TH1D, TH1
#include "art/Framework/Core/detail/Analyzer.h"             // for Analyzer:...
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "cetlib/exempt_ptr.h"                              // for exempt_ptr
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...
#include "fhiclcpp/types/Comment.h"                         // for Comment
#include "fhiclcpp/types/Name.h"                            // for Name
#include "fhiclcpp/types/Table.h"                           // for Table::me...
#include "fhiclcpp/types/detail/validationException.h"      // for validatio...

namespace mu2e {

  class SimParticleTimeMapAnalyzer : public art::EDAnalyzer {
  public:
    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;
      fhicl::Atom<art::InputTag> input{ Name("input"), Comment("Tag of the SimParticleTimeMap to analyze.")};

      fhicl::Atom<unsigned> nbins{Name("nbins"), Comment("Number of bins in the histogram"), 250u};
      fhicl::Atom<double> tmin{Name("tmin"), Comment("Histogram tmin"), -500.};
      fhicl::Atom<double> tmax{Name("tmax"), Comment("Histogram tmax"), +2000.};
    };

    typedef art::EDAnalyzer::Table<Config> Parameters;

    explicit SimParticleTimeMapAnalyzer(const Parameters& conf);
    void analyze(const art::Event& evt) override;
  private:
    Config conf_;
    TH1 *histTime_;
  };

  //================================================================
  SimParticleTimeMapAnalyzer::SimParticleTimeMapAnalyzer(const Parameters& conf)
    : art::EDAnalyzer(conf)
    , conf_(conf())
    , histTime_(art::ServiceHandle<art::TFileService>()->
                make<TH1D>("timeOffset", "Time offset",
                           conf().nbins(), conf().tmin(), conf().tmax()))
  {}

  //================================================================
  void SimParticleTimeMapAnalyzer::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<SimParticleTimeMap>(conf_.input());
    for(const auto& entry: *ih) {
      histTime_->Fill(entry.second);
    }
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::SimParticleTimeMapAnalyzer);
