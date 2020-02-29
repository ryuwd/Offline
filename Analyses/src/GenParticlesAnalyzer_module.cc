// Fills histograms for a GenParticle collection.
//
// Andrei Gaponenko, 2013

#include <exception>                                 // for exception
#include <string>                                           // for allocator
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "MCDataProducts/inc/GenParticle.hh"                // for GenPartic...
#include "Mu2eUtilities/inc/GeneratorSummaryHistograms.hh"  // for Generator...
#include "art/Framework/Core/detail/Analyzer.h"             // for Analyzer:...
#include "cetlib/exempt_ptr.h"                              // for exempt_ptr
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...
#include "fhiclcpp/types/Atom.h"                            // for Atom
#include "fhiclcpp/types/Comment.h"                         // for Comment
#include "fhiclcpp/types/Name.h"                            // for Name
#include "fhiclcpp/types/Table.h"                           // for Table::me...

namespace art {
class Run;
}  // namespace art

namespace mu2e {

  class GenParticlesAnalyzer : public art::EDAnalyzer {
  public:

    struct Config {
      fhicl::Atom<art::InputTag> inputs{
        fhicl::Name("inputs"),
          fhicl::Comment("The InputTag of a GenParticle collection to analyze. ")
          };
    };

    using Parameters = art::EDAnalyzer::Table<Config>;
    explicit GenParticlesAnalyzer(const Parameters& conf);

    void beginRun(const art::Run&) override;
    void analyze(const art::Event& e) override;

  private:
    art::InputTag inputs_;
    GeneratorSummaryHistograms genSummary_;
  };

  GenParticlesAnalyzer::GenParticlesAnalyzer(const Parameters& conf) :
      art::EDAnalyzer(conf),
      inputs_(conf().inputs()),
      genSummary_()
  {}

  // Can't book GeneratorSummaryHistograms in beginJob
  // because it uses the geometry service
  void GenParticlesAnalyzer::beginRun(const art::Run&) {
    genSummary_.book();
  }

  void GenParticlesAnalyzer::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<GenParticleCollection>(inputs_);
    genSummary_.fill(*ih);
  }

}  // end namespace mu2e

DEFINE_ART_MODULE(mu2e::GenParticlesAnalyzer);
