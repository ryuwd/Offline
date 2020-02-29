// Print out a SimParticleCollection.
//
// Andrei Gaponenko, 2013

#include <exception>                                   // for exception
#include <string>                                             // for allocator
#include <iostream>                                           // for cout
#include <algorithm>                                          // for all_of
#include <memory>                                             // for unique_ptr
#include <typeinfo>                                           // for type_info

#include "art/Framework/Core/EDAnalyzer.h"                    // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                  // for DEFINE_...
#include "art/Framework/Principal/Event.h"                    // for Event
#include "art/Framework/Principal/Handle.h"                   // for ValidHa...
#include "canvas/Utilities/InputTag.h"                        // for InputTag
#include "Mu2eUtilities/inc/SimParticleCollectionPrinter.hh"  // for SimPart...
#include "MCDataProducts/inc/SimParticle.hh"                  // for SimPart...
#include "art/Framework/Core/detail/Analyzer.h"               // for Analyze...
#include "cetlib/exempt_ptr.h"                                // for exempt_ptr
#include "fhiclcpp/exception.h"                               // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"         // for Allowed...
#include "fhiclcpp/types/Atom.h"                              // for Atom
#include "fhiclcpp/types/Comment.h"                           // for Comment
#include "fhiclcpp/types/Name.h"                              // for Name
#include "fhiclcpp/types/Table.h"                             // for Table

namespace mu2e {

  //================================================================
  class SimParticlesPrinter : public art::EDAnalyzer {
  public:
    struct Config {
      using Name = fhicl::Name;
      using Comment = fhicl::Comment;
      fhicl::Atom<art::InputTag> inputCollection {Name("inputCollection"),
          Comment("Collection to print")
          };

      fhicl::Table<SimParticleCollectionPrinter::Config> printer { Name("printer") };
    };

    using Parameters = art::EDAnalyzer::Table<Config>;
    explicit SimParticlesPrinter(const Parameters& pars);

    void analyze(const art::Event& evt) override;

  private:
    art::InputTag input_;
    SimParticleCollectionPrinter pr_;
  };

  //================================================================
  SimParticlesPrinter::SimParticlesPrinter(const Parameters& pars)
    : art::EDAnalyzer(pars)
    , input_(pars().inputCollection())
    , pr_(pars().printer())
  {}

  //================================================================
  void SimParticlesPrinter::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<SimParticleCollection>(input_);
    pr_.print(std::cout, *ih);
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::SimParticlesPrinter);
