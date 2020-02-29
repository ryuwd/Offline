// Print out a StepPointMCCollection.
//
// Andrei Gaponenko, 2013

#include <exception>                            // for exception
#include <string>                                      // for string, allocator
#include <iostream>                                    // for operator<<
#include <memory>                                      // for unique_ptr
#include <typeinfo>                                    // for type_info
#include <vector>                                      // for vector

#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "art/Framework/Principal/Event.h"             // for Event
#include "art/Framework/Principal/Handle.h"            // for ValidHandle
#include "MCDataProducts/inc/StepPointMC.hh"           // for StepPointMCCol...
#include "canvas/Persistency/Provenance/EventID.h"     // for operator<<
#include "fhiclcpp/ParameterSet.h"                     // for ParameterSet
#include "fhiclcpp/exception.h"                        // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...

namespace mu2e {

  //================================================================
  class StepPointsPrinter : public art::EDAnalyzer {
  public:
    explicit StepPointsPrinter(fhicl::ParameterSet const& pset);
    void analyze(const art::Event& evt) override;
  private:
    std::string input_;
  };

  //================================================================
  StepPointsPrinter::StepPointsPrinter(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , input_(pset.get<std::string>("inputCollection"))
  {}

  //================================================================
  void StepPointsPrinter::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<StepPointMCCollection>(input_);
    std::cout<<"Hits for "<<input_<<" in "<<event.id()<<": ("<<ih->size()<<")"<<std::endl;
    for(const auto& hit : *ih) {
      std::cout<<"   "<<hit<<std::endl;
    }
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::StepPointsPrinter);
