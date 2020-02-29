// Print out a StepPointMCCollection, based on StepPointsPrinter by
// Andrei Gaponenko.
//
// In this version, can set a list of volumeIds to selectively print out.
//
// Andrei Gaponenko, 2013

#include <exception>                            // for exception
#include <string>                                      // for string, allocator
#include <iostream>                                    // for operator<<
#include <algorithm>                                   // for max
#include <memory>                                      // for unique_ptr
#include <typeinfo>                                    // for type_info
#include <vector>                                      // for vector

#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "art/Framework/Principal/Event.h"             // for Event
#include "art/Framework/Principal/Handle.h"            // for ValidHandle
#include "MCDataProducts/inc/StepPointMC.hh"           // for StepPointMCCol...
#include "CLHEP/Vector/ThreeVector.h"                  // for operator<<

#include "MCDataProducts/inc/SimParticle.hh"           // for SimParticle
#include "canvas/Persistency/Common/Ptr.h"             // for Ptr
#include "canvas/Persistency/Provenance/EventID.h"     // for operator<<
#include "cetlib_except/exception.h"                   // for operator<<
#include "fhiclcpp/ParameterSet.h"                     // for ParameterSet
#include "fhiclcpp/coding.h"                           // for ps_sequence_t
#include "fhiclcpp/exception.h"                        // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...

namespace mu2e {

  //================================================================
  class SelectiveStepPtPrinter : public art::EDAnalyzer {
  public:
    explicit SelectiveStepPtPrinter(fhicl::ParameterSet const& pset);
    void analyze(const art::Event& evt) override;
  private:
    std::string            input_;
    std::vector<int>       volumes_;
  };

  //================================================================
  SelectiveStepPtPrinter::SelectiveStepPtPrinter(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , input_(pset.get<std::string>("inputCollection"))
    , volumes_(pset.get<std::vector<int> >("volumes"))
  {}

  //================================================================
  void SelectiveStepPtPrinter::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<StepPointMCCollection>(input_);
    std::cout<<"Hits for "<<input_<<" in "<<event.id()<<": ("<<ih->size()<<")"<<std::endl;
    for(const auto& hit : *ih) {
      int id = hit.volumeId();
      CLHEP::Hep3Vector r = hit.position();
      for ( const auto& testId : volumes_ ) {
	//	std::cout << "id is " << id << ", while testId is " << testId << std::endl;
	if ( id == testId ) {
	  std::cout <<  "Volume:  " << id << ", particle: " << hit.simParticle()->pdgId() << ", position:  " << r << std::endl;
	}
      }
      //      std::cout<<"   "<<hit<<std::endl;
    }
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::SelectiveStepPtPrinter);
