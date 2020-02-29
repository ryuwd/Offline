// Print out an EventIDSequence
//
// Andrei Gaponenko, 2020

#include <exception>                            // for exception
#include <string>                                      // for allocator, string
#include <iostream>                                    // for operator<<
#include <memory>                                      // for unique_ptr
#include <typeinfo>                                    // for type_info
#include <vector>                                      // for vector

#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "art/Framework/Principal/Event.h"             // for Event
#include "art/Framework/Principal/Handle.h"            // for ValidHandle
#include "art/Framework/IO/ProductMix/MixTypes.h"      // for EventIDSequence
#include "canvas/Utilities/InputTag.h"                 // for InputTag, oper...
#include "art/Framework/Core/detail/Analyzer.h"        // for Analyzer::Table
#include "canvas/Persistency/Provenance/EventID.h"     // for operator<<
#include "cetlib/exempt_ptr.h"                         // for exempt_ptr
#include "fhiclcpp/exception.h"                        // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...
#include "fhiclcpp/types/Atom.h"                       // for Atom
#include "fhiclcpp/types/Comment.h"                    // for Comment
#include "fhiclcpp/types/Name.h"                       // for Name
#include "fhiclcpp/types/Table.h"                      // for Table::members_t


namespace mu2e {

  //================================================================
  class EventIDSequencePrinter : public art::EDAnalyzer {
  public:


    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;

      fhicl::Atom<art::InputTag> input { Name("input"),
          Comment("The EventIDSequence to print") };

    };

    using Parameters = art::EDAnalyzer::Table<Config>;
    explicit EventIDSequencePrinter(const Parameters& conf);
    void analyze(const art::Event& evt) override;
  private:
    art::InputTag input_;
  };

  //================================================================
  EventIDSequencePrinter::EventIDSequencePrinter(const Parameters& pars)
    : art::EDAnalyzer(pars)
    , input_(pars().input())
  {}

  //================================================================
  void EventIDSequencePrinter::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<art::EventIDSequence>(input_);
    std::cout<<"EventIDSequence "<<input_<<" : ";
    for(const auto& id : *ih) {
      std::cout<<" ( "<<id<<" ) ";
    }
    std::cout<<std::endl;
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::EventIDSequencePrinter);
