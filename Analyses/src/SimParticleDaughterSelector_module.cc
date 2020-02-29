// Select daughers of a given set of particles based on their creation
// codes, and write the result into a new SimParticlePtrCollection.
//
// Andrei Gaponenko, 2014

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <vector>                                           // for vector
#include <utility>                                          // for move
#include <set>                                              // for set, oper...
#include <algorithm>                                        // for max
#include <memory>                                           // for unique_ptr
#include <ostream>                                          // for operator<<
#include <typeinfo>                                         // for type_info

#include "art/Framework/Core/EDProducer.h"                  // for EDProducer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "MCDataProducts/inc/ProcessCode.hh"                // for ProcessCo...
#include "MCDataProducts/inc/SimParticlePtrCollection.hh"   // for SimPartic...
#include "MCDataProducts/inc/SimParticle.hh"                // for SimParticle
#include "TH1.h"                                            // for TH1D, TH1
#include "art/Framework/Core/ProducerTable.h"               // for ProducerT...
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...
#include "fhiclcpp/types/Atom.h"                            // for Atom, Ato...
#include "fhiclcpp/types/Comment.h"                         // for Comment
#include "fhiclcpp/types/Name.h"                            // for Name
#include "fhiclcpp/types/Sequence.h"                        // for Sequence

namespace mu2e {
  namespace {
    std::string processToString(const SimParticle& p) {
      std::ostringstream os;

      if(static_cast<int>(p.pdgId()) < 1000000000) {
        os<<p.pdgId()<<" "<<ProcessCode(p.creationCode());
      }
      else {
        os<<"nucleus "<<ProcessCode(p.creationCode());
      }

      return os.str();
    }
  }

  class SimParticleDaughterSelector : public art::EDProducer {
  public:

    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;
      fhicl::Atom<art::InputTag> particleInput{ Name("particleInput"), Comment("The input collection.") };
      fhicl::Sequence<std::string> processes{ Name("processes"),
          Comment("A list of production process names, like \"DIO\", \"NuclearCapture\".\n"
                  "SimParticles with the given production codes will be written  to the output.\n"
                  "If the process list is empty, all particles are passed to the output.\n"
                  "You can use this mode to produce diagnostic histograms that will contain\nthe names of possible processes."
                  )
          };
    };

    using Parameters = art::EDProducer::Table<Config>;
    explicit SimParticleDaughterSelector(const Parameters& conf);

    void produce(art::Event& evt) override;
  private:
    art::InputTag particleInput_;
    std::set<ProcessCode::enum_type> procs_;

    art::ServiceHandle<art::TFileService> tfs() { return art::ServiceHandle<art::TFileService>(); }
    TH1* haccepted_;
    TH1* hignored_;
  };

  //================================================================
  SimParticleDaughterSelector::SimParticleDaughterSelector(const Parameters& conf)
    : EDProducer{conf}
    , particleInput_(conf().particleInput())
    , haccepted_(tfs()->make<TH1D>("accepted", "Accepted pdgId and process code pairs", 1, 0., 1.))
    , hignored_(tfs()->make<TH1D>("ignored", "Ignored pdgId and process code pairs", 1, 0., 1.))
  {
    produces<SimParticlePtrCollection>();
    const auto& processes = conf().processes();
    for(const auto& proc: processes) {
      procs_.insert(ProcessCode::findByName(proc).id());
    }

  }

  //================================================================
  void SimParticleDaughterSelector::produce(art::Event& event) {

    std::unique_ptr<SimParticlePtrCollection> output(new SimParticlePtrCollection());

    auto ih = event.getValidHandle<SimParticlePtrCollection>(particleInput_);
    for(const auto& i: *ih) {
      const auto& particle = *i;
      for(const auto& daughter: particle.daughters()) {
        const std::string strid(processToString(*daughter));
        if(procs_.empty() || (procs_.find(daughter->creationCode().id()) != procs_.end())) {
          output->emplace_back(daughter);
          haccepted_->Fill(strid.c_str(), 1.);
        }
        else {
          hignored_->Fill(strid.c_str(), 1.);
        }
      }
    }

    event.put(std::move(output));
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::SimParticleDaughterSelector);
