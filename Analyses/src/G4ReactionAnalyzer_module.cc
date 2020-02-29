// Ntuple dumper for StepPointMCs.
//
// Andrei Gaponenko, 2013

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <vector>                                           // for vector
#include <sstream>                                          // for operator<<
#include <map>                                              // for map, map<...
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <utility>                                          // for pair

#include "TH1.h"                                            // for TH1D, TH1
#include "TH2.h"                                            // for TH2, TH2D
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "MCDataProducts/inc/SimParticle.hh"                // for SimParticle
#include "MCDataProducts/inc/ProcessCode.hh"                // for operator<<
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "cetlib/map_vector.h"                              // for map_vector
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace mu2e {

  //================================================================
  class G4ReactionAnalyzer : public art::EDAnalyzer {
    art::InputTag inputs_;
    int pdgId_;

    TH1* hStoppingCodes_;
    TH1* hDaugherCreationCodes_;
    TH2* hDaugherMultiplicity_;

    art::ServiceHandle<art::TFileService> tfs() { return art::ServiceHandle<art::TFileService>(); }

  public:
    explicit G4ReactionAnalyzer(const fhicl::ParameterSet& pset);
    virtual void analyze(const art::Event& event);
  };

  //================================================================
  G4ReactionAnalyzer::G4ReactionAnalyzer(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , inputs_(pset.get<std::string>("inputs"))
    , pdgId_(pset.get<int>("pdgId"))
    , hStoppingCodes_(tfs()->make<TH1D>("stoppingCodes", "stopping codes", 1, 0., 1.))
    , hDaugherCreationCodes_(tfs()->make<TH1D>("creationCodes", "daughter creation codes", 1, 0., 1.))
    , hDaugherMultiplicity_(tfs()->make<TH2D>("multiplicity", "daughter multiplicity", 1, 0., 1., 25, 0.5, 25.5))
  {
    hDaugherMultiplicity_->SetOption("colz");
  }

  //================================================================
  void G4ReactionAnalyzer::analyze(const art::Event& event) {
    typedef std::map<std::string, int> StringStats;

    const auto& coll = event.getValidHandle<SimParticleCollection>(inputs_);
    for(const auto& i : *coll) {
      const auto& particle = i.second;
      if(particle.pdgId() == pdgId_) {
        hStoppingCodes_->Fill(particle.stoppingCode().name().c_str(), 1.);
        StringStats ccount;
        for(const auto& daughter: particle.daughters()) {
          std::ostringstream pp;
          if(static_cast<int>(daughter->pdgId()) < 1000000000) {
            pp<<daughter->pdgId()<<" "<<daughter->creationCode();
          }
          else {
            pp<<"nucleus "<<daughter->creationCode();
          }

          hDaugherCreationCodes_->Fill(pp.str().c_str(), 1.);
          ++ccount[pp.str()];
        }
        for(const auto& i : ccount) {
          hDaugherMultiplicity_->Fill(i.first.c_str(), i.second, 1.);
        }
      }
    }
  } // analyze(event)

    //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::G4ReactionAnalyzer);
