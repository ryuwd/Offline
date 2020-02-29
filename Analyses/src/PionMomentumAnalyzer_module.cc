// Momentum distribution of our pions
//
// Andrei Gaponenko, 2018

#include <exception>                                     // for excep...
#include <string>                                               // for alloc...
#include <algorithm>                                            // for all_of
#include <memory>                                               // for uniqu...
#include <typeinfo>                                             // for type_...
#include <utility>                                              // for pair

#include "art/Framework/Core/EDAnalyzer.h"                      // for EDAna...
#include "art/Framework/Principal/Event.h"                      // for Event
#include "art/Framework/Core/ModuleMacros.h"                    // for DEFIN...
#include "canvas/Utilities/InputTag.h"                          // for InputTag
#include "art_root_io/TFileDirectory.h"                         // for TFile...
#include "art_root_io/TFileService.h"                           // for TFile...
#include "art/Framework/Services/Registry/ServiceHandle.h"      // for Servi...
#include "GlobalConstantsService/inc/GlobalConstantsHandle.hh"  // for Globa...
#include "GlobalConstantsService/inc/ParticleDataTable.hh"      // for Parti...
#include "TH1.h"                                                // for TH1D
#include "TH2.h"                                                // for TH2D
#include "CLHEP/Vector/LorentzVector.h"                         // for HepLo...

#include "CLHEP/Vector/ThreeVector.h"                           // for Hep3V...

#include "DataProducts/inc/PDGCode.hh"                          // for PDGCode
#include "HepPDT/ParticleData.hh"                               // for Parti...

#include "MCDataProducts/inc/ProcessCode.hh"                    // for Proce...
#include "MCDataProducts/inc/SimParticle.hh"                    // for SimPa...
#include "art/Framework/Core/detail/Analyzer.h"                 // for Analy...
#include "art/Framework/Principal/Handle.h"                     // for Valid...
#include "canvas/Persistency/Common/Ptr.h"                      // for Ptr
#include "canvas/Utilities/Exception.h"                         // for Excep...
#include "cetlib/exempt_ptr.h"                                  // for exemp...
#include "cetlib/map_vector.h"                                  // for map_v...
#include "cetlib_except/exception.h"                            // for opera...
#include "fhiclcpp/exception.h"                                 // for excep...
#include "fhiclcpp/types/AllowedConfigurationMacro.h"           // for Allow...
#include "fhiclcpp/types/Atom.h"                                // for Atom
#include "fhiclcpp/types/Comment.h"                             // for Comment
#include "fhiclcpp/types/Name.h"                                // for Name
#include "fhiclcpp/types/Table.h"                               // for Table...
#include "fhiclcpp/types/detail/validationException.h"          // for valid...

namespace mu2e {

  class PionMomentumAnalyzer : public art::EDAnalyzer {
  public:

    struct ConfigStruct {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;

      fhicl::Atom<art::InputTag> inputs{Name("inputs"), Comment("Input SimParticleCollection")};

      fhicl::Atom<int>    nPBins{Name("nPBins"), Comment("number of bins for momentum histograms")};
      fhicl::Atom<double> pmin{Name("pmin"), Comment("Momentum histogram lower limit")};
      fhicl::Atom<double> pmax{Name("pmax"), Comment("Momentum histogram upper limit")};
    };

    typedef art::EDAnalyzer::Table<ConfigStruct> Conf;

    //----------------------------------------------------------------

    explicit PionMomentumAnalyzer(const Conf& config);
    explicit PionMomentumAnalyzer(const Conf& config, art::TFileDirectory tfdir);

    virtual void beginJob() override;
    virtual void analyze(const art::Event&) override;

  private:
    Conf conf_;

    TH1* h_p_all_;
    TH2* h_p_by_parent_;
    TH2* h_p_by_process_;

    const ParticleDataTable *particleTable_;

    static bool is_muon_daughter(const SimParticle& p);
  };

  //================================================================
  PionMomentumAnalyzer::PionMomentumAnalyzer(const Conf& config)
    : PionMomentumAnalyzer(config, *art::ServiceHandle<art::TFileService>())
  {}

  PionMomentumAnalyzer::PionMomentumAnalyzer(const Conf& c, art::TFileDirectory tf)
    : art::EDAnalyzer(c)
    , conf_{c}

    , h_p_all_{tf.make<TH1D>("p_pion", "Pion production momentum", c().nPBins(), c().pmin(), c().pmax())}
    , h_p_by_parent_{tf.make<TH2D>("p_pion_by_parent", "Pion production momentum vs parent PID",  1, 0., 0., c().nPBins(), c().pmin(), c().pmax())}
    , h_p_by_process_{tf.make<TH2D>("p_pion_by_process", "Pion production momentum vs production process",  1, 0., 0., c().nPBins(), c().pmin(), c().pmax())}
    , particleTable_{nullptr}
  {
    h_p_by_parent_->SetOption("colz");
    h_p_by_process_->SetOption("colz");
  }

  //================================================================
  void PionMomentumAnalyzer::beginJob() {
    GlobalConstantsHandle<ParticleDataTable> ph;
    particleTable_ = &(*ph);
  }

  //================================================================
  void PionMomentumAnalyzer::analyze(const art::Event& evt) {
    const auto sc = evt.getValidHandle<SimParticleCollection>(conf_().inputs());
    for(const auto& spe: *sc) {
      const SimParticle& p = spe.second;

      if(p.pdgId() == PDGCode::pi_minus) {

        const double momentum = p.startMomentum().vect().mag();
        h_p_all_->Fill(momentum);

        const SimParticle& parent{*p.parent()};

        const auto pref = particleTable_->particle(parent.pdgId()).ref();
        std::string parentName = pref.name();
        h_p_by_parent_->Fill(parentName.c_str(), momentum, 1.0);

        std::string codename = p.creationCode().name(); // need to bind for the c_str() call below
        h_p_by_process_->Fill(codename.c_str(), momentum, 1.0);
      }
    }
  }

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::PionMomentumAnalyzer);
