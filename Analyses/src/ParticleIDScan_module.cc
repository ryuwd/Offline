// Visualize inputs to the E/p based particle ID.
//
// Andrei Gaponenko, 2017

#include <exception>                                 // for exception
#include <string>                                           // for allocator
#include <algorithm>                                        // for all_of

#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art_root_io/TFileDirectory.h"                     // for TFileDire...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "ParticleID/inc/PIDLogLRatio.hh"                   // for PIDLogLRa...
#include "ParticleID/inc/PIDLogLEp.hh"                      // for PIDLogLEp
#include "TH2.h"                                            // for TH2, TH2D
#include "TAxis.h"                                          // for TAxis
#include "art/Framework/Core/detail/Analyzer.h"             // for Analyzer:...
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "cetlib/exempt_ptr.h"                              // for exempt_ptr
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...
#include "fhiclcpp/types/Atom.h"                            // for Atom
#include "fhiclcpp/types/Comment.h"                         // for Comment
#include "fhiclcpp/types/Name.h"                            // for Name
#include "fhiclcpp/types/Table.h"                           // for Table::me...

namespace art {
class Event;
class Run;
}  // namespace art

namespace mu2e {

  class ParticleIDScan : public art::EDAnalyzer {
  public:
    typedef PIDLogLRatio<PIDLogLEp> PIDEp;

    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;
      fhicl::Table<PIDEp::Config> pid_ep_conf{Name("PIDEp"), Comment("E/p based PID config")};

      fhicl::Atom<int>    nPathBins{Name("nPathBins"), Comment("scan nPathBins")};
      fhicl::Atom<double> pathMin{Name("pathMin"), Comment("scan pathMin")};
      fhicl::Atom<double> pathMax{Name("pathMax"), Comment("scan pathMax")};

      fhicl::Atom<int>    neopbins{Name("neopbins"), Comment("scan neopbins")};
      fhicl::Atom<double> eopmin{Name("eopmin"), Comment("scan eopmin")};
      fhicl::Atom<double> eopmax{Name("eopmax"), Comment("scan eopmax")};
    };

    //----------------------------------------------------------------

    explicit ParticleIDScan(art::EDAnalyzer::Table<Config> const & config);
    explicit ParticleIDScan(art::EDAnalyzer::Table<Config> const & config, art::TFileDirectory tfdir);

    virtual void analyze(const art::Event&) override {};
    virtual void beginRun(const art::Run& run) override;

  private:
    art::EDAnalyzer::Table<Config> conf_;
    PIDEp pid_ep_;

    TH2* h_signalLL_;
    TH2* h_backgroundLL_;
    TH2* h_llRatio_;
  };

  //================================================================
  ParticleIDScan::ParticleIDScan(art::EDAnalyzer::Table<Config> const & config)
    : ParticleIDScan(config, *art::ServiceHandle<art::TFileService>())
  {}

  ParticleIDScan::ParticleIDScan(art::EDAnalyzer::Table<Config> const & c, art::TFileDirectory tf)
    : art::EDAnalyzer(c)
    , conf_{c}
    , pid_ep_{c().pid_ep_conf()}
    , h_signalLL_{tf.make<TH2D>("signalLL", "signal LL scan",  c().nPathBins(), c().pathMin(), c().pathMax(), c().neopbins(), c().eopmin(), c().eopmax())}
    , h_backgroundLL_{tf.make<TH2D>("backgroundLL", "background LL scan",  c().nPathBins(), c().pathMin(), c().pathMax(), c().neopbins(), c().eopmin(), c().eopmax())}
    , h_llRatio_{tf.make<TH2D>("llRatio", "LL ratio scan",  c().nPathBins(), c().pathMin(), c().pathMax(), c().neopbins(), c().eopmin(), c().eopmax())}
  {
    h_signalLL_->SetOption("colz");
    h_backgroundLL_->SetOption("colz");
    h_llRatio_->SetOption("colz");
  }

  //================================================================
  void ParticleIDScan::beginRun(const art::Run& run) {
    const auto xaxis = h_llRatio_->GetXaxis();
    const auto yaxis = h_llRatio_->GetYaxis();

    for(int ix=1; ix<=conf_().nPathBins(); ++ix) {
      const double x = xaxis->GetBinCenter(ix);

      for(int iy=1; iy<=conf_().neopbins(); ++iy) {
        const double y = yaxis->GetBinCenter(iy);

        h_signalLL_->SetBinContent(ix, iy, pid_ep_.signalLogLikelihood().value(y,x));
        h_backgroundLL_->SetBinContent(ix, iy, pid_ep_.backgroundLogLikelihood().value(y,x));
        h_llRatio_->SetBinContent(ix, iy, pid_ep_.value(y,x));
      }
    }
  }

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::ParticleIDScan);
