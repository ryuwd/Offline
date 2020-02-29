// Histogram a proton bunch intensity distribution.
//
// Andrei Gaponenko, 2018

#include <exception>                                 // for exception
#include <algorithm>                                        // for all_of
#include <memory>                                           // for allocator
#include <string>                                           // for string
#include <typeinfo>                                         // for type_info

#include "fhiclcpp/types/Atom.h"                            // for Atom
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for Handle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "MCDataProducts/inc/ProtonBunchIntensity.hh"       // for ProtonBun...
#include "TH1.h"                                            // for TH1D, TH1
#include "art/Framework/Core/detail/Analyzer.h"             // for Analyzer:...
#include "art/Framework/Principal/SubRun.h"                 // for SubRun
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

  class ProtonBunchIntensityAnalyzer : public art::EDAnalyzer {
  public:
    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;
      fhicl::Atom<art::InputTag> input{ Name("input"), Comment("Tag of the ProtonBunchIntensity object to analyze.")};
      fhicl::Atom<art::InputTag> meanPBItag{Name("MeanBeamIntensity"), Comment("Tag for MeanBeamIntensity"), art::InputTag()};

      fhicl::Atom<unsigned> nbins{Name("nbins"), Comment("Number of bins in the histogram"), 250u};
      fhicl::Atom<double> hmin{Name("hmin"), Comment("Histogram min"), 0.};
      fhicl::Atom<double> hmax{Name("hmax"), Comment("Absolute PBI histogram max"), 1.e8};
      fhicl::Atom<double> rmax{Name("rmax"), Comment("Relative PBI histogram max"), 3.0};
    };

    typedef art::EDAnalyzer::Table<Config> Parameters;

    explicit ProtonBunchIntensityAnalyzer(const Parameters& conf);
    void analyze(const art::Event& evt) override;
    void beginSubRun(const art::SubRun& subrun) override;
  private:
    Config conf_;
    TH1 *hh_, *hhr_;
    double meanPBI_;
  };

  //================================================================
  ProtonBunchIntensityAnalyzer::ProtonBunchIntensityAnalyzer(const Parameters& conf)
    : art::EDAnalyzer(conf)
    , conf_(conf())
    , hh_(art::ServiceHandle<art::TFileService>()->
          make<TH1D>("pbi", "Absolute proton bunch intensity",
                     conf().nbins(), conf().hmin(), conf().hmax()))
    , hhr_(art::ServiceHandle<art::TFileService>()->
          make<TH1D>("rpbi", "Relative proton bunch intensity",
                     conf().nbins(), 0.0, conf().rmax()))
    , meanPBI_(-1.0)
  {
    hh_->StatOverflows();
    hhr_->StatOverflows();
  }

  void ProtonBunchIntensityAnalyzer::beginSubRun(const art::SubRun & subrun ) {
    // mean number of protons on target
    art::Handle<ProtonBunchIntensity> PBIHandle;
    subrun.getByLabel(conf_.meanPBItag(), PBIHandle);
    if(PBIHandle.isValid())
      meanPBI_ = PBIHandle->intensity();
  }

  //================================================================
  void ProtonBunchIntensityAnalyzer::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<ProtonBunchIntensity>(conf_.input());
    hh_->Fill(ih->intensity());
    if(meanPBI_>0.0)hhr_->Fill(ih->intensity()/meanPBI_);
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::ProtonBunchIntensityAnalyzer);
