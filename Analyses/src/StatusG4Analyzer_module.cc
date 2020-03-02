// Andrei Gaponenko, 2015, with most of the code picked from Rob's DiagnosticG4.

#include <exception>                                 // for exception
#include <cmath>                                           // for log10
#include <string>                                           // for allocator
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info

#include "TH1.h"                                            // for TH1D
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "MCDataProducts/inc/StatusG4.hh"                   // for StatusG4
#include "art/Framework/Core/detail/Analyzer.h"             // for Analyzer:...
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "cetlib/exempt_ptr.h"                              // for exempt_ptr
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...
#include "fhiclcpp/types/Atom.h"                            // for Atom
#include "fhiclcpp/types/Comment.h"                         // for Comment
#include "fhiclcpp/types/Name.h"                            // for Name
#include "fhiclcpp/types/Table.h"                           // for Table::me...

namespace mu2e {

  //================================================================
  class StatusG4Analyzer : public art::EDAnalyzer {
    art::InputTag input_;
    TH1D *hStatusValue_ = nullptr;
    TH1D *hNG4Tracks_ = nullptr;
    TH1D *hNG4TracksLog_ = nullptr;
    TH1D *hOverflowSimPart_ = nullptr;
    TH1D *hNKilledStep_ = nullptr;
    TH1D *hRealTime_ = nullptr;
    TH1D *hRealTimeWide_ = nullptr;
    TH1D *hCPUTime_ = nullptr;
    TH1D *hCPUTimeWide_ = nullptr;
  public:

    struct Config {
      fhicl::Atom<art::InputTag> input {
        fhicl::Name("input"),
          fhicl::Comment("The InputTag of the StatusG4 object to analyze.")
          };
    };

    using Parameters = art::EDAnalyzer::Table<Config>;
    explicit StatusG4Analyzer(const Parameters& conf);

    virtual void analyze(const art::Event& event);
  };

  //================================================================
  StatusG4Analyzer::StatusG4Analyzer(const Parameters& conf)
    : art::EDAnalyzer(conf)
    , input_(conf().input())
  {
    art::ServiceHandle<art::TFileService> tfs;
    hStatusValue_     = tfs->make<TH1D>("statusValue", "Non-zero values of the G4 status", 20, 0., 20.);
    hNG4Tracks_       = tfs->make<TH1D>("numG4Tracks", "Number of tracks created by G4", 200, 0., 2000.);
    hNG4TracksLog_    = tfs->make<TH1D>("numG4TracksLog", "log10(Number of tracks created by G4)", 100, 0., 10.);
    hOverflowSimPart_ = tfs->make<TH1D>("overflowSimParticles", "Count of events with SimParticle overflows", 1, -0.5, 0.5);
    hNKilledStep_     = tfs->make<TH1D>( "hNKilledStep",   "Number Killed by Step Limit",              100,  0.,   100.   );
    hRealTime_        = tfs->make<TH1D>( "hRealTime",      "Wall Clock time/event (10 ms ticks);(ms)",  50,  0.,   500.   );
    hRealTimeWide_    = tfs->make<TH1D>( "hRealTimeWide",  "Wall Clock time/event (10 ms ticks);(ms)", 100,  0., 10000.   );
    hCPUTime_         = tfs->make<TH1D>( "hCPUTime",       "CPU  time/event(10 ms ticks);(ms)",         50,  0.,   500.   );
    hCPUTimeWide_     = tfs->make<TH1D>( "hCPUTimeWide",   "CPU  time/event(10 ms ticks);(ms)",        100,  0., 10000.   );
  }

  //================================================================
  void StatusG4Analyzer::analyze(const art::Event& event) {
    const auto sh = event.getValidHandle<StatusG4>(input_);
    if ( sh->status() != 0 ){
      hStatusValue_->Fill(sh->status());
    }
    hNG4Tracks_->Fill(sh->nG4Tracks());
    hNG4TracksLog_->Fill( (sh->nG4Tracks() > 0) ? log10(sh->nG4Tracks()) : 0 );

    if(sh->overflowSimParticles()) hOverflowSimPart_->Fill(0.);
    if ( sh->nKilledStepLimit() > 0 ) { hNKilledStep_->Fill(sh->nKilledStepLimit()); }

    // Convert times to ms;
    double cpu = sh->cpuTime()*1000.;
    double real = sh->realTime()*1000.;
    hRealTime_    ->Fill(real);
    hRealTimeWide_->Fill(real);
    hCPUTime_     ->Fill(cpu);
    hCPUTimeWide_ ->Fill(cpu);
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::StatusG4Analyzer);
