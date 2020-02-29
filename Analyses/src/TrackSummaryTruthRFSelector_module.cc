// Selector modules picks the "best" match from the many-to-many map
// produced by TrackSummaryTruthMaker.  The "RF" selector uses
// the fraction of (common/reco) hits as its FOM.
//
// Andrei Gaponenko, 2014

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <vector>                                           // for vector<>:...
#include <memory>                                           // for unique_ptr
#include <algorithm>                                        // for max, min
#include <map>                                              // for _Rb_tree_...
#include <type_traits>                                      // for __decay_a...
#include <typeinfo>                                         // for type_info
#include <utility>                                          // for pair, mak...

#include "cetlib_except/exception.h"                        // for operator<<
#include "art/Framework/Core/EDProducer.h"                  // for EDProducer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "RecoDataProducts/inc/TrackSummary.hh"             // for TrackSummary
#include "MCDataProducts/inc/TrackSummaryTruthAssns.hh"     // for TrackSumm...
#include "TH1.h"                                            // for TH1D, TH1
#include "TH2.h"                                            // for TH2, TH2D
#include "canvas/Persistency/Common/Assns.h"                // for Assns
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr, oper...
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace mu2e {
struct SimParticle;

  // The "nice" declaration does not work. Similar to redmine #6420
  // double commonHitFraction(typename TrackSummaryTruthAssns::assn_t& p, const TrackSummaryMatchInfo& mi) {
  // Spell things out by hand for now
  double commonHitFraction(const std::pair<art::Ptr<SimParticle>, art::Ptr<TrackSummary> >& p, const TrackSummaryMatchInfo& mi) {
    unsigned numRecoHits = p.second->nactive();
    return double(mi.nPrincipal())/double(numRecoHits);
  }

  class TrackSummaryTruthRFSelector : public art::EDProducer {
  public:
    explicit TrackSummaryTruthRFSelector(fhicl::ParameterSet const& pset);
    void produce(art::Event& evt) override;
  private:
    art::InputTag input_;
    double cutMinCommonFraction_;

    TH1 *hInputMapSize_;
    TH1 *hFinalMapSize_;

    TH2 *hSelection_;

    TH1 *hRFBeforeCut_;
    TH1 *hRFFinal_;

    TH1 *hWrongPrincipal_;
    TH1 *hNonContrib_;
  };

  //================================================================
  TrackSummaryTruthRFSelector::TrackSummaryTruthRFSelector(const fhicl::ParameterSet& pset)
    : art::EDProducer{pset}
    , input_(pset.get<std::string>("TrackTruthInput"))
    , cutMinCommonFraction_(pset.get<double>("cutMinCommonFraction"))
  {
    produces<TrackSummaryTruthAssns>();
    art::ServiceHandle<art::TFileService> tfs;

    hInputMapSize_ = tfs->make<TH1D>("inputMapSize", "input map size", 50, -0.5, 49.5);
    hFinalMapSize_ = tfs->make<TH1D>("finalMapSize", "final map size", 50, -0.5, 49.5);

    hSelection_ = tfs->make<TH2D>("selection", "other vs better", 100, 0., 1.+1.e-6, 100, 0., 1.+1.e-6);
    hSelection_->SetOption("colz");

    hRFBeforeCut_ = tfs->make<TH1D>("matchFracBefore", "(nCommon princ)/nActive, before cut", 100, 0., 1.+1.e-6);
    hRFFinal_ = tfs->make<TH1D>("matchFracFinal", "(nCommon princ)/nActive, final", 100, 0., 1.+1.e-6);

    hWrongPrincipal_ = tfs->make<TH1D>("numWrongPrincipal", "nActive - nCommon principal, final", 50, -0.5, 49.5);
    hNonContrib_ = tfs->make<TH1D>("nonContrib", "nActive - nCommon any, final", 50, -0.5, 49.5);
  }

  //================================================================
  void TrackSummaryTruthRFSelector::produce(art::Event& event) {
    std::unique_ptr<TrackSummaryTruthAssns> out(new TrackSummaryTruthAssns());
    auto ih = event.getValidHandle<TrackSummaryTruthAssns>(input_);

    hInputMapSize_->Fill(ih->size());

    // Select the best match per sim particle.
    // Map value is an index into the original Assns
    typedef std::map<art::Ptr<SimParticle>, unsigned> SimBestMatches;
    SimBestMatches sm;
    for(unsigned i=0; i<ih->size(); ++i) {
      const art::Ptr<SimParticle> part = ih->at(i).first;
      const auto iter = sm.find(part);
      if(iter == sm.end()) {
        sm.insert(std::make_pair(part, i));
      }
      else {
        const double rfOld = commonHitFraction(ih->at(iter->second), ih->data(iter->second));
        const double rfNew = commonHitFraction(ih->at(i), ih->data(i));
        hSelection_->Fill(std::max(rfOld,rfNew), std::min(rfOld, rfNew));
        if(rfOld < rfNew) {
          sm[part] = i;
        }
      }
    }

    // Out of one-per-particle matches, select the best match per
    // track to get a one-to-one correspondence.  Map value is an
    // index into the original Assns
    typedef std::map<art::Ptr<TrackSummary>, unsigned> RecoBestMatches;
    RecoBestMatches sr;
    for(const auto& entry : sm) {
      const art::Ptr<TrackSummary> trk = ih->at(entry.second).second;
      const auto iter = sr.find(trk);
      if(iter == sr.end()) {
        sr.insert(std::make_pair(trk, entry.second));
      }
      else {
        const double rfOld = commonHitFraction(ih->at(iter->second), ih->data(iter->second));
        const double rfNew = commonHitFraction(ih->at(entry.second), ih->data(entry.second));
        hSelection_->Fill(std::max(rfOld,rfNew), std::min(rfOld, rfNew));
        if(rfOld < rfNew) {
          sr[trk] = entry.second;
        }
      }
    }

    // Discard remaining bad matches and fill the output collection
    for(const auto& entry: sr) {
      const double rf = commonHitFraction(ih->at(entry.second), ih->data(entry.second));
      hRFBeforeCut_->Fill(rf);
      if(rf > cutMinCommonFraction_) {

        const art::Ptr<SimParticle>& part = ih->at(entry.second).first;
        const art::Ptr<TrackSummary>& trk = ih->at(entry.second).second;
        const TrackSummaryMatchInfo& mi = ih->data(entry.second);

        out->addSingle(part, trk, mi);

        hRFFinal_->Fill(rf);
        hWrongPrincipal_->Fill(trk->nactive() - mi.nPrincipal());
        hNonContrib_->Fill(trk->nactive() - mi.nAll());
      }
    }

    hFinalMapSize_->Fill(out->size());

    event.put(std::move(out));
  }

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::TrackSummaryTruthRFSelector);
