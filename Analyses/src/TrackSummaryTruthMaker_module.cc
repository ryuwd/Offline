// Andrei Gaponenko, 2014

#include <exception>                                // for exception
#include <string>                                          // for string
#include <vector>                                          // for vector
#include <memory>                                          // for unique_ptr
#include <algorithm>                                       // for max
#include <map>                                             // for map, map<>...
#include <set>                                             // for set
#include <typeinfo>                                        // for type_info
#include <utility>                                         // for move, pair

#include "cetlib_except/exception.h"                       // for exception
#include "art/Framework/Core/EDProducer.h"                 // for EDProducer
#include "art/Framework/Core/ModuleMacros.h"               // for DEFINE_ART...
#include "art/Framework/Principal/Event.h"                 // for Event
#include "art/Framework/Principal/Handle.h"                // for ValidHandle
#include "MCDataProducts/inc/StrawDigiMC.hh"               // for StrawDigiMC
#include "RecoDataProducts/inc/TrackSummaryRecoMap.hh"     // for TrackSumma...
#include "BTrkData/inc/TrkStrawHit.hh"                     // for TrkStrawHit
#include "MCDataProducts/inc/TrackSummaryTruthAssns.hh"    // for TrackSumma...
#include "MCDataProducts/inc/SimParticlePtrCollection.hh"  // for SimParticl...
#include "Mu2eUtilities/inc/particleEnteringG4Volume.hh"   // for particleEn...
#include "BTrk/KalmanTrack/KalRep.hh"                      // for KalRep
#include "BTrk/TrkBase/TrkHit.hh"                          // for TrkHit
#include "DataProducts/inc/StrawEnd.hh"                    // for StrawEnd
#include "DataProducts/inc/StrawId.hh"                     // for operator<<
#include "TrackerGeom/inc/Straw.hh"                        // for Straw
#include "canvas/Persistency/Common/Assns.h"               // for Assns, Ass...
#include "canvas/Persistency/Common/Ptr.h"                 // for Ptr, opera...
#include "canvas/Utilities/InputTag.h"                     // for InputTag
#include "fhiclcpp/ParameterSet.h"                         // for ParameterSet
#include "fhiclcpp/exception.h"                            // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"      // for AllowedCon...

namespace mu2e {
struct SimParticle;

  class TrackSummaryTruthMaker : public art::EDProducer {
  public:
    explicit TrackSummaryTruthMaker(fhicl::ParameterSet const& pset);
    void produce(art::Event& evt) override;
  private:
    art::InputTag recoMapInput_;
    art::InputTag strawHitDigiMCInput_;

    // Cuts for storing a match
    unsigned minPrincipalHits_;
    unsigned minAllHits_;
  };

  //================================================================
  TrackSummaryTruthMaker::TrackSummaryTruthMaker(const fhicl::ParameterSet& pset)
    : art::EDProducer{pset}
    , recoMapInput_(pset.get<std::string>("recoMapInput"))
    , strawHitDigiMCInput_(pset.get<std::string>("strawHitDigiMCInput"))
    , minPrincipalHits_(pset.get<unsigned>("minPrincipalHits"))
    , minAllHits_(pset.get<unsigned>("minAllHits"))
  {
    produces<TrackSummaryTruthAssns>();
    produces<SimParticlePtrCollection>(); // list of particles of interest for filtering
  }

  //================================================================
  void TrackSummaryTruthMaker::produce(art::Event& event) {
    std::unique_ptr<SimParticlePtrCollection> spptrs(new SimParticlePtrCollection());
    std::unique_ptr<TrackSummaryTruthAssns> output(new TrackSummaryTruthAssns());
    auto ireco = event.getValidHandle<TrackSummaryRecoMap>(recoMapInput_);
    auto imc = event.getValidHandle<StrawDigiMCCollection>(strawHitDigiMCInput_);

    StrawEnd end(StrawEnd::cal);

    typedef std::map<art::Ptr<SimParticle>, unsigned> PerParticleCount;
    PerParticleCount nPrincipal;
    PerParticleCount nAll;

    for(const auto recoMapEntry: *ireco) {
      const KalRep& krep = **recoMapEntry.first;
      for(const auto hot : krep.hitVector()) {
        const TrkStrawHit *hit = dynamic_cast<const TrkStrawHit*>(hot);
        if(!hit) {
          throw cet::exception("BADINPUT")<<"TrackSummaryTruthMaker: an entry in KalRep::hitVector() is not a TrkStrawHit.\n";
        }

        const StrawDigiMC& dmc = imc->at(hit->index());
        if(hit->straw().id() != dmc.strawId()) {
          throw cet::exception("BADINPUTS")<<"TrackSummaryTruthMaker: mismatched input data: "
                                           <<"straw id="<<hit->straw().id()
                                           <<" != StrawDigiMC index="<<dmc.strawId()
                                           <<"\n";
        }

        if(hit->isActive()) {
          ++nPrincipal[particleEnteringG4Volume(*dmc.strawGasStep(end))];
          // Aggregate all the steps, so that each particle is counted no more than once per hit
          std::set<art::Ptr<SimParticle> > parts;
	  parts.insert(particleEnteringG4Volume(*dmc.strawGasStep(end)));
          for(const auto& p: parts) {
            ++nAll[p];
          }
        }
      }

      // The "principal" particles should form a subset of "all"
      // Therefore it is sufficient to iterate over "all".
      for(const auto& p : nAll) {
        TrackSummaryMatchInfo mi(nPrincipal[p.first], p.second);
        if((mi.nPrincipal() >= minPrincipalHits_) || (mi.nAll() >= minAllHits_)) {
          output->addSingle(p.first, recoMapEntry.second, mi);
          spptrs->emplace_back(p.first);
        }
      }
    }

    event.put(std::move(output));
    event.put(std::move(spptrs));
  }

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::TrackSummaryTruthMaker);
