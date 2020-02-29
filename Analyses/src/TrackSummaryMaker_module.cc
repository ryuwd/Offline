// Andrei Gaponenko, 2014

#include <exception>                             // for exception
#include <string>                                       // for string
#include <vector>                                       // for vector
#include <memory>                                       // for unique_ptr
#include <algorithm>                                    // for min, max
#include <typeinfo>                                     // for type_info
#include <utility>                                      // for move

#include "cetlib_except/exception.h"                    // for exception
#include "CLHEP/Vector/ThreeVector.h"                   // for Hep3Vector
#include "art/Framework/Core/EDProducer.h"              // for EDProducer
#include "art/Framework/Core/ModuleMacros.h"            // for DEFINE_ART_MO...
#include "art/Framework/Principal/Event.h"              // for Event
#include "art/Framework/Principal/Handle.h"             // for ValidHandle
#include "RecoDataProducts/inc/KalRepPtrCollection.hh"  // for KalRepPtrColl...
#include "BTrk/KalmanTrack/KalRep.hh"                   // for KalRep
#include "BTrk/KalmanTrack/KalHit.hh"                   // for KalHit
#include "BTrk/TrkBase/TrkHelixUtils.hh"                // for TrkHelixUtils
#include "BTrk/BbrGeom/BbrVectorErr.hh"                 // for BbrVectorErr
#include "GeometryService/inc/GeomHandle.hh"            // for GeomHandle
#include "GeometryService/inc/DetectorSystem.hh"        // for DetectorSystem
#include "GeometryService/inc/VirtualDetector.hh"       // for VirtualDetector
#include "DataProducts/inc/VirtualDetectorId.hh"        // for VirtualDetect...
#include "Mu2eUtilities/inc/toHepPoint.hh"              // for fromHepPoint
#include "RecoDataProducts/inc/TrackSummary.hh"         // for TrackSummaryC...
#include "RecoDataProducts/inc/TrackSummaryRecoMap.hh"  // for TrackSummaryR...
#include "BTrk/BbrGeom/BbrError.hh"                     // for BbrError
#include "BTrk/BbrGeom/HepPoint.h"                      // for HepPoint
#include "BTrk/KalmanTrack/KalSite.hh"                  // for KalSite
#include "BTrk/TrkBase/TrkDifTraj.hh"                   // for TrkDifTraj
#include "BTrk/TrkBase/TrkErrCode.hh"                   // for TrkErrCode
#include "BTrk/TrkBase/TrkHit.hh"                       // for TrkHit
#include "BTrk/TrkBase/TrkT0.hh"                        // for TrkT0

#include "art/Framework/Core/ProducerTable.h"           // for ProducerTable
#include "canvas/Persistency/Common/Assns.h"            // for Assns
#include "canvas/Persistency/Common/Ptr.h"              // for Ptr
#include "canvas/Persistency/Provenance/ProductID.h"    // for ProductID
#include "canvas/Utilities/InputTag.h"                  // for InputTag, ope...
#include "fhiclcpp/exception.h"                         // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"   // for AllowedConfig...
#include "fhiclcpp/types/Atom.h"                        // for Atom
#include "fhiclcpp/types/Comment.h"                     // for Comment
#include "fhiclcpp/types/Name.h"                        // for Name

namespace art {
class EDProductGetter;
}  // namespace art

namespace mu2e {

  class TrackSummaryMaker : public art::EDProducer {
  public:

    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;
      fhicl::Atom<art::InputTag> trackInput{ Name("trackInput"), Comment("The input collection.") };
    };

    using Parameters = art::EDProducer::Table<Config>;
    explicit TrackSummaryMaker(const Parameters& conf);

    void produce(art::Event& evt) override;
  private:
    art::InputTag trackInput_;
  };

  //================================================================
  TrackSummaryMaker::TrackSummaryMaker(const Parameters& conf)
    : art::EDProducer{conf}
    , trackInput_(conf().trackInput())
  {
    produces<TrackSummaryCollection>();
    produces<TrackSummaryRecoMap>();
  }

  //================================================================
  void TrackSummaryMaker::produce(art::Event& event) {
    GeomHandle<DetectorSystem> det;
    GeomHandle<VirtualDetector> vdg;

    std::unique_ptr<TrackSummaryCollection> output(new TrackSummaryCollection());
    std::unique_ptr<TrackSummaryRecoMap> recomap(new TrackSummaryRecoMap());

    const art::ProductID trackSummaryPID = event.getProductID<TrackSummaryCollection>();
    const art::EDProductGetter *trackSummaryGetter = event.productGetter(trackSummaryPID);

    auto ih = event.getValidHandle<KalRepPtrCollection>(trackInput_);
    for(unsigned itrack=0; itrack<ih->size(); ++itrack) {
      const auto& krep = (*ih)[itrack];
      if(krep->fitCurrent()){
        TrackSummary sum(krep->fitStatus().success(),
                         krep->charge(), krep->nActive(),
                         krep->nDof(), krep->chisq(),
                         krep->t0().t0(), krep->t0().t0Err(),
                         krep->flt0());

        // The following code is based on KalFitMC.cc
        CLHEP::Hep3Vector entpos = det->toDetector(vdg->getGlobal(VirtualDetectorId::TT_FrontPA));
        double zent = entpos.z();
        double firsthitfltlen = krep->firstHit()->kalHit()->hit()->fltLen();
        double lasthitfltlen = krep->lastHit()->kalHit()->hit()->fltLen();
        double entlen = std::min(firsthitfltlen,lasthitfltlen);
        if(!TrkHelixUtils::findZFltlen(krep->traj(),zent,entlen,0.1)) {
          throw cet::exception("RUNTIME")<<"Error from findZFltlen()\n";
        }

        double loclen(0.0);
        TrackSummary::HelixParams helix(*krep->localTrajectory(entlen,loclen));
        TrackSummary::TrackStateAtPoint st(helix,
                                           fromHepPoint(krep->position(entlen)),
                                           krep->momentum(entlen),
                                           krep->momentumErr(entlen).covMatrix(),
                                           krep->arrivalTime(entlen),
                                           entlen
                                           );

        sum.addState(st);

        recomap->addSingle(art::Ptr<KalRepPtr>(ih, itrack),
                           art::Ptr<TrackSummary>(trackSummaryPID, output->size(), trackSummaryGetter));

        output->emplace_back(sum);
      }
      else {
        throw cet::exception("BADINPUT")<<"TrackSummaryMaker: do not know what to do with a fitCurrent==0 track\n";
      }
    }

    event.put(std::move(output));
    event.put(std::move(recomap));
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::TrackSummaryMaker);
