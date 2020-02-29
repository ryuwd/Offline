// Ntuple dumper for 1st StepPointMCs.
//
// Nam Tran, 2018

#include <exception>                                     // for excep...
#include <string>                                               // for string
#include <vector>                                               // for vecto...
#include <limits>                                               // for numer...
#include <cmath>                                                // for sqrt
#include <algorithm>                                            // for max
#include <memory>                                               // for uniqu...
#include <typeinfo>                                             // for type_...

#include "cetlib_except/exception.h"                            // for excep...
#include "CLHEP/Vector/ThreeVector.h"                           // for Hep3V...
#include "TTree.h"                                              // for TTree
#include "canvas/Utilities/InputTag.h"                          // for InputTag
#include "fhiclcpp/ParameterSet.h"                              // for Param...
#include "art/Framework/Core/EDAnalyzer.h"                      // for EDAna...
#include "art/Framework/Principal/Event.h"                      // for Event
#include "art/Framework/Core/ModuleMacros.h"                    // for DEFIN...
#include "art_root_io/TFileService.h"                           // for TFile...
#include "MCDataProducts/inc/StepPointMC.hh"                    // for StepP...
#include "GlobalConstantsService/inc/GlobalConstantsHandle.hh"  // for Globa...
#include "GlobalConstantsService/inc/ParticleDataTable.hh"      // for Parti...
#include "Mu2eUtilities/inc/SimParticleTimeOffset.hh"           // for SimPa...

#include "DataProducts/inc/PDGCode.hh"                          // for PDGCode
#include "HepPDT/Measurement.hh"                                // for Measu...

#include "HepPDT/ParticleData.hh"                               // for Parti...
#include "MCDataProducts/inc/SimParticle.hh"                    // for SimPa...
#include "art/Framework/Principal/Handle.h"                     // for Handle
#include "art/Framework/Services/Registry/ServiceHandle.h"      // for Servi...
#include "canvas/Persistency/Common/Ptr.h"                      // for Ptr
#include "canvas/Persistency/Provenance/EventID.h"              // for EventID
#include "canvas/Utilities/Exception.h"                         // for Excep...
#include "fhiclcpp/coding.h"                                    // for ps_se...
#include "fhiclcpp/exception.h"                                 // for excep...
#include "fhiclcpp/types/AllowedConfigurationMacro.h"           // for Allow...

namespace mu2e {

  //================================================================
  double getCharge(PDGCode::type pdgId) {
    // unlike generic conditions, MC particle data
    // should not change run-to-run, so static is safe
    // use static for efficiency
    static GlobalConstantsHandle<ParticleDataTable> pdt;

    ParticleDataTable::maybe_ref info = pdt->particle(pdgId);

    if(!info.isValid()) {
      throw cet::exception("MISSINGINFO")<<"No valid PDG info for pdgId = "<<pdgId<<"\n";
    }

    return info.ref().charge();
  }

  //================================================================
  double getKineticEnergy(const StepPointMC& hit) {
    // unlike generic conditions, MC particle data
    // should not change run-to-run, so static is safe
    // use static for efficiency
    static GlobalConstantsHandle<ParticleDataTable> pdt;

    ParticleDataTable::maybe_ref info = pdt->particle(hit.simParticle()->pdgId());

    if(!info.isValid()) {
      throw cet::exception("MISSINGINFO")<<"No valid PDG info for hit = "<<hit<<"\n";
    }

    const double mass = info.ref().mass();
    return sqrt(hit.momentum().mag2() + std::pow(mass, 2)) - mass;
  }

  //================================================================
  struct VDHit {
    float x;
    float y;
    float z;

    float InitX;
    float InitY;
    float InitZ;

    float time;

    float px;
    float py;
    float pz;
    float pmag;
    float ek;

    float charge;
    int   pdgId;
    unsigned particleId;
    unsigned volumeCopyNumber;
    int eventId;
    int subrunId;

    VDHit() : x(std::numeric_limits<double>::quiet_NaN())
            , y(std::numeric_limits<double>::quiet_NaN())
            , z(std::numeric_limits<double>::quiet_NaN())

	    , InitX(std::numeric_limits<double>::quiet_NaN())
	    , InitY(std::numeric_limits<double>::quiet_NaN())
            , InitZ(std::numeric_limits<double>::quiet_NaN())

            , time(std::numeric_limits<double>::quiet_NaN())

            , px(std::numeric_limits<double>::quiet_NaN())
            , py(std::numeric_limits<double>::quiet_NaN())
            , pz(std::numeric_limits<double>::quiet_NaN())
            , pmag(std::numeric_limits<double>::quiet_NaN())
            , ek(std::numeric_limits<double>::quiet_NaN())

      , charge(std::numeric_limits<double>::quiet_NaN())
      , pdgId(0)
      , particleId(-1U)
      , volumeCopyNumber(-1U)
      , eventId(0)
      , subrunId(0)
    {}

    //----------------------------------------------------------------
    VDHit(const SimParticleTimeOffset& toff, const art::Event& event, const StepPointMC& hit)
      : x(hit.position().x())
      , y(hit.position().y())
      , z(hit.position().z())

      , InitX(hit.simParticle()->startPosition().x())
      , InitY(hit.simParticle()->startPosition().y())
      , InitZ(hit.simParticle()->startPosition().z())

      , time(toff.timeWithOffsetsApplied(hit))

      , px(hit.momentum().x())
      , py(hit.momentum().y())
      , pz(hit.momentum().z())

      , pmag(hit.momentum().mag())
      , ek(getKineticEnergy(hit))

      , charge(getCharge(hit.simParticle()->pdgId()))

      , pdgId(hit.simParticle()->pdgId())
      , particleId(hit.simParticle()->id().asUint())
      , volumeCopyNumber(hit.volumeId())

      , eventId(event.id().event())
      , subrunId(event.id().subRun())
    {}

  }; // struct VDHit

  //================================================================
  class StepPointMC1stHitDumper : public art::EDAnalyzer {
    typedef std::vector<std::string> VS;
    typedef std::vector<StepPointMCCollection> VspMC;

    art::InputTag hitsInputTag_;
    SimParticleTimeOffset toff_;

    bool writeProperTime_;
    VS tauHitCollections_;
    std::vector<int> decayOffCodes_;

    // Members needed to write the ntuple
    TTree *nt_;
    VDHit hit_;
    float tau_;

  public:
    explicit StepPointMC1stHitDumper(const fhicl::ParameterSet& pset);
    virtual void beginJob();
    virtual void analyze(const art::Event& event);
  };

  //================================================================
  StepPointMC1stHitDumper::StepPointMC1stHitDumper(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , hitsInputTag_(pset.get<std::string>("hitsInputTag"))
    , toff_(pset.get<fhicl::ParameterSet>("TimeOffsets"))
    , writeProperTime_(pset.get<bool>("writeProperTime", false))
    , tauHitCollections_( writeProperTime_ ? pset.get<VS>("tauHitCollections") : VS() )
    , nt_(0)
    , tau_()
  {
    if(writeProperTime_) {
      decayOffCodes_ = pset.get<std::vector<int> >("decayOffPDGCodes");
      // must sort to use binary_search in SimParticleGetTau
      std::sort(decayOffCodes_.begin(), decayOffCodes_.end());
    }
  }

  //================================================================
  void StepPointMC1stHitDumper::beginJob() {
    art::ServiceHandle<art::TFileService> tfs;
    static const char branchDesc[] = "x/F:y/F:z/F:InitX/F:InitY/F:InitZ/F:time/F:px/F:py/F:pz/F:pmag/F:ek/F:charge/F:pdgId/I:particleId/i:volumeCopy/i:eventId/I:subrunId/I";
    nt_ = tfs->make<TTree>( "nt", "StepPointMC1stHitDumper ntuple");
    nt_->Branch("hits", &hit_, branchDesc);
    if(writeProperTime_) {
      nt_->Branch("tau", &tau_, "tauNormalized/F");
    }
  }

  //================================================================
  void StepPointMC1stHitDumper::analyze(const art::Event& event) {
    toff_.updateMap(event);

    VspMC spMCColls;
    for ( const auto& iColl : tauHitCollections_ ){
      auto spColl = event.getValidHandle<StepPointMCCollection>(iColl);
      spMCColls.push_back( *spColl );
    }

    art::Handle<std::vector<mu2e::StepPointMC>> spHndl;
    bool gotIt = event.getByLabel(hitsInputTag_, spHndl);
    if (gotIt) {
      std::vector<mu2e::StepPointMC> stepPoints = *spHndl;
      // for (unsigned int i = 0; i < stepPoints.size(); ++i) {
        // std::cout << event.id() << ": " << stepPoints.at(i) << std::endl;
      // }
      if (stepPoints.size()) {
        hit_ = VDHit(toff_, event, stepPoints.at(0));
        // std::cout << event.id() << ", " << hit_.pdgId
          // << ", " << hit_.time << ", " << hit_.charge
          // << std::endl;
        nt_->Fill();
      }
    }
  } // analyze(event)

    //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::StepPointMC1stHitDumper);
