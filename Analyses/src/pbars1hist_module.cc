// Andrei Gaponenko, 2016

#include <exception>                                     // for excep...
#include <iostream>                                             // for opera...
#include <string>                                               // for string
#include <memory>                                               // for uniqu...
#include <typeinfo>                                             // for type_...
#include <utility>                                              // for pair

#include "TH1.h"                                                // for TH1D
#include "TH2.h"                                                // for TH2
#include "fhiclcpp/ParameterSet.h"                              // for Param...
#include "art/Framework/Core/EDAnalyzer.h"                      // for EDAna...
#include "art/Framework/Principal/Event.h"                      // for Event
#include "art/Framework/Core/ModuleMacros.h"                    // for DEFIN...
#include "art_root_io/TFileService.h"                           // for TFile...
#include "canvas/Utilities/InputTag.h"                          // for InputTag
#include "CLHEP/Vector/LorentzVector.h"                         // for HepLo...
#include "CLHEP/Vector/ThreeVector.h"                           // for Hep3V...
#include "CLHEP/Units/SystemOfUnits.h"                          // for degree
#include "ProductionTargetGeom/inc/ProductionTarget.hh"         // for Produ...
#include "GeometryService/inc/GeomHandle.hh"                    // for GeomH...
#include "GlobalConstantsService/inc/GlobalConstantsHandle.hh"  // for Globa...
#include "GlobalConstantsService/inc/ParticleDataTable.hh"      // for Parti...

#include "CLHEP/Vector/Rotation.h"                              // for HepRo...


#include "HepPDT/ParticleData.hh"                               // for Parti...
#include "MCDataProducts/inc/SimParticle.hh"                    // for SimPa...
#include "TAxis.h"                                              // for TAxis
#include "art/Framework/Principal/Handle.h"                     // for Valid...
#include "art/Framework/Services/Registry/ServiceHandle.h"      // for Servi...
#include "canvas/Persistency/Common/Ptr.h"                      // for Ptr
#include "canvas/Persistency/Provenance/EventID.h"              // for opera...
#include "canvas/Utilities/Exception.h"                         // for Excep...
#include "cetlib/map_vector.h"                                  // for map_v...
#include "cetlib_except/exception.h"                            // for opera...
#include "fhiclcpp/exception.h"                                 // for excep...
#include "fhiclcpp/types/AllowedConfigurationMacro.h"           // for Allow...

namespace art {
class Run;
}  // namespace art

namespace mu2e {

  //================================================================
  class pbars1hist : public art::EDAnalyzer {
    art::InputTag particlesTag_;

    const ProductionTarget *targetGeom_;
    const ParticleDataTable *particleTable_;

    // This is a workaround for geometry not being available at beginJob()
    bool booked_;
    TH1 *numPbars_;
    TH1 *pbarMomentum_;
    TH2 *pbarThetaVsMomentum_;
    TH2 *pbarRvsZ_;

    TH1 *pbarParentPDG_;
    TH2 *pbarParentMomentum_;

    //----------------
    CLHEP::Hep3Vector mu2eToTarget_momentum(const CLHEP::Hep3Vector& mu2emom) {
        return targetGeom_->productionTargetRotation() * mu2emom;
    }

    CLHEP::Hep3Vector mu2eToTarget_position(const CLHEP::Hep3Vector& mu2epos) {
      const CLHEP::Hep3Vector rel(mu2epos - targetGeom_->position());
      return targetGeom_->productionTargetRotation() * rel;
    }

  public:
    explicit pbars1hist(const fhicl::ParameterSet& pset);
    virtual void beginRun(const art::Run& run) override;
    virtual void analyze(const art::Event& event) override;
  };

  //================================================================
  pbars1hist::pbars1hist(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , particlesTag_(pset.get<std::string>("particles"))
    , targetGeom_(nullptr)
    , particleTable_(nullptr)
    , booked_(false)
  {}

  //================================================================
  void pbars1hist::beginRun(const art::Run& run) {
    // This is a workaround for geometry not being available at beginJob()
    if(!booked_) {
      booked_ = true;

      GeomHandle<ProductionTarget> gh;
      targetGeom_ = gh.get();

      GlobalConstantsHandle<ParticleDataTable> ph;
      particleTable_ = &(*ph);

      art::ServiceHandle<art::TFileService> tfs;
      numPbars_ = tfs->make<TH1D>("npbar","Number of pbars in event", 5, -0.5, 4.5);

      pbarMomentum_ = tfs->make<TH1D>("pbarMom","pbar momentum", 70, 0., 7000.);
      pbarMomentum_->GetXaxis()->SetTitle("p, MeV/c");

      pbarThetaVsMomentum_ = tfs->make<TH2D>("pbarThetaVsMom","pbar target angle vs momentum", 70, 0., 7000., 18, 0., 180.);
      pbarThetaVsMomentum_->SetOption("colz");
      pbarThetaVsMomentum_->GetXaxis()->SetTitle("p, MeV/c");
      pbarThetaVsMomentum_->GetYaxis()->SetTitle("target theta, degrees");

      pbarRvsZ_ = tfs->make<TH2D>("pbarRvsZ","pbar R vs Z", 32, -80., 80., 8, 0., 4.);
      pbarRvsZ_->SetOption("colz");
      pbarRvsZ_->GetXaxis()->SetTitle("target Z, mm");
      pbarRvsZ_->GetYaxis()->SetTitle("target R, mm");

      // We use automatic binning on the X axis to accommodate an
      // apriory unknown set of parent IDs.  ROOT's kCanRebin is a
      // property of a histogram, not an axis, therefore the other axis
      // may also be auto-rebinned, whether we want it or not.

      pbarParentMomentum_ = tfs->make<TH2D>("pbarParentMomentum", "pbar parent START momentum vs parent name",
                                            1, 0., 0., 900, 0, 9000.);
      pbarParentMomentum_->SetOption("colz");

      pbarParentPDG_ = tfs->make<TH1D>("pbarParentPDG", "pbar parent PDG Id", 1, 0., 0.);
    }
  }

  //================================================================
  void pbars1hist::analyze(const art::Event& event) {
    //GeomHandle<ExtMonFNAL::ExtMon> extmon;

    int pbarCount=0;

    auto particles = event.getValidHandle<SimParticleCollection>(particlesTag_);
    for(const auto& it: *particles) {
      const SimParticle& p = it.second;
      if(p.pdgId() == -2212) {
        ++pbarCount;

        const CLHEP::Hep3Vector& mom = mu2eToTarget_momentum(p.startMomentum());
        const CLHEP::Hep3Vector& pos = mu2eToTarget_position(p.startPosition());

        const double pmag = mom.mag();

        // primary proton direction is (0,0,-1).  Compute pbar angle w.r.t. to that.
        const double theta = 180. - mom.theta()/CLHEP::degree;

        pbarMomentum_->Fill(pmag);
        pbarThetaVsMomentum_->Fill(pmag, theta);

        pbarRvsZ_->Fill(pos.z(), pos.perp());

        const SimParticle& parent = *p.parent();
        auto parentId = parent.pdgId();

        std::string parentName = particleTable_->particle(parentId).ref().name();
        const double parentStartMomentum = parent.startMomentum().vect().mag();
        //const double parentEndMomentum = parent.endMomentum().vect().mag();
        //std::cout<<"parent "<<parentName<<", pstart = "<<parentStartMomentum<<", pend = "<<parentEndMomentum
        //         <<((parentStartMomentum > 8889.)? "LARGE": "")
        //         <<std::endl;

        if(parentStartMomentum > 8889.) {
          std::cout<<"Event "<<event.id()<<": parent "<<parentName<<", pstart = "<<parentStartMomentum<<std::endl;
        }

        pbarParentPDG_->Fill(parentName.c_str(), 1.0);
        pbarParentMomentum_->Fill(parentName.c_str(), parentStartMomentum, 1.0);
      }

      //----------------
      static int validationCount = 0;
      if((p.id().asUint() == 1)&&(validationCount < 5)) {
        ++validationCount;
        std::cout<<"primary proton (pdgId = "<<p.pdgId()<<"):"
                 <<" orig p = "<<p.startMomentum()
                 <<", p in target = "<<mu2eToTarget_momentum(p.startMomentum())
                 <<", orig pos = "<<p.startPosition()
                 <<", pos in target = "<<mu2eToTarget_position(p.startPosition())
                 <<std::endl;
      }

    }

    numPbars_->Fill(pbarCount);
  }

  //================================================================
} // namespace mu2e

DEFINE_ART_MODULE(mu2e::pbars1hist);
