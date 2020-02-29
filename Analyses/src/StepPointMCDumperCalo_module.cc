// Ntuple dumper for StepPointMCs.
//
// Andrei Gaponenko, 2013

#include <exception>                                     // for excep...
#include <string>                                               // for string
#include <cmath>                                                // for sqrt
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
#include "art/Framework/Principal/Handle.h"                     // for Valid...
#include "art/Framework/Services/Registry/ServiceHandle.h"      // for Servi...
#include "canvas/Persistency/Common/Ptr.h"                      // for Ptr
#include "canvas/Utilities/Exception.h"                         // for Excep...
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
  class StepPointMCDumperCalo : public art::EDAnalyzer
  {

    public:
      explicit StepPointMCDumperCalo(const fhicl::ParameterSet& pset);
      virtual void beginJob();
      virtual void analyze(const art::Event& event);


    private:

      art::InputTag           hitsInputTag_;
      SimParticleTimeOffset   toff_;
      int _nProcess;


      TTree *nt_;

      int   _nStep, _stepCharge[16384],_stepPdgId[16384],_stepPartId[16384],_stepVolumeId[16384],_stepCrCode[16384];
      float _stepX[16384],_stepY[16384],_stepZ[16384],_stepT[16384],_stepPx[16384],_stepPy[16384],_stepPz[16384],_stepEk[16384],_stepEdep[16384];

  };



  //================================================================
  StepPointMCDumperCalo::StepPointMCDumperCalo(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , hitsInputTag_(pset.get<std::string>("hitsInputTag"))
    , toff_(pset.get<fhicl::ParameterSet>("TimeOffsets"))
    , _nProcess(0)
    , nt_(0)

  {}

  //================================================================
  void StepPointMCDumperCalo::beginJob()
  {

      art::ServiceHandle<art::TFileService> tfs;
      nt_ = tfs->make<TTree>( "nt", "StepPointMCDumperCalo ntuple");

      nt_->Branch("nStep",        &_nStep ,        "nStep/I");
      nt_->Branch("stepX",        &_stepX,         "stepX[nStep]/F");
      nt_->Branch("stepY",        &_stepY,         "stepY[nStep]/F");
      nt_->Branch("stepZ",        &_stepZ,         "stepZ[nStep]/F");
      nt_->Branch("stepT",        &_stepT,         "stepT[nStep]/F");
      nt_->Branch("stepPx",       &_stepPx,        "stepPx[nStep]/F");
      nt_->Branch("stepPy",       &_stepPy,        "stepPy[nStep]/F");
      nt_->Branch("stepPz",       &_stepPz,        "stepPz[nStep]/F");
      nt_->Branch("stepEk",       &_stepEk,        "stepEk[nStep]/F");
      nt_->Branch("stepEdep",     &_stepEdep,      "stepEdep[nStep]/F");
      nt_->Branch("stepCharge",   &_stepCharge,    "stepCharge[nStep]/I");
      nt_->Branch("stepPdgId",    &_stepPdgId,     "stepPdgId[nStep]/I");
      nt_->Branch("stepPartId",   &_stepPartId,    "stepPartId[nStep]/I");
      nt_->Branch("stepVolId",    &_stepVolumeId,  "stepVolId[nStep]/I");
      nt_->Branch("stepCrCode",   &_stepCrCode,    "stepCrCode[nStep]/I");
   }





  //================================================================
  void StepPointMCDumperCalo::analyze(const art::Event& event)
  {

    ++_nProcess;
    toff_.updateMap(event);
    const auto& ih = event.getValidHandle<StepPointMCCollection>(hitsInputTag_);
    StepPointMCCollection const& hits(*ih);

    _nStep = hits.size();

    for(unsigned int i=0; i < hits.size(); ++i)
    {
           const auto& hit = hits[i];
           art::Ptr<SimParticle> grandMother = hit.simParticle();
           while (grandMother->startPosition().z() > 11840 && grandMother->hasParent())grandMother = grandMother->parent();
           //while (grandMother->hasParent()){ std::cout<< grandMother->pdgId()<<" "<<grandMother->startPosition().z()  <<std::endl; grandMother = grandMother->parent();}
           //std::cout<<"--"<<std::endl;

	   _stepX[i]        = hit.position().x();
	   _stepY[i]        = hit.position().y();
	   _stepZ[i]        = hit.position().z();
	   _stepT[i]        = toff_.timeWithOffsetsApplied(hit);

	   _stepPx[i]       = hit.momentum().x();
	   _stepPy[i]       = hit.momentum().y();
	   _stepPz[i]       = hit.momentum().z();
	   _stepEk[i]       = getKineticEnergy(hit);
	   _stepEdep[i]     = hit.totalEDep();

	   _stepCharge[i]   = getCharge(hit.simParticle()->pdgId());
	   _stepPdgId[i]    = hit.simParticle()->pdgId();
	   _stepPartId[i]   = hit.simParticle()->id().asUint();
	   _stepVolumeId[i] = hit.volumeId();
	   _stepCrCode[i]   = grandMother->pdgId();

     }

    nt_->Fill();


  }

    //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::StepPointMCDumperCalo);
