// Write out info from a StepPointMC collection for re-sampling in the following stage.
//
// Andrei Gaponenko, 2015

#include <exception>                                 // for exception
#include <string>                                           // for string
#include <memory>                                           // for unique_ptr
#include <typeinfo>                                         // for type_info
#include <vector>                                           // for vector

#include "cetlib_except/exception.h"                        // for operator<<
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Principal/Handle.h"                 // for ValidHandle
#include "canvas/Utilities/InputTag.h"                      // for InputTag
#include "art_root_io/TFileService.h"                       // for TFileService
#include "MCDataProducts/inc/StepPointMC.hh"                // for StepPointMC
#include "GeneralUtilities/inc/RSNTIO.hh"                   // for InFlightP...
#include "TTree.h"                                          // for TTree
#include "CLHEP/Vector/ThreeVector.h"                       // for Hep3Vector

#include "MCDataProducts/inc/SimParticle.hh"                // for SimParticle
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Common/Ptr.h"                  // for Ptr
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace mu2e {

  //================================================================
  class InFlightStepDumper : public art::EDAnalyzer {
  public:
    explicit InFlightStepDumper(fhicl::ParameterSet const& pset);
    void beginJob() override;
    void analyze(const art::Event& evt) override;
  private:
    art::InputTag input_;
    TTree *nt_;
    int pie_; // particle number in the current event
    IO::InFlightParticleD data_;
  };

  //================================================================
  InFlightStepDumper::InFlightStepDumper(const fhicl::ParameterSet& pset)
    : art::EDAnalyzer(pset)
    , input_(pset.get<std::string>("inputCollection"))
    , nt_()
    , pie_()
  {}

  //================================================================
  void InFlightStepDumper::beginJob() {
    art::ServiceHandle<art::TFileService> tfs;
    nt_ = tfs->make<TTree>( "particles", "In-flight particles ntuple");
    nt_->Branch("particles", &data_, IO::InFlightParticleD::branchDescription());
    nt_->Branch("pie", &pie_);
  }

  //================================================================
  void InFlightStepDumper::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<StepPointMCCollection>(input_);
    pie_ = 0;
    for(const auto& hit : *ih) {
      data_.x = hit.position().x();
      data_.y = hit.position().y();
      data_.z = hit.position().z();

      data_.time = hit.time();

      data_.px = hit.momentum().x();
      data_.py = hit.momentum().y();
      data_.pz = hit.momentum().z();

      data_.pdgId = hit.simParticle()->pdgId();

      nt_->Fill();
      ++pie_;
    }
  }

  //================================================================
} // namespace mu2e

DEFINE_ART_MODULE(mu2e::InFlightStepDumper);
