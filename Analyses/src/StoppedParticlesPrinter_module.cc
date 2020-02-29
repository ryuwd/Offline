// Print out particles from a SimParticlePtrCollections into a text file.
//
// Andrei Gaponenko, 2013

#include <exception>                                // for exception
#include <string>                                          // for string
#include <fstream>                                         // for operator<<
#include <iomanip>                                         // for operator<<
#include <memory>                                          // for unique_ptr
#include <typeinfo>                                        // for type_info
#include <vector>                                          // for vector

#include "art/Framework/Core/EDAnalyzer.h"                 // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"               // for DEFINE_ART...
#include "art/Framework/Principal/Event.h"                 // for Event
#include "art/Framework/Principal/Handle.h"                // for ValidHandle
#include "canvas/Utilities/InputTag.h"                     // for InputTag
#include "MCDataProducts/inc/SimParticlePtrCollection.hh"  // for SimParticl...
#include "CLHEP/Vector/ThreeVector.h"                      // for Hep3Vector

#include "MCDataProducts/inc/SimParticle.hh"               // for SimParticle
#include "canvas/Persistency/Common/Ptr.h"                 // for Ptr
#include "fhiclcpp/ParameterSet.h"                         // for ParameterSet
#include "fhiclcpp/exception.h"                            // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"      // for AllowedCon...

namespace mu2e {

  //================================================================
  class StoppedParticlesPrinter : public art::EDAnalyzer {
  public:
    explicit StoppedParticlesPrinter(fhicl::ParameterSet const& pset);
    void analyze(const art::Event& evt) override;
  private:
    art::InputTag input_;
    std::ofstream outFile_;
  };

  //================================================================
  StoppedParticlesPrinter::StoppedParticlesPrinter(const fhicl::ParameterSet& pset):
    art::EDAnalyzer(pset),
    input_(pset.get<std::string>("inputCollection"))
  {
    const std::string outFileName(pset.get<std::string>("outFileName"));
    outFile_.open(outFileName.c_str());
    if(pset.get<bool>("writeBeginDataMarker", true)) {
      outFile_<< "begin data" << std::endl;
    }
  }

  //================================================================
  void StoppedParticlesPrinter::analyze(const art::Event& event) {
    auto ih = event.getValidHandle<SimParticlePtrCollection>(input_);
    for(const auto& p : *ih) {
      outFile_<< std::setprecision(8)
              << p->endPosition().x() << " "
              << p->endPosition().y() << " "
              << p->endPosition().z() << " "
              << p->endGlobalTime()
              << std::endl;
    }
  }

  //================================================================

} // namespace mu2e

DEFINE_ART_MODULE(mu2e::StoppedParticlesPrinter);
