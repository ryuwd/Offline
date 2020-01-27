// Andrei Gaponenko, 2013

#include <utility>

#include "Mu2eUtilities/inc/SimParticleCollectionPrinter.hh"
#include "CLHEP/Vector/LorentzVector.h"
#include "CLHEP/Vector/ThreeVector.h"
#include "MCDataProducts/inc/ProcessCode.hh"
#include "boost/lexical_cast/bad_lexical_cast.hpp"
#include "boost/numeric/conversion/converter_policies.hpp"
#include "cetlib/map_vector.h"
#include "fhiclcpp/exception.h"

namespace mu2e {
  //================================================================
  SimParticleCollectionPrinter::SimParticleCollectionPrinter(const fhicl::ParameterSet& pset)
    : prefix_(pset.get<std::string>("prefix", ""))
    , enabled_(pset.get<bool>("enabled", true))
    , primariesOnly_(pset.get<bool>("primariesOnly", false))
  {}

  //================================================================
  std::ostream& SimParticleCollectionPrinter::print(std::ostream& os, const SimParticle& p) {
    os<<" id = "<<p.id()
      <<", parent="<<p.parentId()
      <<", pdgId="<<p.pdgId()
      <<", start="<<p.startPosition()
      <<", pstart="<<p.startMomentum()
      <<", end="<<p.endPosition()
      <<", pend="<<p.endMomentum()
      <<", nSteps = "<<p.nSteps()
      <<", preLastStepKE = "<<p.preLastStepKineticEnergy()
      <<", creationCode = "<<p.creationCode()
      <<", stoppingCode = "<<p.stoppingCode()
      <<", startG4Status = "<<p.startG4Status()
      <<", endG4Status = "<<p.endG4Status();
    return os;
  }

  //================================================================
  std::ostream& SimParticleCollectionPrinter::print(std::ostream& os, const SimParticleCollection& particles) const {
    if(enabled_) {
      for(const auto& i: particles) {
        const SimParticle& p{i.second};
        if(!primariesOnly_ || p.isPrimary()) {
          os<<prefix_;
          print(os, p);
          os<<std::endl;
        }
      }
    }

    return os;
  }

  //================================================================
  fhicl::ParameterSet SimParticleCollectionPrinter::defaultPSet() {
    fhicl::ParameterSet res;
    res.put<bool>("enabled", false);
    return res;
  }

  //================================================================
}
