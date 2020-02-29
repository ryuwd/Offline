//
// Test of Ptr to GenParticles and SimParticles.
//
// $Id: PtrTest0_module.cc,v 1.8 2013/10/21 20:44:04 genser Exp $
// $Author: genser $
// $Date: 2013/10/21 20:44:04 $
//
// Original author Rob Kutschke
//

#include <ext/alloc_traits.h>                          // for __alloc_traits...
#include <stddef.h>                                    // for size_t
// C++ includes.
#include <iostream>                                    // for operator<<
#include <vector>                                      // for vector, vector...
#include <algorithm>                                   // for max
#include <memory>                                      // for unique_ptr
#include <string>                                      // for string
#include <typeinfo>                                    // for type_info
#include <utility>                                     // for pair

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "art/Framework/Principal/Event.h"             // for Event
#include "art/Framework/Principal/Handle.h"            // for Handle
#include "canvas/Persistency/Common/Ptr.h"             // for Ptr


#include "MCDataProducts/inc/GenId.hh"                 // for operator<<
#include "MCDataProducts/inc/GenParticle.hh"           // for GenParticleCol...
#include "MCDataProducts/inc/SimParticle.hh"           // for SimParticle
#include "cetlib/map_vector.h"                         // for map_vector<>::...
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...

namespace fhicl {
class ParameterSet;
}  // namespace fhicl

using namespace std;

namespace mu2e {

  class PtrTest0 : public art::EDAnalyzer {
  public:

    explicit PtrTest0(fhicl::ParameterSet const& pset);
    virtual ~PtrTest0() { }

    // The framework calls this for each event.
    void analyze(const art::Event& e);

  private:

  };

  PtrTest0::PtrTest0(fhicl::ParameterSet const& pset) :
    art::EDAnalyzer(pset)
  {}

  void PtrTest0::analyze(const art::Event& event) {


    art::Handle<GenParticleCollection> gensHandle;
    event.getByLabel("generate",gensHandle);
    GenParticleCollection const& gens = *gensHandle;

    art::Handle<SimParticleCollection> simsHandle;
    event.getByLabel("g4run",simsHandle);
    SimParticleCollection const& sims = *simsHandle;

    // Fill a vector of Ptr into a collection that, under the covers, is a std::vector<T>.
    vector<art::Ptr<GenParticle> > genPtrs;
    for ( GenParticleCollection::const_iterator i=gens.begin();
          i!=gens.end(); ++i){
      GenParticle const& gen = *i;
      size_t offset = &(*i)-&gens.front();
      cout << "Gen: "
           << offset << " "
           << gen.pdgId() <<  " "
           << gen.generatorId() << " "
           << gen.time() << " "
           << endl;
      genPtrs.push_back( art::Ptr<GenParticle>(gensHandle,offset) );
    }

    // Read back the Ptrs.
    for ( size_t i=0; i< genPtrs.size(); ++i ){
      GenParticle const& gen = *genPtrs.at(i);
      cout << "Gen from Ptr: "
           << i << " "
           << gen.pdgId()       <<  " "
           << gen.generatorId() << " "
           << gen.time()
           << endl;
    }

    // Fill a vector of Ptr into a collection that, under the covers, is a cet::map_vector<T>.
    vector<art::Ptr<SimParticle> > simPtrs;
    for ( SimParticleCollection::const_iterator i = sims.begin();
          i != sims.end(); ++i ){

      SimParticleCollection::key_type key = i->first;
      SimParticle const& sim = i->second;

      if ( sim.fromGenerator() ){
        cout << "Sim: "
             << key.asInt()          << " "
             << sim.id().asInt()     << " "
             << sim.generatorIndex() << " "
             << sim.startGlobalTime() << " "
             << endl;

        simPtrs.push_back( art::Ptr<SimParticle>() );
        simPtrs.back() = art::Ptr<SimParticle>(simsHandle,key.asInt());


      }
    } // end loop filling the Ptr to SimParticles


    // Read back the Ptrs.
    for ( size_t i=0; i< simPtrs.size(); ++i ){

      SimParticle const& sim = *simPtrs.at(i);

      cout << "Sim from Ptr: "
           << i << " "
           << sim.id().asInt()     << " "
           << sim.generatorIndex() << " "
           << sim.startGlobalTime() << " "
           << endl;

    }


  } // end PtrTest0::analyze



}  // end namespace mu2e

using mu2e::PtrTest0;
DEFINE_ART_MODULE(PtrTest0);
