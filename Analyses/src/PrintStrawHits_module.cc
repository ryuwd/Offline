// Prints out all StrawHits in a collection.
//
// $Id: PrintStrawHits_module.cc,v 1.1 2014/06/27 15:57:03 murat Exp $
// $Author: murat $
// $Date: 2014/06/27 15:57:03 $
//
// Original author Pavel Murat
//

#include <exception>                            // for exception
#include <stdio.h>                                     // for printf
#include <iostream>                                    // for operator<<
#include <string>                                      // for string, operat...
#include <memory>                                      // for unique_ptr
#include <typeinfo>                                    // for type_info
#include <vector>                                      // for vector, vector...

#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Principal/Event.h"             // for Event
#include "art/Framework/Principal/Handle.h"            // for Handle
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "RecoDataProducts/inc/StrawHit.hh"            // for StrawHitCollec...
#include "DataProducts/inc/StrawId.hh"                 // for StrawId
#include "fhiclcpp/ParameterSet.h"                     // for ParameterSet
#include "fhiclcpp/exception.h"                        // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...

namespace mu2e {

  class PrintStrawHits : public art::EDAnalyzer {
  public:

    explicit PrintStrawHits(fhicl::ParameterSet const& pset);

    void analyze(const art::Event& e);

  private:

    // The two strings specify what collection to print

    std::string _makeSHmoduleLabel;
    std::string _makeSHinstance;
    int         _printStrawHits;
  };

  PrintStrawHits::PrintStrawHits(const fhicl::ParameterSet& pset) :
    art::EDAnalyzer   (pset),
    _makeSHmoduleLabel(pset.get<std::string>("makeSHModuleLabel","makeSH")),
    _makeSHinstance   (pset.get<std::string>("makeSHinstance"   ,""      )),
    _printStrawHits   (pset.get<int>        ("printStrawHits"   ,       0))
  {}

  void PrintStrawHits::analyze(const art::Event& event) {

    if (_printStrawHits != 0) {

      art::Handle<StrawHitCollection> h;
      event.getByLabel(_makeSHmoduleLabel, "", h);
      const StrawHitCollection&     coll(*h);

      std::cout<<"PrintStrawHits: begin printing collection moduleLabel = \"" << _makeSHmoduleLabel
	       <<"\", makeSHinstance = \""<<_makeSHinstance<<"\""
	       <<std::endl;
      for (StrawHitCollection::const_iterator i = coll.begin(); i != coll.end(); ++i) {
	printf("Straw hit index: %8i time: %10.3f dt: %10.3f eDep: %10.5f\n",
	       i->strawId().asUint16(), i->time(), i->dt(), i->energyDep());
      }
      std::cout<<"PrintStrawHits:   end printing collection moduleLabel = \""<<_makeSHmoduleLabel
	       <<"\", makeSHinstance = \""<<_makeSHinstance<<"\""
	       <<std::endl;
    }
  } // analyze()

}  // end namespace mu2e

// Register the module with the framework
//using mu2e::PrintStrawHits;
DEFINE_ART_MODULE(mu2e::PrintStrawHits);
