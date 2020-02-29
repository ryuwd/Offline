//
// Write the event ids of all events.
//
// $Id: EventLister_module.cc,v 1.3 2014/06/26 03:40:05 murat Exp $
// $Author: murat $
// $Date: 2014/06/26 03:40:05 $
//
// Original author Rob Kutschke
//

#include <stdio.h>                                     // for printf
// C++ includes.
#include <iostream>                                    // for std

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Principal/Event.h"             // for Event
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...

namespace fhicl {
class ParameterSet;
}  // namespace fhicl

using namespace std;

namespace mu2e {

  class EventLister : public art::EDAnalyzer {
  public:

    explicit EventLister(fhicl::ParameterSet const& pset);

    virtual void analyze ( const art::Event& event);

  private:
    int _ievent;
  };

  EventLister::EventLister(fhicl::ParameterSet const& pset):
        art::EDAnalyzer(pset)
  {
    _ievent = 0;
  }

  void EventLister::analyze(const art::Event& event) {
    _ievent++;
    printf("Event: %8i run: %10i subRun: %5i event: %10i\n",
	   _ievent,event.run(),event.subRun(),event.event());
    //    cout << "Event: " << event.id() << endl;
  }

}  // end namespace mu2e

DEFINE_ART_MODULE(mu2e::EventLister);
