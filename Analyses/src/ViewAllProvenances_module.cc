//
//  Look at all provenances stored in the registry
//
//  $Id: ViewAllProvenances_module.cc,v 1.2 2013/10/21 20:34:14 gandr Exp $
//  $Author: gandr $
//  $Date: 2013/10/21 20:34:14 $
//
//  Original author Rob Kutschke
//

#include <iostream>                                    // for operator<<, endl
#include <string>                                      // for operator<<
#include <utility>                                     // for pair

#include "art/Framework/Core/EDAnalyzer.h"             // for EDAnalyzer
#include "art/Framework/Core/ModuleMacros.h"           // for DEFINE_ART_MODULE
#include "fhiclcpp/ParameterSet.h"                     // for ParameterSet
#include "fhiclcpp/ParameterSetRegistry.h"             // for ParameterSetRe...
#include "fhiclcpp/types/AllowedConfigurationMacro.h"  // for AllowedConfigu...

namespace art {
class Event;
}  // namespace art

using namespace std;

namespace mu2e {

  class ViewAllProvenances : public art::EDAnalyzer {

  public:
    explicit ViewAllProvenances(fhicl::ParameterSet const& pset) : art::EDAnalyzer(pset) {}

    void analyze(const art::Event& event) override;

  private:

  };

  void ViewAllProvenances::analyze(const art::Event& event){

    cout << "Number of enteries in the parameter set registry: " <<  fhicl::ParameterSetRegistry::get().size() << endl;

    // The type of tmp is some sort of key value pair.
    for ( auto const& tmp : fhicl::ParameterSetRegistry::get() ){

      fhicl::ParameterSet const& pset = tmp.second;

      cout << "\nParameter set content: " << endl;
      cout << "--------------------------------------" << endl;
      cout << pset.to_indented_string();
      cout << "--------------------------------------" << endl;
    }


  } // end analyze

} // end namespace mu2e

DEFINE_ART_MODULE(mu2e::ViewAllProvenances);
