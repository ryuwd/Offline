// Helper templates for extracting from fhicl::ParameterSet objects of
// types that are not directly supported by fhicl.  After including this
// file you will be able to do, for example
//
//    pset.get<CLHEP::Hep3Vector>("position")
//
// in you code.  The fhicl file syntax for specifying vectors is
//
//    position : [ 1.1, 2.2, 3.3 ]
//
// Andrei Gaponenko, 2012

#ifndef GeneralUtilities_inc_ParameterSetHelpers_hh
#define GeneralUtilities_inc_ParameterSetHelpers_hh

#include <string>
#include "fhiclcpp/ParameterSet.h"

namespace CLHEP { class Hep3Vector; }

namespace fhicl {

  template<> bool ParameterSet::get_if_present<CLHEP::Hep3Vector>(std::string const & key, CLHEP::Hep3Vector& value) const;

}

#endif/*GeneralUtilities_inc_ParameterSetHelpers_hh*/
