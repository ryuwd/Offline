#ifndef RecoDataProducts_LineSeed_hh
#define RecoDataProducts_LineSeed_hh
//
// Seed for a straight line track
// Richie Bonventre
//

// Mu2e includes
#include "RecoDataProducts/inc/TimeCluster.hh"
#include "canvas/Persistency/Common/Ptr.h"
#include <vector>

namespace mu2e {
  struct LineSeed {

    CLHEP::Hep3Vector _seedDir;
    CLHEP::Hep3Vector _seedInt; // y intercept
    int _seedSize;
    art::Ptr<TimeCluster>    _timeCluster; // associated time cluster
  };
   typedef std::vector<mu2e::LineSeed> LineSeedCollection;
} // namespace mu2e

#endif /* RecoDataProducts_LneSeed_hh */
