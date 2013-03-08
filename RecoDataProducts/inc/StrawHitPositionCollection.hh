#ifndef RecoDataProducts_StrawHitPositionCollection_hh
#define RecoDataProducts_StrawHitPositionCollection_hh

//
// Define a type for a collection of StrawHitPosition objects.
//
// $Id: StrawHitPositionCollection.hh,v 1.1 2013/03/08 04:29:49 brownd Exp $
// $Author: brownd $
// $Date: 2013/03/08 04:29:49 $
//
// Original author David Brown
//

#include <vector>

#include "RecoDataProducts/inc/StrawHitPosition.hh"

namespace mu2e {
   typedef std::vector<mu2e::StrawHitPosition> StrawHitPositionCollection;
}

#endif /* RecoDataProducts_StrawHitPositionCollection_hh */
