#ifndef ProductionSolenoidGeom_PSShieldMaker_hh
#define ProductionSolenoidGeom_PSShieldMaker_hh
//
// Class to construct and return PSShield
//
// $Id: PSShieldMaker.hh,v 1.4 2013/03/15 15:52:05 kutschke Exp $
// $Author: kutschke $
// $Date: 2013/03/15 15:52:05 $
//
// Original author Andrei Gaponenko
//

#include <memory>
#include "ProductionSolenoidGeom/inc/PSShield.hh"

namespace CLHEP { class Hep3Vector; }

namespace mu2e {

  class SimpleConfig;

  class PSShieldMaker {
    static PSShield::Groove readGroove(int i, const SimpleConfig& config);

  public:

    static std::unique_ptr<PSShield> make(const SimpleConfig& config,
                                        // The HRS is placed on PS axis
                                        const CLHEP::Hep3Vector& psEndRefPoint,
                                        // The z position is relative to the proton target
                                        const CLHEP::Hep3Vector& productionTargetCenter
                                        );
  };

}  //namespace mu2e

#endif /* ProductionSolenoidGeom_PSShieldMaker_hh */
