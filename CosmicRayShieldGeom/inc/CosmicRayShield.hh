#ifndef CosmicRayShieldGeom_CosmicRayShield_hh
#define CosmicRayShieldGeom_CosmicRayShield_hh

//
// Representation of CosmicRayShield
//
// $Id: CosmicRayShield.hh,v 1.15 2013/09/13 06:42:44 ehrlich Exp $
// $Author: ehrlich $
// $Date: 2013/09/13 06:42:44 $
//
// Original author KLG
//

// c++ includes
#include <map>
#include <string>

// clhep includes
#include "CLHEP/Vector/Rotation.h"
#include "CLHEP/Vector/ThreeVector.h"

// Includes from Mu2e
#include "CosmicRayShieldGeom/inc/CRSScintillatorShield.hh"
#include "CosmicRayShieldGeom/inc/CRSSteelShield.hh"
#include "Mu2eInterfaces/inc/Detector.hh"


namespace mu2e 
{

  // Forward reference.
  class SimpleConfig;
  class CosmicRayShieldMaker;

  class CosmicRayShield : virtual public Detector 
  {

    friend class CosmicRayShieldMaker;

    public:

    CosmicRayShield() {}
    ~CosmicRayShield() {}

    // Get ScintillatorShield
    CRSScintillatorShield const & getCRSScintillatorShield(const CRSScintillatorShieldId& id) const
    {
      return _scintillatorShields.at(id);
    }

    CRSScintillatorModule const & getModule( const CRSScintillatorModuleId& moduleid ) const
    {
      return _scintillatorShields.at(moduleid.getShieldNumber()).getModule(moduleid);
    }

    CRSScintillatorLayer const & getLayer( const CRSScintillatorLayerId& lid ) const
    {
      return _scintillatorShields.at(lid.getShieldNumber()).getLayer(lid);
    }

    CRSScintillatorBar const & getBar( const CRSScintillatorBarId& bid ) const
    {
      return _scintillatorShields.at(bid.getShieldNumber()).getBar(bid);
    }

    std::vector<CRSScintillatorShield> const & getCRSScintillatorShields() const 
    {
      return _scintillatorShields;
    }

    std::vector<CRSScintillatorBar> const & getAllCRSScintillatorBars() const 
    {
      return _allCRSScintillatorBars;
    }

    const CRSScintillatorBar& getBar ( CRSScintillatorBarIndex index ) const 
    {
      return _allCRSScintillatorBars.at(index.asInt());
    }

    private:

    std::vector<CRSScintillatorShield>  _scintillatorShields;
    std::vector<CRSScintillatorBar>     _allCRSScintillatorBars;  // global holder of all scintillator bars
  };

}
#endif /* CosmicRayShieldGeom_CosmicRayShield_hh */
