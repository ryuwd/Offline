#ifndef TrackerConditions_StrawPhysics_hh
#define TrackerConditions_StrawPhysics_hh
//
// StrawPhysics collects the electronics response behavior of a Mu2e straw in
// several functions and parameters
//
// $Id: StrawPhysics.hh,v 1.2 2014/03/08 00:55:21 brownd Exp $
// $Author: brownd $
// $Date: 2014/03/08 00:55:21 $
//
// Original author David Brown, LBNL
//

// C++ includes
#include <iostream>
#include <array>
#include <vector>
#include <utility>
// CLHEP
#include "CLHEP/Random/RandGaussQ.h"
#include "CLHEP/Random/RandFlat.h"
// Mu2e includes
#include "Mu2eInterfaces/inc/ConditionsEntity.hh"
#include "fhiclcpp/ParameterSet.h"
#include <array>
#include <vector>
#include <algorithm>

namespace mu2e {
  class StrawPhysics : virtual public ConditionsEntity {
    public:
      // construct from parameters
      StrawPhysics(fhicl::ParameterSet const& pset);
      virtual ~StrawPhysics();
    // models.  Note these are different from the corresponding
    // functions used in reconstruction, as those can be wire-
    // dependent and have emergent properties
      unsigned nePerIon(double urand) const; // number of electrons for a given ionization, given a flat random number 0 < x < 1
      unsigned nePerEIon(double Eion) const; // number of electrons for a given ionization energy (approximate)
      double strawGain() const { return _gasgain; } // nominal gain
      double clusterGain(CLHEP::RandGaussQ& rgauss, CLHEP::RandFlat& rflat, unsigned nele) const;
      double driftDistanceToTime(double ddist, double phi) const;  // single cluster!
      double driftTimeSpread(double ddist, double phi) const; // single cluster!
      double propagationAttenuation(double wdist) const; 
      double propagationTime(double wdist) const;
      double velocityDispersion() const { return _vdisp; } 
      double meanFreePath() const { return _meanpath; }
      double ionizationEnergy(unsigned nele=1) const { return _EIonize*nele*(nele+1)/2; } // approximate total energy for an ionization of "nele" electrons, assuming all electrons come from the same shell
      double ionizationEnergy(double q) const { return _EAverage*q/_QAverage; }  // energy to produce a given charge.  This assumes the internal distribution of the number of electrons/ionization
      double ionizationCharge(unsigned nele=1) const { return _QIonize*nele; } 
    private:
      double _EIonize; // energy to create a single ionization electron (MeV)
      double _EAverage; // Average energy of ionization electrons (MeV)
      double _meanpath; // mean free path (mm)
      double _QIonize; // charge of a single ionization electron (=e, pC)
      double _QAverage; // average charge produced per ionization
      std::vector<double> _intNProb; // integrated probability distribution of the number of e produced per cluster
      double _gasgain; // nominal (average) avalanche gain
      double _polyaA; // 'A' parameter of Polya function used in gain fluctuation
      double _gslope; // slope of gain relative RMS vs 1/sqrt(n)
      unsigned _nggauss; // number of electrons/cluster to switch to a Gaussian model
      // attenuation length of charge down the wire; note
      // there is a short and a long component, each with it's own amplitude
      double _attlen[2];
      double _longfrac;
      double _vprop; // (average) propagation velocity
      double _vdisp; // dispersion of propagation velocity (dv/dl)
    // parameters describing cluster DtoT
      std::vector<double> _cdpoly;
      std::vector<double> _cdsigmapoly;


  };
}
#endif

