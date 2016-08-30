//
//  Collection of tools useful for dealing with various helices
//  Original Author Dave Brown (LBNL) 26 Aug. 2016
//
// Mu2e
#include "RecoDataProducts/inc/RobustHelix.hh"
#include "GeneralUtilities/inc/Angles.hh"
// BTrk
#include "BTrk/TrkBase/HelixTraj.hh"
// CLHEP
#include "CLHEP/Vector/ThreeVector.h"
#include "CLHEP/Matrix/Vector.h"
#include "CLHEP/Matrix/SymMatrix.h"
//C++
#include <math.h>
using CLHEP::Hep3Vector;
using CLHEP::HepSymMatrix;
using CLHEP::HepVector;
using namespace std;
namespace mu2e {
  namespace TrkHelixTools {

    bool RobustHelix2Traj (RobustHelix const& helix, HelixTraj &traj, float amsign) {
      bool retval(false);
      // compare the input with this configuration's helicity: these must be the same
      // radius = 0 or lambda=0 are degenerate cases that this representation can't handle
      if(helix.radius() > 0.0 && helix.lambda() != 0.0){
	HepVector helParams(5);
	// radius and omega have inverse magnitude, omega is signed by the angular momentum 
	helParams[HelixTraj::omegaIndex] = amsign/helix.radius();
	// phi0 is the azimuthal angle of the particle velocity vector at the point
	// of closest approach to the origin.  It's sign also depends on the angular
	// momentum.  To translate from the center, we need to reverse coordinates
	helParams[HelixTraj::phi0Index] = atan2(-amsign*helix.center().x(),amsign*helix.center().y());
	// d0 describes the distance to the origin at closest approach.
	// It is signed by the particle angular momentum WRT the origin.
	// The Helix fit radial bias is anti-correlated with d0; correct for it here.
	helParams[HelixTraj::d0Index] = amsign*(helix.center().perp() - helix.radius());
	// the dip angle is measured WRT the perpendicular, signed by the z component of linear momentum
	helParams[HelixTraj::tanDipIndex] = amsign*helix.lambda()/helix.radius();
	// must change conventions here: fz0 is the phi at z=0, z0 is defined at the point of closest approach
	// resolve the loop ambiguity such that the POCA is closest to z=0.
	double refphi = helix.fz0()+amsign*M_PI_2;
	double phi = helParams[HelixTraj::phi0Index];
	double dphi = Angles::deltaPhi(phi,refphi);
	// choose z0 (which loop) so that f=0 is as close to z=0 as possible
	helParams[HelixTraj::z0Index] = dphi*helParams[HelixTraj::tanDipIndex]/helParams[HelixTraj::omegaIndex]; 
	// setup a dummy error matrix.
	HepVector perr(5,0);
	// estimated parameter errors based on average performance.  These should be parameters, FIXME!!!
	// These get scaled up by the fit in early iterations
	perr[HelixTraj::d0Index]     = 34.0;
	perr[HelixTraj::phi0Index]   = 0.02;
	perr[HelixTraj::omegaIndex]  = 0.0002;
	perr[HelixTraj::tanDipIndex] = 0.05;
	perr[HelixTraj::z0Index]     = 15.0;

	CLHEP::HepSymMatrix covar = vT_times_v(perr);
	traj = HelixTraj(helParams,covar);
	retval = true;
      }
      return retval;
    }

    void RobustHelixFromMom(Hep3Vector const& pos, Hep3Vector const& mom, double charge, double Bz, RobustHelix& helix){
      // speed of light in mm/nsec
      static double clight =299.792;  // This value should come from conditions FIXME!!!	
      // translation factor from MeV/c to curvature radius.
      double momToRad = 1000.0/(charge*Bz*clight);
      // compute some simple useful parameters
      double pt = mom.perp();
      // transverse radius of the helix
      helix.radius() = fabs(pt*momToRad);
      //longitudinal wavelength; sign convention goes with angular rotation
      helix.lambda() = -mom.z()*momToRad;
      // circle center
      helix.center() = Hep3Vector(pos.x() + mom.y()*momToRad,
	  pos.y() - mom.x()*momToRad, 0.0);
      // phi at z=0
      double phi = (pos - helix.center()).phi() - pos.z()/helix.lambda();
      // reset to be close to 0
      Angles::deltaPhi(phi);
      helix.fz0() = phi;
    }
  } // TrkHelixTools
}// mu2e
