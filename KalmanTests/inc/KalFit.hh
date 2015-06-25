//
// Object to perform BaBar Kalman fit
//
// $Id: KalFit.hh,v 1.29 2014/08/01 18:56:10 gandr Exp $
// $Author: gandr $ 
// $Date: 2014/08/01 18:56:10 $
//
#ifndef KalFit_HH
#define KalFit_HH

// framework

#ifndef __GCCXML__
#include "fhiclcpp/ParameterSet.h"
#endif/*__GCCXML__*/

// data
#include "RecoDataProducts/inc/StrawHitCollection.hh"
// tracker
#include "TrackerGeom/inc/Tracker.hh"
#include "TrackerGeom/inc/Straw.hh"
// BaBar
#include "BTrk/BaBar/BaBar.hh"
// KalFit objects
#include "KalmanTests/inc/TrkFitDirection.hh"
#include "KalmanTests/inc/TrkDef.hh"
#include "KalmanTests/inc/KalFitResult.hh"
#include "KalmanTests/inc/TrkStrawHit.hh"
#include "KalmanTests/inc/AmbigResolver.hh"
#include "BTrk/KalmanTrack/KalContext.hh"
#include "BTrk/KalmanTrack/KalRep.hh"
#include "BTrk/BField/BField.hh"
#include "BTrk/TrkBase/TrkParticle.hh"
// diagnostics
#include "KalmanTests/inc/KalDiag.hh"
//CLHEP
#include "CLHEP/Units/PhysicalConstants.h"
// C++

namespace mu2e 
{
  class KalFit : public KalContext
  {
  public:
// define different ambiguity resolution strategies
    enum ambigStrategy {fixedambig=0,pocaambig=1,hitambig=2,panelambig=3,doubletambig=4};
// parameter set should be passed in on construction
#ifndef __GCCXML__
    explicit KalFit(fhicl::ParameterSet const&,KalDiag * kdiag=0);
#endif/*__GCCXML__*/

    virtual ~KalFit();
// main function: given a track definition, create a fit object from it
    virtual void makeTrack(KalFitResult& kres);
// add a set of hits to an existing fit
    virtual void addHits(KalFitResult& kres,const StrawHitCollection* straws, std::vector<hitIndex> indices, double maxchi);
// Try to put back inactive hits
    bool unweedHits(KalFitResult& kres, double maxchi);
// KalContext interface
    virtual const TrkVolume* trkVolume(trkDirection trkdir) const ;
    BField const& bField() const;

    int  decisionMode() { return _decisionMode ; }
    void setDecisionMode (int Mode) { _decisionMode = Mode; }

  protected:
    // configuration parameters
    int _debug;
    std::vector<bool> _weedhits;
//    bool _weedhits;
    double _maxhitchi;
    unsigned _maxweed;
    std::vector<double> _herr;
    double _maxdriftpull;
    fhicl::ParameterSet*        _darPset;         // 2015-04-12 P.Murat: parameter set for doublet ambig resolver
    std::vector<AmbigResolver*> _ambigresolver;
    bool _initt0;
    bool _updatet0;
    std::vector<double> _t0tol;
    bool fitable(TrkDef const& tdef);
    bool weedHits(KalFitResult& kres, int Iteration, int Final);
    void initT0(TrkDef const& tdef, TrkT0& t0);
    bool updateT0(KalFitResult& kres);
    void fitTrack(KalFitResult& kres);
    virtual void makeHits(KalFitResult& kres, TrkT0 const& t0);
    virtual void makeMaterials(KalFitResult& kres);

  private:
    double _t0errfac; // fudge factor for the calculated t0 error
    double _mint0doca; // minimum (?) doca for t0 hits
    double _t0nsig; // # of sigma to include when selecting hits for t0
    bool _removefailed;
    unsigned _minnstraws;
    TrkParticle _tpart;
    TrkFitDirection _fdir;
    std::vector<int> _ambigstrategy;
    bool             _resolveAfterWeeding;  // 2015-04-12 P.Murat: temp flag to mark changes
    int              _decisionMode; // 0:decision is not forced; 1:decision has to be made
    mutable BField* _bfield;
    KalDiag* _kdiag; //optional diagnostics
    // helper functions
//-----------------------------------------------------------------------------
// 'Final'=1: final iteration, may involve special decision making mode
//        =0: do not force decision on the hit drift sign if not enough information
//-----------------------------------------------------------------------------
    void fitIteration(KalFitResult& kres,size_t iiter,int Final);
    void updateHitTimes(KalFitResult& kres);
    void findBoundingHits(std::vector<TrkStrawHit*>& hits, double flt0,
	std::vector<TrkStrawHit*>::reverse_iterator& ilow,
	std::vector<TrkStrawHit*>::iterator& ihigh);
  };
}
#endif
