//
// Simple accessor to Kalman fit
//
// $Id: KalFitResult.hh,v 1.4 2012/12/05 21:14:14 tassiell Exp $
// $Author: tassiell $ 
// $Date: 2012/12/05 21:14:14 $
//
#ifndef KalFitResult_HH
#define KalFitResult_HH

#include "BTrk/BaBar/BaBar.hh"
// KalFit objects
#include "KalmanTests/inc/TrkFitDirection.hh"
#include "KalmanTests/inc/TrkStrawHit.hh"
#include "KalmanTests/inc/TrkDef.hh"
#include "BTrk/KalmanTrack/KalRep.hh"
#include "BTrk/TrkBase/TrkParticle.hh"
#include "KalmanTests/inc/Doublet.hh"

// C++

namespace mu2e 
{
// struct defining the Kalman fit inputs and output
  struct KalFitResult {
    KalFitResult(): _tdef(0) ,_krep(0), _fit(TrkErrCode::fail), _nt0iter(0), _nweediter(0), _nunweediter(0), _ninter(0) {}

// must initialize with a TrkDef
    KalFitResult(TrkDef const* tdef) : _tdef(tdef) ,_krep(0), _fit(TrkErrCode::fail), _nt0iter(0), _nweediter(0), _nunweediter(0), _ninter(0) {}
    ~KalFitResult() {}

    void    removeFailed() { if(_fit.failure())deleteTrack(); }
    void    fit() { if(_fit.success()) _fit = _krep->fit(); }
    void    deleteTrack();
    KalRep* stealTrack() { KalRep* retval = _krep; _krep=0; _hits.clear(); return retval; }
// data payload
    TrkDef const*                _tdef; // original track definition on which this is based
    KalRep*                      _krep; // Kalman rep, owned by the collection
    std::vector<TrkStrawHit*>    _hits; // straw hits, owned by the KalRep
    std::vector<DetIntersection> _detinter; // material intersections, used by the KalRep
    TrkErrCode                   _fit; // error code from last fit
    unsigned                     _nt0iter; // number of times t0 was iterated
    unsigned                     _nweediter; // number of iterations on hit weeding
    unsigned                     _nunweediter; // number of iterations on hit unweeding
    unsigned                     _ninter; // number of iterations on material intersections
    int                          _decisionMode;
    std::vector <Doublet>        _listOfDoublets;
  };

  typedef std::vector<KalFitResult> KalFitResultCollection;
} 

#endif
