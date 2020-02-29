//
//
//
// $Id: ReadExtrapol_module.cc,v 1.18 2014/08/20 14:23:09 murat Exp $
// $Author: murat $
// $Date: 2014/08/20 14:23:09 $
//
// Original author G. Pezzullo
//

#include <exception>                                 // for exception
#include <stdio.h>                                          // for printf
#include <iostream>                                         // for operator<<
#include <memory>                                           // for allocator
#include <string>                                           // for string
#include <typeinfo>                                         // for type_info

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Principal/Handle.h"                 // for Handle
// From the art tool-chain
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
// data
#include "RecoDataProducts/inc/TrackClusterMatch.hh"        // for TrackClus...
#include "RtypesCore.h"                                     // for Float_t
#include "TTree.h"                                          // for TTree
#include "canvas/Persistency/Provenance/EventID.h"          // for EventID
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...


using namespace std;

namespace mu2e {

  static int ncalls(0);

  class ReadTrackCaloMatching : public art::EDAnalyzer {
  public:
    explicit ReadTrackCaloMatching(fhicl::ParameterSet const& pset):
      art::EDAnalyzer(pset),
      _diagLevel(pset.get<int>("diagLevel")),
      _trackClusterMatchModuleLabel(pset.get<std::string>("trackClusterMatchModuleLabel")),
      _Ntup(0){}

    virtual ~ReadTrackCaloMatching() {
    }
    void beginJob();
    void endJob() {}

    void analyze(art::Event const& e );

  private:

    void readTracClusterMatch(art::Event const& evt, bool skip);

    // Diagnostic level
    int _diagLevel;

    // Label of the extrapolated impact points
    std::string _trackClusterMatchModuleLabel;

    bool _skipEvent;

    TTree* _Ntup;//Ntupla which contains informations about the extrapolation starting from MC

    Int_t _evt,//event Id
      _ntrks;

    Float_t  _xtrk[100],
      _ytrk[100],
      _ztrk[100],
      _ttrk[100],
      _nx[100],
      _ny[100],
      _nz[100],
      _dx[100],
      _dy[100],
      _dz[100],
      _dt[100],
      _du[100],
      _dv[100],
      _ep[100],
      _chi2[100],
      _chi2_time[100],
      _int_depth[100],
      _ds[100];

  };



  void ReadTrackCaloMatching::beginJob( ) {

    cout << "start ReadTrackCaloMatching..."<<endl;

  }



  void ReadTrackCaloMatching::analyze(art::Event const& evt ) {

    ++ncalls;

    if (ncalls == 1) {

      art::ServiceHandle<art::TFileService> tfs;
      _Ntup        = tfs->make<TTree>("trkClu", "track-cluster match info");

      _Ntup->Branch("evt"    , &_evt ,   "evt/F");
      _Ntup->Branch("ntrks"  , &_ntrks , "ntrks/I");

      _Ntup->Branch("xtrk", _xtrk, "xtrk[ntrks]/F");
      _Ntup->Branch("ytrk", &_ytrk, "ytrk[ntrks]/F");
      _Ntup->Branch("ztrk", &_ztrk, "ztrk[ntrks]/F");
      _Ntup->Branch("ttrk", &_ttrk, "ttrk[ntrks]/F");
      _Ntup->Branch("nx", &_nx, "nx[ntrks]/F");
      _Ntup->Branch("ny", &_ny, "ny[ntrks]/F");
      _Ntup->Branch("nz", &_nz, "nz[ntrks]/F");
      _Ntup->Branch("dx", &_dx, "dx[ntrks]/F");
      _Ntup->Branch("dy", &_dy, "dy[ntrks]/F");
      _Ntup->Branch("dz", &_dz, "dz[ntrks]/F");
      _Ntup->Branch("dt", &_dt, "dt[ntrks]/F");
      _Ntup->Branch("du", &_du, "du[ntrks]/F");
      _Ntup->Branch("dv", &_dv, "dv[ntrks]/F");
      _Ntup->Branch("ep", &_ep, "ep[ntrks]/F");
      _Ntup->Branch("chi2", &_chi2, "chi2[ntrks]/F");
      _Ntup->Branch("chi2time", &_chi2_time, "chi2time[ntrks]/F");
      _Ntup->Branch("intdepth", &_int_depth, "intdepth[ntrks]/F");
      _Ntup->Branch("ds", &_ds, "ds[ntrks]/F");
    }


    readTracClusterMatch(evt, _skipEvent);

  } // end of analyze


  void ReadTrackCaloMatching::readTracClusterMatch(art::Event const& evt, bool skip){

    art::Handle<TrackClusterMatchCollection>  handle;
    evt.getByLabel(_trackClusterMatchModuleLabel, handle);
    const TrackClusterMatchCollection*       coll;
    const TrackClusterMatch* obj;

    if (handle.isValid()){
      coll = handle.product();
    }else {
      printf(">>> ERROR in ReadTrackCaloMatching::doExtrapolation: failed to locate collection");
      printf(". BAIL OUT. \n");
      return;
    }

    _evt   = evt.id().event();
    _ntrks = coll->size();

    for(int i=0; i<_ntrks; ++i){

      obj     = &coll->at(i);

      _xtrk[i]   = obj->xtrk();
      _ytrk[i]   = obj->ytrk();
      _ztrk[i]   = obj->ztrk();
      _ttrk[i]   = obj->ttrk();
      _nx[i]   = obj->nx();
      _ny[i]   = obj->ny();
      _nz[i]   = obj->nz();
      _dx[i]   = obj->dx();
      _dy[i]   = obj->dy();
      _dz[i]   = obj->dz();
      _dt[i]   = obj->dt();
      _du[i]   = obj->du();
      _dv[i]   = obj->dv();
      _ep[i]   = obj->ep();
      _chi2[i]   = obj->chi2();
      _chi2_time[i]   = obj->chi2_time();
      _int_depth[i]   = obj->int_depth();
      _ds[i]      = obj->ds();


    }

    _Ntup->Fill();

    if(evt.id().event() % 100 == 0){
      cout << "Event "<<evt.id().event()<<" ReadTrackCaloMatching done..."<<endl;
    }
  }



}



using mu2e::ReadTrackCaloMatching;
DEFINE_ART_MODULE(ReadTrackCaloMatching);
