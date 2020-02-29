//
//
// Original author G. Pezzullo
//

#include <exception>                                    // for exception
#include <iostream>                                            // for std
#include <memory>                                              // for allocator
#include <string>                                              // for string
#include <typeinfo>                                            // for type_info

// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"                     // for EDAnal...
#include "art/Framework/Principal/Event.h"                     // for Event
#include "art/Framework/Core/ModuleMacros.h"                   // for DEFINE...
#include "art/Framework/Services/Registry/ServiceHandle.h"     // for Servic...
#include "art_root_io/TFileService.h"                          // for TFileS...
#include "art/Framework/Principal/Handle.h"                    // for Handle
//CLHEP includes
#include "CLHEP/Vector/ThreeVector.h"                          // for Hep3Ve...
#include "BTrk/BbrGeom/HepPoint.h"                             // for HepPoint
// From the art tool-chain
#include "fhiclcpp/ParameterSet.h"                             // for Parame...
#include "BTrk/KalmanTrack/KalRep.hh"                          // for KalRep
// data
#include "RecoDataProducts/inc/TrkCaloIntersectCollection.hh"  // for TrkCal...

#include "RecoDataProducts/inc/KalRepPtrCollection.hh"         // for KalRepPtr
#include "RecoDataProducts/inc/TrkCaloIntersect.hh"            // for TrkCal...
#include "RtypesCore.h"                                        // for Float_t
#include "TTree.h"                                             // for TTree
#include "canvas/Persistency/Provenance/EventID.h"             // for EventID
#include "canvas/Utilities/Exception.h"                        // for Exception
#include "fhiclcpp/exception.h"                                // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"          // for Allowe...


using namespace std;

namespace mu2e {


  class ReadTrkExtrapolMVA : public art::EDAnalyzer {

     public:

	 explicit ReadTrkExtrapolMVA(fhicl::ParameterSet const& pset):
	   art::EDAnalyzer(pset),
	   _diagLevel(pset.get<int>("diagLevel",0)),
	   _trkExtrapolModuleLabel(pset.get<std::string>("trkExtrapolModuleLabel")),
	   _Ntup(0)
	 {}

	 virtual ~ReadTrkExtrapolMVA() {}

	 void beginJob();
	 void endJob() {}

	 void analyze(art::Event const& event );



     private:

	  int          _diagLevel;
	  std::string  _trkExtrapolModuleLabel;

	  TTree* _Ntup;
	  Int_t _evt,_ntrks;
	  Float_t _caloSec[100],_trkId[100],_trkTime[100],_trkTimeErr[100], _trkPathLenghtIn[100], _trkPathLenghtInErr[100], _trkPathLenghtOut[100];
	  Float_t _trkMom[100],_trkMomX[100], _trkMomY[100],_trkMomZ[100], _trkPosX[100], _trkPosY[100], _trkPosZ[100];

  };



  void ReadTrkExtrapolMVA::beginJob( ) {


      art::ServiceHandle<art::TFileService> tfs;
      _Ntup  = tfs->make<TTree>("trkExt", "Extrapolated track info");

      _Ntup->Branch("evt"    , &_evt ,   "evt/F");
      _Ntup->Branch("ntrks"  , &_ntrks , "ntrks/I");
      _Ntup->Branch("caloSec", &_caloSec, "caloSec[ntrks]/F");
      _Ntup->Branch("trkTime", &_trkTime, "trkTime[ntrks]/F");
      _Ntup->Branch("trkPathLenghtIn", &_trkPathLenghtIn, "trkPathLenghtIn[ntrks]/F");
      _Ntup->Branch("trkPathLenghtInErr", &_trkPathLenghtInErr, "trkPathLenghtInErr[ntrks]/F");
      _Ntup->Branch("trkPathLenghtOut", &_trkPathLenghtOut, "trkPathLenghtOut[ntrks]/F");
      _Ntup->Branch("trkMom", &_trkMom, "trkMom[ntrks]/F");
      _Ntup->Branch("trkMomX", &_trkMomX, "trkMomX[ntrks]/F");
      _Ntup->Branch("trkMomY", &_trkMomY, "trkMomY[ntrks]/F");
      _Ntup->Branch("trkMomZ", &_trkMomZ, "trkMomZ[ntrks]/F");
      _Ntup->Branch("trkPosX", &_trkPosX, "trkPosX[ntrks]/F");
      _Ntup->Branch("trkPosY", &_trkPosY, "trkPosY[ntrks]/F");
      _Ntup->Branch("trkPosZ", &_trkPosZ, "trkPosZ[ntrks]/F");


  }



  void ReadTrkExtrapolMVA::analyze(art::Event const& event )
  {

        art::Handle<TrkCaloIntersectCollection>  trjExtrapolHandle;
        event.getByLabel(_trkExtrapolModuleLabel, trjExtrapolHandle);
        TrkCaloIntersectCollection const& trkExtrapols(*trjExtrapolHandle);

        _evt   = event.id().event();
        _ntrks = trkExtrapols.size();

	int it(0);
	for (auto const& extrapol: trkExtrapols)
	{
	    double pathLength       = extrapol.pathLengthEntrance();
	    _caloSec[it]            = extrapol.diskId();
	    _trkTime[it]            = extrapol.trk()->arrivalTime(pathLength);
	    _trkPathLenghtIn[it]    = extrapol.pathLengthEntrance();
	    _trkPathLenghtInErr[it] = extrapol.pathLenghtEntranceErr();
	    _trkPathLenghtOut[it]   = extrapol.pathLengthExit();
	    _trkMom[it]             = extrapol.trk()->momentum(pathLength).mag();
	    _trkMomX[it]            = extrapol.trk()->momentum(pathLength).x();
	    _trkMomY[it]            = extrapol.trk()->momentum(pathLength).y();
	    _trkMomZ[it]            = extrapol.trk()->momentum(pathLength).z();
	    _trkPosX[it]            = extrapol.trk()->position(pathLength).x();
	    _trkPosY[it]            = extrapol.trk()->position(pathLength).y();
	    _trkPosZ[it]            = extrapol.trk()->position(pathLength).z();

	    ++it;
	}

        _Ntup->Fill();
  }


}



using mu2e::ReadTrkExtrapolMVA;
DEFINE_ART_MODULE(ReadTrkExtrapolMVA);
