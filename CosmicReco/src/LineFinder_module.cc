//
// $Id: LineFinder_module.cc,v 1.2 2014/08/30 12:19:38 tassiell Exp $
// $Author: tassiell $
// $Date: 2014/08/30 12:19:38 $
//
// Original author D. Brown and G. Tassielli
//

#include "fhiclcpp/ParameterSet.h"
#include "art/Framework/Principal/Event.h"
#include "fhiclcpp/ParameterSet.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Core/EDProducer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art_root_io/TFileService.h"
#include "art/Utilities/make_tool.h"
#include "canvas/Persistency/Common/Ptr.h"

#include "RecoDataProducts/inc/ComboHit.hh"
#include "RecoDataProducts/inc/TimeCluster.hh"
#include "RecoDataProducts/inc/LineSeed.hh"

#include "CLHEP/Units/PhysicalConstants.h"
#include "CLHEP/Matrix/Vector.h"
#include "CLHEP/Matrix/SymMatrix.h"

#include "TH2F.h"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <utility>
#include <functional>
#include <float.h>
#include <vector>
#include <map>

namespace mu2e{

  class LineFinder : public art::EDProducer {
    public:
      struct Config{
        using Name=fhicl::Name;
        using Comment=fhicl::Comment;
        fhicl::Atom<int> diag{Name("diag"), Comment("Create diag histograms"),0};
        fhicl::Atom<int> thetabins{Name("thetaBins"), Comment("Theta bins in accumulator"),15};
        fhicl::Atom<int> phibins{Name("phiBins"), Comment("Phi bins in accumulator"),15};
        fhicl::Atom<int> xbins{Name("xBins"), Comment("x bins in accumulator"),100};
        fhicl::Atom<int> zbins{Name("zBins"), Comment("z bins in accumulator"),100};
        fhicl::Atom<float> maxx{Name("xBins"), Comment("x bins in accumulator"),5000};
        fhicl::Atom<float> maxz{Name("zBins"), Comment("z bins in accumulator"),5000};
        fhicl::Atom<float> mindistance{Name("minDistance"), Comment("Minimum distance required between hits"),20};
        fhicl::Atom<int> minhits{Name("minHits"), Comment("Minimum hits in time cluster"),5};
        fhicl::Atom<int> minpeak{Name("minPeak"), Comment("Minimum hits in accumulator peak"),3};
        fhicl::Atom<art::InputTag> chToken{Name("ComboHitCollection"),Comment("tag for combo hit collection")};
        fhicl::Atom<art::InputTag> tcToken{Name("TimeClusterCollection"),Comment("tag for time cluster collection")};
      };
      typedef art::EDProducer::Table<Config> Parameters;
      explicit LineFinder(const Parameters& conf);
      virtual ~LineFinder(){};
      virtual void produce(art::Event& event ) override;

    private:

      Config _conf;

      //config parameters:
      int _diag;
      int _thetaBins, _phiBins, _xBins, _zBins;
      float _maxX, _maxZ;
      float _minDistance;
      int _minHits, _minPeak;
      art::InputTag  _chToken;
      art::InputTag  _tcToken;

      void findLine(const ComboHitCollection& chC, int itc, CLHEP::Hep3Vector &seedDirection, CLHEP::Hep3Vector &seedIntercept, int &seedsize);
  };


 LineFinder::LineFinder(const Parameters& conf) :
   art::EDProducer(conf),
   	_diag (conf().diag()),
        _thetaBins (conf().thetabins()),
        _phiBins (conf().phibins()),
        _xBins (conf().xbins()),
        _zBins (conf().zbins()),
        _maxX (conf().maxx()),
        _maxZ (conf().maxz()),
        _minDistance (conf().mindistance()),
        _minHits (conf().minhits()),
        _minPeak (conf().minpeak()),
    	_chToken (conf().chToken()),
	_tcToken (conf().tcToken())
{
  consumes<ComboHitCollection>(_chToken);
  consumes<TimeClusterCollection>(_tcToken);
  produces<LineSeedCollection>();
	    
 }

void LineFinder::produce(art::Event& event ) {

  auto const& chH = event.getValidHandle<ComboHitCollection>(_chToken);
  const ComboHitCollection& chcol(*chH);
  auto  const& tcH = event.getValidHandle<TimeClusterCollection>(_tcToken);
  const TimeClusterCollection& tccol(*tcH);


  std::unique_ptr<LineSeedCollection> seed_col(new LineSeedCollection());

  for (size_t index=0;index< tccol.size();++index) {
    const auto& tclust = tccol[index];

    const std::vector<StrawHitIndex>& shIndices = tclust.hits();
    ComboHitCollection tchits;
    
    int nhits = 0;
    for (size_t i=0; i<shIndices.size(); ++i) {
      int loc = shIndices[i];
      const ComboHit& ch  = chcol[loc];
      //FIXME add flag check
      //if(ch.flag().hasAnyProperty(_hsel) && !ch.flag().hasAnyProperty(_hbkg) ) {
      tchits.push_back(ComboHit(ch));
      nhits += ch.nStrawHits();
    }
    
    if (nhits < _minHits)
      continue;

    LineSeed lseed;
    lseed._timeCluster = art::Ptr<TimeCluster>(tcH,index);
    findLine(tchits, index, lseed._seedDir, lseed._seedInt, lseed._seedSize);

    if (lseed._seedSize >= _minPeak){
      LineSeedCollection*  lcol  = seed_col.get();
      lcol->push_back(lseed);
    }
  }

  event.put(std::move(seed_col));    
}

void LineFinder::findLine(const ComboHitCollection& chC, int itc, CLHEP::Hep3Vector &seedDirection, CLHEP::Hep3Vector &seedIntercept, int &seedsize){
  std::vector<uint16_t> dirAccumulator(_thetaBins*_phiBins,0);
  std::vector<uint16_t> intAccumulator(_xBins*_zBins,0);

  std::vector<int> pairthetas(chC.size()*chC.size());
  std::vector<int> pairphis(chC.size()*chC.size());
  std::vector<CLHEP::Hep3Vector> pairdirs(chC.size()*chC.size());
  std::vector<CLHEP::Hep3Vector> pairintercepts(chC.size()*chC.size());

  // loop over all pairs of hits
  for (size_t i=0;i<chC.size();i++){
    for (size_t j=i+1;j<chC.size();j++){
      CLHEP::Hep3Vector thisdir = (chC[i].posCLHEP()-chC[j].posCLHEP());
      // require that hits are separated by some minimum distance
      if (thisdir.mag() < _minDistance)
        continue;
      thisdir = thisdir.unit();
      // make all directions point down
      if (thisdir.y() > 0){
        thisdir *= -1;
      }
      int tbin = (thisdir.theta() + M_PI)/(2*M_PI)*_thetaBins;
      int pbin = (thisdir.phi() + M_PI)/(2*M_PI)*_phiBins; //FIXME

      // add direction to accumulator
      dirAccumulator[tbin*_phiBins + pbin] += 1;

      // calculate y-intercept
      // store for later calculations
      CLHEP::Hep3Vector thisintercept = chC[i].posCLHEP() - thisdir*chC[i].posCLHEP().y()/(thisdir.y()+1e-8);
      pairthetas[i*chC.size()+j] = tbin;
      pairphis[i*chC.size()+j] = pbin;
      pairdirs[i*chC.size()+j] = thisdir;
      pairintercepts[i*chC.size()+j] = thisintercept;

    }
  }
  int peaktheta = 0;
  int peakphi = 0;
  uint16_t peakcount = 0;
  for (size_t i=0;i<(size_t)_thetaBins;i++){
    for (size_t j=0;j<(size_t)_phiBins;j++){
      if (dirAccumulator[i*_phiBins + j] > peakcount){
        peaktheta = i;
        peakphi = j;
        peakcount = dirAccumulator[i*_phiBins + j];
      }
    }
  }
  // now that we have peak direction, calculate peak intercept
  for (size_t i=0;i<chC.size();i++){
    for (size_t j=i+1;j<chC.size();j++){
      if (pairthetas[i*chC.size()+j] == peaktheta && pairphis[i*chC.size()+j] == peakphi){
        int xbin = (pairintercepts[i*chC.size()+j].x()+_maxX)/(2*_maxX)*_xBins;
        int zbin = (pairintercepts[i*chC.size()+j].z()+_maxZ)/(2*_maxZ)*_zBins;
        if (xbin >= 0 && xbin < _xBins && zbin >= 0 && zbin < _zBins){
          intAccumulator[xbin*_zBins + zbin] += 1;
        }
      }
    }
  }
  int peakx = 0;
  int peakz = 0;
  uint16_t peakcount2 = 0;
  for (int i=0;i<_xBins;i++){
    for (int j=0;j<_zBins;j++){
      if (intAccumulator[i*_zBins + j] > peakcount2){
        peakx = i;
        peakz = j;
        peakcount2 = intAccumulator[i*_zBins + j];
      }
    }
  }
  
  // now get avg of things in bin
  seedDirection = CLHEP::Hep3Vector(0,0,0);
  seedIntercept = CLHEP::Hep3Vector(0,0,0);
  for (size_t i=0;i<chC.size();i++){
    for (size_t j=i+1;j<chC.size();j++){
      if (pairthetas[i*chC.size()+j] == peaktheta && pairphis[i*chC.size()+j] == peakphi){
        int xbin = (pairintercepts[i*chC.size()+j].x()+_maxX)/(2*_maxX)*_xBins;
        int zbin = (pairintercepts[i*chC.size()+j].z()+_maxZ)/(2*_maxZ)*_zBins;
        if (xbin == peakx && zbin == peakz){
          seedDirection += pairdirs[i*chC.size()+j];
          seedIntercept += pairintercepts[i*chC.size()+j];
        }
      }
    }
  }

  if (peakcount2 > 0){
    seedDirection /= peakcount2;
    seedIntercept /= peakcount2;
  }

  //seedDirection = CLHEP::Hep3Vector(1,1,1);
  //seedDirection.setTheta(-M_PI + (2*M_PI)/_thetaBins*(peaktheta+0.5));
  //seedDirection.setPhi(-M_PI + (2*M_PI)/_phiBins*(peakphi+0.5)); //FIXME
  seedDirection = seedDirection.unit();

  //seedIntercept = CLHEP::Hep3Vector(-_maxX + (2*_maxX)/_xBins*(peakx+0.5),0,-_maxZ + (2*_maxZ)/_zBins*(peakz+0.5));

  seedsize = peakcount2;

  if (_diag > 0){
    art::ServiceHandle<art::TFileService> tfs;
    static unsigned nhist(0);
    char name[60];
    snprintf(name,60,"dir_%d_%d",nhist,itc);
    TH2F *hdir = tfs->make<TH2F>(name,name,_thetaBins,-M_PI,M_PI,_phiBins,-M_PI,M_PI);
    char name2[60];
    snprintf(name2,60,"int_%d_%d",nhist,itc);
    TH2F *hint = tfs->make<TH2F>(name2,name2,_xBins,-_maxX,_maxX,_zBins,-_maxZ,_maxZ);
    ++nhist;

    for (size_t i=0;i<chC.size();i++){
      for (size_t j=i+1;j<chC.size();j++){
        hdir->Fill(pairdirs[i*chC.size()+j].theta(),pairdirs[i*chC.size()+j].phi());
        hint->Fill(pairintercepts[i*chC.size()+j].x(),pairintercepts[i*chC.size()+j].z());
      }
    }
  }
}

}//end mu2e namespace
using mu2e::LineFinder;
DEFINE_ART_MODULE(LineFinder);
