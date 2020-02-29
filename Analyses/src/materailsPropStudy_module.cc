//
// Module which starts the event display, and transmits the data of each event to the event display.
//

#include <exception>                                 // for exception
// c++
#include <iostream>                                         // for basic_ost...
#include <string>                                           // for string
#include <vector>                                           // for vector
#include <algorithm>                                        // for max

// framework
#include "art/Framework/Core/EDAnalyzer.h"                  // for EDAnalyzer
#include "art/Framework/Principal/Event.h"                  // for Event
#include "fhiclcpp/ParameterSet.h"                          // for ParameterSet
#include "art_root_io/TFileService.h"                       // for TFileService
#include "art/Framework/Core/ModuleMacros.h"                // for DEFINE_AR...
#include "BTrk/MatEnv/MatDBInfo.hh"                         // for MatDBInfo
#include "BTrk/DetectorModel/DetMaterial.hh"                // for DetMaterial
// ROOT
#include "TNtuple.h"                                        // for TNtuple
// CLHEP
#include "CLHEP/Units/SystemOfUnits.h"                      // for cm
#include "BTrk/TrkBase/TrkParticle.hh"                      // for TrkParticle
#include "TDirectory.h"                                     // for TDirectory
#include "art/Framework/Services/Registry/ServiceHandle.h"  // for ServiceHa...
#include "canvas/Persistency/Provenance/EventID.h"          // for EventID
#include "canvas/Utilities/Exception.h"                     // for Exception
#include "fhiclcpp/coding.h"                                // for ps_sequen...
#include "fhiclcpp/exception.h"                             // for exception
#include "fhiclcpp/types/AllowedConfigurationMacro.h"       // for AllowedCo...

namespace art {
class Run;
}  // namespace art


namespace mu2e
{
  class materailsPropStudy : public art::EDAnalyzer
  {
    public:
    explicit materailsPropStudy(fhicl::ParameterSet const&);
    virtual ~materailsPropStudy() { }
    virtual void beginJob();
    virtual void beginRun(art::Run const&) override;
    void endJob();
    //void produce(art::Event& e);
    void analyze(art::Event const&);

    private:

    MatDBInfo* matdbinfo(){
      static MatDBInfo mat;
      return &mat;
    }

    //std::string _materialdb;
    std::string _matname;
    TrkParticle _tpart; // particle type being searched for
    double _partmom;
    double _Tcut;
    std::vector<double> _pathlengths;
    double _maxTCut_dEdx;
    unsigned int _nStepTCut_dEdx;

    const DetMaterial *_mat;

    TNtuple *_meanEloss;
    TNtuple *_dEdx;

    // Save directory from beginJob so that we can go there in endJob. See note 3.
    TDirectory* _directory;

  };

  materailsPropStudy::materailsPropStudy(fhicl::ParameterSet const& pset) :
    art::EDAnalyzer(pset),
    //_materialdb(pset.get<std::string>("materialdb", "")),
    _matname(pset.get<std::string>("matname", "")),
    _tpart((TrkParticle::type)(pset.get<int>("particle",TrkParticle::e_minus))),
    _partmom(pset.get<double>("partmom",100.0)),
    _Tcut(pset.get<double>("Tcut",-1)),
    _pathlengths(pset.get< std::vector<double> >("pathlengths")),
    _maxTCut_dEdx(pset.get<double>("maxTCut_dEdx",0)),
    _nStepTCut_dEdx(pset.get<unsigned int>("nStepTCut_dEdx",0)),
    _mat(0),
    _meanEloss(0),
    _dEdx(0),
    _directory(0)
  {
  }

  void materailsPropStudy::beginJob()
  {
          art::ServiceHandle<art::TFileService> tfs;
          _meanEloss       = tfs->make<TNtuple>("meanEloss","meanEloss","path:elossBB:elossTCut_20:elossTCut_500:elossTCut_1000");
          _dEdx       = tfs->make<TNtuple>("dEdx","dEdx","TCut:dedx");

          // See note 3.
          _directory = gDirectory;
  }

  void materailsPropStudy::beginRun(art::Run const&){
    //std::cout<<"init dchdetector"<<std::endl;
    //DchGDchP gdch(_materialdb,_kfit.useDetailedWrSuppDescr(),_kfit.useSingleCellDescr());
    //dchdet=new DchDetector(gdch,true);
  }

  //void materailsPropStudy::produce(art::Event& event)
  void materailsPropStudy::analyze(art::Event const& event)
  {
          int iev=event.id().event();
          std::cout<<"ev "<<iev<<std::endl;
          if (iev==1) {
                  //RecoMatFactory* matFactory = RecoMatFactory::getInstance();
                  //MatMaterialList* mtrList = new MatMaterialList(_materialdb);
                  //((MatMtrDictionary*)matFactory->materialDictionary())->FillMtrDict(mtrList);
                  //MatDBInfo* mtdbinfo=new MatDBInfo;

                  _mat = matdbinfo()->findDetMaterial(_matname);

                  //_mat->printAll(std::cout);
                  //double pathlen = 1.0*CLHEP::cm;
                  //std::cout<<"part mass "<<_tpart.mass()<<std::endl;
                  //std::cout<<"mat density "<<_mat->density()<<std::endl;
                  //std::cout<<"dE/dx "<<_mat->dEdx(_partmom,DetMaterial::loss,_tpart)/_mat->density()/CLHEP::cm/CLHEP::cm<<std::endl;
                  //std::cout<<"eloss in "<<pathlen<<" mm = "<<_mat->energyLoss(_partmom, pathlen, _tpart, DetMaterial::loss)<<std::endl;


                  float ntpData[5];
                  for (std::vector<double>::iterator pl_it = _pathlengths.begin(); pl_it!= _pathlengths.end(); ++pl_it) {
                          ntpData[0]=*pl_it/CLHEP::cm;
                          (const_cast<DetMaterial*> (_mat))->setDEDXtype( DetMaterial::loss );
                          ntpData[1]=-_mat->energyLoss(_partmom, *pl_it, _tpart);
                          (const_cast<DetMaterial*> (_mat))->setCutOffEnergy(0.02);
                          ntpData[2]=-_mat->energyLoss(_partmom, *pl_it, _tpart);
                          (const_cast<DetMaterial*> (_mat))->setCutOffEnergy(0.5);
                          ntpData[3]=-_mat->energyLoss(_partmom, *pl_it, _tpart);
                          (const_cast<DetMaterial*> (_mat))->setCutOffEnergy(1);
                          ntpData[4]=-_mat->energyLoss(_partmom, *pl_it, _tpart);
                          _meanEloss->Fill(ntpData);
                  }

                  float ntpdEdxData[2];
                  double tcutStep = _maxTCut_dEdx/((double)_nStepTCut_dEdx);
                  ntpdEdxData[0]=0.0;
                  for (unsigned int istep=1; istep<=_nStepTCut_dEdx; ++istep) {
                          //ntpdEdxData[0] = tcutStep*((double)istep);
                          ntpdEdxData[0]+=tcutStep;
                          (const_cast<DetMaterial*> (_mat))->setCutOffEnergy(ntpdEdxData[0]);
                          ntpdEdxData[1] = -_mat->dEdx(_partmom,DetMaterial::deposit,_tpart)/_mat->density()/CLHEP::cm/CLHEP::cm;
                          _dEdx->Fill(ntpdEdxData);
                  }
                  ntpdEdxData[0]+=tcutStep;
                  ntpdEdxData[1] = -_mat->dEdx(_partmom,DetMaterial::loss,_tpart)/_mat->density()/CLHEP::cm/CLHEP::cm;
                  _dEdx->Fill(ntpdEdxData);


          }

  }

  void materailsPropStudy::endJob()
  {

          // cd() to correct root directory. See note 3.
          TDirectory* save = gDirectory;
          _directory->cd();

          // Write canvas.  See note 4.
      //    _canvas->Write();

          // cd() back to where we were.  See note 3.
          save->cd();

  }

}  // end namespace mu2e

using mu2e::materailsPropStudy;
DEFINE_ART_MODULE(materailsPropStudy);
