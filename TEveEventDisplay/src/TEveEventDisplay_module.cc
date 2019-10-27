//Author: SMiddleton
//Purpose: To make TEVe based event displaysin Offleint
// ... libCore
#include <TApplication.h>
#include <TString.h>
#include <TSystem.h>
#include <TList.h>
#include <TObjArray.h>
// ... libRIO
#include <TFile.h>
// ... libGui
#include <TGString.h>
#include <TGLabel.h>
#include <TGButton.h>
#include <TGButtonGroup.h>
#include <TGTextEntry.h>
#include <TGTextView.h>
#include <TGLayout.h>
#include <TGTab.h>
#include <TG3DLine.h>
// ... libGeom
#include <TGeoManager.h>
#include <TGeoTube.h>
#include <TGeoCompositeShape.h>
#include <TGeoBoolNode.h>
#include <TGeoNode.h>
#include <TGeoPhysicalNode.h>
// ... libEG
#include <TParticle.h>
// ... libRGL
#include <TGLViewer.h>
// ... libEve
#include <TEveManager.h>
#include <TEveEventManager.h>
#include <TEveBrowser.h>
#include <TEveGeoNode.h>
#include <TEveViewer.h>
#include <TEveScene.h>
#include <TEveProjectionManager.h>
#include <TEveProjectionAxes.h>
#include <TEvePointSet.h>
#include <TEveTrack.h>
#include <TEveTrackPropagator.h>
#include <TEveStraightLineSet.h>

//#include <sstream>
#include "fstream"

//#include  "TEveEventDisplay/inc/EventDisplay3D.h"
#include  "TEveEventDisplay/inc/NavState.h"
#include  "TEveEventDisplay/inc/EvtDisplayUtils.h"
// Mu2e Utilities
#include "GeometryService/inc/GeomHandle.hh"
#include "Mu2eUtilities/inc/SimParticleTimeOffset.hh"
#include "TrkDiag/inc/TrkMCTools.hh"

//Mu2e Tracker Geom:
#include "TrackerGeom/inc/Tracker.hh"
#include "TrackerGeom/inc/Straw.hh"


#include "TrkDiag/inc/ComboHitInfo.hh"
//#include "GeneralUtilities/inc/ParameterSetHelpers.hh"


// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art_root_io/TFileService.h"
#include "art/Framework/Core/ModuleMacros.h"


// Mu2e diagnostics
using namespace mu2e;

class point {
	public:
	float x;
	float y;
	float z;
	};
    
class traj {
	public:
	int id;
	std::vector<point> tv;
};

void setRecursiveColorTransp(TGeoVolume *vol, Int_t color, Int_t transp)
  {
     if(color>=0)vol->SetLineColor(color);
     if(transp>=0)vol->SetTransparency(transp);
     Int_t nd = vol->GetNdaughters();
     for (Int_t i=0; i<nd; i++) {
        setRecursiveColorTransp(vol->GetNode(i)->GetVolume(), color, transp);
     }
  }

namespace mu2e 
{
  class TEveEventDisplay : public art::EDAnalyzer {
    public:
      explicit TEveEventDisplay(fhicl::ParameterSet const& pset);
      virtual ~TEveEventDisplay();
      virtual void beginJob();
      virtual void beginRun(const art::Run& run);
      void analyze(const art::Event& e);
      bool _mcdiag;
      Int_t _evt; 

      std::string moduleLabel_;
      const ComboHitCollection* _chcol;
      art::InputTag   _chtag;//combo
      // Label of the modules that created the data products.
     art::InputTag strawStepsTag_;
     art::InputTag strawDigisTag_;
     art::InputTag strawHitsTag_;
     art::InputTag comboHitsTag_;
     std::string generatorModuleLabel_;
     std::string g4ModuleLabel_;
     std::string hitMakerModuleLabel_;

     // Name of the tracker StepPoint collection
     std::string trackerStepPoints_;

     // Cuts used inside SimParticleWithHits:
     //  - drop hits with too little energy deposited.
     //  - drop SimParticles with too few hits.
     double minEnergyDep_;
     size_t minHits_;

      bool doDisplay_;
      bool clickToAdvance_;
	//List of functions:
      bool            drawGenTracks_;
      bool            drawHits_;
      Double_t        hitMarkerSize_;
      Double_t        trkMaxR_;
      Double_t        trkMaxZ_;
      Double_t        trkMaxStepSize_;
      Double_t        camRotateCenterH_;
      Double_t        camRotateCenterV_;
      Double_t        camDollyDelta_;

    //art::ServiceHandle<Geometry>          geom_;
    //art::ServiceHandle<PDT>               pdt_;

     
      TEveGeoShape* fSimpleGeom;

      TEveViewer *fXYView;
      TEveViewer *fRZView;
      TEveProjectionManager *fXYMgr;
      TEveProjectionManager *fRZMgr;
      TEveScene *fDetXYScene;
      TEveScene *fDetRZScene;
      TEveScene *fEvtXYScene;
      TEveScene *fEvtRZScene;

      TGTextEntry      *fTeRun,*fTeEvt;
      TGLabel          *fTlRun,*fTlEvt;
   
      TEveTrackList *fTrackList;
      TEveElementList *fHitsList;
      
      EvtDisplayUtils *visutil_ = new EvtDisplayUtils();
      //trkMaxStepSize_ = 0.1;
   
      void makeNavPanel();
      void InsideDS( TGeoNode * node, bool inDSVac );
      void hideTop(TGeoNode* node);
      void hideNodesByName(TGeoNode* node, const std::string& str,bool onOff) ;
      void hideNodesByMaterial(TGeoNode* node, const std::string& mat, bool onOff);
      void hideBuilding(TGeoNode* node);
      void AddHits(const art::Event& event);
      bool FindData(const art::Event& event);
};

TEveEventDisplay::TEveEventDisplay(fhicl::ParameterSet const& pset) :
	art::EDAnalyzer(pset),
	
	_mcdiag		(pset.get<bool>("MCdiag",true)),
	_chtag		(pset.get<art::InputTag>("ComboHitCollection")),
        strawStepsTag_(pset.get<string>("strawStepsTag")),
        strawDigisTag_(pset.get<string>("strawDigisTag")),
        strawHitsTag_(pset.get<string>("strawHitsTag")),
        comboHitsTag_(pset.get<string>("comboHitsTag")),
        generatorModuleLabel_(pset.get<std::string>("generatorModuleLabel")),
        g4ModuleLabel_(pset.get<std::string>("g4ModuleLabel")),
        hitMakerModuleLabel_(pset.get<std::string>("hitMakerModuleLabel")),
        trackerStepPoints_(pset.get<std::string>("trackerStepPoints")),
        minEnergyDep_(pset.get<double>("minEnergyDep")),
        minHits_(pset.get<unsigned>("minHits")),
	doDisplay_(pset.get<bool>("doDisplay",false)),
        clickToAdvance_(pset.get<bool>("clickToAdvance",false)){}
/*
        fSimpleGeom(pset.get<TEveGeoShape>("fSimpleGeom",0)),
        fXYView(pset.get<TEveViewer>("fXYView","")),
        fRZView(pset.get<TEveViewer>("fRZView","")),
	fXYMgr(pset.get<TEveProjectionManager>("fXYMgr","")),
	fRZMgr(pset.get<TEveProjectionManager>("fRZMgr","")),
	fDetXYScene(pset.get<TEveScene>("fDetXYScene","")),
	fDetRZScene(pset.get<TEveScene>("fDetRZScene","")),
	fEvtXYScene(pset.get<TEveScene>("EvtXYScene","")),
	fEvtRZScene(pset.get<TEveScene>("fEvtRZScene","")),
	fTeRun(pset.get<TGTextEntry>("fTeRun"," "),
	fTeEvt(pset.get<TGTextEntry>("fTeEvt"," ")),
	fTlRun(pset.get<TGLabel>("fTlRun"," ")),
	fTlEvt(pset.get<TGLabel>("fTlEvt"," ")),
	fTrackList(pset.get<TEveTrackList>("fTrackLaist","")),
	fHitsList(pset.get<TEveElementList>("fHitsList","")){}*/

TEveEventDisplay::~TEveEventDisplay(){}

/*-------Create Control Panel For Event Navigation----""*/
void TEveEventDisplay::makeNavPanel()
{
  TEveBrowser* browser = gEve->GetBrowser();
  browser->StartEmbedding(TRootBrowser::kLeft); 

  TGMainFrame* frmMain = new TGMainFrame(gClient->GetRoot(), 1000, 600);
  frmMain->SetWindowName("EVT NAV");
  frmMain->SetCleanup(kDeepCleanup);

  TGHorizontalFrame* navFrame = new TGHorizontalFrame(frmMain);
  TGVerticalFrame* evtidFrame = new TGVerticalFrame(frmMain);
  {
    TString icondir(TString::Format("%s/icons/", gSystem->Getenv("ROOTSYS")) );
    TGPictureButton* b = 0;

    // ... Create back button and connect to "PrevEvent" rcvr in visutils
    b = new TGPictureButton(navFrame, gClient->GetPicture(icondir + "GoBack.gif"));
    navFrame->AddFrame(b);
    b->Connect("Clicked()", "EvtDisplayUtils", visutil_, "PrevEvent()");

    // ... Create forward button and connect to "NextEvent" rcvr in visutils
    b = new TGPictureButton(navFrame, gClient->GetPicture(icondir + "GoForward.gif"));
    navFrame->AddFrame(b);
    b->Connect("Clicked()", "EvtDisplayUtils", visutil_, "NextEvent()");

    // ... Create run num text entry widget and connect to "GotoEvent" rcvr in visutils
    TGHorizontalFrame* runoFrame = new TGHorizontalFrame(evtidFrame);
    fTlRun = new TGLabel(runoFrame,"Run Number");
    fTlRun->SetTextJustify(kTextLeft);
    fTlRun->SetMargins(5,5,5,0);
    runoFrame->AddFrame(fTlRun);
    
    fTeRun = new TGTextEntry(runoFrame, visutil_->fTbRun = new TGTextBuffer(5), 1);
    visutil_->fTbRun->AddText(0, "1");
    fTeRun->Connect("ReturnPressed()","EvtDisplayUtils", visutil_,"GotoEvent()");
    runoFrame->AddFrame(fTeRun,new TGLayoutHints(kLHintsExpandX));

    // ... Create evt num text entry widget and connect to "GotoEvent" rcvr in visutils
    TGHorizontalFrame* evnoFrame = new TGHorizontalFrame(evtidFrame);
    fTlEvt = new TGLabel(evnoFrame,"Evt Number");
    fTlEvt->SetTextJustify(kTextLeft);
    fTlEvt->SetMargins(5,5,5,0);
    evnoFrame->AddFrame(fTlEvt);

    fTeEvt = new TGTextEntry(evnoFrame, visutil_->fTbEvt = new TGTextBuffer(5), 1);
    visutil_->fTbEvt->AddText(0, "1");
    fTeEvt->Connect("ReturnPressed()","EvtDisplayUtils", visutil_,"GotoEvent()");
    evnoFrame->AddFrame(fTeEvt,new TGLayoutHints(kLHintsExpandX));

    // ... Add horizontal run & event number subframes to vertical evtidFrame
    evtidFrame->AddFrame(runoFrame,new TGLayoutHints(kLHintsExpandX));
    evtidFrame->AddFrame(evnoFrame,new TGLayoutHints(kLHintsExpandX));

    // ... Add navFrame and evtidFrame to MainFrame
    frmMain->AddFrame(navFrame);
    TGHorizontal3DLine *separator = new TGHorizontal3DLine(frmMain);
    frmMain->AddFrame(separator, new TGLayoutHints(kLHintsExpandX));
    frmMain->AddFrame(evtidFrame);

    frmMain->MapSubwindows();
    frmMain->Resize();
    frmMain->MapWindow();

    browser->StopEmbedding();
    browser->SetTabTitle("Event Nav", 0);
  }
}

void TEveEventDisplay::beginJob(){
  
  if ( !doDisplay_ ) return;

  // Initialize global Eve application manager (return gEve)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TEveManager::Create();

  // Create detector and event scenes for ortho views
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  fDetXYScene = gEve->SpawnNewScene("Det XY Scene", "");
  fDetRZScene = gEve->SpawnNewScene("Det RZ Scene", "");
  fEvtXYScene = gEve->SpawnNewScene("Evt XY Scene", "");
  fEvtRZScene = gEve->SpawnNewScene("Evt RZ Scene", "");

  // Create XY/RZ projection mgrs, draw projected axes, & add them to scenes
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  fXYMgr = new TEveProjectionManager(TEveProjection::kPT_RPhi);
  TEveProjectionAxes* axes_xy = new TEveProjectionAxes(fXYMgr);
  fDetXYScene->AddElement(axes_xy);

  fRZMgr = new TEveProjectionManager(TEveProjection::kPT_RhoZ);
  TEveProjectionAxes* axes_rz = new TEveProjectionAxes(fRZMgr);
  fDetRZScene->AddElement(axes_rz);

  // Create side-by-side ortho XY & RZ views in new tab & add det/evt scenes
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  TEveWindowSlot *slot = 0;
  TEveWindowPack *pack = 0;

  slot = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
  pack = slot->MakePack();
  pack->SetElementName("Ortho Views");
  pack->SetHorizontal();
  pack->SetShowTitleBar(kFALSE);

  pack->NewSlot()->MakeCurrent();
  fXYView = gEve->SpawnNewViewer("XY View", "");
  fXYView->GetGLViewer()->SetCurrentCamera(TGLViewer::kCameraOrthoXOY);
  fXYView->AddScene(fDetXYScene);
  fXYView->AddScene(fEvtXYScene);

  pack->NewSlot()->MakeCurrent();
  fRZView = gEve->SpawnNewViewer("RZ View", "");
  fRZView->GetGLViewer()->SetCurrentCamera(TGLViewer::kCameraOrthoXOY);
  fRZView->AddScene(fDetRZScene);
  fRZView->AddScene(fEvtRZScene);

  gEve->GetBrowser()->GetTabRight()->SetTab(0);

  // Create navigation panel
  // ~~~~~~~~~~~~~~~~~~~~~~~~
  makeNavPanel();

  // Add new Eve event into the "Event" scene and make it the current event
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // (Subsequent elements added using "AddElements" will be added to this event)
  gEve->AddEvent(new TEveEventManager("Event", "Toy Detector Event"));

  
  TGLViewer *glv = gEve->GetDefaultGLViewer();
  glv->SetGuideState(TGLUtil::kAxesEdge, kTRUE, kFALSE, 0);
  glv->CurrentCamera().RotateRad(camRotateCenterH_,camRotateCenterV_);
  glv->CurrentCamera().Dolly(camDollyDelta_,kFALSE,kFALSE);

}

void TEveEventDisplay::beginRun(art::Run& run){
  if(gGeoManager){
    gGeoManager->GetListOfNodes()->Delete();
    gGeoManager->GetListOfVolumes()->Delete();
    gGeoManager->GetListOfShapes()->Delete();
  }
  gEve->GetGlobalScene()->DestroyElements();
  fDetXYScene->DestroyElements();
  fDetRZScene->DestroyElements();

  TGeoManager* geom = TGeoManager::Import("mu2e.gdml");

  TGeoVolume* topvol = geom->GetTopVolume();

  gGeoManager->SetTopVolume(topvol);
  gGeoManager->SetTopVisible(kTRUE);
  int nn = gGeoManager->GetNNodes();
  printf("nodes in geom = %d\n",nn);

  TGeoNode* topnode = gGeoManager->GetTopNode();
  TEveGeoTopNode* etopnode = new TEveGeoTopNode(gGeoManager, topnode);
  etopnode->SetVisLevel(4);
  etopnode->GetNode()->GetVolume()->SetVisibility(kFALSE);

  setRecursiveColorTransp(etopnode->GetNode()->GetVolume(), kWhite-10,70);
  InsideDS( topnode, false );
  hideBuilding(topnode);
  hideTop(topnode);
  //hideTop2();

  // ... Add static detector geometry to global scene
  gEve->AddGlobalElement(etopnode);

  // Draw the 2D projections using the EVE element list created above 
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // .... Add the EVE element list to the global scene first
  TGeoShape *composite=0;
  //TEveElementList *orthodet = new TEveElementList("OrthoDet");
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Using the detector specifications provided by the geometry service, create
  // a TGeoCompositeShape for drawing the detector in the main 3D view and an 
  // TEveElementList for drawing the detector in the 2D orthographic views
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  GeomHandle<Tracker> th; 
  const Tracker* tracker_mu2e = th.get(); 
  TubsParams envelope(tracker_mu2e->getInnerTrackerEnvelopeParams());
  //double rmax = envelope.outerRadius();
  //double rmin = envelope.innerRadius();
  //double dz = envelope.zHalfLength();

  //GeomHandle<Calorimeter> ca; 
  //const Calorimeter* calo_mu2e = ca.get(); 
  /*
  std::string layerid="Layer %d";
  Int_t i=0;
  for ( auto const& shell : geom_->tracker().shells() ){
    Double_t dz = 0.2*shell.halfLength();
    Double_t rmin = 0.1*shell.radius() ;
    Double_t rmax = rmin+0.1*shell.thickness();
    if(i==0){
      composite = new TGeoTube(Form(layerid.c_str(),i),rmin,rmax,dz); 
    }else{
      TGeoShape *gs = new TGeoTube(Form(layerid.c_str(),i),rmin,rmax,dz); 
      TGeoBoolNode *bn = new TGeoUnion(composite,gs);
      composite = new TGeoCompositeShape("TGeoCompositeShape", bn);
    }
    TEveGeoShape *egs = new TEveGeoShape(Form(layerid.c_str(),i));
    egs->SetShape(new TGeoTube(rmin, rmax, dz));
    egs->SetMainColor(kPink+7);
    orthodet->AddElement(egs);
    i++;
  }
   */
  // Draw the 3D detector using the composite shape created above
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ... Define some materials
  TGeoMaterial *matVac = new TGeoMaterial("Vac", 0,0,0);
  TGeoMaterial *matSi = new TGeoMaterial("Si", 28.085,14,2.33);

  // ... Define some media
  TGeoMedium *Vacuum = new TGeoMedium("Vacuum",1, matVac);
  TGeoMedium *Si = new TGeoMedium("Silicon",2, matSi);

  // ... Define top volume as an empty box
  TGeoVolume *dettopvol = gGeoManager->MakeBox("Detector", Vacuum, 1000., 1000., 1000.);
  gGeoManager->SetTopVolume(dettopvol);

  // ... Create tracker out of Silicon using the composite shape defined above
  TGeoVolume *tracker = new TGeoVolume("Tracker",composite, Si);
  tracker->SetVisLeaves(kTRUE);
  dettopvol->AddNode(tracker, 1, new TGeoTranslation(0,0,0));
  gGeoManager->CloseGeometry();

  TGeoNode* dettopnode = gGeoManager->GetTopNode();
  TEveGeoTopNode* detetopnode = new TEveGeoTopNode(gGeoManager, dettopnode);
  detetopnode->SetVisLevel(4);
  detetopnode->GetNode()->GetVolume()->SetVisibility(kTRUE);

  // ... Use helper to recursively make inner/outer tracker descendants 
  //     transparent & set custom colors
  setRecursiveColorTransp(etopnode->GetNode()->GetVolume(), kBlack-10, 70);

  // ... Add static detector geometry to global scene
  gEve->AddGlobalElement(detetopnode);

   /*Draw the 2D projections using the EVE element list created above 
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // .... Add the EVE element list to the global scene first
  gEve->AddGlobalElement(orthodet);

  // ... Import elements of the list into the projected views
  fXYMgr->ImportElements(orthodet, fDetXYScene);
  fRZMgr->ImportElements(orthodet, fDetRZScene);

  // ... Turn OFF rendering of duplicate detector in main 3D view
  gEve->GetGlobalScene()->FindChild("OrthoDet")->SetRnrState(kFALSE);

  // ... Turn ON rendering of detector in RPhi and RZ views
  fDetXYScene->FindChild("OrthoDet [P]")->SetRnrState(kTRUE);
  fDetRZScene->FindChild("OrthoDet [P]")->SetRnrState(kTRUE);
  */
   
}


void TEveEventDisplay::InsideDS( TGeoNode * node, bool inDSVac ){
  std::string _name = (node->GetVolume()->GetName());

  if ( node->GetMotherVolume() ) {
    std::string motherName(node->GetMotherVolume()->GetName());
    if ( motherName == "DS2Vacuum" || motherName == "DS3Vacuum" ){
      inDSVac = true;
    }
  }

  if ( inDSVac && _name.find("Virtual") != 0 ) {
    node->SetVisibility(kTRUE);
  } else{
    node->SetVisibility(kFALSE);
  }

  // Descend into each daughter TGeoNode.
  int ndau = node->GetNdaughters();
  for ( int i=0; i<ndau; ++i ){
    TGeoNode * dau = node->GetDaughter(i);
    InsideDS( dau, inDSVac );
  }

}

void TEveEventDisplay::analyze(const art::Event& event){
  _evt = event.id().event();
  FindData(event);
  
  std::ostringstream sstr;
  sstr << event.id().run();
  visutil_->fTbRun->Clear();
  visutil_->fTbRun->AddText(0,sstr.str().c_str());
  gClient->NeedRedraw(fTeRun);

  sstr.str("");
  sstr << event.id().event();
  visutil_->fTbEvt->Clear();
  visutil_->fTbEvt->AddText(0,sstr.str().c_str());
  gClient->NeedRedraw(fTeEvt);
  
  // ... Delete visualization structures associated with previous event
  gEve->GetViewers()->DeleteAnnotations();
  gEve->GetCurrentEvent()->DestroyElements();
  //AddHits(event);


} // end TEveEventDisplay::analyze




void TEveEventDisplay::hideTop(TGeoNode* node) {

  TString name = node->GetName();
  if(name.Index("Shield")>0) {
    std::cout << name << " " <<  name.Index("mBox_") << std::endl;
  }
  bool test = false;

  // from gg1
  if(name.Index("mBox_45_")>=0) test = true;
  if(name.Index("mBox_46_")>=0) test = true;
  if(name.Index("mBox_47_")>=0) test = true;
  if(name.Index("mBox_48_")>=0) test = true;
  if(name.Index("mBox_49_")>=0) test = true;
  if(name.Index("mBox_74_")>=0) test = true;

  // from 542
  /*
  if(name.Index("mBox_51_")>=0) test = true;
  if(name.Index("mBox_52_")>=0) test = true;
  if(name.Index("mBox_53_")>=0) test = true;
  if(name.Index("mBox_54_")>=0) test = true;
  if(name.Index("mBox_55_")>=0) test = true;
  if(name.Index("mBox_64_")>=0) test = true;
  if(name.Index("mBox_83_")>=0) test = true;
  */
  if(test) {
    std::cout << "turning off " << name << std::endl;
    node->SetVisibility(false);
  }


  /*
  std::string str("ExtShield");
  if ( name.find(str) != std::string::npos ){
    TGeoVolume* gv = node->GetVolume(); 
    printf("vol %ul\n",gv);
    TGeoShape* gs = gv->GetShape();
    printf("ExtShield shape %s\n",gs->IsA()->GetName());
    Double_t dx,dy,dz;
  }
  */
  
  // Descend recursively into each daughter TGeoNode.
  int ndau = node->GetNdaughters();
  for ( int i=0; i<ndau; ++i ){
    TGeoNode * dau = node->GetDaughter(i);
    hideTop( dau );
  }

}

void TEveEventDisplay::hideNodesByName(TGeoNode* node, const std::string& str,
				     bool onOff) {

  std::string name(node->GetName());
  if ( name.find(str) != std::string::npos ){
    node->SetVisibility(onOff);
    //std::cout <<"hiding "<< name << std::endl;
  }

  // Descend recursively into each daughter TGeoNode.
  int ndau = node->GetNdaughters();
  for ( int i=0; i<ndau; ++i ){
    TGeoNode * dau = node->GetDaughter(i);
    hideNodesByName( dau, str, onOff);
  }

}

void TEveEventDisplay::hideNodesByMaterial(TGeoNode* node, 
					 const std::string& mat, bool onOff) {

  std::string material(node->GetVolume()->GetMaterial()->GetName());
  if ( material.find(mat) != std::string::npos ) node->SetVisibility(onOff);

  // Descend recursively into each daughter TGeoNode.
  int ndau = node->GetNdaughters();
  for ( int i=0; i<ndau; ++i ){
    TGeoNode * dau = node->GetDaughter(i);
    hideNodesByMaterial( dau, mat, onOff);
  }

}

void TEveEventDisplay::hideBuilding(TGeoNode* node) {

  static std::vector <std::string> substrings  { "Ceiling",
      "backfill", "dirt", "concrete", "VirtualDetector",
      "pipeType","CRSAluminium","CRV","CRS", "ExtShield", "PSShield"};
  for(auto& i: substrings) hideNodesByName(node,i,kFALSE);

  // Volumes with these material names will be made invisible.
  //"CONCRETE"
  static std::vector <std::string> materials { "MBOverburden", "CONCRETE"};
  for(auto& i: materials) hideNodesByMaterial(node,i,kFALSE);

  // add back in extshield
  //std::string name("ExtShield");
  //hideNodesByName(node,name,kTRUE);

}


void TEveEventDisplay::AddHits(const art::Event& event){
/*
  int drawHits_=1;
  if (drawHits_) {
    std::vector<art::Handle<IntersectionCollection>> hitsHandles;
    event.getManyByType(hitsHandles);

    if (fHitsList == 0) {
      fHitsList = new TEveElementList("Hits"); 
      fHitsList->IncDenyDestroy();              // protect element against destruction
    }
    else {
      fHitsList->DestroyElements();             // destroy children of the element
    }

    TEveElementList* KpHitsList  = new TEveElementList("K+ Hits"); 
    TEveElementList* KmHitsList  = new TEveElementList("K- Hits"); 
    TEveElementList* BkgHitsList = new TEveElementList("Bkg Hits"); 

    int ikp=0,ikm=0,ibkg=0;
    for ( auto const& handle: hitsHandles ){
      for ( auto const& hit: *handle ){
        if ( hit.genTrack()->pdgId() == PDGCode::K_plus ){
          drawHit("K+",kGreen,hitMarkerSize_,ikp++,hit,KpHitsList);
        } else if ( hit.genTrack()->pdgId() == PDGCode::K_minus ){
          drawHit("K-",kYellow,hitMarkerSize_,ikm++,hit,KmHitsList);
        } else{
          drawHit("Bkg",kViolet+1,hitMarkerSize_,ibkg++,hit,BkgHitsList);
        }
      }
    }
    fHitsList->AddElement(KpHitsList);  
    fHitsList->AddElement(KmHitsList);  
    fHitsList->AddElement(BkgHitsList);  
    gEve->AddElement(fHitsList);
  }
*/
}


bool TEveEventDisplay::FindData(const art::Event& evt){
	_chcol = 0; 
        
	auto chH = evt.getValidHandle<ComboHitCollection>(_chtag);
	_chcol = chH.product();
	
	return _chcol != 0;
       }   
	
}
/*
void eve() {
  TEveEventDisplay ed3d;
  ed3d.beginJob();
  ed3d.beginRun();
  ed3d.analyze(const art::Event& event);

}
*/
