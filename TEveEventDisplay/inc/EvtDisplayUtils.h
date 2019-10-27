#ifndef EvtDisplayUtils_h
#define EvtDisplayUtils_h

#include <TObject.h>
#include <TApplication.h>
#include <TGTextBuffer.h>
#include <iostream>

#include "TROOT.h"


  class EvtDisplayUtils
  {
    public:
      explicit EvtDisplayUtils();
      void PrevEvent();
      void NextEvent();
      void GotoEvent();
      TGTextBuffer *fTbRun;
      TGTextBuffer *fTbEvt;
    //ClassDef(EvtDisplayUtils,0);
  };

#endif
