// Andrei Gaponenko, 2016

#include <exception>                   // for exception
#include <memory>                             // for unique_ptr
#include <typeinfo>                           // for type_info

#include "Mu2eUtilities/inc/EventWeightHelper.hh"
#include "fhiclcpp/ParameterSet.h"            // for ParameterSet
#include "art/Framework/Principal/Event.h"    // for Event
#include "art_root_io/TFileDirectory.h"       // for TFileDirectory
#include "MCDataProducts/inc/EventWeight.hh"  // for EventWeight
#include "TH1.h"                              // for TH1D, TH1
#include "art/Framework/Principal/Handle.h"   // for ValidHandle
#include "fhiclcpp/coding.h"                  // for ps_sequence_t, sequence_t
#include "fhiclcpp/exception.h"               // for exception

namespace mu2e {

  EventWeightHelper::EventWeightHelper(const fhicl::ParameterSet& pset,
                                       art::TFileDirectory topdir,
                                       const std::string& subdir)
    : inputs_{pset.is_empty() ? InputTags() : pset.get<InputTags>("inputs")}
    , weight_{-1.}
    , hfinal_{nullptr}
  {
    if(!pset.is_empty()) {
      art::TFileDirectory tfdir = subdir.empty() ? topdir : topdir.mkdir(subdir.c_str());
      const int nbins = pset.get<unsigned>("hist.nbins", 100);
      const int xmin = pset.get<unsigned>("hist.xmin", 0.);
      const int xmax = pset.get<unsigned>("hist.xmax", 2.);
      hfinal_ = tfdir.make<TH1D>("finalWeight", "Final event weight", nbins, xmin, xmax);
    }
  }

  void EventWeightHelper::update(const art::Event& evt) {
    weight_ = 1.;
    for(const auto& in: inputs_) {
      const auto& wh = evt.getValidHandle<EventWeight>(in);
      weight_ *= wh->weight();
    }
    if(hfinal_) {
      hfinal_->Fill(weight_);
    }
  }
}
