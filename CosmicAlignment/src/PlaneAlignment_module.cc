// Ryunosuke O'Neil, 2019
// Module calling the MilleWrapper to set up plane alignment

#include "art/Framework/Core/EDAnalyzer.h"

#include "CosmicAlignment/inc/MilleWrapper.h"

#include "GeometryService/inc/GeometryService.hh"
#include "GeometryService/inc/GeomHandle.hh"
#include "TrackerGeom/inc/Tracker.hh"

#include "RecoDataProducts/inc/ComboHit.hh"
#include "RecoDataProducts/inc/CosmicTrack.hh"
#include "RecoDataProducts/inc/CosmicTrackSeed.hh"
#include "art/Framework/Core/ModuleMacros.h"

namespace mu2e
{

class PlaneAlignment : public art::EDAnalyzer
{
private:
    /* data */

public:
    struct Config
    {
        using Name = fhicl::Name;
        using Comment = fhicl::Comment;

        //fhicl::Atom<art::InputTag> chtag{Name("ComboHitCollection"), Comment("tag for combo hit collection")};
        fhicl::Atom<art::InputTag> costag{Name("CosmicTrackSeedCollection"), Comment("tag for cosmic track seed collection")};
    };
    typedef art::EDAnalyzer::Table<Config> Parameters;

    virtual void beginJob();
    virtual void analyze(art::Event const &) override;

    PlaneAlignment(const Parameters &conf) : art::EDAnalyzer(conf),
                                             //_chtag(conf().chtag()),
                                             _costag(conf().costag()) {}

    virtual ~PlaneAlignment();

    Config _conf;

    int _diag;
    bool _mcdiag;

    //art::InputTag _chtag;
    art::InputTag _costag;

    //const ComboHitCollection *_chcol;
    const CosmicTrackSeedCollection *_coscol;


    MilleWrapper millepede("mu2e_plane_alignment.bin");
};

PlaneAlignment::~PlaneAlignment()
{
}

void PlaneAlignment::beginJob()
{
    mu2e::GeomHandle<mu2e::Tracker> th;
    const Tracker *tracker = th.get();

    // get all planes and register them as alignable objects
    for (auto const& plane : tracker->getPlanes())
        millepede.RegisterAlignableObject(plane.id().asUint16(), 6);
    millepede.StartRegisteringHits();
}

void PlaneAlignment::analyze(art::Event const &event)
{
    auto stH = event.getValidHandle<CosmicTrackSeedCollection>(_costag);
	_coscol = stH.product();

    for(size_t ist = 0;ist < _coscol->size(); ++ist)
    {
        CosmicTrackSeed sts =(*_coscol)[ist];
        CosmicTrack st = sts._track;

        TrkFitFlag const& status = sts._status;

        if (!status.hasAllProperties(TrkFitFlag::helixOK) ){continue;}
        if(st.converged == false or st.minuit_converged  == false) { continue;}


        // get all unbiased residuals and their derivatives with respect
        // to all local and global parameters
        // get also plane id hit by straw hits

        //millepede.RegisterTrackHit()

    }
}

}; // namespace mu2e


using mu2e::PlaneAlignment;
DEFINE_ART_MODULE(PlaneAlignment);

