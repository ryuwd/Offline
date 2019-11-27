// Ryunosuke O'Neil, 2019
// Module calling the MilleWrapper to set up plane alignment

#include "art/Framework/Core/EDAnalyzer.h"

#include "CosmicAlignment/inc/Mille.h"

#include "GeometryService/inc/GeometryService.hh"
#include "GeometryService/inc/GeomHandle.hh"
#include "TrackerGeom/inc/Tracker.hh"

#include "RecoDataProducts/inc/ComboHit.hh"
#include "RecoDataProducts/inc/CosmicTrack.hh"
#include "RecoDataProducts/inc/CosmicTrackSeed.hh"

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

        fhicl::Atom<art::InputTag> chtag{Name("ComboHitCollection"), Comment("tag for combo hit collection")};
        fhicl::Atom<art::InputTag> costag{Name("CosmicTrackSeedCollection"), Comment("tag for cosmic track seed collection")};
    };
    typedef art::EDAnalyzer::Table<Config> Parameters;

    virtual void beginJob();
    virtual void analyze(art::Event const &) override;

    PlaneAlignment(const Parameters &conf) : art::EDAnalyzer(conf),
                                             _chtag(conf().chtag()),
                                             _costag(conf().costag()) {}

    virtual ~PlaneAlignment();

    Config _conf;

    int _diag;
    bool _mcdiag;

    art::InputTag _chtag;
    art::InputTag _costag;

    const ComboHitCollection *_chcol;
    const CosmicTrackSeedCollection *_coscol;
};

PlaneAlignment::~PlaneAlignment()
{
}

void PlaneAlignment::beginJob()
{

}

void PlaneAlignment::analyze(art::Event const &event)
{
}

}; // namespace mu2e