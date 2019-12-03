// Ryunosuke O'Neil, 2019
// Module calling the MilleWrapper to set up plane alignment

#include "art/Framework/Core/EDAnalyzer.h"
#include "art_root_io/TFileService.h"

#include "ProditionsService/inc/ProditionsHandle.hh"
#include "GeometryService/inc/GeometryService.hh"
#include "GeometryService/inc/GeomHandle.hh"
#include "TrackerGeom/inc/Tracker.hh"

#include "RecoDataProducts/inc/ComboHit.hh"
#include "RecoDataProducts/inc/CosmicTrack.hh"
#include "RecoDataProducts/inc/CosmicTrackSeed.hh"
#include "art/Framework/Core/ModuleMacros.h"

#include "CosmicAlignment/inc/MilleWrapper.hh"
#include "CosmicAlignment/inc/AlignableObjects.hh"
#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"

#include "Mu2eUtilities/inc/TwoLinePCA_XYZ.hh"

#include "CosmicReco/inc/DriftFitUtils.hh"


#include "TH1F.h"


using namespace mu2e;
/*
    p1, t1: track
    p2, t2: wire

    // Cosine of angle between the two line directions.
    double c(_t1.Dot(_t2));

    // Sine-squared corresponding to c.
    double sinsq(1.-c*c);

      XYZVec delta(_p1-_p2);
      double dDotT1 = delta.Dot(_t1);
      double dDotT2 = delta.Dot(_t2);

      _s1 =  (dDotT2*c-dDotT1)/sinsq;
      _s2 = -(dDotT1*c-dDotT2)/sinsq;

      _pca1 = _p1 + _t1*_s1;
      _pca2 = _p2 + _t2*_s2;
      _LRambig = _s2 > 0 ? 1 : -1;//int ambig_sign= ambig > 0 ? 1 : -1;

    XYZVec diff = (_pca1-_pca2);
    _dca   = sqrt(diff.Mag2());
    _dca2d = sqrt(diff.Perp2());



def DOCA(track_pos, track_dir, wire_pos, wire_dir):
    alpha = track_dir.dot(wire_dir)
    beta = 1 - alpha*alpha
    delta = track_pos - wire_pos

    ddotT1 = delta.dot(track_dir)
    ddotT2 = delta.dot(wire_dir)

    s1 = (ddotT2 * alpha - ddotT1)/beta
    s2 = -(ddotT1 * alpha - ddotT2)/beta

    pca1 = track_pos + track_dir * s1
    pca2 = wire_pos + wire_dir * s2

    diff = pca1 - pca2

    doca = sqrt(diff.dot(diff))

    return sympy.Piecewise((doca, s2 > 0), (-doca, True))
*/

double DOCA(Straw const& straw, double a0, double a1, double b0, double b1) {
	XYZVec track_position(a0,b0,0);
	XYZVec track_direction(a1,b1,1);

	const CLHEP::Hep3Vector& spos = straw.getMidPoint();
	const CLHEP::Hep3Vector& sdir = straw.getDirection();
	XYZVec wire_position  = Geom::toXYZVec(spos);
    XYZVec wire_direction = Geom::toXYZVec(sdir);

	TwoLinePCA_XYZ PCA = TwoLinePCA_XYZ(track_position,
                track_direction,
                wire_position,
                wire_direction,
                1.e-8);
	return PCA.LRambig() * PCA.dca();
}

namespace mu2e
{

class PlaneAlignment : public art::EDAnalyzer
{
private:
    TH1F *residuum;


public:
    struct Config
    {
        using Name = fhicl::Name;
        using Comment = fhicl::Comment;

        //fhicl::Atom<art::InputTag> chtag{Name("ComboHitCollection"), Comment("tag for combo hit collection")};
        fhicl::Atom<art::InputTag> costag{Name("CosmicTrackSeedCollection"), Comment("tag for cosmic track seed collection")};
        fhicl::Atom<std::string> millefile{Name("MillepedeBinaryOutputFile"), Comment("Output filename for millepede binary")};

    };
    typedef art::EDAnalyzer::Table<Config> Parameters;

    virtual void beginJob();
    virtual void endJob();
    virtual void beginRun(art::Run const&);

    virtual void analyze(art::Event const &) override;

    PlaneAlignment(const Parameters &conf) : art::EDAnalyzer(conf),
                                             //_chtag(conf().chtag()),
                                             _costag(conf().costag()),
                                             millepede(conf().millefile()) {}

    virtual ~PlaneAlignment();

    Config _conf;

    int _diag;
    bool _mcdiag;

    //art::InputTag _chtag;
    art::InputTag _costag;

    //const ComboHitCollection *_chcol;
    const CosmicTrackSeedCollection *_coscol;


    MilleWrapper millepede;

    Tracker const *tracker = nullptr;
    //ProditionsHandle<StrawResponse> srep_h;
};

PlaneAlignment::~PlaneAlignment()
{
}

void PlaneAlignment::beginJob()
{

	art::ServiceHandle<art::TFileService> tfs;
    residuum = tfs->make<TH1F>("residuum","Straw Hit Residuals ",100,-40, 25);
    residuum->GetXaxis()->SetTitle("Residual (DOCA - Estimated Drift Distance) (mm)");

}

void PlaneAlignment::beginRun(art::Run const&)
{
    if (tracker == nullptr)
    {
        mu2e::GeomHandle<mu2e::Tracker> th;
        tracker = th.get();

        // get all planes and register them as alignable objects
        for (auto const& p : tracker->getPlanes())
        {
            AlignablePlane a(p);
            millepede.RegisterAlignableObject(a);

        }
        millepede.StartRegisteringHits();
    }
}

void PlaneAlignment::endJob()
{
    millepede.Save();
}

void PlaneAlignment::analyze(art::Event const &event)
{
    //StrawResponse const& _srep = *srep_h.getPtr(event.id());

    assert(tracker != nullptr && "Check tracker instance is available");

    auto stH = event.getValidHandle<CosmicTrackSeedCollection>(_costag);
	_coscol = stH.product();

    for(size_t ist = 0;ist < _coscol->size(); ++ist)
    {
        CosmicTrackSeed sts =(*_coscol)[ist];
        CosmicTrack st = sts._track;

        TrkFitFlag const& status = sts._status;

        if (!status.hasAllProperties(TrkFitFlag::helixOK) ){continue;}
        if(st.converged == false or st.minuit_converged  == false) { continue;}

        if (isnan(st.MinuitFitParams.A0)) continue;

        XYZVec track_dir(st.MinuitFitParams.A1, st.MinuitFitParams.B1, 0);
        XYZVec track_dir_unit = track_dir.unit();

        // get residuum and their derivatives with respect
        // to all local and global parameters
        // get also plane id hit by straw hits
        for (auto const&straw_hit : sts._straw_chits)
        {
            auto const &plane_origin = tracker->getPlane(
                straw_hit.strawId().plane()).origin();
            auto const &straw = tracker->getStraw(
                straw_hit.strawId());
            auto straw_mp = straw.getMidPoint();
            auto wire_dir = straw.getDirection().unit();

            // now calculate the derivatives.
            auto const &align_obj = millepede.GetAlignableObject<AlignablePlane>((int)straw_hit.strawId().plane());
            auto derivs_local = RigidBodyDOCADerivatives_local(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                track_dir_unit.X(),
                track_dir_unit.Y(),
                straw_mp.x(), straw_mp.y(), straw_mp.z(), // TODO: is this suitable?
                wire_dir.x(), wire_dir.y(), wire_dir.z(),
                plane_origin.x(), plane_origin.y(), plane_origin.z()
                );
            auto derivs_global = RigidBodyDOCADerivatives_global(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                track_dir_unit.X(),
                track_dir_unit.Y(),
                straw_mp.x(), straw_mp.y(), straw_mp.z(), // TODO: is this suitable?
                wire_dir.x(), wire_dir.y(), wire_dir.z(),
                plane_origin.x(), plane_origin.y(), plane_origin.z()
                );
            // FIXME! use _srep utilities to do this properly!
            float drift_prediction = straw_hit.driftTime() * 0.065; //_srep.driftTimeToDistance(straw_hit.strawId(), straw_hit.driftTime(), straw_hit.);

            float resid = DOCA(straw, st.MinuitFitParams.A0,
                    st.MinuitFitParams.A1, st.MinuitFitParams.B0,
                    st.MinuitFitParams.B1) - drift_prediction;
            std::cout << "residual " << resid << std::endl;
            float resid_compar = RigidBodyDOCADerivatives_DOCAfn(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                track_dir_unit.X(),
                track_dir_unit.Y(),
                straw_mp.x(), straw_mp.y(), straw_mp.z(), // TODO: is this suitable?
                wire_dir.x(), wire_dir.y(), wire_dir.z()) - drift_prediction;
            std::cout << "residual used in python: " << resid_compar << std::endl;

            std::cout << "derivative example " << derivs_global[0] << std::endl;

            if (isnan(resid)) continue;
            millepede.RegisterTrackHit(align_obj,
                derivs_global,
                derivs_local,
                resid, // FIXME! (ask about correct drift distance estimate method)
                resid*0.1); // FIXME! need to estimate residual error
            residuum->Fill(resid);
        }
        millepede.WriteTrack();

    }
}

}; // namespace mu2e


using mu2e::PlaneAlignment;
DEFINE_ART_MODULE(PlaneAlignment);

