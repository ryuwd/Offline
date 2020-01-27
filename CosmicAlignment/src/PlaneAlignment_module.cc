// Ryunosuke O'Neil, 2019
// Module calling the MilleWrapper to set up plane alignment

#include "art/Framework/Core/EDAnalyzer.h"
#include "art_root_io/TFileService.h"

#include "ProditionsService/inc/ProditionsHandle.hh"
#include "TrackerGeom/inc/Tracker.hh"

#include "RecoDataProducts/inc/ComboHit.hh"
#include "RecoDataProducts/inc/CosmicTrack.hh"
#include "RecoDataProducts/inc/CosmicTrackSeed.hh"
#include "art/Framework/Core/ModuleMacros.h"

#include "Mu2eUtilities/inc/TwoLinePCA_XYZ.hh"
#include "DataProducts/inc/StrawId.hh"
#include "CosmicReco/inc/DriftFitUtils.hh"


#include "CosmicAlignment/inc/Mille.h"
#include "CosmicAlignment/inc/RigidBodyDOCADeriv.hh"

#include "TH1F.h"


using namespace mu2e;
namespace mu2e
{

class PlaneAlignment : public art::EDAnalyzer
{
private:
    // Histograms for diagnostic purposes
    TH1F *residuum;


public:
    struct Config
    {
        using Name = fhicl::Name;
        using Comment = fhicl::Comment;
        fhicl::Atom<int> diaglvl { Name("diagLevel"), Comment("diagnostic level")};
        fhicl::Atom<art::InputTag> costag{Name("CosmicTrackSeedCollection"), Comment("tag for cosmic track seed collection")};
        fhicl::Atom<std::string> millefile{Name("MillepedeBinaryOutputFile"), Comment("Output filename for millepede binary")};

    };
    typedef art::EDAnalyzer::Table<Config> Parameters;

    override void beginJob();
    override void endJob();
    override void beginRun(art::Run const&);

    void analyze(art::Event const &) override;

    explicit PlaneAlignment(const Parameters &conf) : art::EDAnalyzer(conf),
                                            _diag(conf().diaglvl()),
                                            _costag(conf().costag()),
                                            _outfilename(conf().millefile())
    {
        // generate hashtable of plane number to DOF labels for planes

        int counter = 0;
        int dof_per_plane = 6;

        for (uint16_t i = 0; i < StrawId::_nplanes; i++)
        {
            std::vector<int> labels;
            for (size_t dof_n = 0; dof_n < dof_per_plane; dof_n++)
                labels.push_back(counter++);
            dof_labels[i] = std::move(labels);
        }

        if (_diag > 0)
        {
            std::cout << "Plane d.o.f. labels occupy range [0," << counter - 1 << "] inclusive" << std::endl;
        }
    }

    virtual ~PlaneAlignment() = default;

    Config _conf;

    int _diag;
    art::InputTag _costag;
    const CosmicTrackSeedCollection *_coscol;

    std::string _outfilename;
    std::unique_ptr<Mille> millepede;
    ProditionsHandle<Tracker> _alignedTracker_h;

    std::unordered_map<uint16_t, std::vector<int>> dof_labels;
    //ProditionsHandle<StrawResponse> srep_h;
};

void PlaneAlignment::beginJob()
{
    millepede = std::make_unique<Mille>(_outfilename.c_str());

    if (_diag > 0)
    {
        art::ServiceHandle<art::TFileService> tfs;
        residuum = tfs->make<TH1F>("residuum","Straw Hit Residuals ",100,-40, 25);
        residuum->GetXaxis()->SetTitle("Residual (DOCA - Estimated Drift Distance) (mm)");
    }
}

void PlaneAlignment::beginRun(art::Run const&)
{
}

void PlaneAlignment::endJob()
{
    // ensure the file is closed once the job finishes
    millepede->~Mille();
}

void PlaneAlignment::analyze(art::Event const &event)
{
    //StrawResponse const& _srep = srep_h.get(event.id());
    Tracker const& tracker = _alignedTracker_h.get(event.id());

    auto stH = event.getValidHandle<CosmicTrackSeedCollection>(_costag);
	_coscol = stH.product();

    for (CosmicTrackSeed const& sts : *_coscol)
    {
        CosmicTrack const& st = sts._track;

        TrkFitFlag const& status = sts._status;

        if (!status.hasAllProperties(TrkFitFlag::helixOK) ){continue;}
        if (!st.converged or !st.minuit_converged) { continue;}

        if (isnan(st.MinuitFitParams.A0)) continue;

        XYZVec track_pos(st.MinuitFitParams.A0, st.MinuitFitParams.B0, 0);
        XYZVec track_dir(st.MinuitFitParams.A1, st.MinuitFitParams.B1, 1);

        // get residuals and their derivatives with respect
        // to all local and global parameters
        // get also plane id hit by straw hits
        for (auto const&straw_hit : sts._straw_chits)
        {
            // straw and plane info
            StrawId straw_id = straw_hit.strawId();
            Straw const& straw = tracker.getStraw(straw_id);
            auto plane_id = straw_id.plane();

            // geometry info
            auto const& plane_origin = tracker.getPlane(plane_id).origin();
            auto const &straw_mp = straw.getMidPoint();
            auto const &wire_dir = straw.getDirection().unit();

            // now calculate the derivatives.
            auto derivs_local = RigidBodyDOCADerivatives_local(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                st.MinuitFitParams.A1,
                st.MinuitFitParams.B1,
                straw_mp.x(), straw_mp.y(), straw_mp.z(), // TODO: is this suitable?
                wire_dir.x(), wire_dir.y(), wire_dir.z(),
                plane_origin.x(), plane_origin.y(), plane_origin.z()
                );

            auto derivs_global = RigidBodyDOCADerivatives_global(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                st.MinuitFitParams.A1,
                st.MinuitFitParams.B1,
                straw_mp.x(), straw_mp.y(), straw_mp.z(), // TODO: is this suitable?
                wire_dir.x(), wire_dir.y(), wire_dir.z(),
                plane_origin.x(), plane_origin.y(), plane_origin.z()
                );


            // FIXME! use _srep utilities to do this properly!
            double drift_prediction = straw_hit.driftTime() * 0.065; //_srep.driftTimeToDistance(straw_hit.strawId(), straw_hit.driftTime(), straw_hit.);

            // Distance of Closest Approach (DOCA)
            TwoLinePCA_XYZ PCA = TwoLinePCA_XYZ(
                    track_pos,
                    track_dir,
                    Geom::toXYZVec(straw_mp),
                    Geom::toXYZVec(wire_dir),
                    1.e-8);
            double residual = (PCA.LRambig() * PCA.dca()) - drift_prediction;
            double residual_error = 0; // use TrkStrawHit (?) ::radialErr()

            if (isnan(residual)) continue;

            // write the track hit to the track buffer
            millepede->mille(
                    derivs_local.size(),
                    derivs_local.data(),
                    derivs_global.size(),
                    derivs_global.data(),
                    dof_labels[plane_id].data(),
                    residual,
                    residual_error);

            // diagnostic information
            if (_diag > 0)
            {
                residuum->Fill(residual);
            }
        }

        // Write the track buffer to file
        millepede->end();
    }
}

}; // namespace mu2e


using mu2e::PlaneAlignment;
DEFINE_ART_MODULE(PlaneAlignment);

