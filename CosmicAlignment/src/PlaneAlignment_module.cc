// Ryunosuke O'Neil, 2019
// Module calling upon Mille to set up bootstrap alignment

#include "art/Framework/Core/EDAnalyzer.h"
#include "art_root_io/TFileService.h"

#include "GeometryService/inc/GeomHandle.hh"
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
    const size_t _dof_per_plane = 6; // dx, dy, dz, a, b, g (translation, rotation)
    const size_t _ndof = StrawId::_nplanes * _dof_per_plane;

    struct Config
    {
        using Name = fhicl::Name;
        using Comment = fhicl::Comment;
        fhicl::Atom<int> diaglvl { Name("diagLevel"), Comment("diagnostic level")};
        fhicl::Atom<art::InputTag> costag{Name("CosmicTrackSeedCollection"), Comment("tag for cosmic track seed collection")};
        fhicl::Atom<std::string> millefile{Name("MillepedeOutputFile"), Comment("Output filename for Millepede data file")};
        fhicl::Atom<int> use_proditions { Name("UseProditions"), Comment("Set to 1 to use Proditions AlignedTracker, 0 to use nominal tracker (set 0 if performing MC alignment validation)")};

    };
    typedef art::EDAnalyzer::Table<Config> Parameters;

    void beginJob();
    void endJob();
    void beginRun(art::Run const&);

    void analyze(art::Event const &);

    explicit PlaneAlignment(const Parameters &conf) : art::EDAnalyzer(conf),
                                            _diag(conf().diaglvl()),
                                            _use_proditions(conf().use_proditions()),
                                            _costag(conf().costag()),
                                            _output_filename(conf().millefile())
    {
        // generate hashtable of plane number to DOF labels for planes

        int counter = 0;
        for (uint16_t i = 0; i < StrawId::_nplanes; i++)
        {
            std::vector<int> labels;
            for (size_t dof_n = 0; dof_n < _dof_per_plane; dof_n++)
                labels.push_back(counter++);
            dof_labels[i] = std::move(labels);
        }

        if (_diag > 0)
        {
            std::cout << "PlaneAlignment: Total number of plane degrees of freedom = " << _ndof << std::endl;
            std::cout << "PlaneAlignment: Plane d.o.f. labels occupy range [0," << counter - 1 << "] inclusive" << std::endl;
        }
    }

    virtual ~PlaneAlignment() { }

    Config _conf;

    int _diag;
    int _use_proditions;

    art::InputTag _costag;
    const CosmicTrackSeedCollection *_coscol;

    std::string _output_filename;
    std::unique_ptr<Mille> millepede;

    // We need both Trackers. Why?
    // 1. Misalignments are simulated by modifications to the Proditions Tracker geometry.
    // 2. However, in alignment validation and alignment generally we should not presume to know
    //     what that misaligned geometry is.
    // In actual running, we use Proditions likely with some seed survey measurements
    // In alignment validation, we use Proditions to apply misalignments to the Tracker used in track reco.
    // Since in validation we are trying to determine the alignment constants that we misaligned to start with,
    // our input to Millepede should use nominal Tracker Straw and Plane position information, not Proditions.
    // Millepede then calculates the global alignment constant corrections, which hopefully resemble those
    // we applied in Proditions to misalign the Tracker.

    // Additional consideration: If we can use Millepede to provide starting alignment, then perhaps Proditions shouldn't
    // be used at all - however, Proditions seems like the most intuitive interface for people to work with.

    ProditionsHandle<Tracker> _alignedTracker_h;
    GeomHandle<Tracker> _nominalTracker_h;

    std::unordered_map<uint16_t, std::vector<int>> dof_labels;
    ProditionsHandle<StrawResponse> srep_h;
};

void PlaneAlignment::beginJob()
{
    millepede = std::make_unique<Mille>(_output_filename.c_str());

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
    StrawResponse const& _srep = srep_h.get(event.id());


    const Tracker * t_ptr = _nominalTracker_h.get();
    if (_use_proditions)
        t_ptr = _alignedTracker_h.getPtr(event.id()).get();

    Tracker const& tracker = *t_ptr;

    auto stH = event.getValidHandle<CosmicTrackSeedCollection>(_costag);
	_coscol = stH.product();

    for (CosmicTrackSeed const& sts : *_coscol)
    {
        CosmicTrack const& st = sts._track;
        TrkFitFlag const& status = sts._status;

        if (!status.hasAllProperties(TrkFitFlag::helixOK) ) {continue;}
        if (!st.converged or !st.minuit_converged) { continue;}

        if (isnan(st.MinuitFitParams.A0)) continue;

        XYZVec track_pos(st.MinuitFitParams.A0, st.MinuitFitParams.B0, 0);
        XYZVec track_dir(st.MinuitFitParams.A1, st.MinuitFitParams.B1, 1);

        // get residuals and their derivatives with respect
        // to all local and global parameters
        // get also plane id hit by straw hits
        for (ComboHit const&straw_hit : sts._straw_chits)
        {
            // straw and plane info
            StrawId const& straw_id = straw_hit.strawId();
            Straw const& straw = tracker.getStraw(straw_id);
            auto plane_id = straw_id.plane();

            // geometry info
            auto const &plane_origin = tracker.getPlane(plane_id).origin();
            auto const &straw_mp = straw.getMidPoint();
            auto const &wire_dir = straw.getDirection().unit();

            // now calculate the derivatives.
            auto derivativesLocal = RigidBodyDOCADerivatives_local(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                st.MinuitFitParams.A1,
                st.MinuitFitParams.B1,
                straw_mp.x(), straw_mp.y(), straw_mp.z(),
                wire_dir.x(), wire_dir.y(), wire_dir.z(),

                // warning: this is not changed in AlignedTrackerMaker
                // this is a problem if our starting geometry is not simply the nominal geometry
                // e.g. if we have survey measurements we wish to take into account
                plane_origin.x(), plane_origin.y(), plane_origin.z()
             );

            auto derivativesGlobal = RigidBodyDOCADerivatives_global(
                st.MinuitFitParams.A0,
                st.MinuitFitParams.B0,
                st.MinuitFitParams.A1,
                st.MinuitFitParams.B1,
                straw_mp.x(), straw_mp.y(), straw_mp.z(),
                wire_dir.x(), wire_dir.y(), wire_dir.z(),
                plane_origin.x(), plane_origin.y(), plane_origin.z()
            );

            Hep3Vector td(st.MinuitFitParams.A1, st.MinuitFitParams.B1, 1);
            td = td.unit();
            Hep3Vector rperp = td - (td.dot(straw.getDirection()) * straw.getDirection());

            // Distance of Closest Approach (DOCA)
            TwoLinePCA_XYZ PCA = TwoLinePCA_XYZ(
                    track_pos,
                    track_dir,
                    Geom::toXYZVec(straw_mp),
                    Geom::toXYZVec(wire_dir),
                    1.e-8);

            double phi = rperp.theta();
            double drift_distance = _srep.driftTimeToDistance(straw_id, straw_hit.driftTime(), phi); //straw_hit.driftTime() * 0.065;
            double residual = (PCA.LRambig() * PCA.dca()) - drift_distance;
            double residual_error = _srep.driftDistanceError(straw_id, drift_distance, phi, PCA.dca());

            if (isnan(residual)) continue;

            // write the hit to the track buffer
            millepede->mille(
                    derivativesLocal.size(),
                    derivativesLocal.data(),
                    derivativesGlobal.size(),
                    derivativesGlobal.data(),
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

