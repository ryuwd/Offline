// Ryunosuke O'Neil, 2019
// Millepede-II Mille routine wrapper

#ifndef F1708379_AFCE_4CEC_A9F1_86381EDEE222
#define F1708379_AFCE_4CEC_A9F1_86381EDEE222

#include "CosmicAlignment/inc/Mille.h"
#include "CosmicAlignment/inc/AlignableObjects.hh"

#include <vector>
#include <memory>
#include <algorithm>


/**
 * @brief MilleWrapper is designed to make it easy to construct an alignment 'problem' with Millepede.
 * All of the handling of Mille routines is confined to this class only.
 */
class MilleWrapper
{
private:
    std::unique_ptr<Mille> millepede;
    std::vector<mu2e::AlignableObject> objects;

    bool have_sorted = false;

public:
    MilleWrapper(std::string filename);

    ~MilleWrapper() { }

    void RegisterAlignableObject(mu2e::AlignableObject &);

    /**
     * @brief Get the AlignableObject by its class specific ID
     * e.g. GetAlignableObject<AlignablePlane>((mu2e::Plane)plane.id())
     *
     * @tparam T AlignableObject type e.g. AlignablePlane, AlignablePanel.
     *  If not derived from AlignableObject the compiler will complain.
     * @param object_id the class-specific ID of the object to align
     * @return AlignableObject const& fetched object.
     */
    template <class T>
    mu2e::AlignableObject const& GetAlignableObject(int object_id);

    void StartRegisteringHits();

    void RegisterTrackHit(mu2e::AlignableObject const& element,
        std::vector<float> const& global_derivatives,
        std::vector<float> const& local_derivatives,
        float residual,
        float residual_error);

    void Save();
};


template <class T>
mu2e::AlignableObject const& MilleWrapper::GetAlignableObject(int object_id)
{
    static_assert(std::is_convertible<T, mu2e::AlignableObject>::value,
        "T in GetAlignableObject must be an alignable object");
    assert(have_sorted && "Check that StartRegisteringHits() has been called.");

    // O(n log n) array search
    return objects[std::equal_range(objects.begin(), objects.end(),
        mu2e::AlignableObject::calc_unique_id(T::class_id, T::class_size, object_id)).first - objects.begin()];
}
#endif /* F1708379_AFCE_4CEC_A9F1_86381EDEE222 */
