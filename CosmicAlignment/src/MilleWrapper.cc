// Ryunosuke O'Neil, 2019
// Millepede-II Mille routine wrapper


#include "CosmicAlignment/inc/MilleWrapper.hh"
#include <vector>
#include <algorithm>    // std::sort
#include <assert.h>

/**
 * @brief Construct a new Mille Wrapper object.
 *
 * How to use:
 *
 * 1. Create a new MilleWrapper and choose a filename unique to this specific alignment procedure.
 * 2. Register all the objects you want to align and their free parameters. Each object must be assigned an ID (can be anything as long as its unique)
 * 3. Call StartRegisteringHits();
 * 4. For each hit in a track, find the ID of the object that you want to align which was hit,
 *    the derivatives with respect to each track parameter and global parameter (of that object),
 *    the residual, and its error. Pass this information to RegisterTrackHit( ... );
 * 5. Call Save(); - This will write a binary file which can be passed to ./pede
 *
 * @param output_file the filename for the mille binary file.
 */
MilleWrapper::MilleWrapper(std::string output_file)
{
    millepede = std::make_unique<Mille>(output_file.c_str());
}

/**
 * @brief Register an object (with a unique ID) to be aligned using N free parameters.
 *
 * @param object_id the unique ID of the object
 * @param no_free_parameters the number of free parameters that Millepede will use to align the object
 */
void MilleWrapper::RegisterAlignableObject(int object_id, int no_free_parameters)
{
    objects.emplace_back(object_id, no_free_parameters);

    if (have_sorted) have_sorted = false;
}

/**
 * @brief Call me after registering all objects with RegisterAlignableObject,
 * and ALWAYS before calling RegisterTrackHit
 *
 */
void MilleWrapper::StartRegisteringHits()
{
    std::sort(objects.begin(), objects.end());
    have_sorted = true;
}

/**
 * @brief Get AlignableObject instance in this object with ID object_id
 *
 * @param object_id the ID of the AlignableObject to fetch.
 * @return AlignableObject const&
 */
AlignableObject const& MilleWrapper::GetAlignableObject(int object_id)
{
    // O(n log n) array search
    return *std::equal_range(objects.begin(), objects.end(), object_id).first;
}

/**
 * @brief Register a track hit residual measurement on detector element ID 'object_id'
 *
 * @param object_id the ID of the object that has been 'hit' by the track e.g. a plane or panel
 * @param global_derivatives the array of partial derivatives with respect to each global alignment parameter
 * @param local_derivatives the array of partial derivatives with respect to each track parameter
 * @param residual the measured residual.
 * @param residual_error the error on the measured residual.
 */
void MilleWrapper::RegisterTrackHit(int object_id,
    std::vector<float> const& global_derivatives,
    std::vector<float> const& local_derivatives,
    float residual,
    float residual_error)
{
    //if (!millepede) return;

    //if (!have_sorted) return;

    // use assert since in release/production
    // we will be calling this f'n for hundreds of thousands
    // of hits
    assert(millepede);
    // We cannot rely on O(n log n) binary search on an unsorted array.
    assert(have_sorted);

    AlignableObject const& element = GetAlignableObject(object_id);

    millepede->mille(
        local_derivatives.size(),
        local_derivatives.data(),
        global_derivatives.size(),
        global_derivatives.data(),
        element.get_param_labels(),
        residual,
        residual_error);
}


/**
 * @brief Dump the Mille buffer to file.
 *
 */
void MilleWrapper::Save()
{
    millepede->end();

    // the Mille documentation states that the Mille
    // object must go out of scope or be destroyed to properly save the file.
    millepede.release();
}
