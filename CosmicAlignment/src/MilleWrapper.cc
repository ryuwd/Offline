// Ryunosuke O'Neil, 2019
// Millepede-II Mille routine wrapper


#include "CosmicAlignment/inc/MilleWrapper.hh"
#include "CosmicAlignment/inc/AlignableObjects.hh"
#include <vector>
#include <algorithm>    // std::sort
#include <cassert>
#include <type_traits>

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
 * 6. Stop using this object.
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
void MilleWrapper::RegisterAlignableObject(mu2e::AlignableObject & obj)
{
    objects.emplace_back(obj);

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
 * @brief Register a track hit residual measurement on detector element ID 'object_id'
 *
 * @param object_id the ID of the object that has been 'hit' by the track e.g. a plane or panel
 * @param global_derivatives the array of partial derivatives with respect to each global alignment parameter
 * @param local_derivatives the array of partial derivatives with respect to each track parameter
 * @param residual the measured residual.
 * @param residual_error the error on the measured residual.
 */
void MilleWrapper::RegisterTrackHit(mu2e::AlignableObject const& element,
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
    assert(millepede && "Check that RegisterTrackHit() has not been called after Save().");
    // We cannot rely on O(n log n) binary search on an unsorted array.

    //AlignableObject const& element = GetAlignableObject(object_id);
    assert(global_derivatives.size() == element.get_param_labels().size && "check param label n corresponds to global derivative N");
    std::cout << element.get_param_labels().size() << global_derivatives.size() << std::endl;
    millepede->mille(
        local_derivatives.size(),
        local_derivatives.data(),
        global_derivatives.size(),
        global_derivatives.data(),
        element.get_param_labels().data(),
        residual,
        residual_error);
}


/**
 * @brief Dump the Mille buffer to file.
 *
 */
void MilleWrapper::Save()
{
    // the Mille documentation states that the Mille
    // object must go out of scope or be destroyed to properly save the file.
    millepede->~Mille();
}
