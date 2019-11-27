// Ryunosuke O'Neil, 2019
// Millepede-II Mille routine wrapper


#include "CosmicAlignment/inc/MilleWrapper.hh"

/**
 * @brief Construct a new Mille Wrapper object.
 *
 * @param output_file the filename for the millepede binary configuration file.
 */
MilleWrapper::MilleWrapper(std::string output_file)
{
    millepede = std::make_unique<Mille>(output_file.c_str());
}

/**
 * @brief This method facilitates, in Millepede II terms,
 * the definition of the 'global parameters' i.e. free parameters
 * of the alignment problem. i.e. the translation vector and rotation
 * angles of a plane object.
 *
 *
 */
void MilleWrapper::RegisterAlignableObject(AlignableObject const& object)
{
    //millepede->mille()
}

/**
 * @brief Add a new track to the Mille buffer.
 *
 */
void MilleWrapper::RegisterTrackHit(AlignableObject const& element,
    GlobalDerivativeCollection const& global_derivatives,
    std::vector<double> const& labels)
{
    if (!millepede) return;
}

void MilleWrapper::Save()
{
    millepede->end();
    millepede.release();
}
