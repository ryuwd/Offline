// Ryunosuke O'Neil, 2019
// Millepede-II Mille routine wrapper

#include "CosmicAlignment/inc/Mille.h"

#include <vector>
#include <memory>

typedef std::vector<std::pair<unsigned int, double>> GlobalDerivativeCollection;


/**
 * @brief A object that represents an 'alignable' object and its
 * free parameters
 *
 */
class AlignableObject
{
private:
    unsigned int object_id;
    unsigned int n_parameters;
public:
    AlignableObject(unsigned int n_params) : n_parameters(n_params) { };
    ~AlignableObject() { };

    /**
     * @brief Get the Millepede label 'l' unique id of the global free parameter.
     *
     * @param param_id
     * @return unsigned int
     */
    unsigned int get_id(unsigned int param_id) { return object_id * 10 + param_id; }
};


/**
 * @brief MilleWrapper is designed to make it easy to construct an alignment 'problem'
 * in an object oriented way. All of the handling of Mille routines is confined to
 * this class only.
 *
 * I'd like to keep separate mu2e types from the invokation of Millepede, for clarity and readability.
 *
 */
class MilleWrapper
{
private:
    /* data */
    std::unique_ptr<Mille> millepede;

public:
    MilleWrapper(std::string filename);

    ~MilleWrapper() { }

    void RegisterAlignableObject(const AlignableObject &);

    void RegisterTrackHit(AlignableObject const& element,
        GlobalDerivativeCollection const& global_derivatives,
        std::vector<double> const& labels);

    void Save();
};
