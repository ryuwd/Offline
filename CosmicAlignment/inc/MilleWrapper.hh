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
    int object_id;
    int n_parameters;

    std::vector<int> labels;
public:
    AlignableObject(int o_id, int n_params) : n_parameters(n_params), object_id(o_id)
    {
        int start_id = get_param_id(0);
        for (int label_id = start_id; label_id < start_id + n_params; label_id++)
            labels.push_back(label_id);
    }

    ~AlignableObject() { };

    /**
     * @brief Get the Millepede label 'l' unique id of the global free parameter.
     *
     * @param param_id
     * @return unsigned int
     */
    int get_param_id(int param_id) const { return object_id * 10 + param_id; }

    /**
     * @brief Get the Millepede global param labels
     * as a C array (input to mille())
     *
     * @return const int*
     */
    const int* get_param_labels() const
    {
        return labels.data();
    }

    /**
     * @brief Get the object ID.
     *
     * @return int
     */
    int get_id() const { return object_id; }


    bool operator<(AlignableObject const&other)
    {
        return object_id < other.object_id;
    }

    bool operator<(const int &other)
    {
        return object_id < other;
    }

};


/**
 * @brief MilleWrapper is designed to make it easy to construct an alignment 'problem'.
 * All of the handling of Mille routines is confined to this class only.
 *
 * I'd like to keep separate mu2e types from the invokation of Millepede, for clarity and readability.
 *
 */
class MilleWrapper
{
private:
    /* data */
    std::unique_ptr<Mille> millepede;

    std::vector<AlignableObject> objects;

public:
    MilleWrapper(std::string filename);

    ~MilleWrapper() { }

    void RegisterAlignableObject(int, int);

    AlignableObject const& GetAlignableObject(int);

    void StartRegisteringHits();

    void RegisterTrackHit(int object_id,
        std::vector<float> const& global_derivatives,
        std::vector<float> const& local_derivatives,
        float residual,
        float residual_error);

    void Save();
};
