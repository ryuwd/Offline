/**
 * @author Ryunosuke O'Neil
 * @email roneil@fnal.gov or ryunoneil@gmail.com
 * @create date 2019-12-01 19:27:27
 * @modify date 2019-12-01 19:27:27
 * @desc Alignable Object hierarchy
 */
#ifndef ALIGNABLEOBJECTS__H
#define ALIGNABLEOBJECTS__H
#include <vector>
#include "TrackerGeom/inc/Plane.hh"
#include "TrackerGeom/inc/Panel.hh"

namespace mu2e
{

/**
 * @brief A object that represents an 'alignable' object and its
 * free parameters
 * This class is necessary as Millepede requires unique parameter 'labels'
 * to uniquely identify each free parameter considered in the alignment.
 *
 * An object: belongs to a class of N objects
 * The ID is guaranteed to lie in the range: (clsID) <-> (clsID * N)
 * A parameter label is: object_unique_id * 10 + parameter_idx;
 * which supports up to 10 free parameters per object.
 */
class AlignableObject
{
private:
    int object_class;
    int object_class_size;
    int object_id;
    int n_parameters;
    int unique_id;

    std::vector<int> labels;

protected:
    /**
     * @brief Construct a new Alignable Object
     * Cannot construct a standalone AlignableObject - must create a Derived class description
     *
     * @param o_cla object class ID i.e. 0 for planes, 1 for panels, 2 for straws, etc...
     * @param o_cla_size number of objects in this class
     * @param o_id class specific object ID
     * @param n_params number of free parameters (currently supports up to 10)
     */
    AlignableObject(const int o_cla, const int o_cla_size, const int o_id, const int n_params) : object_class(o_cla), object_class_size(o_cla_size), object_id(o_id), n_parameters(n_params)
    {
        unique_id = AlignableObject::calc_unique_id(o_cla, o_cla_size, o_id);

        // set up the vector of unique free parameter labels for this object.
        int start_id = get_param_id(0);
        for (int label_id = start_id; label_id < start_id + n_params; label_id++)
            labels.push_back(label_id);
    }

public:
    AlignableObject(AlignableObject const &other) : object_class(other.object_class), object_class_size(other.object_class_size),
                                                    object_id(other.object_id), n_parameters(other.n_parameters), unique_id(other.unique_id), labels(other.labels) {}

    static const int calc_unique_id(const int object_class, const int object_class_size, const int object_id)
    {
        return (object_class)*object_class_size + object_id;
    }

    virtual ~AlignableObject(){};

    /**
     * @brief Get the Millepede label 'l' unique id of the global free parameter.
     *
     * @param param_id
     * @return unsigned int
     */
    int get_param_id(int param_id) const { return unique_id * 10 + param_id; }

    /**
     * @brief Get the Millepede global param labels
     * as a C array (input to mille())
     *
     * @return std::vector<int> const&
     */
    std::vector<int> const &get_param_labels() const
    {
        if (labels.size() == 0)
            std::cout << "Warning: label vector empty..." << std::endl;

        return labels;
    }

    /**
     * @brief Get the class-specific (e.g. Plane, Panel) object ID.
     *
     * @return int
     */
    int get_object_id() const { return object_id; }

    /**
     * @brief Get the unique id
     *
     * @return int
     */
    int get_unique_id() const { return unique_id; }

    bool operator<(AlignableObject const &other)
    {
        return unique_id < other.unique_id;
    }

    bool operator<(int const &other)
    {
        return unique_id < other;
    }

    friend bool operator<(int const &, AlignableObject const &);
};

bool operator<(int const &lhs, AlignableObject const &rhs)
{
    return lhs < rhs.unique_id;
}

/**
 * @brief Aligning planes as rigid bodies... (6 free parameters)
 * Class ID: 0
 * Free Parameters: 6
 *
 */
class AlignablePlane : public AlignableObject
{
public:
    static const int class_id = 0;
    static const int class_size = mu2e::StrawId::_nplanes;
    static const int free_parameters = 6;

    AlignablePlane(mu2e::Plane const &p) : AlignableObject(class_id, class_size, (const int)p.id().plane(), free_parameters)
    {
        // nothing complicated required
    }
    AlignablePlane(AlignableObject const &other) : AlignableObject(other) {}
};

/**
 * @brief Aligning panels as rigid bodies.
 *  Class ID: 1
 *  Free Parameters: 6
 */
class AlignablePanel : public AlignableObject
{
public:
    static const int class_id = 1;
    static const int class_size = mu2e::StrawId::_nplanes * mu2e::StrawId::_npanels;
    static const int free_parameters = 6;

    AlignablePanel(mu2e::Panel const &p) : AlignableObject(class_id, class_size, (const int)p.id().asUint16(), free_parameters)
    {
        // nothing complicated required
    }
};

} // namespace mu2e
#endif