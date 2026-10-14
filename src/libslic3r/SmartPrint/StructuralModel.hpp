#pragma once

#include "Materials.hpp"
#include "PrintStructure.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r { namespace SmartPrint {

using Vec3 = Eigen::Vector3d;

// Non-owning view of a triangle mesh in millimetres, Z = build direction.
// Orca's indexed_triangle_set maps onto it without copying (see SmartPrintOrca.hpp).
struct MeshView
{
    const float* vertices     = nullptr; // xyz triplets
    size_t       num_vertices = 0;
    const int*   indices      = nullptr; // vertex index triplets
    size_t       num_faces    = 0;
};

// All mesh nodes within `radius` of `center`; when none are, the nearest few nodes.
struct Region
{
    Vec3   center = Vec3::Zero();
    double radius = 1.0; // mm
};

struct RegionForce
{
    Region region;
    Vec3   force = Vec3::Zero(); // N, total over the region
};

// A torque about `axis` through the region centre, applied as a ring of tangential forces.
struct RegionTorque
{
    Region region;
    Vec3   axis   = Vec3::UnitZ();
    double torque = 0.0; // N*mm
};

// A mass carried by the part (motor, battery, camera). Used for acceleration loads and vibration.
struct RegionMass
{
    Region region;
    double mass_g = 0.0;
};

struct LoadCase
{
    std::string               id;
    std::string               name;
    std::vector<RegionForce>  forces;
    std::vector<RegionTorque> torques;
    // Acceleration in g applied to the part's own mass and the attached masses (crash, hard landing).
    Vec3 acceleration_g = Vec3::Zero();
};

struct MeshOptions
{
    int    target_elements    = 30000; // used when element_size is 0
    double element_size       = 0.0;   // mm, 0 = automatic
    int    max_cells_per_axis = 320;
    double min_fill           = 0.08;  // voxels less filled than this are dropped
    double poisson            = 0.35;
};

struct CaseResult
{
    std::string id;
    bool        ok = false;
    std::string error;

    double max_displacement = 0.0; // mm
    Vec3   max_displacement_point = Vec3::Zero();
    double min_safety_factor      = 0.0;
    Vec3   weakest_point          = Vec3::Zero();
    bool   interlayer_governs     = false; // the weakest spot fails by layers separating
    double max_wall_stress        = 0.0;   // MPa, von Mises in the solid plastic
    int    iterations             = 0;     // solver iterations spent; 0 = answered from cached responses

    std::vector<float> element_safety;    // per element, capped at 100
    std::vector<float> element_stress;    // per element, MPa in the solid plastic
    std::vector<float> node_displacement; // per node, mm
};

// Linear-elastic finite-element model of a printed part on a voxel grid.
//
// The part is voxelized once. Each voxel's stiffness and strength come from how much of it is
// walls / top-bottom shells (solid plastic) versus sparse infill, using the actual slicer settings.
// Elements are 8-node bricks with incompatible modes, which bend accurately even with only two
// elements through a thin wall.
//
// Speed: the solver is conjugate gradients with a geometric multigrid preconditioner, so its cost grows
// linearly with the model. Each solve is the response to one unit load (a unit force or torque at one
// place, or the part's own weight) and is cached; a load case is a weighted sum of them. Changing the
// motors, propellers, battery, force magnitudes or the material therefore needs no new solve at all.
// Changing walls or infill re-solves the cached loads, starting from the previous answers.
class StructuralModel
{
public:
    StructuralModel();
    ~StructuralModel();
    StructuralModel(const StructuralModel&)            = delete;
    StructuralModel& operator=(const StructuralModel&) = delete;

    bool build(const MeshView& mesh, const MeshOptions& options, std::string* error = nullptr);
    bool is_built() const;

    double             element_size() const;
    size_t             num_elements() const;
    size_t             num_nodes() const;
    Vec3               element_center(size_t element) const;
    Vec3               node_position(size_t node) const;
    double             part_volume() const; // mm3 of plastic envelope
    Eigen::AlignedBox3d bounding_box() const;

    // Boundary conditions. Changing them clears all cached load responses.
    // Returns the number of clamped nodes. Parts of the mesh not connected to a support are ignored.
    int  set_supports(const std::vector<Region>& fixed);
    void set_attached_masses(const std::vector<RegionMass>& masses);
    int  num_ignored_elements() const; // elements in pieces not connected to any support

    void            set_material(const Material& material);
    const Material& material() const;

    // Walls and infill. Clears the cached load responses (kept as starting guesses).
    void                     set_structure(const StructureSettings& structure);
    const StructureSettings& structure() const;
    // Builds the solver for the current structure (done automatically by the first solve).
    bool prepare_solver(std::string* error = nullptr);
    bool solver_ready() const;

    // Printed weight of the part alone, for the current or any other settings (no solve needed).
    double estimated_mass_g() const;
    double estimated_mass_g(const StructureSettings& structure, const Material& material) const;
    // Volume of plastic actually deposited, mm3.
    double extruded_volume(const StructureSettings& structure) const;

    CaseResult solve(const LoadCase& load_case);
    // Lowest natural frequency in Hz for vibration along x, y and z, with the attached masses, by
    // Rayleigh's method on the static deflection under the part's own weight. Within a few percent of the
    // exact first mode for arms and beams, and almost free once those responses are cached.
    Vec3 natural_frequencies();

    // Per-element values sampled at points (e.g. the original mesh vertices) for display.
    // take_min = true shows the weakest nearby element instead of an average.
    std::vector<float> sample_elements(const std::vector<float>& element_values,
                                       const float*              points,
                                       size_t                    num_points,
                                       bool                      take_min) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m;
};

}} // namespace Slic3r::SmartPrint
