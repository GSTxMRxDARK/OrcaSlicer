#pragma once

#include "Materials.hpp"
#include "PrintStructure.hpp"
#include "Propulsion.hpp"
#include "StructuralModel.hpp"

#include <nlohmann/json.hpp>

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r { namespace SmartPrint {

struct PrinterCapabilities
{
    std::string name;
    double      max_nozzle_temp = 300;
    bool        hardened_nozzle = false;
    bool        enclosure       = false;
    double      nozzle_diameter = 0.4;
};

// A filament preset the user owns.
struct OwnedFilament
{
    std::string preset;
    std::string type;                 // Orca filament_type
    std::string vendor;
    double      density              = 0; // g/cm3 from the preset, 0 = use the library value
    double      max_volumetric_speed = 0; // mm3/s from the preset, 0 = unknown
};

struct Environment
{
    double max_temp_c = 35;
    bool   uv         = false;
    int    impact     = 1; // 0 none, 1 occasional knocks, 2 crashes / drops
};

struct MaterialRanking
{
    OwnedFilament            filament;
    Material                 material;   // library family, density from the preset when known
    bool                     matched = false;
    bool                     usable  = false;
    double                   score   = 0;
    std::vector<std::string> issues;     // codes: too_hot, needs_hardened_nozzle, needs_enclosure, ...
    std::vector<std::string> strengths;  // codes: stiff_light, strong_light, tough, fatigue, heat, uv, easy
};

class UseCaseLibrary
{
public:
    bool                  load_file(const std::string& path, std::string* error = nullptr);
    const nlohmann::json& json() const { return m_root; }
    const nlohmann::json* find(const std::string& id) const;
    Environment           environment(const std::string& id) const;

private:
    nlohmann::json m_root;
};

// Ranks the owned filaments for a use case; usable ones first, best score first.
std::vector<MaterialRanking> rank_materials(const nlohmann::json&             use_case,
                                            const Environment&                env,
                                            const PrinterCapabilities&        printer,
                                            const std::vector<OwnedFilament>& owned,
                                            const MaterialLibrary&            library);

// One of the Light / Balanced / Strong options.
struct TierResult
{
    std::string       id;
    double            target = 0;      // safety factor asked for (0 for rule-based tiers)
    StructureSettings settings;
    double            mass_g       = 0;
    double            safety       = 0; // under the main load case (0 when not analysed)
    bool              reached      = true;
    double            extruded_mm3 = 0;
    double            print_minutes = 0;
};

// Returns false from the callback to cancel; the argument is progress 0..1.
using Progress = std::function<bool(double)>;

// Searches walls and infill density for the lightest settings meeting each safety target under the
// governing load cases. The search runs on `model` (coarse, quick); when `verify` is given (the model
// shown to the user), each pick is checked there and re-aimed, so the reported safety matches the map.
// Both models must have supports set.
std::vector<TierResult> optimize_tiers(StructuralModel&                              model,
                                       StructuralModel*                              verify,
                                       const Material&                               material,
                                       const StructureSettings&                      base,
                                       const std::vector<LoadCase>&                  governing,
                                       const std::vector<std::pair<std::string, double>>& targets,
                                       const Progress&                               progress);

// Tiers straight from the use case's rule table (for parts without a load analysis).
std::vector<TierResult> rule_tiers(const nlohmann::json& use_case, const StructureSettings& base);

double estimate_print_minutes(double extruded_mm3, double max_volumetric_speed);

// --- Drones -------------------------------------------------------------------------------------

struct DroneStyle
{
    std::string id             = "freestyle";
    double      dynamic_factor = 1.5; // full-throttle thrust multiplier for punch-outs and vibration
    double      landing_g      = 8;
    double      crash_g        = 10;
};

struct DroneLayout
{
    std::vector<Vec3> motors;         // motor mount centres
    std::vector<int>  spin;           // +1 / -1 propeller direction per motor
    double            mount_radius = 12;
    Vec3              center       = Vec3::Zero();
    double            center_radius = 20;
    Vec3              up           = Vec3::UnitZ();
};

// Finds motor mounts as the outermost points in `motor_count` directions around the frame centre.
DroneLayout detect_drone_layout(const MeshView& mesh, int motor_count, double mount_radius, const Vec3& up = Vec3::UnitZ());
double      mount_radius_for(const Motor& motor);

// Load cases: "punch" (full throttle with thrust and torque on every arm), "landing" (hard landing),
// "arm_hit" (the frame lands on one arm).
std::vector<LoadCase>   drone_load_cases(const DroneLayout& layout, const PowerSystem& system, const PowerReport& report, const DroneStyle& style);
std::vector<RegionMass> drone_masses(const DroneLayout& layout, const PowerSystem& system);

nlohmann::json structure_to_json(const StructureSettings& s);
StructureSettings structure_from_json(const nlohmann::json& j, const StructureSettings& base);
nlohmann::json tier_to_json(const TierResult& t);

}} // namespace Slic3r::SmartPrint
