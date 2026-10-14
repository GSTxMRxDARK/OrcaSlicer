#pragma once

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <vector>

namespace Slic3r { namespace SmartPrint {

enum class EnclosureNeed { None, Recommended, Required };

// Mechanical properties of a printed material family, as an FDM part (not injection-molded data).
// "xy" is along the printed layers, "z" across them (layer adhesion).
struct Material
{
    std::string              id;
    std::string              name;
    std::string              brand;
    std::string              summary;
    std::vector<std::string> aliases;

    double density          = 1.24; // g/cm3
    double modulus_xy       = 2600; // MPa
    double modulus_z        = 2300; // MPa
    double tensile_xy       = 45;   // MPa
    double tensile_z        = 28;   // MPa
    double elongation       = 4;    // %
    double heat_deflection  = 55;   // degC
    double min_nozzle_temp  = 200;  // degC
    int    impact           = 2;    // 1..5
    int    fatigue          = 2;    // 1..5
    int    creep_resistance = 2;    // 1..5

    bool          uv_resistant       = false;
    bool          moisture_sensitive = false;
    bool          abrasive           = false;
    bool          flexible           = false;
    bool          support_only       = false;
    EnclosureNeed enclosure          = EnclosureNeed::None;
};

class MaterialLibrary
{
public:
    bool load_file(const std::string& path, std::string* error = nullptr);
    bool load_json(const nlohmann::json& root, std::string* error = nullptr);

    const std::vector<Material>& materials() const { return m_materials; }
    const Material*              find_by_id(const std::string& id) const;
    // Maps an Orca filament_type ("PA6-CF", "PETG Basic", "PLA Silk") to the closest family.
    // Returns nullptr when nothing matches.
    const Material* find_for_filament_type(const std::string& filament_type) const;

private:
    std::vector<Material> m_materials;
};

std::string    normalize_material_name(const std::string& name);
nlohmann::json material_to_json(const Material& material);

}} // namespace Slic3r::SmartPrint
