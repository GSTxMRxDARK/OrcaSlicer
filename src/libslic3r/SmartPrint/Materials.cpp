#include "Materials.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>

namespace Slic3r { namespace SmartPrint {

std::string normalize_material_name(const std::string& name)
{
    std::string out;
    out.reserve(name.size());
    for (char c : name) {
        if (c == ' ' || c == '_')
            c = '-';
        out.push_back(char(std::toupper(static_cast<unsigned char>(c))));
    }
    // Trim dashes left over from leading/trailing spaces.
    while (!out.empty() && out.back() == '-')
        out.pop_back();
    const size_t first = out.find_first_not_of('-');
    return first == std::string::npos ? std::string() : out.substr(first);
}

static EnclosureNeed parse_enclosure(const std::string& value)
{
    if (value == "required")
        return EnclosureNeed::Required;
    if (value == "recommended")
        return EnclosureNeed::Recommended;
    return EnclosureNeed::None;
}

static const char* enclosure_name(EnclosureNeed need)
{
    switch (need) {
    case EnclosureNeed::Required: return "required";
    case EnclosureNeed::Recommended: return "recommended";
    default: return "none";
    }
}

bool MaterialLibrary::load_file(const std::string& path, std::string* error)
{
    std::ifstream file(path);
    if (!file) {
        if (error)
            *error = "Cannot open " + path;
        return false;
    }
    nlohmann::json root = nlohmann::json::parse(file, nullptr, false);
    if (root.is_discarded()) {
        if (error)
            *error = "Invalid JSON in " + path;
        return false;
    }
    return load_json(root, error);
}

bool MaterialLibrary::load_json(const nlohmann::json& root, std::string* error)
{
    try {
        std::vector<Material> loaded;
        for (const nlohmann::json& j : root.at("materials")) {
            Material m;
            m.id               = j.at("id").get<std::string>();
            m.name             = j.value("name", m.id);
            m.brand            = j.value("brand", "");
            m.summary          = j.value("summary", "");
            m.aliases          = j.value("aliases", std::vector<std::string>{});
            m.density          = j.value("density", m.density);
            m.modulus_xy       = j.value("modulus_xy", m.modulus_xy);
            m.modulus_z        = j.value("modulus_z", m.modulus_xy * 0.9);
            m.tensile_xy       = j.value("tensile_xy", m.tensile_xy);
            m.tensile_z        = j.value("tensile_z", m.tensile_xy * 0.6);
            m.elongation       = j.value("elongation", m.elongation);
            m.heat_deflection  = j.value("heat_deflection", m.heat_deflection);
            m.min_nozzle_temp  = j.value("min_nozzle_temp", m.min_nozzle_temp);
            m.impact           = j.value("impact", m.impact);
            m.fatigue          = j.value("fatigue", m.fatigue);
            m.creep_resistance = j.value("creep_resistance", m.creep_resistance);
            m.uv_resistant     = j.value("uv_resistant", false);
            m.moisture_sensitive = j.value("moisture_sensitive", false);
            m.abrasive         = j.value("abrasive", false);
            m.flexible         = j.value("flexible", false);
            m.support_only     = j.value("support_only", false);
            m.enclosure        = parse_enclosure(j.value("enclosure", "none"));
            loaded.push_back(std::move(m));
        }
        // A later entry with the same id (e.g. a user override file merged in) replaces the earlier one.
        for (Material& m : loaded) {
            auto it = std::find_if(m_materials.begin(), m_materials.end(), [&](const Material& e) { return e.id == m.id; });
            if (it != m_materials.end())
                *it = std::move(m);
            else
                m_materials.push_back(std::move(m));
        }
    } catch (const std::exception& e) {
        if (error)
            *error = e.what();
        return false;
    }
    return true;
}

const Material* MaterialLibrary::find_by_id(const std::string& id) const
{
    for (const Material& m : m_materials)
        if (m.id == id)
            return &m;
    return nullptr;
}

const Material* MaterialLibrary::find_for_filament_type(const std::string& filament_type) const
{
    const std::string key = normalize_material_name(filament_type);
    if (key.empty())
        return nullptr;

    // 1) Exact id or alias. 2) Longest id/alias the type starts with ("PETG-CF10" -> "PETG-CF").
    const Material* best     = nullptr;
    size_t          best_len = 0;
    auto consider = [&](const Material& m, const std::string& name) {
        const std::string n = normalize_material_name(name);
        if (n.empty())
            return false;
        if (n == key) {
            best = &m;
            return true;
        }
        // Only accept a prefix when it ends at a word boundary ("PA" must not match "PAHT").
        if (key.size() > n.size() && key.compare(0, n.size(), n) == 0) {
            const char next = key[n.size()];
            if ((next == '-' || std::isdigit(static_cast<unsigned char>(next)) != 0) && n.size() > best_len) {
                best     = &m;
                best_len = n.size();
            }
        }
        return false;
    };
    for (const Material& m : m_materials) {
        if (consider(m, m.id))
            return best;
        for (const std::string& alias : m.aliases)
            if (consider(m, alias))
                return best;
    }
    if (best == nullptr)
        return nullptr;

    // A fibre-filled type that only matched its base family ("ABS-CF15" -> "ABS"): prefer the filled family.
    const bool has_cf = key.find("CF") != std::string::npos;
    const bool has_gf = key.find("GF") != std::string::npos;
    if ((has_cf || has_gf) && best->id.find("CF") == std::string::npos && best->id.find("GF") == std::string::npos) {
        if (const Material* filled = find_by_id(best->id + (has_cf ? "-CF" : "-GF")))
            return filled;
        if (const Material* filled = find_by_id(best->id + "-CF"))
            return filled;
    }
    return best;
}

nlohmann::json material_to_json(const Material& m)
{
    return {{"id", m.id},
            {"name", m.name},
            {"brand", m.brand},
            {"summary", m.summary},
            {"density", m.density},
            {"modulus_xy", m.modulus_xy},
            {"modulus_z", m.modulus_z},
            {"tensile_xy", m.tensile_xy},
            {"tensile_z", m.tensile_z},
            {"elongation", m.elongation},
            {"heat_deflection", m.heat_deflection},
            {"min_nozzle_temp", m.min_nozzle_temp},
            {"impact", m.impact},
            {"fatigue", m.fatigue},
            {"creep_resistance", m.creep_resistance},
            {"uv_resistant", m.uv_resistant},
            {"moisture_sensitive", m.moisture_sensitive},
            {"abrasive", m.abrasive},
            {"flexible", m.flexible},
            {"support_only", m.support_only},
            {"enclosure", enclosure_name(m.enclosure)}};
}

}} // namespace Slic3r::SmartPrint
