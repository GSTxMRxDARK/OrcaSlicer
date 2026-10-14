#pragma once

#include "Advisor.hpp"
#include "Materials.hpp"
#include "Propulsion.hpp"
#include "StructuralModel.hpp"

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace Slic3r { namespace SmartPrint {

// Everything the Smart Print assistant does, behind a JSON request/response protocol, so the GUI
// only has to collect context from Orca and relay messages to the web page.
//
// Requests ({"cmd": ...}):
//   "libraries"  -> materials, motors, props, batteries, use cases, infill patterns
//   "rank"       -> the user's filaments ranked for a use case and environment
//   "power"      -> thrust / weight / flight-time report of a drone power system
//   "layout"     -> automatically detected motor mounts and centre support
//   "analyze"    -> stresses, safety factors (per mesh vertex), deflection and vibration per load case
//   "optimize"   -> Light / Balanced / Strong settings with mass, print time and safety per load case
//   "save_user_item" -> stores a user's own motor, propeller or battery
//
// Not thread-safe: the caller runs one request at a time (on a worker thread is fine).
class Session
{
public:
    Session();
    ~Session();

    // resources_dir holds materials.json, propulsion.json, usecases.json; user_dir the user's additions.
    bool load_resources(const std::string& resources_dir, const std::string& user_dir, std::string* error = nullptr);

    // Mesh in printer coordinates (mm, Z up = build direction).
    void set_mesh(std::vector<float> vertices, std::vector<int> indices, const std::string& name);
    // {"printer": {...}, "filaments": [{"preset","type","vendor","density","max_volumetric_speed"}], "structure": {...}}
    void set_context(const nlohmann::json& context);

    nlohmann::json handle(const nlohmann::json& request, const Progress& progress = {});

    // The mesh as base64 arrays, for the 3D view.
    nlohmann::json mesh_json() const;

private:
    nlohmann::json libraries() const;
    nlohmann::json rank(const nlohmann::json& req) const;
    nlohmann::json power(const nlohmann::json& req) const;
    nlohmann::json layout(const nlohmann::json& req) const;
    nlohmann::json analyze(const nlohmann::json& req, const Progress& progress);
    nlohmann::json optimize(const nlohmann::json& req, const Progress& progress);
    nlohmann::json save_user_item(const nlohmann::json& req);

    struct Problem; // supports, masses and load cases of a request
    bool           build_problem(const nlohmann::json& req, Problem& out, std::string& error) const;
    StructuralModel* model(int target_elements, std::string& error);
    void           apply_problem(StructuralModel& model, const Problem& problem);

    Environment         environment(const nlohmann::json& req) const;
    PrinterCapabilities printer(const nlohmann::json& req) const;
    Material            material(const nlohmann::json& filament) const;
    PowerSystem         power_system(const nlohmann::json& j) const;

    MaterialLibrary   m_materials;
    PropulsionLibrary m_propulsion;
    UseCaseLibrary    m_use_cases;
    std::string       m_user_dir;

    std::vector<float> m_vertices;
    std::vector<int>   m_indices;
    std::string        m_name;
    nlohmann::json     m_context = nlohmann::json::object();

    struct CachedModel
    {
        int                              target = 0;
        std::unique_ptr<StructuralModel> model;
        std::string                      supports_key;
    };
    std::vector<CachedModel> m_models;
};

std::string base64_encode(const void* data, size_t size);

}} // namespace Slic3r::SmartPrint
