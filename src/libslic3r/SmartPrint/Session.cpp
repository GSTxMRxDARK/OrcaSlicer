#include "Session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <type_traits>

namespace Slic3r { namespace SmartPrint {

namespace {

constexpr int kFastElements     = 12000;
constexpr int kDetailedElements = 35000;
constexpr int kSearchElements   = 5000;

Vec3 vec_from(const nlohmann::json& j, const Vec3& fallback = Vec3::Zero())
{
    if (!j.is_array() || j.size() != 3)
        return fallback;
    return Vec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

nlohmann::json vec_to(const Vec3& v) { return nlohmann::json::array({v.x(), v.y(), v.z()}); }

nlohmann::json floats_b64(const std::vector<float>& values) { return base64_encode(values.data(), values.size() * sizeof(float)); }

double seconds_since(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::string read_text(const std::string& path)
{
    std::ifstream     file(path);
    std::stringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

} // namespace

std::string base64_encode(const void* data, size_t size)
{
    static const char* table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const auto*        bytes = static_cast<const unsigned char*>(data);
    std::string        out;
    out.reserve((size + 2) / 3 * 4);
    for (size_t i = 0; i < size; i += 3) {
        const unsigned v = (unsigned(bytes[i]) << 16) | (i + 1 < size ? unsigned(bytes[i + 1]) << 8 : 0u) |
                           (i + 2 < size ? unsigned(bytes[i + 2]) : 0u);
        out.push_back(table[(v >> 18) & 63]);
        out.push_back(table[(v >> 12) & 63]);
        out.push_back(i + 1 < size ? table[(v >> 6) & 63] : '=');
        out.push_back(i + 2 < size ? table[v & 63] : '=');
    }
    return out;
}

struct Session::Problem
{
    std::string                                  analysis = "none";
    std::vector<Region>                          supports;
    std::vector<RegionMass>                      masses;
    std::vector<LoadCase>                        cases;
    std::vector<std::string>                     governing;
    std::vector<std::pair<std::string, double>>  targets;
    bool                                         has_power = false;
    PowerSystem                                  system;
    PowerReport                                  report;
    DroneLayout                                  layout;
    DroneStyle                                   style;
};

Session::Session()  = default;
Session::~Session() = default;

bool Session::load_resources(const std::string& resources_dir, const std::string& user_dir, std::string* error)
{
    m_user_dir = user_dir;
    const std::string dir = resources_dir.empty() || resources_dir.back() == '/' || resources_dir.back() == '\\' ? resources_dir : resources_dir + "/";
    if (!m_materials.load_file(dir + "materials.json", error) || !m_propulsion.load_file(dir + "propulsion.json", error) ||
        !m_use_cases.load_file(dir + "usecases.json", error))
        return false;
    if (!user_dir.empty()) {
        // Optional user additions; a missing file is fine.
        std::string ignored;
        const std::string user = user_dir + "/user_library.json";
        if (std::ifstream(user).good()) {
            m_propulsion.load_file(user, &ignored);
            m_materials.load_file(user, &ignored);
        }
    }
    return true;
}

void Session::set_mesh(std::vector<float> vertices, std::vector<int> indices, const std::string& name)
{
    m_vertices = std::move(vertices);
    m_indices  = std::move(indices);
    m_name     = name;
    m_models.clear();
}

void Session::set_context(const nlohmann::json& context) { m_context = context; }

nlohmann::json Session::mesh_json() const
{
    return {{"name", m_name},
            {"vertices", base64_encode(m_vertices.data(), m_vertices.size() * sizeof(float))},
            {"indices", base64_encode(m_indices.data(), m_indices.size() * sizeof(int))},
            {"num_vertices", m_vertices.size() / 3},
            {"num_faces", m_indices.size() / 3}};
}

nlohmann::json Session::handle(const nlohmann::json& req, const Progress& progress)
{
    const std::string cmd = req.value("cmd", "");
    nlohmann::json    out;
    try {
        if (cmd == "libraries")
            out = libraries();
        else if (cmd == "rank")
            out = rank(req);
        else if (cmd == "power")
            out = power(req);
        else if (cmd == "layout")
            out = layout(req);
        else if (cmd == "analyze")
            out = analyze(req, progress);
        else if (cmd == "optimize")
            out = optimize(req, progress);
        else if (cmd == "save_user_item")
            out = save_user_item(req);
        else if (cmd == "mesh")
            out = mesh_json();
        else
            out = {{"error", "unknown command " + cmd}};
    } catch (const std::exception& e) {
        out = {{"error", e.what()}};
    }
    out["cmd"] = cmd;
    if (req.contains("request_id"))
        out["request_id"] = req["request_id"];
    return out;
}

nlohmann::json Session::libraries() const
{
    nlohmann::json materials = nlohmann::json::array();
    for (const Material& m : m_materials.materials())
        materials.push_back(material_to_json(m));
    nlohmann::json motors = nlohmann::json::array(), props = nlohmann::json::array(), batteries = nlohmann::json::array();
    for (const Motor& m : m_propulsion.motors())
        motors.push_back(motor_to_json(m));
    for (const Propeller& p : m_propulsion.props())
        props.push_back(prop_to_json(p));
    for (const Battery& b : m_propulsion.batteries())
        batteries.push_back(battery_to_json(b));
    nlohmann::json patterns = nlohmann::json::array();
    for (const InfillPatternModel& p : infill_pattern_models())
        patterns.push_back({{"id", p.id}, {"label", p.label}, {"structural", p.structural}});
    return {{"materials", materials},
            {"motors", motors},
            {"props", props},
            {"batteries", batteries},
            {"use_cases", m_use_cases.json()},
            {"patterns", patterns},
            {"context", m_context},
            {"mesh_name", m_name}};
}

Environment Session::environment(const nlohmann::json& req) const
{
    Environment env = m_use_cases.environment(req.value("environment", "indoor"));
    if (req.contains("max_temp"))
        env.max_temp_c = req["max_temp"].get<double>();
    if (req.contains("uv"))
        env.uv = req["uv"].get<bool>();
    env.impact = req.value("impact", 1);
    return env;
}

PrinterCapabilities Session::printer(const nlohmann::json& req) const
{
    PrinterCapabilities caps;
    const nlohmann::json p = req.contains("printer") ? req["printer"] : m_context.value("printer", nlohmann::json::object());
    caps.name            = p.value("name", "");
    caps.hardened_nozzle = p.value("hardened_nozzle", false);
    caps.enclosure       = p.value("enclosure", false);
    caps.max_nozzle_temp = p.value("max_nozzle_temp", 300.0);
    caps.nozzle_diameter = p.value("nozzle_diameter", 0.4);
    return caps;
}

Material Session::material(const nlohmann::json& f) const
{
    // Same resolution as rank_materials: library family, density from the user's preset.
    OwnedFilament owned{f.value("preset", ""), f.value("type", "PLA"), f.value("vendor", ""), f.value("density", 0.0),
                        f.value("max_volumetric_speed", 0.0)};
    nlohmann::json use_case = {{"material_weights", nlohmann::json::object()}};
    std::vector<MaterialRanking> r = rank_materials(use_case, Environment{}, PrinterCapabilities{}, {owned}, m_materials);
    Material m = r.front().material;
    if (!r.front().matched)
        if (const Material* pla = m_materials.find_by_id("PLA"))
            m = *pla;
    return m;
}

nlohmann::json Session::rank(const nlohmann::json& req) const
{
    const nlohmann::json* use_case = m_use_cases.find(req.value("use_case", ""));
    if (use_case == nullptr)
        return {{"error", "unknown use case"}};
    std::vector<OwnedFilament> owned;
    for (const nlohmann::json& f : m_context.value("filaments", nlohmann::json::array()))
        owned.push_back({f.value("preset", ""), f.value("type", ""), f.value("vendor", ""), f.value("density", 0.0),
                         f.value("max_volumetric_speed", 0.0)});
    nlohmann::json list = nlohmann::json::array();
    for (const MaterialRanking& r : rank_materials(*use_case, environment(req), printer(req), owned, m_materials))
        list.push_back({{"preset", r.filament.preset},
                        {"type", r.filament.type},
                        {"vendor", r.filament.vendor},
                        {"density", r.filament.density},
                        {"max_volumetric_speed", r.filament.max_volumetric_speed},
                        {"matched", r.matched},
                        {"usable", r.usable},
                        {"score", r.score},
                        {"issues", r.issues},
                        {"strengths", r.strengths},
                        {"material", material_to_json(r.material)}});
    return {{"materials", list}};
}

PowerSystem Session::power_system(const nlohmann::json& j) const
{
    PowerSystem sys;
    auto pick = [](const nlohmann::json& value, auto find, auto from_json) {
        using T = std::decay_t<decltype(from_json(nlohmann::json::object()))>;
        if (value.is_string())
            if (const T* item = find(value.get<std::string>()))
                return *item;
        return value.is_object() ? from_json(value) : T{};
    };
    sys.motor   = pick(j.value("motor", nlohmann::json()), [&](const std::string& id) { return m_propulsion.find_motor(id); }, motor_from_json);
    sys.prop    = pick(j.value("prop", nlohmann::json()), [&](const std::string& id) { return m_propulsion.find_prop(id); }, prop_from_json);
    sys.battery = pick(j.value("battery", nlohmann::json()), [&](const std::string& id) { return m_propulsion.find_battery(id); }, battery_from_json);
    sys.motor_count       = j.value("motor_count", 4);
    sys.payload_g         = j.value("payload_g", 0.0);
    sys.measured_thrust_g = j.value("measured_thrust_g", 0.0);
    return sys;
}

nlohmann::json Session::power(const nlohmann::json& req) const
{
    const PowerSystem sys = power_system(req.value("system", nlohmann::json::object()));
    return {{"report", report_to_json(PropulsionModel::analyse(sys, req.value("frame_g", 0.0)))}};
}

nlohmann::json Session::layout(const nlohmann::json& req) const
{
    if (m_vertices.empty())
        return {{"error", "no mesh"}};
    const MeshView mv{m_vertices.data(), m_vertices.size() / 3, m_indices.data(), m_indices.size() / 3};
    const DroneLayout l = detect_drone_layout(mv, req.value("motor_count", 4), req.value("mount_radius", 12.0), vec_from(req.value("up", nlohmann::json()), Vec3::UnitZ()));
    nlohmann::json motors = nlohmann::json::array();
    for (const Vec3& p : l.motors)
        motors.push_back(vec_to(p));
    return {{"layout", {{"motors", motors}, {"spin", l.spin}, {"mount_radius", l.mount_radius}, {"center", vec_to(l.center)},
                        {"center_radius", l.center_radius}, {"up", vec_to(l.up)}}}};
}

StructuralModel* Session::model(int target, std::string& error)
{
    for (CachedModel& c : m_models)
        if (c.target == target)
            return c.model.get();
    if (m_vertices.empty()) {
        error = "No object to analyse.";
        return nullptr;
    }
    CachedModel c;
    c.target = target;
    c.model  = std::make_unique<StructuralModel>();
    MeshOptions options;
    options.target_elements = target;
    const MeshView mv{m_vertices.data(), m_vertices.size() / 3, m_indices.data(), m_indices.size() / 3};
    if (!c.model->build(mv, options, &error))
        return nullptr;
    m_models.push_back(std::move(c));
    return m_models.back().model.get();
}

bool Session::build_problem(const nlohmann::json& req, Problem& p, std::string& error) const
{
    const nlohmann::json* use_case = m_use_cases.find(req.value("use_case", ""));
    if (use_case == nullptr) {
        error = "Unknown use case.";
        return false;
    }
    p.analysis = use_case->value("analysis", "none");
    const nlohmann::json targets = use_case->value("safety_targets", nlohmann::json::object());
    for (const auto& [id, value] : targets.items())
        p.targets.emplace_back(id, value.get<double>());
    if (req.contains("targets"))
        for (auto& t : p.targets)
            t.second = req["targets"].value(t.first, t.second);
    // Keep Light, Balanced, Strong order whatever the JSON order.
    auto order = [](const std::string& id) { return id == "light" ? 0 : id == "balanced" ? 1 : 2; };
    std::sort(p.targets.begin(), p.targets.end(), [&](const auto& a, const auto& b) { return order(a.first) < order(b.first); });

    if (p.analysis == "drone") {
        const nlohmann::json d = req.value("drone", nlohmann::json::object());
        p.system               = power_system(d.value("system", nlohmann::json::object()));
        p.has_power            = true;
        const nlohmann::json l = d.value("layout", nlohmann::json::object());
        p.layout.up            = vec_from(l.value("up", nlohmann::json()), Vec3::UnitZ()).normalized();
        p.layout.mount_radius  = l.value("mount_radius", mount_radius_for(p.system.motor));
        for (const nlohmann::json& m : l.value("motors", nlohmann::json::array()))
            p.layout.motors.push_back(vec_from(m));
        p.layout.spin          = l.value("spin", std::vector<int>{});
        p.layout.center        = vec_from(l.value("center", nlohmann::json()));
        p.layout.center_radius = l.value("center_radius", 20.0);
        if (p.layout.motors.empty()) {
            const MeshView mv{m_vertices.data(), m_vertices.size() / 3, m_indices.data(), m_indices.size() / 3};
            p.layout = detect_drone_layout(mv, p.system.motor_count, p.layout.mount_radius, p.layout.up);
        }
        while (p.layout.spin.size() < p.layout.motors.size())
            p.layout.spin.push_back(p.layout.spin.size() % 2 == 0 ? 1 : -1);
        const std::string style_id = d.value("style", "freestyle");
        for (const nlohmann::json& s : use_case->value("styles", nlohmann::json::array()))
            if (s.value("id", "") == style_id) {
                p.style.id             = style_id;
                p.style.dynamic_factor = s.value("dynamic_factor", p.style.dynamic_factor);
                p.style.landing_g      = s.value("landing_g", p.style.landing_g);
                p.style.crash_g        = s.value("crash_g", p.style.crash_g);
            }
        p.supports  = {Region{p.layout.center, p.layout.center_radius}};
        p.masses    = drone_masses(p.layout, p.system);
        p.governing = {"punch"};
    } else if (p.analysis == "custom") {
        const nlohmann::json c = req.value("custom", nlohmann::json::object());
        for (const nlohmann::json& s : c.value("supports", nlohmann::json::array()))
            p.supports.push_back(Region{vec_from(s.value("center", nlohmann::json())), s.value("radius", 5.0)});
        LoadCase lc;
        lc.id   = "load";
        lc.name = "Your loads";
        for (const nlohmann::json& f : c.value("forces", nlohmann::json::array()))
            lc.forces.push_back({Region{vec_from(f.value("center", nlohmann::json())), f.value("radius", 5.0)}, vec_from(f.value("force", nlohmann::json()))});
        p.cases.push_back(lc);
        p.governing = {"load"};
        if (p.supports.empty()) {
            error = "Add at least one support: a place where the part is held or screwed down.";
            return false;
        }
        if (lc.forces.empty()) {
            error = "Add at least one force: where and how hard the part is pushed or pulled.";
            return false;
        }
    }
    return true;
}

void Session::apply_problem(StructuralModel& model, const Problem& p)
{
    // Supports reset the cached responses, so only touch them when they change.
    std::ostringstream key;
    key.precision(9);
    for (const Region& r : p.supports)
        key << r.center.x() << ',' << r.center.y() << ',' << r.center.z() << ',' << r.radius << ';';
    for (CachedModel& c : m_models)
        if (c.model.get() == &model && c.supports_key != key.str()) {
            model.set_supports(p.supports);
            c.supports_key = key.str();
        }
    model.set_attached_masses(p.masses);
}

nlohmann::json Session::analyze(const nlohmann::json& req, const Progress& progress)
{
    const auto start = std::chrono::steady_clock::now();
    Problem    p;
    std::string error;
    if (!build_problem(req, p, error))
        return {{"error", error}};
    if (p.analysis == "none")
        return {{"error", "This use case has no load analysis."}};
    StructuralModel* m = model(req.value("quality", "fast") == "detailed" ? kDetailedElements : kFastElements, error);
    if (m == nullptr)
        return {{"error", error}};
    const Material          mat       = material(req.value("filament", nlohmann::json::object()));
    const StructureSettings structure = structure_from_json(req.value("structure", nlohmann::json::object()), StructureSettings{});
    m->set_material(mat);
    m->set_structure(structure);
    apply_problem(*m, p);
    if (progress && !progress(0.05))
        return {{"cancelled", true}};

    nlohmann::json result;
    const double frame_g = m->estimated_mass_g();
    if (p.has_power) {
        p.report = PropulsionModel::analyse(p.system, frame_g);
        p.cases  = drone_load_cases(p.layout, p.system, p.report, p.style);
        result["power"] = report_to_json(p.report);
        nlohmann::json motors = nlohmann::json::array();
        for (const Vec3& v : p.layout.motors)
            motors.push_back(vec_to(v));
        result["layout"] = {{"motors", motors}, {"spin", p.layout.spin}, {"mount_radius", p.layout.mount_radius},
                            {"center", vec_to(p.layout.center)}, {"center_radius", p.layout.center_radius}, {"up", vec_to(p.layout.up)}};
    }

    nlohmann::json cases = nlohmann::json::array();
    int            done  = 0;
    for (const LoadCase& lc : p.cases) {
        const CaseResult r = m->solve(lc);
        nlohmann::json   c = {{"id", lc.id}, {"name", lc.name}, {"ok", r.ok}};
        if (!r.ok) {
            c["error"] = r.error;
        } else {
            c["min_safety"]       = r.min_safety_factor;
            c["weakest"]          = vec_to(r.weakest_point);
            c["max_displacement"] = r.max_displacement;
            c["max_displacement_at"] = vec_to(r.max_displacement_point);
            c["interlayer"]       = r.interlayer_governs;
            c["max_stress"]       = r.max_wall_stress;
            c["iterations"]       = r.iterations;
            c["vertex_safety"]    = floats_b64(m->sample_elements(r.element_safety, m_vertices.data(), m_vertices.size() / 3, true));
        }
        cases.push_back(c);
        if (progress && !progress(0.1 + 0.8 * double(++done) / double(p.cases.size())))
            return {{"cancelled", true}};
    }
    result["cases"] = cases;

    const Vec3 freq = m->natural_frequencies();
    result["frequencies"] = vec_to(freq);
    if (p.has_power && p.report.can_hover) {
        // Motors sweep from idle to full speed; a frame mode inside that range is excited at some throttle.
        const double full_hz  = p.report.full.rpm / 60.0;
        const double hover_hz = p.report.hover.rpm / 60.0;
        nlohmann::json vib     = nlohmann::json::array();
        const char*    axes[3] = {"x", "y", "z"};
        for (int a = 0; a < 3; ++a)
            if (freq[a] > 0 && freq[a] < full_hz)
                vib.push_back({{"axis", axes[a]}, {"frequency", freq[a]}, {"throttle", freq[a] / full_hz},
                               {"near_hover", std::abs(freq[a] - hover_hz) < 0.15 * hover_hz}});
        result["vibration"] = {{"hover_hz", hover_hz}, {"full_hz", full_hz}, {"modes_in_range", vib}};
    }
    const double volume = m->extruded_volume(structure);
    const double speed  = req.value("filament", nlohmann::json::object()).value("max_volumetric_speed", 0.0);
    result["mass_g"]        = frame_g;
    result["extruded_mm3"]  = volume;
    result["print_minutes"] = estimate_print_minutes(volume, speed);
    result["elements"]      = m->num_elements();
    result["element_size"]  = m->element_size();
    result["ignored_elements"] = m->num_ignored_elements();
    result["material"]      = material_to_json(mat);
    result["seconds"]       = seconds_since(start);
    if (progress)
        progress(1.0);
    return result;
}

nlohmann::json Session::optimize(const nlohmann::json& req, const Progress& progress)
{
    const auto  start = std::chrono::steady_clock::now();
    Problem     p;
    std::string error;
    if (!build_problem(req, p, error))
        return {{"error", error}};
    const nlohmann::json*   use_case  = m_use_cases.find(req.value("use_case", ""));
    StructuralModel*        m         = model(kSearchElements, error);
    if (m == nullptr)
        return {{"error", error}};
    const nlohmann::json    filament  = req.value("filament", nlohmann::json::object());
    const Material          mat       = material(filament);
    StructureSettings       base      = structure_from_json(req.value("structure", nlohmann::json::object()), StructureSettings{});
    if (!req.value("structure", nlohmann::json::object()).contains("pattern"))
        base.infill_pattern = use_case->value("pattern", base.infill_pattern);
    m->set_material(mat);
    m->set_structure(base);
    const double speed = filament.value("max_volumetric_speed", 0.0);

    std::vector<TierResult> tiers;
    nlohmann::json          result;
    if (p.analysis == "none") {
        tiers = rule_tiers(*use_case, base);
        for (TierResult& t : tiers)
            if (!use_case->value("tiers", nlohmann::json::object()).value(t.id, nlohmann::json::object()).contains("pattern"))
                t.settings.infill_pattern = base.infill_pattern;
    } else {
        // Search on the quick model, verify on the one the user sees, so the numbers match the map.
        StructuralModel* shown = model(req.value("quality", "fast") == "detailed" ? kDetailedElements : kFastElements, error);
        if (shown == nullptr)
            return {{"error", error}};
        shown->set_material(mat);
        apply_problem(*m, p);
        apply_problem(*shown, p);
        if (p.has_power) {
            p.report = PropulsionModel::analyse(p.system, m->estimated_mass_g());
            p.cases  = drone_load_cases(p.layout, p.system, p.report, p.style);
        }
        std::vector<LoadCase> governing;
        for (const LoadCase& lc : p.cases)
            if (std::find(p.governing.begin(), p.governing.end(), lc.id) != p.governing.end())
                governing.push_back(lc);
        tiers = optimize_tiers(*m, shown, mat, base, governing, p.targets, [&](double f) { return !progress || progress(0.85 * f); });
        if (tiers.empty())
            return {{"cancelled", true}};
        m = shown;
    }

    nlohmann::json list = nlohmann::json::array();
    for (TierResult& t : tiers) {
        t.mass_g        = m->estimated_mass_g(t.settings, mat);
        t.extruded_mm3  = m->extruded_volume(t.settings);
        t.print_minutes = estimate_print_minutes(t.extruded_mm3, speed);
        nlohmann::json j = tier_to_json(t);
        if (p.analysis != "none") {
            // How each option copes with every load case, for the "what to expect" text.
            m->set_structure(t.settings);
            nlohmann::json per_case = nlohmann::json::object();
            for (const LoadCase& lc : p.cases) {
                const CaseResult r = m->solve(lc);
                if (r.ok)
                    per_case[lc.id] = {{"safety", r.min_safety_factor}, {"displacement", r.max_displacement}, {"interlayer", r.interlayer_governs}};
            }
            j["cases"] = per_case;
            // Bending strength grows with the square of thickness: how much thicker the weak sections must be.
            if (t.safety > 0 && t.safety < t.target)
                j["thickness_factor_needed"] = std::sqrt(t.target / t.safety);
            if (p.has_power)
                j["thrust_to_weight"] = PropulsionModel::analyse(p.system, t.mass_g).thrust_to_weight;
        }
        list.push_back(j);
    }
    result["tiers"]    = list;
    result["analysis"] = p.analysis;
    result["material"] = material_to_json(mat);
    result["seconds"]  = seconds_since(start);
    if (progress)
        progress(1.0);
    return result;
}

nlohmann::json Session::save_user_item(const nlohmann::json& req)
{
    const std::string kind = req.value("kind", "");
    nlohmann::json    item = req.value("item", nlohmann::json::object());
    if ((kind != "motors" && kind != "props" && kind != "batteries") || item.value("id", "").empty())
        return {{"error", "invalid item"}};
    if (m_user_dir.empty())
        return {{"error", "no user folder"}};
    const std::string path = m_user_dir + "/user_library.json";
    nlohmann::json    root = nlohmann::json::parse(read_text(path), nullptr, false);
    if (root.is_discarded() || !root.is_object())
        root = nlohmann::json::object();
    nlohmann::json& list = root[kind];
    if (!list.is_array())
        list = nlohmann::json::array();
    bool replaced = false;
    for (nlohmann::json& existing : list)
        if (existing.value("id", "") == item["id"].get<std::string>()) {
            existing = item;
            replaced = true;
        }
    if (!replaced)
        list.push_back(item);
    std::ofstream file(path);
    if (!file)
        return {{"error", "cannot write " + path}};
    file << root.dump(2);
    m_propulsion.load_json({{kind, nlohmann::json::array({item})}});
    return {{"ok", true}, {"kind", kind}, {"item", item}};
}

}} // namespace Slic3r::SmartPrint
