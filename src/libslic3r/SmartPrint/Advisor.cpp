#include "Advisor.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <string>

namespace Slic3r { namespace SmartPrint {

namespace {

constexpr double kG  = 9.80665;
constexpr double kPi = 3.14159265358979323846;

double feature(const Material& m, const std::string& name)
{
    if (name == "stiffness_per_weight")
        return m.modulus_xy / m.density;
    if (name == "strength_per_weight")
        return m.tensile_xy / m.density;
    if (name == "layer_strength")
        return m.tensile_z;
    if (name == "impact")
        return m.impact;
    if (name == "fatigue")
        return m.fatigue;
    if (name == "creep")
        return m.creep_resistance;
    if (name == "heat")
        return m.heat_deflection;
    if (name == "light")
        return 1.0 / m.density;
    if (name == "ease") {
        double e = 1.0;
        if (m.enclosure == EnclosureNeed::Required)
            e -= 0.4;
        else if (m.enclosure == EnclosureNeed::Recommended)
            e -= 0.2;
        if (m.abrasive)
            e -= 0.15;
        if (m.moisture_sensitive)
            e -= 0.15;
        if (m.flexible)
            e -= 0.2;
        return std::max(e, 0.05);
    }
    return 0.0;
}

} // namespace

bool UseCaseLibrary::load_file(const std::string& path, std::string* error)
{
    std::ifstream file(path);
    if (!file) {
        if (error)
            *error = "Cannot open " + path;
        return false;
    }
    m_root = nlohmann::json::parse(file, nullptr, false);
    if (m_root.is_discarded() || !m_root.contains("use_cases")) {
        if (error)
            *error = "Invalid use case file " + path;
        m_root = nlohmann::json::object();
        return false;
    }
    return true;
}

const nlohmann::json* UseCaseLibrary::find(const std::string& id) const
{
    if (!m_root.contains("use_cases"))
        return nullptr;
    for (const nlohmann::json& uc : m_root["use_cases"])
        if (uc.value("id", "") == id)
            return &uc;
    return nullptr;
}

Environment UseCaseLibrary::environment(const std::string& id) const
{
    Environment env;
    if (m_root.contains("environments"))
        for (const nlohmann::json& e : m_root["environments"])
            if (e.value("id", "") == id) {
                env.max_temp_c = e.value("max_temp", env.max_temp_c);
                env.uv         = e.value("uv", false);
            }
    return env;
}

std::vector<MaterialRanking> rank_materials(const nlohmann::json&             use_case,
                                            const Environment&                env,
                                            const PrinterCapabilities&        printer,
                                            const std::vector<OwnedFilament>& owned,
                                            const MaterialLibrary&            library)
{
    std::vector<MaterialRanking> out;
    const bool needs_flexible = use_case.value("requires_flexible", false);
    for (const OwnedFilament& f : owned) {
        MaterialRanking r;
        r.filament = f;
        if (const Material* m = library.find_for_filament_type(f.type)) {
            r.material = *m;
            r.matched  = true;
        } else {
            r.material.id   = f.type;
            r.material.name = f.type;
            r.issues.push_back("unknown_material");
        }
        Material& m = r.material;
        if (f.density > 0.3 && f.density < 3.0 && m.id != "PLA-AERO" && m.id != "ASA-AERO")
            m.density = f.density;

        r.usable = r.matched;
        if (m.support_only) {
            r.usable = false;
            r.issues.push_back("support_material");
        }
        if (m.flexible != needs_flexible) {
            r.usable = false;
            r.issues.push_back(needs_flexible ? "not_flexible" : "too_flexible");
        }
        if (m.heat_deflection < env.max_temp_c + 5) {
            r.usable = false;
            r.issues.push_back("too_hot");
        }
        if (m.abrasive && !printer.hardened_nozzle) {
            r.usable = false;
            r.issues.push_back("needs_hardened_nozzle");
        }
        if (m.min_nozzle_temp > printer.max_nozzle_temp) {
            r.usable = false;
            r.issues.push_back("nozzle_too_cold");
        }
        if (m.enclosure == EnclosureNeed::Required && !printer.enclosure)
            r.issues.push_back("needs_enclosure");
        else if (m.enclosure == EnclosureNeed::Recommended && !printer.enclosure)
            r.issues.push_back("enclosure_recommended");
        if (env.uv && !m.uv_resistant)
            r.issues.push_back("uv_degrades");
        if (m.moisture_sensitive)
            r.issues.push_back("dry_before_printing");
        if (env.impact >= 2 && m.impact <= 1)
            r.issues.push_back("brittle");
        if (m.creep_resistance <= 2 && use_case.value("analysis", "") == "custom")
            r.issues.push_back("creeps_under_load");
        out.push_back(std::move(r));
    }

    // Score: weighted features, each normalized by the best value among the matched filaments.
    const nlohmann::json weights = use_case.value("material_weights", nlohmann::json::object());
    std::map<std::string, double> best;
    for (const auto& [name, w] : weights.items())
        for (const MaterialRanking& r : out)
            if (r.matched)
                best[name] = std::max(best[name], feature(r.material, name));
    for (MaterialRanking& r : out) {
        if (!r.matched)
            continue;
        double score = 0.0;
        for (const auto& [name, w] : weights.items())
            if (best[name] > 0)
                score += w.get<double>() * feature(r.material, name) / best[name];
        if (env.impact >= 2)
            score += 0.15 * r.material.impact / 5.0;
        for (const std::string& issue : r.issues) {
            if (issue == "needs_enclosure")
                score -= 0.3;
            else if (issue == "enclosure_recommended")
                score -= 0.08;
            else if (issue == "uv_degrades")
                score -= 0.1;
            else if (issue == "brittle")
                score -= 0.15;
            else if (issue == "creeps_under_load")
                score -= 0.05;
        }
        r.score = score;
        const Material& m = r.material;
        if (m.modulus_xy / m.density > 3000)
            r.strengths.push_back("stiff_light");
        if (m.tensile_xy / m.density > 50)
            r.strengths.push_back("strong_light");
        if (m.impact >= 4)
            r.strengths.push_back("tough");
        if (m.fatigue >= 4)
            r.strengths.push_back("fatigue");
        if (m.heat_deflection >= 90)
            r.strengths.push_back("heat");
        if (m.uv_resistant)
            r.strengths.push_back("uv");
        if (feature(m, "ease") >= 0.95)
            r.strengths.push_back("easy");
        if (m.density < 1.0)
            r.strengths.push_back("very_light");
    }
    std::stable_sort(out.begin(), out.end(), [](const MaterialRanking& a, const MaterialRanking& b) {
        if (a.usable != b.usable)
            return a.usable;
        return a.score > b.score;
    });
    return out;
}

double estimate_print_minutes(double extruded_mm3, double max_volumetric_speed)
{
    // Average flow is far below the hotend's maximum: walls run slower, travel and layer changes add time.
    const double flow = (max_volumetric_speed > 0 ? max_volumetric_speed : 12.0) * 0.45;
    return extruded_mm3 / flow / 60.0 + 3.0;
}

std::vector<TierResult> optimize_tiers(StructuralModel&                                   model,
                                       StructuralModel*                                   verify,
                                       const Material&                                    material,
                                       const StructureSettings&                           base,
                                       const std::vector<LoadCase>&                       governing,
                                       const std::vector<std::pair<std::string, double>>& targets,
                                       const Progress&                                    progress)
{
    static const int    kWalls[]   = {2, 3, 4, 5, 6};
    static const double kDensity[] = {0.10, 0.30, 1.00};
    constexpr int       nw = int(std::size(kWalls)), nd = int(std::size(kDensity));
    model.set_material(material);
    if (verify)
        verify->set_material(material);

    auto settings_for = [&](int walls, double density) {
        StructureSettings s = base;
        s.wall_loops        = walls;
        s.infill_density    = density;
        // Shells follow the walls so the skin is as strong as the sides.
        const int shells = std::max(base.top_layers, int(std::ceil(walls * base.wall_line_width / base.layer_height)));
        s.top_layers     = std::max(s.top_layers, std::min(shells, 12));
        s.bottom_layers  = std::max(s.bottom_layers, std::min(shells, 12));
        return s;
    };
    auto safety_of = [&](StructuralModel& m, const StructureSettings& s) {
        m.set_structure(s);
        double sf = std::numeric_limits<double>::max();
        for (const LoadCase& lc : governing) {
            const CaseResult r = m.solve(lc);
            if (!r.ok)
                return 0.0;
            sf = std::min(sf, r.min_safety_factor);
        }
        return sf;
    };

    const int total_steps = nw * nd + int(targets.size()) * (verify ? 3 : 1);
    int       step        = 0;
    auto      advance     = [&]() { return !progress || progress(double(++step) / total_steps); };

    // 1) Safety over a grid of walls x density on the quick model.
    double sf[nw][nd];
    for (int w = 0; w < nw; ++w)
        for (int d = 0; d < nd; ++d) {
            sf[w][d] = safety_of(model, settings_for(kWalls[w], kDensity[d]));
            if (!advance())
                return {};
        }

    // Lightest grid-interpolated settings whose quick-model safety reaches `target`.
    auto pick = [&](double target, StructureSettings& out) {
        double best_mass = std::numeric_limits<double>::max();
        for (int w = 0; w < nw; ++w) {
            // Smallest density meeting the target, interpolating log(density) against log(safety).
            double density = -1.0;
            if (sf[w][0] >= target) {
                const double slope = std::log(sf[w][1] / std::max(sf[w][0], 1e-9)) / std::log(kDensity[1] / kDensity[0]);
                density = slope > 1e-3 ? kDensity[0] * std::exp(std::log(target / sf[w][0]) / slope) : kDensity[0];
                density = std::clamp(density, 0.05, kDensity[0]);
            } else {
                for (int d = 1; d < nd; ++d)
                    if (sf[w][d] >= target) {
                        const double t = std::log(target / sf[w][d - 1]) / std::log(sf[w][d] / sf[w][d - 1]);
                        density = std::exp(std::log(kDensity[d - 1]) + t * std::log(kDensity[d] / kDensity[d - 1]));
                        break;
                    }
            }
            if (density < 0)
                continue;
            density                   = std::min(1.0, std::ceil(density * 20.0 - 1e-6) / 20.0); // 5% steps
            const StructureSettings s = settings_for(kWalls[w], density);
            const double            mass = model.estimated_mass_g(s, material);
            if (mass < best_mass) {
                best_mass = mass;
                out       = s;
            }
        }
        if (best_mass < std::numeric_limits<double>::max())
            return true;
        out = settings_for(kWalls[nw - 1], 1.0); // nothing reaches it: the strongest combination
        return false;
    };

    // 2) Per target: pick on the quick model, check on the detailed one, and re-aim with the measured
    //    ratio between them (the coarse voxels blur thin walls, so the two differ by 10-30%).
    std::vector<TierResult> out;
    for (const auto& [id, target] : targets) {
        TierResult best;
        best.id     = id;
        best.target = target;
        bool   have = false;
        double aim  = target;
        for (int attempt = 0; attempt < (verify ? 3 : 1); ++attempt) {
            StructureSettings s;
            const bool        on_curve = pick(aim, s);
            const double      quick    = verify ? safety_of(model, s) : 0.0;
            const double      actual   = verify ? safety_of(*verify, s) : safety_of(model, s);
            const double      mass     = model.estimated_mass_g(s, material);
            if (actual >= target && (!have || mass < best.mass_g)) {
                best.settings = s;
                best.safety   = actual;
                best.mass_g   = mass;
                have          = true;
            } else if (!have && actual > best.safety) {
                best.settings = s; // closest so far, kept if nothing reaches the target
                best.safety   = actual;
                best.mass_g   = mass;
            }
            if (!advance())
                return {};
            if (!verify || (actual >= target && actual <= target * 1.2) || (!on_curve && actual < target))
                break;
            const double next = target * quick / std::max(actual, 1e-9);
            if (std::abs(next - aim) < 0.02 * target)
                break;
            aim = next;
        }
        while (verify && step < nw * nd + int(out.size() + 1) * 3)
            advance();
        if (!have && best.settings.infill_density < 1.0) {
            // The re-aimed picks all fell short: fall back to the strongest combination.
            best.settings = settings_for(kWalls[nw - 1], 1.0);
            best.safety   = safety_of(verify ? *verify : model, best.settings);
            best.mass_g   = model.estimated_mass_g(best.settings, material);
            have          = best.safety >= target;
        }
        best.reached      = have;
        best.extruded_mm3 = model.extruded_volume(best.settings);
        out.push_back(best);
    }
    return out;
}

StructureSettings structure_from_json(const nlohmann::json& j, const StructureSettings& base)
{
    StructureSettings s = base;
    s.wall_loops        = j.value("walls", s.wall_loops);
    s.infill_density    = j.value("infill", s.infill_density);
    s.infill_pattern    = j.value("pattern", s.infill_pattern);
    s.top_layers        = j.value("top", s.top_layers);
    s.bottom_layers     = j.value("bottom", s.bottom_layers);
    s.layer_height      = j.value("layer_height", s.layer_height);
    s.wall_line_width   = j.value("line_width", s.wall_line_width);
    return s;
}

nlohmann::json structure_to_json(const StructureSettings& s)
{
    return {{"walls", s.wall_loops},     {"infill", s.infill_density}, {"pattern", s.infill_pattern},
            {"top", s.top_layers},       {"bottom", s.bottom_layers},  {"layer_height", s.layer_height},
            {"line_width", s.wall_line_width}};
}

std::vector<TierResult> rule_tiers(const nlohmann::json& use_case, const StructureSettings& base)
{
    std::vector<TierResult> out;
    const nlohmann::json tiers = use_case.value("tiers", nlohmann::json::object());
    for (const char* id : {"light", "balanced", "strong"}) {
        TierResult t;
        t.id       = id;
        t.settings = base;
        if (tiers.contains(id))
            t.settings = structure_from_json(tiers[id], base);
        out.push_back(t);
    }
    return out;
}

nlohmann::json tier_to_json(const TierResult& t)
{
    return {{"id", t.id},         {"target", t.target},           {"settings", structure_to_json(t.settings)},
            {"mass_g", t.mass_g}, {"safety", t.safety},           {"reached", t.reached},
            {"extruded_mm3", t.extruded_mm3}, {"print_minutes", t.print_minutes}};
}

// --- Drones -------------------------------------------------------------------------------------

double mount_radius_for(const Motor& motor)
{
    // "2806.5" -> 28 mm stator; the motor base covers a little more than the stator.
    if (motor.stator.size() >= 4 && std::isdigit(static_cast<unsigned char>(motor.stator[0])) &&
        std::isdigit(static_cast<unsigned char>(motor.stator[1]))) {
        const double stator = std::stod(motor.stator.substr(0, 2));
        return std::clamp(stator * 0.5 + 2.0, 4.0, 40.0);
    }
    return 12.0;
}

DroneLayout detect_drone_layout(const MeshView& mesh, int motor_count, double mount_radius, const Vec3& up_in)
{
    DroneLayout layout;
    layout.up           = up_in.normalized();
    layout.mount_radius = mount_radius;
    if (mesh.num_vertices == 0 || motor_count <= 0)
        return layout;
    const Vec3 up = layout.up;
    // In-plane axes.
    const Vec3 ax = (std::abs(up.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY()).cross(up).cross(up).normalized() * -1.0;
    const Vec3 ay = up.cross(ax);
    auto vtx = [&](size_t i) {
        return Vec3(mesh.vertices[3 * i], mesh.vertices[3 * i + 1], mesh.vertices[3 * i + 2]);
    };
    Eigen::AlignedBox3d box;
    for (size_t i = 0; i < mesh.num_vertices; ++i)
        box.extend(vtx(i));
    const Vec3 center = box.center();

    constexpr int       kBins = 360;
    std::vector<double> radius(kBins, -1.0);
    std::vector<Vec3>   point(kBins, center);
    for (size_t i = 0; i < mesh.num_vertices; ++i) {
        const Vec3   d   = vtx(i) - center;
        const double x   = d.dot(ax), y = d.dot(ay);
        const double r   = std::hypot(x, y);
        int          bin = int(std::floor((std::atan2(y, x) + kPi) / (2 * kPi) * kBins)) % kBins;
        if (r > radius[bin]) {
            radius[bin] = r;
            point[bin]  = vtx(i);
        }
    }
    std::vector<char> taken(kBins, 0);
    const int         suppress = std::max(1, int(kBins / motor_count * 0.4));
    std::vector<std::pair<double, Vec3>> found; // angle, position
    for (int k = 0; k < motor_count; ++k) {
        int best = -1;
        for (int b = 0; b < kBins; ++b)
            if (!taken[b] && radius[b] > 0 && (best < 0 || radius[b] > radius[best]))
                best = b;
        if (best < 0)
            break;
        for (int o = -suppress; o <= suppress; ++o)
            taken[(best + o + kBins) % kBins] = 1;
        const Vec3 d   = point[best] - center;
        Vec3       dir = (d - d.dot(up) * up);
        dir.normalize();
        Vec3 pos = center + dir * std::max(radius[best] - mount_radius, 0.0);
        // Sit the mount on the top surface near that spot.
        double top = -std::numeric_limits<double>::max();
        for (size_t i = 0; i < mesh.num_vertices; ++i) {
            const Vec3 v = vtx(i);
            Vec3       q = v - pos;
            q -= q.dot(up) * up;
            if (q.norm() <= mount_radius * 1.2)
                top = std::max(top, v.dot(up));
        }
        if (top > -std::numeric_limits<double>::max())
            pos += (top - pos.dot(up)) * up;
        found.emplace_back(std::atan2(dir.dot(ay), dir.dot(ax)), pos);
    }
    std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < found.size(); ++i) {
        layout.motors.push_back(found[i].second);
        layout.spin.push_back(i % 2 == 0 ? 1 : -1);
        Vec3 d = found[i].second - center;
        min_dist = std::min(min_dist, (d - d.dot(up) * up).norm());
    }
    layout.center        = center;
    layout.center_radius = found.empty() ? 20.0 : std::clamp(0.3 * min_dist, 8.0, 45.0);
    return layout;
}

std::vector<RegionMass> drone_masses(const DroneLayout& layout, const PowerSystem& system)
{
    std::vector<RegionMass> out;
    for (const Vec3& p : layout.motors)
        out.push_back({Region{p, layout.mount_radius}, system.motor.weight_g + system.prop.weight_g});
    return out;
}

std::vector<LoadCase> drone_load_cases(const DroneLayout& layout, const PowerSystem& system, const PowerReport& report, const DroneStyle& style)
{
    std::vector<LoadCase> out;
    const Vec3 up = layout.up.normalized();

    LoadCase punch;
    punch.id   = "punch";
    punch.name = "Full-throttle punch-out";
    for (size_t i = 0; i < layout.motors.size(); ++i) {
        const Region r{layout.motors[i], layout.mount_radius};
        punch.forces.push_back({r, up * report.full.thrust_n * style.dynamic_factor});
        const int spin = i < layout.spin.size() ? layout.spin[i] : 1;
        punch.torques.push_back({r, up, spin * report.full.torque_nm * 1000.0 * style.dynamic_factor});
    }
    out.push_back(punch);

    LoadCase landing;
    landing.id             = "landing";
    landing.name           = "Hard landing";
    landing.acceleration_g = -up * style.landing_g;
    out.push_back(landing);

    if (!layout.motors.empty()) {
        LoadCase hit;
        hit.id   = "arm_hit";
        hit.name = "Crash on one arm";
        // The ground stops the drone through one arm tip.
        const double auw_n = report.all_up_weight_g * 1e-3 * kG;
        hit.forces.push_back({Region{layout.motors[0], layout.mount_radius}, up * auw_n * style.crash_g});
        out.push_back(hit);
    }
    (void) system;
    return out;
}

}} // namespace Slic3r::SmartPrint
