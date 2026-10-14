#include "Propulsion.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>

namespace Slic3r { namespace SmartPrint {

namespace {

constexpr double kPi        = 3.14159265358979323846;
constexpr double kAirDensity = 1.225; // kg/m3, sea level, 15 C
constexpr double kInch      = 0.0254;
constexpr double kG         = 9.80665;

double cell_voltage(const std::string& chemistry)
{
    // Average voltage under load over a flight.
    if (chemistry == "liion")
        return 3.6;
    if (chemistry == "life")
        return 3.2;
    return 3.8;
}

template<class T> const T* find_id(const std::vector<T>& list, const std::string& id)
{
    for (const T& item : list)
        if (item.id == id)
            return &item;
    return nullptr;
}

} // namespace

Motor motor_from_json(const nlohmann::json& j)
{
    Motor m;
    m.id           = j.value("id", "");
    m.brand        = j.value("brand", "");
    m.name         = j.value("name", m.id);
    m.stator       = j.value("stator", "");
    m.source       = j.value("source", "user");
    m.kv           = j.value("kv", m.kv);
    m.weight_g     = j.value("weight_g", m.weight_g);
    m.resistance   = j.value("resistance", m.resistance);
    m.idle_current = j.value("idle_current", m.idle_current);
    m.max_current  = j.value("max_current", m.max_current);
    m.max_power    = j.value("max_power", m.max_power);
    if (j.contains("cells") && j["cells"].is_array() && j["cells"].size() == 2) {
        m.min_cells = j["cells"][0].get<int>();
        m.max_cells = j["cells"][1].get<int>();
    }
    m.props_in = j.value("props_in", std::vector<double>{});
    return m;
}

Propeller prop_from_json(const nlohmann::json& j)
{
    Propeller p;
    p.id          = j.value("id", "");
    p.name        = j.value("name", p.id);
    p.diameter_in = j.value("diameter_in", p.diameter_in);
    p.pitch_in    = j.value("pitch_in", p.pitch_in);
    p.blades      = j.value("blades", p.blades);
    p.weight_g    = j.value("weight_g", p.weight_g);
    p.ct          = j.value("ct", 0.0);
    p.cp          = j.value("cp", 0.0);
    return p;
}

Battery battery_from_json(const nlohmann::json& j)
{
    Battery b;
    b.id                  = j.value("id", "");
    b.name                = j.value("name", b.id);
    b.chemistry           = j.value("chemistry", b.chemistry);
    b.cells               = j.value("cells", b.cells);
    b.capacity_mah        = j.value("capacity_mah", b.capacity_mah);
    b.weight_g            = j.value("weight_g", b.weight_g);
    b.resistance_per_cell = j.value("resistance_per_cell", b.resistance_per_cell);
    return b;
}

nlohmann::json motor_to_json(const Motor& m)
{
    return {{"id", m.id},         {"brand", m.brand},           {"name", m.name},
            {"stator", m.stator}, {"source", m.source},         {"kv", m.kv},
            {"weight_g", m.weight_g}, {"resistance", m.resistance}, {"idle_current", m.idle_current},
            {"max_current", m.max_current}, {"max_power", m.max_power}, {"cells", {m.min_cells, m.max_cells}},
            {"props_in", m.props_in}};
}

nlohmann::json prop_to_json(const Propeller& p)
{
    return {{"id", p.id},         {"name", p.name},     {"diameter_in", p.diameter_in}, {"pitch_in", p.pitch_in},
            {"blades", p.blades}, {"weight_g", p.weight_g}, {"ct", p.ct},              {"cp", p.cp}};
}

nlohmann::json battery_to_json(const Battery& b)
{
    return {{"id", b.id},       {"name", b.name},         {"chemistry", b.chemistry},
            {"cells", b.cells}, {"capacity_mah", b.capacity_mah}, {"weight_g", b.weight_g},
            {"resistance_per_cell", b.resistance_per_cell}};
}

bool PropulsionLibrary::load_file(const std::string& path, std::string* error)
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

bool PropulsionLibrary::load_json(const nlohmann::json& root, std::string* error)
{
    try {
        // Entries with an existing id replace it, so a user file can override the bundled one.
        auto merge = [](auto& list, auto item) {
            for (auto& existing : list)
                if (existing.id == item.id) {
                    existing = std::move(item);
                    return;
                }
            list.push_back(std::move(item));
        };
        for (const nlohmann::json& j : root.value("motors", nlohmann::json::array()))
            merge(m_motors, motor_from_json(j));
        for (const nlohmann::json& j : root.value("props", nlohmann::json::array()))
            merge(m_props, prop_from_json(j));
        for (const nlohmann::json& j : root.value("batteries", nlohmann::json::array()))
            merge(m_batteries, battery_from_json(j));
    } catch (const std::exception& e) {
        if (error)
            *error = e.what();
        return false;
    }
    return true;
}

const Motor*     PropulsionLibrary::find_motor(const std::string& id) const { return find_id(m_motors, id); }
const Propeller* PropulsionLibrary::find_prop(const std::string& id) const { return find_id(m_props, id); }
const Battery*   PropulsionLibrary::find_battery(const std::string& id) const { return find_id(m_batteries, id); }

double PropulsionModel::thrust_coefficient(const Propeller& prop)
{
    if (prop.ct > 0)
        return prop.ct;
    const double ratio = prop.pitch_in / std::max(prop.diameter_in, 0.1);
    return (0.065 + 0.075 * ratio) * std::pow(std::max(prop.blades, 1) / 2.0, 0.55);
}

double PropulsionModel::power_coefficient(const Propeller& prop)
{
    if (prop.cp > 0)
        return prop.cp;
    const double ratio = prop.pitch_in / std::max(prop.diameter_in, 0.1);
    return (0.014 + 0.07 * std::pow(ratio, 1.5)) * std::pow(std::max(prop.blades, 1) / 2.0, 0.85);
}

OperatingPoint PropulsionModel::operating_point(const PowerSystem& sys, double throttle)
{
    OperatingPoint op;
    op.throttle = std::clamp(throttle, 0.0, 1.0);
    const Motor&  motor = sys.motor;
    const double  d     = sys.prop.diameter_in * kInch;
    const double  ct    = thrust_coefficient(sys.prop);
    const double  cp    = power_coefficient(sys.prop);
    const double  kv    = std::max(motor.kv, 1.0);
    const double  r     = std::max(motor.resistance, 1e-4);
    const double  kt    = 60.0 / (2.0 * kPi * kv); // N*m/A
    const double  v_oc  = sys.battery.cells * cell_voltage(sys.battery.chemistry);
    const double  r_bat = sys.battery.cells * sys.battery.resistance_per_cell;
    const int     count = std::max(sys.motor_count, 1);

    double voltage = v_oc;
    for (int sag = 0; sag < 12; ++sag) {
        const double v_eff = op.throttle * voltage;
        auto current = [&](double n) { return std::max((v_eff - 60.0 * n / kv) / r, 0.0); };
        auto prop_torque = [&](double n) { return cp * kAirDensity * n * n * std::pow(d, 5) / (2.0 * kPi); };
        // Balance motor torque against propeller torque; motor torque falls and prop torque rises with speed.
        double lo = 0.0, hi = std::max(v_eff * kv / 60.0, 0.0);
        for (int it = 0; it < 60; ++it) {
            const double n = 0.5 * (lo + hi);
            if (kt * std::max(current(n) - motor.idle_current, 0.0) > prop_torque(n))
                lo = n;
            else
                hi = n;
        }
        const double n = 0.5 * (lo + hi);
        op.rpm         = n * 60.0;
        op.thrust_n    = ct * kAirDensity * n * n * std::pow(d, 4);
        op.torque_nm   = prop_torque(n);
        op.current_a   = n > 0 ? std::max(current(n), motor.idle_current * op.throttle) : 0.0;
        const double next_voltage = std::max(v_oc - count * op.current_a * r_bat, 0.5 * v_oc);
        if (std::abs(next_voltage - voltage) < 1e-3) {
            voltage = next_voltage;
            break;
        }
        voltage = 0.5 * (voltage + next_voltage);
    }
    op.voltage = voltage;
    op.power_w = op.current_a * voltage * op.throttle;
    return op;
}

PowerReport PropulsionModel::analyse(const PowerSystem& sys, double frame_g)
{
    PowerReport rep;
    const int count = std::max(sys.motor_count, 1);
    rep.all_up_weight_g = frame_g + sys.payload_g + sys.battery.weight_g + count * (sys.motor.weight_g + sys.prop.weight_g);

    rep.full = operating_point(sys, 1.0);
    // A bench-measured thrust replaces the prediction; scale every operating point by the same factor.
    double scale = 1.0;
    if (sys.measured_thrust_g > 0 && rep.full.thrust_n > 0)
        scale = sys.measured_thrust_g * 1e-3 * kG / rep.full.thrust_n;
    rep.full.thrust_n *= scale;

    const double weight_n = rep.all_up_weight_g * 1e-3 * kG;
    rep.thrust_to_weight  = weight_n > 0 ? count * rep.full.thrust_n / weight_n : 0.0;
    rep.can_hover         = rep.thrust_to_weight > 1.0;
    if (rep.can_hover) {
        double lo = 0.0, hi = 1.0;
        for (int it = 0; it < 40; ++it) {
            const double t = 0.5 * (lo + hi);
            if (operating_point(sys, t).thrust_n * scale * count < weight_n)
                lo = t;
            else
                hi = t;
        }
        rep.hover = operating_point(sys, hi);
        rep.hover.thrust_n *= scale;
        const double total_current = rep.hover.current_a * count + 0.5; // + electronics
        rep.flight_time_min        = 0.8 * sys.battery.capacity_mah * 1e-3 / total_current * 60.0;
    }

    if (sys.battery.cells < sys.motor.min_cells || sys.battery.cells > sys.motor.max_cells)
        rep.warnings.emplace_back("cells_out_of_range", sys.battery.cells);
    if (rep.full.current_a > sys.motor.max_current * 1.05)
        rep.warnings.emplace_back("over_current", rep.full.current_a);
    if (rep.full.power_w > sys.motor.max_power * 1.1)
        rep.warnings.emplace_back("over_power", rep.full.power_w);
    if (!rep.can_hover)
        rep.warnings.emplace_back("cannot_hover", rep.thrust_to_weight);
    else if (rep.thrust_to_weight < 2.0)
        rep.warnings.emplace_back("low_thrust_to_weight", rep.thrust_to_weight);
    if (rep.can_hover && rep.hover.throttle > 0.65)
        rep.warnings.emplace_back("high_hover_throttle", rep.hover.throttle);
    if (!sys.motor.props_in.empty()) {
        double closest = 1e9;
        for (double p : sys.motor.props_in)
            closest = std::min(closest, std::abs(p - sys.prop.diameter_in));
        if (closest > 0.75)
            rep.warnings.emplace_back("prop_size_mismatch", sys.prop.diameter_in);
    }
    return rep;
}

static nlohmann::json op_to_json(const OperatingPoint& op)
{
    return {{"throttle", op.throttle},   {"rpm", op.rpm},         {"thrust_g", op.thrust_n / kG * 1000.0},
            {"thrust_n", op.thrust_n},   {"torque_nm", op.torque_nm}, {"current_a", op.current_a},
            {"voltage", op.voltage},     {"power_w", op.power_w}};
}

nlohmann::json report_to_json(const PowerReport& r)
{
    nlohmann::json warnings = nlohmann::json::array();
    for (const auto& [code, value] : r.warnings)
        warnings.push_back({{"code", code}, {"value", value}});
    return {{"full", op_to_json(r.full)},
            {"hover", op_to_json(r.hover)},
            {"all_up_weight_g", r.all_up_weight_g},
            {"thrust_to_weight", r.thrust_to_weight},
            {"flight_time_min", r.flight_time_min},
            {"can_hover", r.can_hover},
            {"warnings", warnings}};
}

}} // namespace Slic3r::SmartPrint
