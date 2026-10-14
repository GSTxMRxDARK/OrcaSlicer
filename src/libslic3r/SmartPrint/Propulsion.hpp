#pragma once

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <utility>
#include <vector>

namespace Slic3r { namespace SmartPrint {

struct Motor
{
    std::string         id, brand, name, stator, source;
    double              kv           = 1000;  // rpm per volt
    double              weight_g     = 30;
    double              resistance   = 0.08;  // ohm, phase-to-phase
    double              idle_current = 1.0;   // A at 10 V
    double              max_current  = 30;    // A
    double              max_power    = 600;   // W
    int                 min_cells    = 3;
    int                 max_cells    = 6;
    std::vector<double> props_in;             // recommended propeller diameters
};

struct Propeller
{
    std::string id, name;
    double      diameter_in = 5.0;
    double      pitch_in    = 4.0;
    int         blades      = 2;
    double      weight_g    = 5.0;
    double      ct          = 0.0; // static thrust coefficient, 0 = estimate from geometry
    double      cp          = 0.0; // static power coefficient, 0 = estimate from geometry
};

struct Battery
{
    std::string id, name, chemistry = "lipo";
    int         cells               = 4;
    double      capacity_mah        = 1500;
    double      weight_g            = 180;
    double      resistance_per_cell = 0.005; // ohm
};

class PropulsionLibrary
{
public:
    bool load_file(const std::string& path, std::string* error = nullptr);
    bool load_json(const nlohmann::json& root, std::string* error = nullptr);

    const std::vector<Motor>&     motors() const { return m_motors; }
    const std::vector<Propeller>& props() const { return m_props; }
    const std::vector<Battery>&   batteries() const { return m_batteries; }

    const Motor*     find_motor(const std::string& id) const;
    const Propeller* find_prop(const std::string& id) const;
    const Battery*   find_battery(const std::string& id) const;

private:
    std::vector<Motor>     m_motors;
    std::vector<Propeller> m_props;
    std::vector<Battery>   m_batteries;
};

// A multirotor's power system and everything it carries except the printed frame.
struct PowerSystem
{
    Motor     motor;
    Propeller prop;
    Battery   battery;
    int       motor_count     = 4;
    double    payload_g       = 0;   // flight controller, ESC, camera, VTX, wiring, payload...
    double    measured_thrust_g = 0; // optional bench-tested full-throttle thrust per motor; 0 = predict
};

struct OperatingPoint
{
    double throttle  = 0; // 0..1
    double rpm       = 0;
    double thrust_n  = 0; // per motor
    double torque_nm = 0; // reaction torque per motor
    double current_a = 0; // per motor
    double voltage   = 0; // battery voltage under this load
    double power_w   = 0; // electrical power per motor
};

struct PowerReport
{
    OperatingPoint full;           // full throttle
    OperatingPoint hover;
    double         all_up_weight_g = 0;
    double         thrust_to_weight = 0;
    double         flight_time_min  = 0; // hover, using 80% of the battery
    bool           can_hover        = false;
    // Issues as (code, value): "cells_out_of_range", "over_current" (A), "over_power" (W),
    // "low_thrust_to_weight", "cannot_hover", "high_hover_throttle", "prop_size_mismatch" (in).
    std::vector<std::pair<std::string, double>> warnings;
};

// Static (hover / punch-out) propeller and motor model:
//   thrust = Ct * rho * n^2 * D^4,  power = Cp * rho * n^3 * D^5   (n in rev/s)
//   motor:  I = (V_eff - rpm / Kv) / R,  torque = Kt * (I - I0),  Kt = 60 / (2 pi Kv)
// solved for the rpm where motor torque equals propeller torque, with battery sag.
// Accuracy is typically within 15-20% of bench tests; a measured thrust replaces the prediction.
class PropulsionModel
{
public:
    static double         thrust_coefficient(const Propeller& prop);
    static double         power_coefficient(const Propeller& prop);
    static OperatingPoint operating_point(const PowerSystem& system, double throttle);
    // frame_g: weight of the printed frame and any other printed parts.
    static PowerReport analyse(const PowerSystem& system, double frame_g);
};

nlohmann::json motor_to_json(const Motor& m);
nlohmann::json prop_to_json(const Propeller& p);
nlohmann::json battery_to_json(const Battery& b);
Motor          motor_from_json(const nlohmann::json& j);
Propeller      prop_from_json(const nlohmann::json& j);
Battery        battery_from_json(const nlohmann::json& j);
nlohmann::json report_to_json(const PowerReport& r);

}} // namespace Slic3r::SmartPrint
