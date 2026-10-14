#include "PrintStructure.hpp"

#include <algorithm>
#include <cmath>

namespace Slic3r { namespace SmartPrint {

bool StructureSettings::operator==(const StructureSettings& o) const
{
    return wall_loops == o.wall_loops && wall_line_width == o.wall_line_width && top_layers == o.top_layers &&
           bottom_layers == o.bottom_layers && layer_height == o.layer_height && infill_density == o.infill_density &&
           infill_pattern == o.infill_pattern;
}

const std::vector<InfillPatternModel>& infill_pattern_models()
{
    // Engineering estimates from published FDM infill test data, averaged over load directions.
    static const std::vector<InfillPatternModel> models = {
        {"gyroid", "Gyroid", 1.60, 1.40, true},
        {"tpmsd", "TPMS-D", 1.55, 1.40, true},
        {"tpmsfk", "TPMS-FK", 1.55, 1.40, true},
        {"cubic", "Cubic", 1.35, 1.40, true},
        {"quartercubic", "Quarter cubic", 1.35, 1.40, true},
        {"triangles", "Triangles", 1.30, 1.35, true},
        {"tri-hexagon", "Tri-hexagon", 1.35, 1.40, true},
        {"grid", "Grid", 1.40, 1.40, true},
        {"honeycomb", "Honeycomb", 1.35, 1.35, true},
        {"3dhoneycomb", "3D honeycomb", 1.45, 1.45, true},
        {"lateral-honeycomb", "Lateral honeycomb", 1.40, 1.40, true},
        {"lateral-lattice", "Lateral lattice", 1.40, 1.40, true},
        {"crosshatch", "Cross hatch", 1.45, 1.45, true},
        {"adaptivecubic", "Adaptive cubic", 1.55, 1.60, true},
        {"supportcubic", "Support cubic", 1.70, 1.75, true},
        {"rectilinear", "Rectilinear", 1.60, 1.60, true},
        {"alignedrectilinear", "Aligned rectilinear", 1.60, 1.60, true},
        {"line", "Line", 1.70, 1.70, true},
        {"zigzag", "Zig zag", 1.60, 1.60, true},
        {"crosszag", "Cross zag", 1.55, 1.55, true},
        {"lockedzag", "Locked zag", 1.55, 1.55, true},
        {"monotonic", "Monotonic", 1.60, 1.60, true},
        {"monotonicline", "Monotonic line", 1.70, 1.70, true},
        {"concentric", "Concentric", 1.80, 1.80, true},
        {"spiralinset", "Spiral inset", 1.80, 1.80, true},
        {"hilbertcurve", "Hilbert curve", 1.90, 1.90, true},
        {"archimedeanchords", "Archimedean chords", 1.90, 1.90, true},
        {"octagramspiral", "Octagram spiral", 1.90, 1.90, true},
        {"lightning", "Lightning", 3.00, 3.00, false},
    };
    return models;
}

const InfillPatternModel& infill_pattern_model(const std::string& pattern)
{
    for (const InfillPatternModel& m : infill_pattern_models())
        if (pattern == m.id)
            return m;
    return infill_pattern_models().front();
}

double infill_relative_stiffness(const StructureSettings& s)
{
    const double rho = std::clamp(s.infill_density, 0.0, 1.0);
    if (rho <= 0.0)
        return 0.0;
    return std::pow(rho, infill_pattern_model(s.infill_pattern).stiffness_exponent);
}

double infill_relative_strength(const StructureSettings& s)
{
    const double rho = std::clamp(s.infill_density, 0.0, 1.0);
    if (rho <= 0.0)
        return 0.0;
    return std::pow(rho, infill_pattern_model(s.infill_pattern).strength_exponent);
}

}} // namespace Slic3r::SmartPrint
