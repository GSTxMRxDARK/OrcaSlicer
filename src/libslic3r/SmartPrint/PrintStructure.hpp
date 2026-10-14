#pragma once

#include <string>
#include <vector>

namespace Slic3r { namespace SmartPrint {

// The slicer settings that decide how much plastic carries load inside a printed part.
struct StructureSettings
{
    int         wall_loops      = 2;
    double      wall_line_width = 0.45; // mm
    int         top_layers      = 4;
    int         bottom_layers   = 4;
    double      layer_height    = 0.2;  // mm
    double      infill_density  = 0.15; // 0..1 volume fraction of the sparse infill
    std::string infill_pattern  = "gyroid";

    double wall_thickness() const { return wall_loops * wall_line_width; }
    double top_thickness() const { return top_layers * layer_height; }
    double bottom_thickness() const { return bottom_layers * layer_height; }

    bool operator==(const StructureSettings& o) const;
    bool operator!=(const StructureSettings& o) const { return !(*this == o); }
};

// Gibson-Ashby style scaling of a sparse infill's stiffness and strength with its density:
// property_infill / property_solid = density ^ exponent. Exponents near 1 mean stretch-dominated
// lattices (grid, triangles, cubic), larger ones bending-dominated or non-structural fills.
struct InfillPatternModel
{
    const char* id;
    const char* label;
    double      stiffness_exponent;
    double      strength_exponent;
    bool        structural; // false for fills that only support top skins (lightning)
};

const InfillPatternModel&              infill_pattern_model(const std::string& pattern);
const std::vector<InfillPatternModel>& infill_pattern_models();

// Infill properties relative to the solid printed material.
double infill_relative_stiffness(const StructureSettings& s);
double infill_relative_strength(const StructureSettings& s);

}} // namespace Slic3r::SmartPrint
