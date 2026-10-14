#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/SmartPrint/Advisor.hpp"
#include "libslic3r/SmartPrint/Materials.hpp"
#include "libslic3r/SmartPrint/PrintStructure.hpp"
#include "libslic3r/SmartPrint/Propulsion.hpp"
#include "libslic3r/SmartPrint/StructuralModel.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace Slic3r::SmartPrint;
using Catch::Matchers::WithinRel;

namespace {

const std::string kResources = std::string(PROFILES_DIR) + "/../smartprint/";

struct BoxMesh
{
    std::vector<float> vertices;
    std::vector<int>   indices;
    void add_box(const Vec3& lo, const Vec3& hi)
    {
        const int base = int(vertices.size() / 3);
        for (int i = 0; i < 8; ++i) {
            vertices.push_back(float(i & 1 ? hi.x() : lo.x()));
            vertices.push_back(float(i & 2 ? hi.y() : lo.y()));
            vertices.push_back(float(i & 4 ? hi.z() : lo.z()));
        }
        const int quads[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
        for (const auto& q : quads)
            for (int k : {q[0], q[1], q[2], q[0], q[2], q[3]})
                indices.push_back(base + k);
    }
    MeshView view() const { return {vertices.data(), vertices.size() / 3, indices.data(), indices.size() / 3}; }
};

StructureSettings solid_settings()
{
    StructureSettings s;
    s.wall_loops     = 30;
    s.infill_density = 1.0;
    s.top_layers = s.bottom_layers = 60;
    return s;
}

Material pla()
{
    Material m;
    m.modulus_xy = 2600;
    m.tensile_xy = 45;
    m.tensile_z  = 28;
    m.density    = 1.24;
    return m;
}

} // namespace

// A 100 x 10 x 6 mm solid cantilever with a 10 N tip load: Euler-Bernoulli gives F L^3 / (3 E I).
TEST_CASE("A solid cantilever deflects as beam theory predicts", "[SmartPrint]")
{
    BoxMesh beam;
    beam.add_box({0, 0, 0}, {100, 10, 6});
    StructuralModel model;
    MeshOptions     options;
    options.element_size = 1.0;
    REQUIRE(model.build(beam.view(), options));
    model.set_material(pla());
    model.set_structure(solid_settings());
    // A huge sphere touching only the x = 0 face clamps that end; another loads the x = 100 end.
    model.set_supports({Region{{-1000, 5, 3}, 1000.01}});
    LoadCase tip;
    tip.forces.push_back({Region{{1100, 5, 3}, 1000.01}, {0, 0, -10}});
    const CaseResult r = model.solve(tip);
    REQUIRE(r.ok);

    const double inertia = 10.0 * 6 * 6 * 6 / 12;
    const double expected = 10.0 * 100 * 100 * 100 / (3 * 2600 * inertia);
    CHECK_THAT(r.max_displacement, WithinRel(expected, 0.03));
}

TEST_CASE("The first bending frequency of a cantilever matches theory", "[SmartPrint]")
{
    BoxMesh beam;
    beam.add_box({0, 0, 0}, {100, 10, 6});
    StructuralModel model;
    MeshOptions     options;
    options.element_size = 1.0;
    REQUIRE(model.build(beam.view(), options));
    model.set_material(pla());
    model.set_structure(solid_settings());
    model.set_supports({Region{{-1000, 5, 3}, 1000.01}});
    // f1 = 1.8751^2 / (2 pi) * sqrt(E I / (rho A L^4)), SI units.
    const double e = 2600e6, i = 0.01 * 0.006 * 0.006 * 0.006 / 12, a = 0.01 * 0.006, rho = 1240, l = 0.1;
    const double expected = 1.8751 * 1.8751 / (2 * 3.14159265358979) * std::sqrt(e * i / (rho * a * l * l * l * l));
    CHECK_THAT(model.natural_frequencies().z(), WithinRel(expected, 0.03));
}

TEST_CASE("Thin walls with sparse infill are weaker and lighter than solid", "[SmartPrint]")
{
    BoxMesh beam;
    beam.add_box({0, 0, 0}, {100, 10, 6});
    StructuralModel model;
    MeshOptions     options;
    options.element_size = 0.75;
    REQUIRE(model.build(beam.view(), options));
    model.set_material(pla());
    model.set_supports({Region{{-1000, 5, 3}, 1000.01}});
    LoadCase tip;
    tip.forces.push_back({Region{{1100, 5, 3}, 1000.01}, {0, 0, -10}});

    model.set_structure(solid_settings());
    const CaseResult solid      = model.solve(tip);
    const double     solid_mass = model.estimated_mass_g();
    StructureSettings sparse;
    sparse.wall_loops     = 2;
    sparse.infill_density = 0.15;
    model.set_structure(sparse);
    const CaseResult light = model.solve(tip);

    REQUIRE(solid.ok);
    REQUIRE(light.ok);
    CHECK(light.max_displacement > solid.max_displacement);
    CHECK(light.min_safety_factor < solid.min_safety_factor);
    CHECK(model.estimated_mass_g() < solid_mass);
    CHECK_THAT(solid_mass, WithinRel(100.0 * 10 * 6 * 1.24e-3, 0.03));
}

TEST_CASE("A load response is reused when only its magnitude changes", "[SmartPrint]")
{
    BoxMesh beam;
    beam.add_box({0, 0, 0}, {60, 10, 6});
    StructuralModel model;
    MeshOptions     options;
    options.element_size = 1.5;
    REQUIRE(model.build(beam.view(), options));
    model.set_material(pla());
    model.set_supports({Region{{-1000, 5, 3}, 1000.01}});
    LoadCase tip;
    tip.forces.push_back({Region{{1060, 5, 3}, 1000.01}, {0, 0, -10}});
    const CaseResult first = model.solve(tip);
    tip.forces[0].force *= 2.0;
    const CaseResult doubled = model.solve(tip);
    REQUIRE(first.ok);
    REQUIRE(doubled.ok);
    CHECK(doubled.iterations == 0);
    CHECK_THAT(doubled.max_displacement, WithinRel(2.0 * first.max_displacement, 1e-6));
}

TEST_CASE("Orca filament types map to material families", "[SmartPrint]")
{
    MaterialLibrary library;
    REQUIRE(library.load_file(kResources + "materials.json"));
    auto id = [&](const char* type) {
        const Material* m = library.find_for_filament_type(type);
        return m ? m->id : std::string("none");
    };
    CHECK(id("PLA Basic") == "PLA");
    CHECK(id("PETG-CF10") == "PETG-CF");
    CHECK(id("PA6-CF") == "PA-CF");
    CHECK(id("PAHT-CF") == "PAHT-CF");
    CHECK(id("ABS-GF") == "ABS-CF");
    CHECK(id("TPU-AMS") == "TPU");
}

TEST_CASE("Filaments a printer cannot print are not recommended", "[SmartPrint]")
{
    MaterialLibrary library;
    REQUIRE(library.load_file(kResources + "materials.json"));
    UseCaseLibrary use_cases;
    REQUIRE(use_cases.load_file(kResources + "usecases.json"));
    PrinterCapabilities printer;
    printer.hardened_nozzle = false;
    const std::vector<OwnedFilament> owned = {{"My PLA", "PLA"}, {"My PA-CF", "PA-CF"}};
    const auto ranking = rank_materials(*use_cases.find("drone_frame"), use_cases.environment("indoor"), printer, owned, library);
    REQUIRE(ranking.size() == 2);
    CHECK(ranking.front().filament.preset == "My PLA");
    CHECK_FALSE(ranking.back().usable);
}

TEST_CASE("A 5 inch power system makes plausible thrust", "[SmartPrint]")
{
    PropulsionLibrary library;
    REQUIRE(library.load_file(kResources + "propulsion.json"));
    PowerSystem sys;
    sys.motor     = *library.find_motor("class-2207-1750");
    sys.prop      = *library.find_prop("p-51466-3");
    sys.battery   = *library.find_battery("b-6s-1300");
    sys.payload_g = 200;
    const PowerReport r = PropulsionModel::analyse(sys, 120);
    // Bench tests of this class give roughly 1.3-1.9 kg per motor at 35-50 A.
    const double thrust_g = r.full.thrust_n / 9.80665 * 1000.0;
    CHECK(thrust_g > 1000);
    CHECK(thrust_g < 2200);
    CHECK(r.can_hover);
    CHECK(r.hover.throttle < 0.5);
}

TEST_CASE("Motor mounts are found at the ends of a quad's arms", "[SmartPrint]")
{
    BoxMesh frame;
    frame.add_box({-30, -30, 0}, {30, 30, 4});
    frame.add_box({0, -6, 0}, {120, 6, 5});
    frame.add_box({-120, -6, 0}, {0, 6, 5});
    frame.add_box({-6, 0, 0}, {6, 120, 5});
    frame.add_box({-6, -120, 0}, {6, 0, 5});
    const DroneLayout layout = detect_drone_layout(frame.view(), 4, 12.0);
    REQUIRE(layout.motors.size() == 4);
    for (const Vec3& m : layout.motors) {
        CHECK_THAT(Eigen::Vector2d(m.x(), m.y()).norm(), WithinRel(108.0, 0.1));
        CHECK_THAT(m.z(), WithinRel(5.0, 0.01));
    }
}
