#include "SmartPrintDialog.hpp"

#include "GUI_App.hpp"
#include "GUI_ObjectList.hpp"
#include "I18N.hpp"
#include "ParamsPanel.hpp"
#include "Plater.hpp"
#include "PresetComboBoxes.hpp"
#include "Selection.hpp"
#include "SmartPrintStrings.hpp"
#include "Tab.hpp"

#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

#include <algorithm>
#include <chrono>
#include <set>
#include <utility>
#include <vector>

#include <wx/sizer.h>
#include <wx/stattext.h>

namespace Slic3r { namespace GUI {

namespace {

double first_float(const DynamicPrintConfig& cfg, const char* key, double fallback)
{
    const ConfigOption* opt = cfg.option(key);
    if (opt == nullptr)
        return fallback;
    if (const auto* floats = dynamic_cast<const ConfigOptionFloats*>(opt))
        return floats->values.empty() ? fallback : floats->get_at(0);
    if (const auto* single = dynamic_cast<const ConfigOptionFloat*>(opt))
        return single->value;
    return fallback;
}

std::string first_string(const DynamicPrintConfig& cfg, const char* key)
{
    if (const auto* strings = cfg.option<ConfigOptionStrings>(key))
        return strings->values.empty() ? std::string() : strings->values.front();
    if (const auto* single = cfg.option<ConfigOptionString>(key))
        return single->value;
    return {};
}

bool first_bool(const DynamicPrintConfig& cfg, const char* key)
{
    if (const auto* b = cfg.option<ConfigOptionBool>(key))
        return b->value;
    if (const auto* bs = cfg.option<ConfigOptionBools>(key))
        return !bs->values.empty() && bs->get_at(0);
    return false;
}

} // namespace

SmartPrintDialog::SmartPrintDialog(wxWindow* parent)
    : WebViewHostDialog(parent, wxID_ANY, _L("Smart Print assistant"))
{
    const std::string user_dir = (boost::filesystem::path(data_dir()) / "smartprint").string();
    boost::system::error_code ec;
    boost::filesystem::create_directories(user_dir, ec);
    m_session_ok = m_session.load_resources((boost::filesystem::path(resources_dir()) / "smartprint").string(), user_dir, &m_session_error);
    if (!m_session_ok)
        BOOST_LOG_TRIVIAL(error) << "Smart Print: " << m_session_error;

    collect_object();
    m_session.set_context(collect_context());
    m_worker = std::thread([this]() { worker_loop(); });

    if (!create_webview("web/dialog/SmartPrint/index.html", _L("Smart Print assistant"), wxSize(1280, 820), wxSize(900, 640))) {
        auto* sizer = new wxBoxSizer(wxVERTICAL);
        sizer->Add(new wxStaticText(this, wxID_ANY, _L("Web view is unavailable.")), wxSizerFlags().Border(wxALL, 20));
        SetSizerAndFit(sizer);
    }
    wxGetApp().UpdateDlgDarkUI(this);
}

SmartPrintDialog::~SmartPrintDialog()
{
    m_alive->store(false, std::memory_order_release);
    m_cancel = true;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
        m_queue.clear();
    }
    m_cv.notify_all();
    if (m_worker.joinable())
        m_worker.join();
}

nlohmann::json SmartPrintDialog::localized_strings() const { return smart_print_ui_strings(); }

ModelObject* SmartPrintDialog::target_object() const
{
    // The object may have been deleted while the dialog was open (it is modal, but be safe).
    for (ModelObject* mo : wxGetApp().model().objects)
        if (mo == m_object)
            return mo;
    return nullptr;
}

void SmartPrintDialog::collect_object()
{
    Plater* plater = wxGetApp().plater();
    Model&  model  = wxGetApp().model();
    int     obj    = -1, inst = 0;
    const Selection& sel = plater->get_selection();
    if (!sel.is_empty()) {
        obj  = sel.get_object_idx();
        inst = std::max(sel.get_instance_idx(), 0);
    }
    if (obj < 0 && model.objects.size() == 1)
        obj = 0;
    if (obj < 0 || obj >= int(model.objects.size()))
        return;
    ModelObject* mo = model.objects[size_t(obj)];
    if (mo->instances.empty())
        return;
    m_object      = mo;
    m_instance    = std::min(inst, int(mo->instances.size()) - 1);
    m_object_name = mo->name;

    // Model parts in printer coordinates: the analysis needs the build direction as Z.
    indexed_triangle_set its = mo->raw_indexed_triangle_set();
    its_transform(its, mo->instances[size_t(m_instance)]->get_matrix());
    std::vector<float> vertices;
    vertices.reserve(its.vertices.size() * 3);
    for (const stl_vertex& v : its.vertices) {
        vertices.push_back(v.x());
        vertices.push_back(v.y());
        vertices.push_back(v.z());
    }
    std::vector<int> indices;
    indices.reserve(its.indices.size() * 3);
    for (const stl_triangle_vertex_indices& f : its.indices) {
        indices.push_back(f[0]);
        indices.push_back(f[1]);
        indices.push_back(f[2]);
    }
    m_session.set_mesh(std::move(vertices), std::move(indices), m_object_name);
}

nlohmann::json SmartPrintDialog::collect_context() const
{
    PresetBundle*      bundle = wxGetApp().preset_bundle;
    DynamicPrintConfig cfg    = bundle->full_config();
    if (m_object)
        cfg.apply(m_object->config.get(), true);

    // Printer: hardened nozzle from nozzle_type; Orca has no enclosure flag, so chamber control or air
    // filtration stands in for it (the user can correct both on the page).
    nlohmann::json printer;
    printer["name"] = bundle->printers.get_edited_preset().name;
    bool hardened   = false;
    if (const auto* nt = cfg.option<ConfigOptionEnumsGenericNullable>("nozzle_type"); nt && !nt->values.empty()) {
        const int type = nt->values.front();
        hardened       = type == ntHardenedSteel || type == ntTungstenCarbide;
    }
    printer["hardened_nozzle"] = hardened;
    printer["enclosure"]       = first_bool(cfg, "support_chamber_temp_control") || first_bool(cfg, "support_air_filtration");
    const double nozzle        = first_float(cfg, "nozzle_diameter", 0.4);
    printer["nozzle_diameter"] = nozzle;
    printer["max_nozzle_temp"] = 300;

    // Settings currently in effect for the object.
    auto width = [&](const char* key) {
        const auto* w = cfg.option<ConfigOptionFloatOrPercent>(key);
        return w ? w->get_abs_value(nozzle) : 0.0;
    };
    double line_width = width("inner_wall_line_width");
    if (line_width <= 0)
        line_width = width("line_width");
    if (line_width <= 0)
        line_width = nozzle * 1.125;
    nlohmann::json structure;
    structure["walls"]        = cfg.opt_int("wall_loops");
    structure["infill"]       = cfg.option<ConfigOptionPercent>("sparse_infill_density") ? cfg.option<ConfigOptionPercent>("sparse_infill_density")->value / 100.0 : 0.15;
    structure["pattern"]      = cfg.option("sparse_infill_pattern") ? cfg.option("sparse_infill_pattern")->serialize() : std::string("gyroid");
    structure["top"]          = cfg.opt_int("top_shell_layers");
    structure["bottom"]       = cfg.opt_int("bottom_shell_layers");
    structure["layer_height"] = cfg.opt_float("layer_height");
    structure["line_width"]   = line_width;

    // Filaments the user has: the ones loaded in the plate's slots first, then every visible preset
    // that fits the printer (the filaments ticked in Orca's filament selection).
    nlohmann::json        filaments = nlohmann::json::array();
    std::set<std::string> seen;
    auto add = [&](const Preset& p) {
        if (p.is_default || !seen.insert(p.name).second || filaments.size() >= 120)
            return;
        filaments.push_back({{"preset", p.name},
                             {"type", first_string(p.config, "filament_type")},
                             {"vendor", first_string(p.config, "filament_vendor")},
                             {"density", first_float(p.config, "filament_density", 0.0)},
                             {"max_volumetric_speed", first_float(p.config, "filament_max_volumetric_speed", 0.0)},
                             {"loaded", false}});
    };
    for (const std::string& name : bundle->filament_presets)
        if (const Preset* p = bundle->filaments.find_preset(name, false)) {
            add(*p);
            if (!filaments.empty())
                filaments.back()["loaded"] = true;
        }
    for (const Preset& p : bundle->filaments)
        if (p.is_visible && p.is_compatible)
            add(p);

    return {{"printer", printer}, {"structure", structure}, {"filaments", filaments}, {"object_name", m_object_name}};
}

void SmartPrintDialog::on_script_message(const nlohmann::json& payload)
{
    if (handle_common_script_command(payload))
        return;
    // Out of the native webview callback before touching windows (see PluginsConfigDialog).
    wxGetApp().CallAfter([this, alive = m_alive, payload]() {
        if (alive->load(std::memory_order_acquire))
            handle_web_command(payload);
    });
}

void SmartPrintDialog::handle_web_command(const nlohmann::json& payload)
{
    const std::string command = payload.value("command", "");
    if (command == "ready") {
        if (!m_session_ok) {
            post_to_page({{"command", "init_failed"}, {"error", m_session_error}});
            return;
        }
        enqueue({{"cmd", "__init"}});
    } else if (command == "smartprint_request" && payload.contains("request")) {
        enqueue(payload["request"]);
    } else if (command == "apply") {
        apply_settings(payload);
    }
}

void SmartPrintDialog::enqueue(nlohmann::json request)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push_back(std::move(request));
    }
    m_cv.notify_one();
}

void SmartPrintDialog::post_to_page(nlohmann::json message)
{
    wxGetApp().CallAfter([this, alive = m_alive, message = std::move(message)]() {
        if (alive->load(std::memory_order_acquire))
            call_web_handler(message);
    });
}

void SmartPrintDialog::worker_loop()
{
    for (;;) {
        nlohmann::json request;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [this]() { return m_stop || !m_queue.empty(); });
            if (m_stop)
                return;
            request = std::move(m_queue.front());
            m_queue.pop_front();
        }
        if (request.value("cmd", "") == "__init") {
            post_to_page({{"command", "init"},
                          {"libraries", m_session.handle({{"cmd", "libraries"}})},
                          {"mesh", m_session.mesh_json()},
                          {"object_name", m_object_name}});
            continue;
        }
        const nlohmann::json id = request.value("request_id", nlohmann::json());
        auto last_post = std::chrono::steady_clock::now();
        nlohmann::json result = m_session.handle(request, [&](double value) {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_post > std::chrono::milliseconds(150) || value >= 1.0) {
                last_post = now;
                post_to_page({{"command", "smartprint_progress"}, {"request_id", id}, {"value", value}});
            }
            return !m_cancel.load();
        });
        if (m_cancel.load())
            return;
        post_to_page({{"command", "smartprint_result"}, {"result", std::move(result)}});
    }
}

void SmartPrintDialog::apply_settings(const nlohmann::json& payload)
{
    ModelObject* mo = target_object();
    if (mo == nullptr) {
        post_to_page({{"command", "applied"}, {"ok", false}, {"error", _u8L("The object is no longer on the plate.")}});
        return;
    }
    const nlohmann::json s      = payload.value("settings", nlohmann::json::object());
    Plater*              plater = wxGetApp().plater();
    PresetBundle*        bundle = wxGetApp().preset_bundle;
    try {
        Plater::TakeSnapshot snapshot(plater, _u8L("Apply Smart Print settings"));
        ModelConfigObject&   config = mo->config;
        config.set_key_value("wall_loops", new ConfigOptionInt(s.value("walls", 2)));
        config.set_key_value("sparse_infill_density", new ConfigOptionPercent(s.value("infill", 0.15) * 100.0));
        auto* pattern = new ConfigOptionEnum<InfillPattern>(ipGyroid);
        pattern->deserialize(s.value("pattern", std::string("gyroid")));
        config.set_key_value("sparse_infill_pattern", pattern);
        config.set_key_value("top_shell_layers", new ConfigOptionInt(s.value("top", 4)));
        config.set_key_value("bottom_shell_layers", new ConfigOptionInt(s.value("bottom", 4)));
        // A minimum shell thickness would add layers beyond what was simulated; the layer counts rule.
        config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0));
        config.set_key_value("bottom_shell_thickness", new ConfigOptionFloat(0));

        // Filament: print the object from the slot holding the chosen preset, or put it into the
        // object's current slot.
        const std::string filament = payload.value("filament", "");
        if (!filament.empty() && bundle->filaments.find_preset(filament, false) != nullptr) {
            auto& slots = bundle->filament_presets;
            auto  it    = std::find(slots.begin(), slots.end(), filament);
            if (it != slots.end()) {
                config.set_key_value("extruder", new ConfigOptionInt(int(it - slots.begin()) + 1));
            } else {
                const ConfigOption* ext  = config.option("extruder");
                size_t              slot = ext ? size_t(std::max(1, ext->getInt()) - 1) : 0;
                slot                     = std::min(slot, slots.empty() ? size_t(0) : slots.size() - 1);
                bundle->set_filament_preset(slot, filament);
                if (slot == 0)
                    wxGetApp().get_tab(Preset::TYPE_FILAMENT)->select_preset(filament);
                plater->update_project_dirty_from_presets();
                bundle->export_selections(*wxGetApp().app_config);
                wxGetApp().sidebar().update_dynamic_filament_list();
                for (PlaterPresetComboBox* combo : wxGetApp().sidebar().combos_filament())
                    combo->update();
                plater->on_config_change(bundle->full_config());
            }
        }
    } catch (const std::exception& e) {
        post_to_page({{"command", "applied"}, {"ok", false}, {"error", e.what()}});
        return;
    }

    wxGetApp().obj_list()->object_config_options_changed({mo, nullptr});
    if (auto* tab = dynamic_cast<TabPrintModel*>(wxGetApp().get_model_tab()))
        tab->update_model_config();
    wxGetApp().params_panel()->notify_object_config_changed();
    plater->changed_object(*mo);
    post_to_page({{"command", "applied"}, {"ok", true}, {"object", m_object_name}});
}

}} // namespace Slic3r::GUI
