#ifndef slic3r_GUI_SmartPrintDialog_hpp_
#define slic3r_GUI_SmartPrintDialog_hpp_

#include <slic3r/GUI/Widgets/WebViewHostDialog.hpp>
#include <libslic3r/SmartPrint/Session.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class wxWindow;

namespace Slic3r {
class ModelObject;
namespace GUI {

// Smart Print assistant: recommends material, walls and infill for what a part is used for, simulates
// it under realistic loads (drone motors and propellers, or the user's own supports and forces) and
// applies the chosen settings to the object. The page (resources/web/dialog/SmartPrint) does the UI;
// all engineering runs in SmartPrint::Session on a worker thread so the window stays responsive.
class SmartPrintDialog : public WebViewHostDialog
{
public:
    explicit SmartPrintDialog(wxWindow* parent);
    ~SmartPrintDialog() override;

private:
    void           on_script_message(const nlohmann::json& payload) override;
    nlohmann::json localized_strings() const override;
    void           handle_web_command(const nlohmann::json& payload);

    // Reads the selected object, printer, filaments and current settings from Orca.
    void           collect_object();
    nlohmann::json collect_context() const;
    void           apply_settings(const nlohmann::json& payload);
    ModelObject*   target_object() const;

    // Worker thread: runs Session requests one at a time and posts the results back.
    void enqueue(nlohmann::json request);
    void worker_loop();
    void post_to_page(nlohmann::json message);

    SmartPrint::Session m_session;
    ModelObject*        m_object{nullptr};
    int                 m_instance{0};
    std::string         m_object_name;
    bool                m_session_ok{false};
    std::string         m_session_error;

    std::thread                 m_worker;
    std::mutex                  m_mutex;
    std::condition_variable     m_cv;
    std::deque<nlohmann::json>  m_queue;
    bool                        m_stop{false};
    std::atomic<bool>           m_cancel{false};

    // Guards callbacks queued with CallAfter against the dialog being destroyed first.
    std::shared_ptr<std::atomic<bool>> m_alive = std::make_shared<std::atomic<bool>>(true);
};

}} // namespace Slic3r::GUI

#endif
