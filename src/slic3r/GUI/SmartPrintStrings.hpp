#ifndef slic3r_GUI_SmartPrintStrings_hpp_
#define slic3r_GUI_SmartPrintStrings_hpp_

#include <nlohmann/json.hpp>

namespace Slic3r { namespace GUI {

// Translated strings of the Smart Print assistant page, keyed as in its strings.js.
nlohmann::json smart_print_ui_strings();

}} // namespace Slic3r::GUI

#endif
