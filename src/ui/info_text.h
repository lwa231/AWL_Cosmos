#pragma once
// The text of the Settings window's Controls and Credits tabs, kept in one place so the wording is easy to edit: change the
// tables in info_text.cpp and nothing else. (The Controls table mirrors keyCallback in main.cpp: change both together.)
#include <vector>

namespace cosmos {
namespace ui {

// ---- Controls tab: a read-only list of the key and mouse bindings ----
struct Binding {
    const char* keys;     // "Space", "Shift + F", "Mouse wheel"
    const char* action;
};
struct BindingGroup {
    const char* title;
    std::vector<Binding> bindings;
};
const std::vector<BindingGroup>& controlGroups();

// ---- Credits tab ----
struct CreditEntry {
    const char* name;      // "Dear ImGui 1.93.0"
    const char* licence;   // "MIT"
    const char* what;      // what it is used for / who made it
};
struct CreditGroup {
    const char* title;                  // "Libraries"
    std::vector<CreditEntry> entries;
    const char* note = nullptr;         // attribution text shown under the group (licence-required wording), or null
};
const std::vector<CreditGroup>& creditGroups();
// "Development assistance: Claude (Anthropic)"
const char* developmentCredit();
// Pointer to the full licence texts, shown at the bottom of the tab
const char* licenceFileNote();

} // namespace ui
} // namespace cosmos
