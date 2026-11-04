#include "ui/info_text.h"

// Edit the wording of the Controls and Credits tabs here. The licences and versions match THIRD_PARTY_LICENSES.md.

namespace cosmos {
namespace ui {

const std::vector<BindingGroup>& controlGroups()
{
    static const std::vector<BindingGroup> groups = {
        {"View",
         {{"Mouse wheel", "Zoom in and out at the cursor"},
          {"Left or right mouse drag", "Pan the view"},
          {"Click on an object", "Follow it and select it in the editor"},
          {"+ and -", "Zoom in and out (the keypad keys work too)"},
          {"Page Up and Page Down", "Zoom in and out in bigger steps"},
          {"H or Home", "Fit the whole system in the window"}}},
        {"Time",
         {{"Space", "Start, pause or resume the simulation"},
          {". (full stop)", "Raise the time rate one step up the ladder (1x, 10x ... 1,000,000x)"},
          {", (comma)", "Lower the time rate one step"},
          {"/", "Back to real time (1x)"}}},
        {"Following and frames",
         {{"1 to 9", "Follow the n-th body of the scene (in Lucerna, 4 is Domus)"},
          {"F or Tab", "Follow the next object; hold Shift for the previous one"},
          {"0", "Inertial reference frame"},
          {"B", "Barycentre reference frame"},
          {"R", "Frame relative to the followed body"}}},
        {"Menus",
         {{"Esc", "Stops following the followed object. With nothing followed, opens the pause menu (Resume, Settings, Return to menu, Quit)"},
          {"Esc in the pause menu", "Resume (closes Settings first when it is open)"},
          {"Ctrl + Q", "Quit"}}},
        {"Text fields",
         {{"Enter", "Apply the value typed in an editor field"},
          {"Tab, click elsewhere", "Leave the field (the value is applied when it is valid)"}}},
    };
    return groups;
}

const std::vector<CreditGroup>& creditGroups()
{
    static const std::vector<CreditGroup> groups = {
        {"Libraries",
         {{"Dear ImGui 1.93.0", "MIT", "User interface (Omar Cornut and contributors)"},
          {"ImPlot 1.0", "MIT", "Charts (Evan Pezent and contributors)"},
          {"nlohmann/json 3.12.0", "MIT", "Scene and settings files (Niels Lohmann)"},
          {"doctest 2.5.3", "MIT", "Unit tests (Viktor Kirilov)"},
          {"GLFW", "zlib/libpng", "Window and input (the GLFW developers)"},
          {"GLAD 0.1.36", "Khronos API registry output", "OpenGL 4.5 loader"},
          {"IconFontCppHeaders", "zlib", "Icon name constants (Juliette Foucaut)"}},
         nullptr},
        {"Fonts and icons",
         {{"Roboto 3.016", "SIL Open Font License 1.1", "Text (Google)"},
          {"Font Awesome Free 7.3.1, Solid", "Icons CC BY 4.0; font SIL OFL 1.1; code MIT", "Toolbar and menu icons (Fonticons, Inc.)"}},
         "Icons from Font Awesome Free by Fonticons, Inc. (https://fontawesome.com), licensed under CC BY 4.0 "
         "(https://creativecommons.org/licenses/by/4.0/). The icon font file was converted from woff2 to otf; no glyphs were changed."},
        {"Data",
         {{"JPL Horizons", "Public service", "Positions and velocities of the Real solar system preset (NASA/JPL Solar System Dynamics, ssd.jpl.nasa.gov/horizons)"}},
         nullptr},
    };
    return groups;
}

const char* developmentCredit() { return "Development assistance: Claude (Anthropic)"; }

const char* licenceFileNote() { return "The full licence texts are in THIRD_PARTY_LICENSES.md and next to each library in external/."; }

} // namespace ui
} // namespace cosmos
