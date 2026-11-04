#include "ui/ui_context.h"
#include <GLFW/glfw3.h>
#include <fstream>
#include <iostream>
#include "IconsFontAwesome7.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"
#include "implot.h"
#include "ui/theme.h"

namespace cosmos {
namespace ui {

namespace {

bool fileExists(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

} // namespace

bool UiContext::loadFonts(const UiConfig& config)
{
    const std::string dir = config.fontDir + "/";
    const std::string regular = dir + "Roboto-Regular.ttf";
    const std::string medium = dir + "Roboto-Medium.ttf";
    const std::string icons = dir + "fa-solid-900.otf";
    if (!fileExists(regular) || !fileExists(medium) || !fileExists(icons)) {
        std::cerr << "UI: fonts not found in '" << config.fontDir << "'; using ImGui's built-in font (icons will not show)\n";
        return false;
    }

    ImGuiIO& io = ImGui::GetIO();
    const float px = config.baseFontPx;
    // Font Awesome Solid merged into each Roboto face. Icons are drawn a little smaller than the text and given one fixed
    // advance, so a row of icon buttons lines up; the Font Awesome range starts above ASCII, so letters stay Roboto.
    auto addFace = [&](const std::string& textFont) {
        ImFont* f = io.Fonts->AddFontFromFileTTF(textFont.c_str(), px);
        ImFontConfig merge;
        merge.MergeMode = true;
        merge.GlyphMinAdvanceX = px * 1.1f;
        merge.GlyphOffset = ImVec2(0.0f, 0.5f);
        merge.OversampleH = 2;
        io.Fonts->AddFontFromFileTTF(icons.c_str(), px * 0.9f, &merge);
        return f;
    };
    ImFont* body = addFace(regular);
    fontMedium_ = addFace(medium);
    if (!body || !fontMedium_) return false;
    io.FontDefault = body;
    return true;
}

bool UiContext::init(GLFWwindow* window, const UiConfig& config)
{
    if (initialised_) return true;
    if (!window) return false;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;   // no imgui.ini in the working folder; window layout is code-driven for now

    window_ = window;
    dpiScale_ = config.scaleOverride > 0.0f ? config.scaleOverride : ImGui_ImplGlfw_GetContentScaleForWindow(window);
    if (!(dpiScale_ > 0.25f && dpiScale_ < 8.0f)) dpiScale_ = 1.0f;

    fontsLoaded_ = loadFonts(config);
    applyTheme(dpiScale_);
    ImGui::GetStyle().FontScaleDpi = dpiScale_;
    ImPlot::GetStyle().PlotDefaultSize = ImVec2(0.0f, 0.0f);

    // install_callbacks = true: ImGui saves the callbacks installed so far (the app's) and calls them after its own.
    backendGlfw_ = ImGui_ImplGlfw_InitForOpenGL(window, true);
    backendGl_ = backendGlfw_ && ImGui_ImplOpenGL3_Init("#version 450");
    if (!backendGl_) {
        std::cerr << "UI: ImGui backend initialisation failed\n";
        shutdown();
        return false;
    }
    initialised_ = true;
    return true;
}

void UiContext::shutdown()
{
    if (backendGl_) ImGui_ImplOpenGL3_Shutdown();
    if (backendGlfw_) ImGui_ImplGlfw_Shutdown();
    backendGl_ = backendGlfw_ = false;
    if (ImGui::GetCurrentContext()) {
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
    }
    initialised_ = false;
    fontMedium_ = nullptr;
}

void UiContext::setScale(float scale)
{
    if (!initialised_ || !window_) return;
    float s = scale > 0.0f ? scale : ImGui_ImplGlfw_GetContentScaleForWindow(window_);
    if (!(s > 0.25f && s < 8.0f)) s = 1.0f;
    if (s == dpiScale_) return;
    dpiScale_ = s;
    applyTheme(dpiScale_);
    ImGui::GetStyle().FontScaleDpi = dpiScale_;
}

void UiContext::newFrame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void UiContext::render()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void UiContext::injectMouse(GLFWwindow* window, double x, double y, int button, int action)
{
    if (!initialised_) return;
    ImGui_ImplGlfw_CursorPosCallback(window, x, y);
    if (button >= 0) ImGui_ImplGlfw_MouseButtonCallback(window, button, action, 0);
}

bool UiContext::wantsMouse() const { return initialised_ && ImGui::GetIO().WantCaptureMouse; }
bool UiContext::wantsKeyboard() const { return initialised_ && ImGui::GetIO().WantCaptureKeyboard; }

} // namespace ui
} // namespace cosmos
