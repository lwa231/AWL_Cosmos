#pragma once
// Dear ImGui + ImPlot plumbing (plan sections 9.4, 10.4, M6): context, GLFW and OpenGL3 backends, fonts, DPI scale.
//
// Order in the app (plan 10.4):
//   1. the app installs ITS GLFW callbacks (scroll, cursor, mouse button, key, framebuffer size);
//   2. UiContext::init() installs ImGui's callbacks on top: ImGui handles every event and chains to the app's;
//   3. each frame:  newFrame(), build widgets, then render() AFTER the scene passes and before the buffer swap;
//   4. the app gates its own input on wantsMouse() / wantsKeyboard() (they describe the state of the last newFrame()).
// The OpenGL3 backend saves and restores the GL state it touches (blend, scissor, program, VAO, buffer bindings), so the
// renderer's per-frame state set-up is not disturbed.
#include <string>

struct GLFWwindow;
struct ImFont;

namespace cosmos {
namespace ui {

struct UiConfig {
    std::string fontDir = "assets/fonts";   // Roboto-Regular.ttf, Roboto-Medium.ttf, fa-solid-900.otf (next to the exe)
    float baseFontPx = 16.0f;               // text size at DPI scale 1
    float scaleOverride = 0.0f;             // > 0: use this DPI scale instead of the window's content scale
};

class UiContext {
public:
    UiContext() = default;
    UiContext(const UiContext&) = delete;
    UiContext& operator=(const UiContext&) = delete;
    ~UiContext() { shutdown(); }

    // Needs a current GL 4.5 core context. Installs the ImGui callbacks (so call it after the app's own). false on failure.
    bool init(GLFWwindow* window, const UiConfig& config = {});
    void shutdown();
    bool initialised() const { return initialised_; }

    void newFrame();   // backend frames + ImGui::NewFrame()
    void render();     // ImGui::Render() + draw into the bound framebuffer

    float dpiScale() const { return dpiScale_; }
    bool fontsLoaded() const { return fontsLoaded_; }   // false: ImGui's built-in font is used and icons show as '?'
    ImFont* fontMedium() const { return fontMedium_; }  // Roboto Medium + icons, for emphasis

    // Verification aid: feeds a mouse event through ImGui's GLFW callbacks, which chain to the app's callbacks exactly like
    // a real event. button < 0 only moves the cursor to (x, y) (window units); otherwise action is GLFW_PRESS/GLFW_RELEASE.
    void injectMouse(GLFWwindow* window, double x, double y, int button, int action);

    // What the UI wants for itself; valid after newFrame(). Gate camera input on these (plan 10.4).
    bool wantsMouse() const;
    bool wantsKeyboard() const;

private:
    bool loadFonts(const UiConfig& config);

    bool initialised_ = false;
    bool backendGlfw_ = false;
    bool backendGl_ = false;
    bool fontsLoaded_ = false;
    float dpiScale_ = 1.0f;
    ImFont* fontMedium_ = nullptr;
};

} // namespace ui
} // namespace cosmos
