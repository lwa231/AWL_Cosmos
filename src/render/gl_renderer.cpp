#include "render/renderer.h"
#include "render/gl_shader.h"
#include "render/gl_stream.h"
#include <glad/glad.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace cosmos {
namespace render {

namespace {

// ---- GPU-side layouts. Each must match the GLSL declarations (shaders/*.glsl, *.vert) ----------------------------

// std140 uniform block "FrameData" (shaders/common.glsl)
struct FrameUbo {
    float proj[16];       // mat4, column major: centre-origin pixels -> clip
    float viewport[4];    // width, height, 1/width, 1/height
    float scale[4];       // metres per pixel, pixels per metre, time, minimum body diameter in pixels
    std::uint32_t counts[4];   // bodies, lights, shuttles, 0
    float shuttle[4];     // hull length in metres, minimum hull length in pixels, 0, 0
};
static_assert(sizeof(FrameUbo) == 64 + 16 * 4, "FrameUbo must match the std140 block");

// std430 storage buffer entries
struct BodyGpu {          // 32 bytes
    float x, y, radius, ambient;
    float r, g, b;
    std::uint32_t flags;  // bit 0 flat, bit 1 black hole
};
struct LightGpu {         // 32 bytes
    float x, y, intensity, pad0;
    float r, g, b, pad1;
};
struct ShuttleGpu {       // 64 bytes: pose, then three colour rows
    float x, y, heading, throttle;
    float c0[3], flicker;
    float c1[3], pad1;
    float c2[3], pad2;
};
struct SegmentGpu {       // 64 bytes
    float x0, y0, x1, y1;
    float c0[4];
    float c1[4];
    float width, pad[3];
};
static_assert(sizeof(BodyGpu) == 32 && sizeof(LightGpu) == 32, "body/light layout");
static_assert(sizeof(ShuttleGpu) == 64 && sizeof(SegmentGpu) == 64, "shuttle/segment layout");

constexpr std::uint32_t kBodyFlat = 1u;
constexpr std::uint32_t kBodyBlackHole = 2u;

// Shader binding points (match layout(binding = N) in the shaders)
constexpr GLuint kBindFrame = 0;
constexpr GLuint kBindBodies = 0;
constexpr GLuint kBindLights = 1;
constexpr GLuint kBindShuttles = 2;
constexpr GLuint kBindSegments = 3;

constexpr std::size_t kShuttleVertices = 9;   // 6 flame + 3 hull per instance
constexpr float kTimeWrapSeconds = 3600.0f;   // keeps the float time precise; flicker is visual only

// KHR_debug callback (core since 4.3)
void APIENTRY debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message,
                            const void* userParam)
{
    (void)source; (void)length;
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    RenderTotals* totals = const_cast<RenderTotals*>(static_cast<const RenderTotals*>(userParam));
    const bool isError = type == GL_DEBUG_TYPE_ERROR || type == GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR;
    if (totals) {
        if (isError) ++totals->glErrors; else ++totals->glWarnings;
    }
    std::cerr << "[GL " << (isError ? "ERROR" : "warning") << " id=" << id << " type=0x" << std::hex << type << std::dec
              << "] " << message << "\n";
}

std::size_t roundUp(std::size_t v, std::size_t a) { return (v + a - 1) / a * a; }

class GlRenderer final : public Renderer {
public:
    ~GlRenderer() override { shutdown(); }

    bool init(const RendererConfig& config) override;
    void shutdown() override;
    void resize(int w, int h) override;
    void beginFrame(const app::RenderFrame& frame) override;
    void drawBodies() override;
    void drawShuttles() override;
    void drawLines(LinePass pass) override;
    void endFrame() override;
    const RenderStats& frameStats() const override { return stats_; }
    const RenderTotals& totals() const override { return totals_; }
    bool readBackBuffer(std::vector<std::uint8_t>& rgb, int& width, int& height) override;

private:
    struct Range {
        std::size_t offset = 0;
        std::size_t bytes = 0;
        std::uint32_t count = 0;
    };

    void fillLines(const app::LineBatch& batch, SegmentGpu* out) const;
    std::size_t alignment_ = 256;

    bool initialised_ = false;
    bool debugCallbackSet_ = false;
    unsigned int progBodies_ = 0, progShuttles_ = 0, progLines_ = 0;
    unsigned int vao_ = 0;     // core profile needs a VAO bound even though all data is pulled from storage buffers
    unsigned int frameUbo_ = 0;
    StreamArena arena_;

    const app::RenderFrame* frame_ = nullptr;
    bool inFrame_ = false;
    int width_ = 0, height_ = 0;
    Range bodies_, lights_, shuttles_, lines_[2];
    RenderStats stats_;
    RenderTotals totals_;
    std::uint32_t lastStalls_ = 0;   // arena stall count already folded into totals_
};

bool GlRenderer::init(const RendererConfig& config)
{
    shutdown();

    if (!GLAD_GL_VERSION_4_5) {
        std::cerr << "Renderer: OpenGL 4.5 is required (got " << glGetString(GL_VERSION) << ")\n";
        return false;
    }
    if (config.debugOutput) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);   // errors are reported at the offending call
        glDebugMessageCallback(debugCallback, &totals_);
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
        // glReadPixels of the back buffer (the --screenshot developer option) triggers this performance note; it is expected
        const GLuint pixelPathNote = 131154;
        glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_PERFORMANCE, GL_DONT_CARE, 1, &pixelPathNote, GL_FALSE);
        debugCallbackSet_ = true;
    }

    progBodies_ = buildProgram(config.shaderDir, "body.vert", "body.frag");
    progShuttles_ = buildProgram(config.shaderDir, "shuttle.vert", "shuttle.frag");
    progLines_ = buildProgram(config.shaderDir, "line.vert", "line.frag");
    if (!progBodies_ || !progShuttles_ || !progLines_) {
        shutdown();
        return false;
    }

    GLint uboAlign = 256, ssboAlign = 256;
    glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &uboAlign);
    glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &ssboAlign);
    alignment_ = static_cast<std::size_t>(std::max<GLint>(std::max(uboAlign, ssboAlign), 16));
    if (alignment_ > 256) {
        std::cerr << "Renderer: buffer offset alignment " << alignment_ << " is larger than expected\n";
    }

    glCreateVertexArrays(1, &vao_);
    // Starts at 1 MiB per segment (3 MiB mapped); grows by doubling if a frame needs more.
    if (!arena_.init(std::size_t(1) << 20)) {
        shutdown();
        return false;
    }

    glEnable(GL_MULTISAMPLE);   // no-op unless the window has a multisampled framebuffer (hull triangle edges)
    initialised_ = true;
    return true;
}

void GlRenderer::shutdown()
{
    if (debugCallbackSet_) glDebugMessageCallback(nullptr, nullptr);   // the callback points at totals_
    debugCallbackSet_ = false;
    arena_.destroy();
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (progBodies_) glDeleteProgram(progBodies_);
    if (progShuttles_) glDeleteProgram(progShuttles_);
    if (progLines_) glDeleteProgram(progLines_);
    vao_ = progBodies_ = progShuttles_ = progLines_ = 0;
    initialised_ = false;
    inFrame_ = false;
    frame_ = nullptr;
}

void GlRenderer::resize(int w, int h)
{
    width_ = w;
    height_ = h;
    if (initialised_ && w > 0 && h > 0) glViewport(0, 0, w, h);
}

void GlRenderer::fillLines(const app::LineBatch& batch, SegmentGpu* out) const
{
    auto put = [&out](const app::LineVertex& a, const app::LineVertex& b, float width) {
        SegmentGpu& s = *out++;
        s.x0 = a.x; s.y0 = a.y; s.x1 = b.x; s.y1 = b.y;
        s.c0[0] = a.r; s.c0[1] = a.g; s.c0[2] = a.b; s.c0[3] = a.a;
        s.c1[0] = b.r; s.c1[1] = b.g; s.c1[2] = b.b; s.c1[3] = b.a;
        s.width = width; s.pad[0] = s.pad[1] = s.pad[2] = 0.0f;
    };
    for (const app::LineStrip& strip : batch.strips) {
        if (strip.count < 2 || std::size_t(strip.first) + strip.count > batch.vertices.size()) continue;
        for (std::uint32_t i = 0; i + 1 < strip.count; ++i)
            put(batch.vertices[strip.first + i], batch.vertices[strip.first + i + 1], strip.widthPx);
    }
    for (const app::LineSegment& seg : batch.segments) put(seg.a, seg.b, seg.widthPx);
}

void GlRenderer::beginFrame(const app::RenderFrame& frame)
{
    if (!initialised_) return;
    frame_ = &frame;
    inFrame_ = false;
    stats_ = RenderStats{};
    width_ = frame.viewportWidth;
    height_ = frame.viewportHeight;

    // ---- size the frame's slice of the streaming buffer, then (re)acquire a segment ----
    const std::size_t nBodies = frame.bodies.size();
    const std::size_t nLights = frame.lights.size();
    const std::size_t nShuttles = frame.shuttles.size();
    const std::size_t nUnder = frame.underlay.segmentCount();
    const std::size_t nOver = frame.overlay.segmentCount();

    const std::size_t szFrame = sizeof(FrameUbo);
    const std::size_t szBodies = std::max<std::size_t>(nBodies, 1) * sizeof(BodyGpu);       // >= 1 element so every binding is valid
    const std::size_t szLights = std::max<std::size_t>(nLights, 1) * sizeof(LightGpu);
    const std::size_t szShuttles = std::max<std::size_t>(nShuttles, 1) * sizeof(ShuttleGpu);
    const std::size_t szUnder = std::max<std::size_t>(nUnder, 1) * sizeof(SegmentGpu);
    const std::size_t szOver = std::max<std::size_t>(nOver, 1) * sizeof(SegmentGpu);
    const std::size_t total = roundUp(szFrame, alignment_) + roundUp(szBodies, alignment_) + roundUp(szLights, alignment_) +
                              roundUp(szShuttles, alignment_) + roundUp(szUnder, alignment_) + roundUp(szOver, alignment_);

    if (!arena_.begin(total)) {
        std::cerr << "Renderer: cannot acquire a streaming segment\n";
        return;
    }
    totals_.arenaGrowths = arena_.growths();
    totals_.fenceStalls = arena_.stalls();

    std::size_t offFrame = 0;
    FrameUbo* ubo = static_cast<FrameUbo*>(arena_.alloc(szFrame, alignment_, &offFrame));
    BodyGpu* bodyOut = static_cast<BodyGpu*>(arena_.alloc(szBodies, alignment_, &bodies_.offset));
    LightGpu* lightOut = static_cast<LightGpu*>(arena_.alloc(szLights, alignment_, &lights_.offset));
    ShuttleGpu* shuttleOut = static_cast<ShuttleGpu*>(arena_.alloc(szShuttles, alignment_, &shuttles_.offset));
    SegmentGpu* underOut = static_cast<SegmentGpu*>(arena_.alloc(szUnder, alignment_, &lines_[0].offset));
    SegmentGpu* overOut = static_cast<SegmentGpu*>(arena_.alloc(szOver, alignment_, &lines_[1].offset));
    if (!ubo || !bodyOut || !lightOut || !shuttleOut || !underOut || !overOut) {
        std::cerr << "Renderer: streaming segment exhausted (internal sizing error)\n";
        return;
    }
    bodies_.bytes = szBodies;       bodies_.count = static_cast<std::uint32_t>(nBodies);
    lights_.bytes = szLights;       lights_.count = static_cast<std::uint32_t>(nLights);
    shuttles_.bytes = szShuttles;   shuttles_.count = static_cast<std::uint32_t>(nShuttles);
    lines_[0].bytes = szUnder;      lines_[0].count = static_cast<std::uint32_t>(nUnder);
    lines_[1].bytes = szOver;       lines_[1].count = static_cast<std::uint32_t>(nOver);

    // ---- fill (sequential writes only: the mapped memory is write-combined) ----
    const float w = static_cast<float>(std::max(width_, 1));
    const float h = static_cast<float>(std::max(height_, 1));
    const float mpp = frame.metresPerPixel > 0.0f ? frame.metresPerPixel : 1.0f;
    FrameUbo u{};
    u.proj[0] = 2.0f / w;     // x_clip = x_px * 2 / width
    u.proj[5] = 2.0f / h;     // y_clip = y_px * 2 / height
    u.proj[10] = 1.0f;
    u.proj[15] = 1.0f;
    u.viewport[0] = w; u.viewport[1] = h; u.viewport[2] = 1.0f / w; u.viewport[3] = 1.0f / h;
    u.scale[0] = mpp;
    u.scale[1] = 1.0f / mpp;
    u.scale[2] = static_cast<float>(std::fmod(frame.time, static_cast<double>(kTimeWrapSeconds)));
    u.scale[3] = frame.bodyMinPixels;
    u.counts[0] = bodies_.count; u.counts[1] = lights_.count; u.counts[2] = shuttles_.count;
    u.shuttle[0] = frame.shuttleLengthMetres;
    u.shuttle[1] = frame.shuttleMinPixels;
    std::memcpy(ubo, &u, sizeof(u));

    for (const app::RenderBody& b : frame.bodies) {
        BodyGpu g;
        g.x = b.x; g.y = b.y; g.radius = b.radius; g.ambient = b.ambient;
        g.r = b.colour.r; g.g = b.colour.g; g.b = b.colour.b;
        g.flags = ((b.emissive || !b.receivesLight) ? kBodyFlat : 0u) | (b.isBlackHole ? kBodyBlackHole : 0u);
        *bodyOut++ = g;
    }
    if (nBodies == 0) std::memset(bodyOut, 0, sizeof(BodyGpu));

    for (const app::RenderLight& l : frame.lights) {
        LightGpu g{};
        g.x = l.x; g.y = l.y; g.intensity = l.intensity;
        g.r = l.colour.r; g.g = l.colour.g; g.b = l.colour.b;
        *lightOut++ = g;
    }
    if (nLights == 0) std::memset(lightOut, 0, sizeof(LightGpu));

    for (const app::RenderShuttle& s : frame.shuttles) {
        ShuttleGpu g{};
        g.x = s.x; g.y = s.y; g.heading = s.heading; g.throttle = s.throttle;
        g.c0[0] = s.sector[0].r; g.c0[1] = s.sector[0].g; g.c0[2] = s.sector[0].b; g.flicker = s.flickerPhase;
        g.c1[0] = s.sector[1].r; g.c1[1] = s.sector[1].g; g.c1[2] = s.sector[1].b;
        g.c2[0] = s.sector[2].r; g.c2[1] = s.sector[2].g; g.c2[2] = s.sector[2].b;
        *shuttleOut++ = g;
    }
    if (nShuttles == 0) std::memset(shuttleOut, 0, sizeof(ShuttleGpu));

    fillLines(frame.underlay, underOut);
    if (nUnder == 0) std::memset(underOut, 0, sizeof(SegmentGpu));
    fillLines(frame.overlay, overOut);
    if (nOver == 0) std::memset(overOut, 0, sizeof(SegmentGpu));

    stats_.bodyInstances = bodies_.count;
    stats_.lightCount = lights_.count;
    stats_.shuttleInstances = shuttles_.count;
    stats_.lineSegments = lines_[0].count + lines_[1].count;
    stats_.streamedBytes = arena_.usedBytes();

    // ---- GL state for the frame ----
    glViewport(0, 0, width_, height_);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);    // premultiplied alpha; alpha 0 = additive (shuttle flames)
    glClearColor(frame.clearColour[0], frame.clearColour[1], frame.clearColour[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glBindVertexArray(vao_);
    glBindBufferRange(GL_UNIFORM_BUFFER, kBindFrame, arena_.buffer(), static_cast<GLintptr>(offFrame), sizeof(FrameUbo));
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, kBindBodies, arena_.buffer(), static_cast<GLintptr>(bodies_.offset), static_cast<GLsizeiptr>(bodies_.bytes));
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, kBindLights, arena_.buffer(), static_cast<GLintptr>(lights_.offset), static_cast<GLsizeiptr>(lights_.bytes));
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, kBindShuttles, arena_.buffer(), static_cast<GLintptr>(shuttles_.offset), static_cast<GLsizeiptr>(shuttles_.bytes));
    inFrame_ = true;
}

void GlRenderer::drawBodies()
{
    if (!inFrame_ || bodies_.count == 0) return;
    glUseProgram(progBodies_);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(bodies_.count));
    ++stats_.drawCalls;
    ++stats_.bodyDrawCalls;
}

void GlRenderer::drawShuttles()
{
    if (!inFrame_ || shuttles_.count == 0) return;
    glUseProgram(progShuttles_);
    glDrawArraysInstanced(GL_TRIANGLES, 0, static_cast<GLsizei>(kShuttleVertices), static_cast<GLsizei>(shuttles_.count));
    ++stats_.drawCalls;
    ++stats_.shuttleDrawCalls;
}

void GlRenderer::drawLines(LinePass pass)
{
    const Range& r = lines_[pass == LinePass::Underlay ? 0 : 1];
    if (!inFrame_ || r.count == 0) return;
    glUseProgram(progLines_);
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, kBindSegments, arena_.buffer(), static_cast<GLintptr>(r.offset), static_cast<GLsizeiptr>(r.bytes));
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(r.count));
    ++stats_.drawCalls;
    ++stats_.lineDrawCalls;
}

void GlRenderer::endFrame()
{
    if (!inFrame_) return;
    arena_.end();   // fence: this segment is not rewritten until the GPU has finished the frame's draws
    glBindVertexArray(0);
    glUseProgram(0);
    inFrame_ = false;
    ++totals_.frames;
    frame_ = nullptr;
}

bool GlRenderer::readBackBuffer(std::vector<std::uint8_t>& rgb, int& width, int& height)
{
    if (!initialised_ || width_ <= 0 || height_ <= 0) return false;
    width = width_;
    height = height_;
    const std::size_t rowBytes = static_cast<std::size_t>(width) * 3;
    std::vector<std::uint8_t> bottomUp(rowBytes * static_cast<std::size_t>(height));
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, bottomUp.data());
    rgb.resize(bottomUp.size());
    for (int y = 0; y < height; ++y)   // GL rows run bottom to top; flip to top row first
        std::memcpy(&rgb[static_cast<std::size_t>(y) * rowBytes], &bottomUp[static_cast<std::size_t>(height - 1 - y) * rowBytes], rowBytes);
    return true;
}

} // namespace

std::unique_ptr<Renderer> createGlRenderer() { return std::unique_ptr<Renderer>(new GlRenderer()); }

} // namespace render
} // namespace cosmos
