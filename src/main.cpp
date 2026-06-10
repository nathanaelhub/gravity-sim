#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "Simulation.hpp"

namespace {

// Simulation units are arbitrary: G and the masses are chosen so that
// orbital speeds land around ~1 unit/s and the system fits in a ~2x2 box.
constexpr double kG = 1.0;
constexpr double kSoftening = 0.01;
constexpr double kSunMass = 1.0;
constexpr double kPlanetMass = 1e-4;
constexpr int kSubsteps = 8;

// View half-extent in world units; the projection letterboxes to keep
// circles circular regardless of window aspect ratio.
constexpr double kViewHalfExtent = 1.2;

// Long enough to cover one full orbit of the outer planet (~350 frames).
constexpr std::size_t kTrailLength = 420;

// Circular orbital speed around a central mass M at radius r.
double circularSpeed(double centralMass, double radius) {
    return std::sqrt(kG * centralMass / radius);
}

Simulation makeThreeBodySystem() {
    Simulation sim(kG, kSoftening);

    // Sun at the origin, two planets on circular orbits. The sun gets a
    // small counter-velocity so total momentum is zero and nothing drifts.
    const double r1 = 0.45;
    const double r2 = 0.95;
    const Vector2D v1{0.0, circularSpeed(kSunMass, r1)};
    const Vector2D v2{0.0, -circularSpeed(kSunMass, r2)};
    const Vector2D sunVelocity = -(v1 * kPlanetMass + v2 * kPlanetMass) / kSunMass;

    sim.addParticle(Particle(kSunMass, {0.0, 0.0}, sunVelocity));
    sim.addParticle(Particle(kPlanetMass, {r1, 0.0}, v1));
    sim.addParticle(Particle(kPlanetMass, {-r2, 0.0}, v2));
    return sim;
}

// Chenciner–Montgomery figure-8 choreography: three equal masses chasing
// each other along one lemniscate. Initial conditions from Chenciner &
// Montgomery (2000); period ~6.33 time units. Only survives if the
// integrator is accurate, so it doubles as a correctness demo.
Simulation makeFigureEightSystem() {
    Simulation sim(kG, kSoftening);

    const Vector2D p1{0.97000436, -0.24308753};
    const Vector2D v3{-0.93240737, -0.86473146};
    const Vector2D v1 = -v3 / 2.0;

    sim.addParticle(Particle(1.0, p1, v1));
    sim.addParticle(Particle(1.0, -p1, v1));
    sim.addParticle(Particle(1.0, {0.0, 0.0}, v3));
    return sim;
}

void setProjection(GLFWwindow* window) {
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    const double aspect = height > 0 ? static_cast<double>(width) / height : 1.0;
    double halfW = kViewHalfExtent;
    double halfH = kViewHalfExtent;
    if (aspect >= 1.0) {
        halfW *= aspect;
    } else {
        halfH /= aspect;
    }

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-halfW, halfW, -halfH, halfH, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

using Trail = std::deque<Vector2D>;

void recordTrails(const Simulation& sim, std::vector<Trail>& trails) {
    trails.resize(sim.particles().size());
    for (std::size_t i = 0; i < trails.size(); ++i) {
        trails[i].push_back(sim.particles()[i].position());
        if (trails[i].size() > kTrailLength) {
            trails[i].pop_front();
        }
    }
}

struct Color {
    float r, g, b;
};

Color particleColor(std::size_t index) {
    static const Color palette[] = {
        {1.0f, 0.85f, 0.3f},   // gold
        {0.4f, 0.7f, 1.0f},    // blue
        {1.0f, 0.5f, 0.55f},   // coral
        {0.55f, 1.0f, 0.65f},  // mint
    };
    return palette[index % (sizeof(palette) / sizeof(palette[0]))];
}

// Bodies much heavier than the lightest one render larger (the "sun").
float pointSize(const Simulation& sim, const Particle& p) {
    double minMass = p.mass();
    for (const auto& other : sim.particles()) {
        minMass = std::min(minMass, other.mass());
    }
    return p.mass() > 100.0 * minMass ? 18.0f : 8.0f;
}

void render(const Simulation& sim, const std::vector<Trail>& trails) {
    glClearColor(0.02f, 0.02f, 0.06f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);

    // Trails first so the bodies draw on top. Alpha ramps from 0 at the
    // oldest point to ~0.8 at the newest, giving a comet-tail fade.
    for (std::size_t i = 0; i < trails.size(); ++i) {
        const Trail& trail = trails[i];
        if (trail.size() < 2) continue;
        const Color c = particleColor(i);

        glBegin(GL_LINE_STRIP);
        for (std::size_t k = 0; k < trail.size(); ++k) {
            const float age = static_cast<float>(k) / static_cast<float>(trail.size() - 1);
            glColor4f(c.r, c.g, c.b, 0.8f * age * age);
            glVertex2d(trail[k].x, trail[k].y);
        }
        glEnd();
    }

    for (std::size_t i = 0; i < sim.particles().size(); ++i) {
        const Particle& p = sim.particles()[i];
        const Color c = particleColor(i);
        glPointSize(pointSize(sim, p));
        glBegin(GL_POINTS);
        glColor4f(c.r, c.g, c.b, 1.0f);
        glVertex2d(p.position().x, p.position().y);
        glEnd();
    }
}

// Write the current framebuffer as an uncompressed 24-bit BMP.
// glReadPixels returns rows bottom-up, which is exactly BMP's layout.
bool saveFramebufferBMP(GLFWwindow* window, const std::string& path) {
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);

    const int rowBytes = ((width * 3 + 3) / 4) * 4;  // rows pad to 4 bytes
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(rowBytes) * height, 0);

    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // BMP stores BGR.
    for (int y = 0; y < height; ++y) {
        std::uint8_t* row = pixels.data() + static_cast<std::size_t>(y) * rowBytes;
        for (int x = 0; x < width; ++x) {
            std::swap(row[x * 3], row[x * 3 + 2]);
        }
    }

    const std::uint32_t dataSize = static_cast<std::uint32_t>(pixels.size());
    const std::uint32_t fileSize = 54 + dataSize;

    std::uint8_t header[54] = {};
    header[0] = 'B';
    header[1] = 'M';
    std::memcpy(header + 2, &fileSize, 4);
    const std::uint32_t dataOffset = 54, infoSize = 40;
    std::memcpy(header + 10, &dataOffset, 4);
    std::memcpy(header + 14, &infoSize, 4);
    std::memcpy(header + 18, &width, 4);
    std::memcpy(header + 22, &height, 4);
    const std::uint16_t planes = 1, bpp = 24;
    std::memcpy(header + 26, &planes, 2);
    std::memcpy(header + 28, &bpp, 2);
    std::memcpy(header + 34, &dataSize, 4);

    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out.write(reinterpret_cast<const char*>(header), sizeof(header));
    out.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
    return out.good();
}

}  // namespace

int main(int argc, char** argv) {
    // --preset orbit|figure8 selects the initial system.
    // --frames N --out file.bmp: run N fixed-dt frames, save a
    // screenshot of the final frame, and exit (for README captures).
    long captureFrames = 0;
    std::string capturePath = "screenshot.bmp";
    std::string preset = "orbit";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            captureFrames = std::strtol(argv[++i], nullptr, 10);
        } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            capturePath = argv[++i];
        } else if (std::strcmp(argv[i], "--preset") == 0 && i + 1 < argc) {
            preset = argv[++i];
        }
    }
    const bool captureMode = captureFrames > 0;
    if (preset != "orbit" && preset != "figure8") {
        std::fprintf(stderr, "Unknown preset '%s' (expected: orbit, figure8)\n", preset.c_str());
        return 1;
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    // Legacy fixed-function pipeline: request a compatibility context.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow* window = glfwCreateWindow(900, 900, "Gravity Simulation", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "Failed to create window\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(captureMode ? 0 : 1);  // vsync off when capturing

    Simulation sim = preset == "figure8" ? makeFigureEightSystem() : makeThreeBodySystem();
    std::vector<Trail> trails;

    long frame = 0;
    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        double dt;
        if (captureMode) {
            dt = 1.0 / 60.0;  // deterministic steps for reproducible captures
        } else {
            const double now = glfwGetTime();
            // Clamp the frame delta so a paused/dragged window doesn't blow
            // up the integrator when time "jumps" on resume.
            dt = std::min(now - lastTime, 1.0 / 30.0);
            lastTime = now;
        }

        sim.step(dt, kSubsteps);
        recordTrails(sim, trails);

        setProjection(window);
        render(sim, trails);

        glfwSwapBuffers(window);
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        if (captureMode && ++frame >= captureFrames) {
            // Re-render after the swap so the back buffer we read is the
            // final frame, not the previous one.
            render(sim, trails);
            if (saveFramebufferBMP(window, capturePath)) {
                std::printf("Saved %s after %ld frames\n", capturePath.c_str(), frame);
            } else {
                std::fprintf(stderr, "Failed to write %s\n", capturePath.c_str());
            }
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
