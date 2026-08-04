//
// test_physics.cpp — unit tests for the simulation core (no OpenGL).
//
// Only the physics is exercised here — Vector2D, Particle, and Simulation —
// so the suite builds and runs headless, without GLFW/OpenGL. A tiny
// CHECK/CHECK_NEAR harness keeps it dependency-free.
//
//   c++ -std=c++17 -Iinclude tests/test_physics.cpp src/Simulation.cpp -o tests
//
#include "Simulation.hpp"
#include "Vector2D.hpp"

#include <cmath>
#include <cstdio>

static int g_checks = 0;
static int g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                      \
        ++g_checks;                                                           \
        if (!(cond)) {                                                        \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
            ++g_fail;                                                         \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                      \
        ++g_checks;                                                           \
        double da = (a), db = (b);                                            \
        if (std::fabs(da - db) > (tol)) {                                     \
            std::printf("FAIL %s:%d  %s ~= %s  (%.9g vs %.9g)\n", __FILE__,   \
                        __LINE__, #a, #b, da, db);                            \
            ++g_fail;                                                         \
        }                                                                     \
    } while (0)

// ---------------------------------------------------------------- Vector2D
static void test_vector() {
    Vector2D a{3.0, 4.0}, b{1.0, 2.0};
    CHECK((a + b).x == 4.0 && (a + b).y == 6.0);
    CHECK((a - b).x == 2.0 && (a - b).y == 2.0);
    CHECK((a * 2.0).x == 6.0 && (2.0 * a).y == 8.0);
    CHECK((-a).x == -3.0 && (-a).y == -4.0);
    CHECK_NEAR(a.length(), 5.0, 1e-12);
    CHECK_NEAR(a.lengthSquared(), 25.0, 1e-12);
    CHECK_NEAR(a.normalized().length(), 1.0, 1e-12);
    CHECK(Vector2D{}.normalized().x == 0.0); // zero-safe
    CHECK_NEAR(Vector2D::dot(a, b), 11.0, 1e-12);
    CHECK_NEAR(Vector2D::distance({0, 0}, {3, 4}), 5.0, 1e-12);
}

// ---------------------------------------------------------------- Particle
static void test_particle_dynamics() {
    Particle p{2.0, {0, 0}, {0, 0}};
    p.applyForce({4.0, 0.0});            // a = F/m = 2
    CHECK_NEAR(p.acceleration().x, 2.0, 1e-12);
    p.applyForce({0.0, 2.0});            // accumulates
    CHECK_NEAR(p.acceleration().y, 1.0, 1e-12);
    p.resetAcceleration();
    CHECK(p.acceleration().x == 0.0 && p.acceleration().y == 0.0);

    // semi-implicit Euler: v += a*dt, THEN x += v*dt
    Particle q{1.0, {0, 0}, {0, 0}};
    q.applyForce({1.0, 0.0});            // a = 1
    q.integrate(1.0);
    CHECK_NEAR(q.velocity().x, 1.0, 1e-12); // v = 0 + 1*1
    CHECK_NEAR(q.position().x, 1.0, 1e-12); // x = 0 + 1*1  (uses the NEW v)
}

// ---------------------------------------------------------------- Simulation
static Vector2D totalMomentum(const Simulation& s) {
    Vector2D p;
    for (const auto& b : s.particles()) p += b.mass() * b.velocity();
    return p;
}

static void test_two_bodies_attract() {
    Simulation s{1.0, 0.01};
    s.addParticle(Particle{1.0, {-1.0, 0.0}, {0, 0}});
    s.addParticle(Particle{1.0, {1.0, 0.0}, {0, 0}});
    s.step(0.01);
    // left body accelerates +x (toward the right body) and vice-versa
    CHECK(s.particles()[0].velocity().x > 0.0);
    CHECK(s.particles()[1].velocity().x < 0.0);
}

static void test_momentum_is_conserved() {
    Simulation s{1.0, 0.05};
    s.addParticle(Particle{2.0, {-3.0, 0.0}, {0.0, 0.5}});
    s.addParticle(Particle{5.0, {2.0, 1.0}, {0.1, 0.0}});
    s.addParticle(Particle{1.0, {0.0, -2.0}, {-0.2, 0.3}});
    Vector2D before = totalMomentum(s);
    for (int i = 0; i < 2000; ++i) s.step(0.005);
    Vector2D after = totalMomentum(s);
    // internal forces are equal/opposite -> total momentum is invariant
    CHECK_NEAR(after.x, before.x, 1e-9);
    CHECK_NEAR(after.y, before.y, 1e-9);
}

static void test_softening_bounds_force() {
    // two particles at (almost) the same point must not blow up to inf/nan
    Simulation s{1.0, 0.1};
    s.addParticle(Particle{1.0, {0.0, 0.0}, {0, 0}});
    s.addParticle(Particle{1.0, {0.0, 0.0}, {0, 0}});
    s.step(0.01);
    for (const auto& b : s.particles()) {
        CHECK(std::isfinite(b.velocity().x) && std::isfinite(b.velocity().y));
    }
}

static void test_circular_orbit_stays_bounded() {
    // heavy central mass + orbiter on a circular orbit: v = sqrt(G M / r).
    // semi-implicit Euler should keep the radius bounded (explicit Euler
    // would spiral outward). We only assert it neither collapses nor escapes.
    const double G = 1.0, M = 1000.0, r = 10.0;
    const double v = std::sqrt(G * M / r);
    Simulation s{G, 0.001};
    s.addParticle(Particle{M, {0.0, 0.0}, {0.0, 0.0}});   // central
    s.addParticle(Particle{1.0, {r, 0.0}, {0.0, v}});     // orbiter

    double dMin = r, dMax = r;
    for (int i = 0; i < 4000; ++i) {
        s.step(0.005);
        double d = Vector2D::distance(s.particles()[0].position(),
                                      s.particles()[1].position());
        dMin = std::min(dMin, d);
        dMax = std::max(dMax, d);
    }
    CHECK(dMin > 0.5 * r); // didn't collapse inward
    CHECK(dMax < 2.0 * r); // didn't spiral away
}

int main() {
    test_vector();
    test_particle_dynamics();
    test_two_bodies_attract();
    test_momentum_is_conserved();
    test_softening_bounds_force();
    test_circular_orbit_stays_bounded();

    std::printf("\n%d checks, %d failed\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
