#include "Simulation.hpp"

void Simulation::computeForces() {
    for (auto& p : particles_) {
        p.resetAcceleration();
    }

    // Each unordered pair is visited once; Newton's third law gives the
    // equal-and-opposite force for free.
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        for (std::size_t j = i + 1; j < particles_.size(); ++j) {
            Particle& a = particles_[i];
            Particle& b = particles_[j];

            const Vector2D delta = b.position() - a.position();

            // Plummer softening: r^2 -> r^2 + eps^2 bounds the force as
            // r -> 0, so overlapping particles never divide by zero.
            const double distSq = delta.lengthSquared() + softening_ * softening_;
            const double dist = std::sqrt(distSq);

            const double forceMag = G_ * a.mass() * b.mass() / distSq;
            const Vector2D force = delta * (forceMag / dist);

            a.applyForce(force);
            b.applyForce(-force);
        }
    }
}

void Simulation::step(double dt, int substeps) {
    if (substeps < 1) substeps = 1;
    const double h = dt / substeps;

    for (int s = 0; s < substeps; ++s) {
        computeForces();
        for (auto& p : particles_) {
            p.integrate(h);
        }
    }
}
