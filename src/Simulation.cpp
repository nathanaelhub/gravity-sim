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

double Simulation::kineticEnergy() const {
    double k = 0.0;
    for (const auto& p : particles_) {
        k += 0.5 * p.mass() * p.velocity().lengthSquared();
    }
    return k;
}

double Simulation::potentialEnergy() const {
    double u = 0.0;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
        for (std::size_t j = i + 1; j < particles_.size(); ++j) {
            const double distSq =
                Vector2D::distanceSquared(particles_[i].position(), particles_[j].position()) +
                softening_ * softening_;
            u -= G_ * particles_[i].mass() * particles_[j].mass() / std::sqrt(distSq);
        }
    }
    return u;
}

Vector2D Simulation::momentum() const {
    Vector2D total;
    for (const auto& p : particles_) {
        total += p.velocity() * p.mass();
    }
    return total;
}

double Simulation::angularMomentum() const {
    double l = 0.0;
    for (const auto& p : particles_) {
        l += p.mass() * (p.position().x * p.velocity().y - p.position().y * p.velocity().x);
    }
    return l;
}
