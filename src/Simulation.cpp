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

// Leapfrog / velocity Verlet in kick-drift-kick form:
//
//     v += a(x) * h/2      half kick with the current forces
//     x += v * h           full drift
//     v += a(x') * h/2     half kick with the forces at the new positions
//
// Second-order accurate (semi-implicit Euler, used before, is first order)
// and still symplectic and time-reversible, so energy errors stay bounded
// instead of drifting. The forces computed at the end of one substep are
// exactly the ones the next substep starts with, so they are cached and each
// substep costs a single O(n^2) force pass — the same as Euler.
void Simulation::step(double dt, int substeps) {
    if (substeps < 1) substeps = 1;
    const double h = dt / substeps;

    if (!forcesValid_) {
        computeForces();
        forcesValid_ = true;
    }
    for (int s = 0; s < substeps; ++s) {
        for (auto& p : particles_) {
            p.kick(0.5 * h);
            p.drift(h);
        }
        computeForces();
        for (auto& p : particles_) {
            p.kick(0.5 * h);
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
