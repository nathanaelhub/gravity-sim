#pragma once

#include <vector>

#include "Particle.hpp"

class Simulation {
public:
    // softening: minimum effective separation, keeps the force finite
    // when two particles occupy (nearly) the same point.
    Simulation(double gravitationalConstant, double softening)
        : G_(gravitationalConstant), softening_(softening) {}

    void addParticle(const Particle& p) {
        particles_.push_back(p);
        forcesValid_ = false;  // cached accelerations no longer cover everyone
    }

    const std::vector<Particle>& particles() const { return particles_; }

    // Advance the system by dt, internally split into `substeps`
    // smaller steps. Integration is leapfrog (velocity Verlet, kick-drift-
    // kick): second-order accurate and symplectic, at one force evaluation
    // per substep.
    void step(double dt, int substeps = 1);

    // --- Conserved-quantity diagnostics ------------------------------------
    // A correct integrator should hold these (nearly) constant, so they are
    // the honest way to measure accuracy. The potential uses the same
    // Plummer softening as the force, U = -G m1 m2 / sqrt(r^2 + eps^2), so
    // the force is exactly -grad U and energy is conserved by the physics.
    double kineticEnergy() const;
    double potentialEnergy() const;
    double totalEnergy() const { return kineticEnergy() + potentialEnergy(); }
    Vector2D momentum() const;
    double angularMomentum() const;  // z-component about the origin

private:
    void computeForces();

    double G_;
    double softening_;
    std::vector<Particle> particles_;
    bool forcesValid_ = false;  // accelerations match current positions
};
