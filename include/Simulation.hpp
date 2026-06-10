#pragma once

#include <vector>

#include "Particle.hpp"

class Simulation {
public:
    // softening: minimum effective separation, keeps the force finite
    // when two particles occupy (nearly) the same point.
    Simulation(double gravitationalConstant, double softening)
        : G_(gravitationalConstant), softening_(softening) {}

    void addParticle(const Particle& p) { particles_.push_back(p); }

    const std::vector<Particle>& particles() const { return particles_; }

    // Advance the system by dt, internally split into `substeps`
    // smaller steps for stability at high frame-time spikes.
    void step(double dt, int substeps = 1);

private:
    void computeForces();

    double G_;
    double softening_;
    std::vector<Particle> particles_;
};
