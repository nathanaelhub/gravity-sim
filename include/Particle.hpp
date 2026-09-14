#pragma once

#include "Vector2D.hpp"

class Particle {
public:
    Particle(double mass, const Vector2D& position, const Vector2D& velocity)
        : mass_(mass), position_(position), velocity_(velocity) {}

    double mass() const { return mass_; }
    const Vector2D& position() const { return position_; }
    const Vector2D& velocity() const { return velocity_; }
    const Vector2D& acceleration() const { return acceleration_; }

    void resetAcceleration() { acceleration_ = {}; }

    void applyForce(const Vector2D& force) {
        // a = F / m, accumulated until the next integration step.
        acceleration_ += force / mass_;
    }

    // Semi-implicit Euler: update velocity from the accumulated
    // acceleration first, then advance position with the new velocity.
    // This keeps orbits stable where explicit Euler spirals outward.
    void integrate(double dt) {
        velocity_ += acceleration_ * dt;
        position_ += velocity_ * dt;
    }

    // The two halves of a leapfrog step (see Simulation::step).
    void kick(double dt) { velocity_ += acceleration_ * dt; }
    void drift(double dt) { position_ += velocity_ * dt; }

private:
    double mass_;
    Vector2D position_;
    Vector2D velocity_;
    Vector2D acceleration_;
};
