#pragma once

#include <cmath>

// Plain-value 2D vector. All operations return new vectors except the
// compound assignments, which mutate in place.
struct Vector2D {
    double x = 0.0;
    double y = 0.0;

    Vector2D() = default;
    Vector2D(double x_, double y_) : x(x_), y(y_) {}

    Vector2D operator+(const Vector2D& rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vector2D operator-(const Vector2D& rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vector2D operator*(double s) const { return {x * s, y * s}; }
    Vector2D operator/(double s) const { return {x / s, y / s}; }
    Vector2D operator-() const { return {-x, -y}; }

    Vector2D& operator+=(const Vector2D& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D& operator-=(const Vector2D& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }

    double lengthSquared() const { return x * x + y * y; }
    double length() const { return std::sqrt(lengthSquared()); }

    Vector2D normalized() const {
        const double len = length();
        return len > 0.0 ? Vector2D{x / len, y / len} : Vector2D{};
    }

    static double distance(const Vector2D& a, const Vector2D& b) {
        return (b - a).length();
    }

    static double distanceSquared(const Vector2D& a, const Vector2D& b) {
        return (b - a).lengthSquared();
    }

    static double dot(const Vector2D& a, const Vector2D& b) {
        return a.x * b.x + a.y * b.y;
    }
};

inline Vector2D operator*(double s, const Vector2D& v) { return v * s; }
