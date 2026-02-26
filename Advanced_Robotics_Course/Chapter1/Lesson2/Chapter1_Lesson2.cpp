#include <cmath>
#include <iostream>
#include <array>

struct Vec2 {
    double x, y;
};

Vec2 operator+(const Vec2& a, const Vec2& b) {
    return {a.x + b.x, a.y + b.y};
}

Vec2 operator-(const Vec2& a, const Vec2& b) {
    return {a.x - b.x, a.y - b.y};
}

Vec2 operator*(double s, const Vec2& v) {
    return {s * v.x, s * v.y};
}

double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

double norm(const Vec2& v) {
    return std::sqrt(dot(v, v));
}

std::array<Vec2,3> forwardKinematics(double q1, double q2,
                                      double L1 = 1.0, double L2 = 1.0) {
    Vec2 p0{0.0, 0.0};
    Vec2 p1{L1 * std::cos(q1), L1 * std::sin(q1)};
    Vec2 p2 = {p1.x + L2 * std::cos(q1 + q2),
               p1.y + L2 * std::sin(q1 + q2)};
    return {p0, p1, p2};
}

double segmentPointDistance(const Vec2& a, const Vec2& b, const Vec2& p) {
    Vec2 ab = {b.x - a.x, b.y - a.y};
    double denom = dot(ab, ab) + 1e-12;
    double t = dot(p - a, ab) / denom;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    Vec2 proj = {a.x + t * ab.x, a.y + t * ab.y};
    return norm(p - proj);
}

bool inCollision(double q1, double q2,
                 const Vec2& center, double radius) {
    auto pts = forwardKinematics(q1, q2);
    double d1 = segmentPointDistance(pts[0], pts[1], center);
    double d2 = segmentPointDistance(pts[1], pts[2], center);
    return (d1 <= radius) || (d2 <= radius);
}

int main() {
    Vec2 center{0.8, 0.4};
    double radius = 0.25;
    double q1 = 0.3, q2 = -1.0;
    std::cout << "Collision? " <<
        (inCollision(q1, q2, center, radius) ? "yes" : "no") << std::endl;
    return 0;
}
      
