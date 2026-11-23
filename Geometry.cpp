// Geometry -- XenShizuka7
// Single-file version of ICPC_GEO.pdf (all sections concatenated in order).
//
// Contents:
//   Core, Basics, Transformations, Angles, Lines & Rays, Segments, Polygons,
//   Circles, Convex Hull, Half-planes, Polygon Orientation, Polygon Union
//   (rectangles), Line Sweeping, Delaunay Triangulation, Rectangles,
//   Segment Intersection Sweep, Convex Polygon Clipping, Rotating Calipers,
//   Polygon Triangulation, 3D Geometry Basics, Appendix: Worked Examples
//   (the appendix is wrapped in #if 0 because it redefines Point/cross/etc.)

////////////////////////////////////////// Core //////////////////////////////////////////
// File: 00_core.hpp  Requires: nothing (standalone)

#include <bits/stdc++.h>
using namespace std;

#include <complex>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cassert>


using T = double; // change to `long double` for extra precision
using Point = complex<T>;
const T EPS = 1e-13;
const T PI = 3.14159265358979323846264338327950288L;

/**
 * @brief Conventions and notes for the geometry library:
 * - arg(complex<T>): Angle between x-axis and vector (atan2(y, x)).
 * - Radians to degrees: angle * 180 / PI
 * - Degrees to radians: angle * PI / 180
 * - Sine rule: a / sin(a1) = b / sin(a2) = c / sin(a3) = 2R
 * - Cosine rule for angle: cos(a) = (a^2 + b^2 - c^2) / (2 * a * b)
 * - Vector from magnitude and theta: x = r * cos(a), y = r * sin(a)
 * - Multiply vectors: Magnitude = r1 * r2, angle = theta1 + theta2
 * - Triangle area: 0.5 * a * b * sin(theta)
 * - Circle radius in triangle: area / (0.5 * perimeter) or (a * b * c) / (4 * area)
 * - Polygon orientation: Remove abs from polygonArea; positive area => clockwise
 * - Inradius r = 2 * area / (a + b + c)
 * - Circumradius R = (a * b * c) / (4 * area)
 * - Distance between circumcenter and incenter: OI^2 = R * (R - 2r)
 * - Pick's theorem (lattice polygons): A = I + B/2 - 1 (I: interior points, B: boundary points)
 * - Law of Sines: a/sin(A) = b/sin(B) = c/sin(C) = 2R
 * - Law of Cosines: c^2 = a^2 + b^2 - 2ab*cos(C)
 * - Shoelace formula: A = 0.5 * |Σ (x_i * y_{i+1} - x_{i+1} * y_i)|
 * - Basic shapes:
 * - Rectangle: Area = w * h, Perimeter = 2(w + h)
 * - Square: Area = a^2, Perimeter = 4a
 * - Triangle: Area = 0.5 * b * h, Perimeter = a + b + c
 * - Heron's formula: A = sqrt[s(s-a)(s-b)(s-c)], s = (a+b+c)/2
 * - Parallelogram: Area = b * h, Perimeter = 2(a + b)
 * - Trapezoid: Area = ((b1 + b2) / 2) * h, Perimeter = a + b1 + b2 + c
 * - Circle: Area = π * r^2, Circumference = 2 * π * r
 * - Sector (angle θ in radians): Area = 0.5 * r^2 * θ, Perimeter = r * θ + 2r
 * - Regular n-gon (side a): Area = (n/4) * a^2 * cot(π/n), Perimeter = n * a
 * - 3D Solids:
 * - Cuboid: V = a * b * c, S = 2(ab + bc + ca)
 * - Sphere: V = (4/3) * π * r^3, S = 4 * π * r^2
 * - Cylinder: V = π * r^2 * h, S = 2 * π * r * (h + r)
 * - Cone: V = (1/3) * π * r^2 * h, S = π * r * (r + sqrt(h^2 + r^2))
 * - Pyramid: V = (1/3) * B * h, S = B + 0.5 * P * l (l: slant height)
 */

////////////////////////////////////////// Basics //////////////////////////////////////////
// File: 01_basics.hpp  Requires: 00_core.hpp

/**
 * @brief Determines the sign of a value.
 * @param val The input value.
 * @return -1 if val < -EPS, 1 if val > EPS, 0 otherwise.
 */
int sign(T val) {
    return (val > EPS) - (val < -EPS);
}

/**
 * @brief Computes the squared magnitude of a vector.
 * @param p The input point (vector).
 * @return The squared distance from (0, 0) to p.
 */
T getVectorMagnitudeSquared(Point p) {
    return norm(p);
}

/**
 * @brief Returns the perpendicular vector (rotated 90° left).
 * @param p The input point (vector).
 * @return The perpendicular vector (-y, x).
 */
Point perp(Point p) {
    return {-p.imag(), p.real()};
}

/**
 * @brief Computes the dot product of two vectors.
 * @param v First vector.
 * @param w Second vector.
 * @return The dot product v·w.
 */
T dot(Point v, Point w) {
    return v.real() * w.real() + v.imag() * w.imag();
}

/**
 * @brief Dot product of (A-O) and (B-O), i.e. the dot product of two vectors
 * sharing a common origin O. Convenience wrapper around dot().
 * @param O Common origin.
 * @param A First point.
 * @param B Second point.
 * @return (A-O)·(B-O). Negative means angle AOB is obtuse, zero means right angle.
 */
T dott(Point O, Point A, Point B) {
    return dot(A - O, B - O);
}

/**
 * @brief Computes the cross product of two vectors.
 * @param v First vector.
 * @param w Second vector.
 * @return The cross product v×w.
 */
T cross(Point v, Point w) {
    return v.real() * w.imag() - v.imag() * w.real();
}

/**
 * @brief Checks if two vectors are perpendicular.
 * @param v First vector.
 * @param w Second vector.
 * @return True if the dot product is near zero (within EPS).
 */
bool isPerp(Point v, Point w) {
    return fabs(dot(v, w)) < EPS;
}

/**
 * @brief Checks if three points form a right angle.
 * @param a First point.
 * @param b Second point (vertex of angle).
 * @param c Third point.
 * @return True if angle at b is 90°.
 */
bool isRightAngled(Point a, Point b, Point c) {
    return isPerp(b - a, c - a) || isPerp(a - b, c - b) || isPerp(a - c, b - c);
}

/**
 * @brief Determines the orientation of three points.
 * @param a First point.
 * @param b Second point.
 * @param c Third point.
 * @return +1 if counterclockwise, -1 if clockwise, 0 if collinear.
 */
int orientation(Point a, Point b, Point c) {
    return sign(cross(b - a, c - a));
}

////////////////////////////////////////// Transformations //////////////////////////////////////////
// File: 02_transformations.hpp  Requires: 00_core.hpp, 01_basics.hpp

/**
 * @brief Scales a point around a center by a factor.
 * @param c Center of scaling.
 * @param factor Scaling factor.
 * @param p Point to scale.
 * @return The scaled point.
 */
Point scale(Point c, T factor, Point p) {
    return c + (p - c) * factor;
}

/**
 * @brief Rotates a point around a center by an angle.
 * @param p Point to rotate.
 * @param c Center of rotation.
 * @param angle Rotation angle in radians.
 * @return The rotated point.
 */
Point rotate(Point p, Point c, T angle) {
    Point v = p - c;
    return {c.real() + v.real() * cos(angle) - v.imag() * sin(angle),
            c.imag() + v.real() * sin(angle) + v.imag() *
            cos(angle)};
}

/**
 * @brief Applies a linear transformation based on two point mappings.
 * @param p First source point.
 * @param q Second source point.
 * @param r Point to transform.
 * @param fp Image of p under transformation.
 * @param fq Image of q under transformation.
 * @return The transformed point r.
 */
Point linearTransformation(Point p, Point q, Point r, Point fp, Point fq) {
    Point pq = q - p;
    Point num{cross(pq, fq - fp), dot(pq, fq - fp)};
    return fp + Point{cross(p, num), dot(r - p, num)} / getVectorMagnitudeSquared(pq);
}

////////////////////////////////////////// Angles //////////////////////////////////////////
// File: 03_angles.hpp  Requires: 00_core.hpp, 01_basics.hpp

/**
 * @brief Computes the oriented cross product (relative to AB).
 * @param a First point.
 * @param b Second point.
 * @param c Third point.
 * @return Cross product (b-a)×(c-a). Positive if c is left of ab, negative if right.
 */
T orient(Point a, Point b, Point c) {
    return cross(b - a, c - a);
}

/**
 * @brief Checks if point p is within the counterclockwise angle formed by points b and c around center.
 * @param center Center of the angle.
 * @param pointB Point defining start of angle.
 * @param pointC Point defining end of angle.
 * @param pointP Point to check.
 * @return True if p is within angle <BAC (counterclockwise).
 */
bool inAngle(Point center, Point pointB, Point pointC, Point pointP) {
    auto computeAngle = [&](Point q) {
        T dx = q.real() - center.real();
        T dy = q.imag() - center.imag();
        T a = atan2(dy, dx);
        if (a < 0) a += 2 * PI;
        else if (a >= 2 * PI) a -= 2 * PI;
        if (a < 0) a += 2 * PI;
        return a;
    };

    T            angleB            =
    computeAngle(pointB);  T  angleC
    = computeAngle(pointC); T angleP
    = computeAngle(pointP);

    if (angleC < angleB) angleC += 2 * PI;
    if (angleP < angleB) angleP += 2 * PI;

    return angleB <= angleP && angleP <= angleC;
}

/**
 * @brief Computes the angle between two vectors.
 * @param v First vector.
 * @param w Second vector.
 * @return Angle in radians [0, π].
 */
T angleBetween(Point v, Point w) {
    return acos(clamp(dot(v, w) / abs(v) / abs(w), T(-1.0), T(1.0)));
}

T signedAngleBetween(Point v, Point
    w){ return atan2(cross(v,w),
    dot(v,w));
}

/**
 * @brief Computes the oriented angle <BAC.
 * @param a First point.
 * @param b Vertex of angle.
 * @param c Third point.
 * @return Angle in radians [0, 2π). Returns 2π - angle if clockwise.
 */
T orientedAngle(Point a, Point b, Point c) {
    return orient(a, b, c) >= 0 ? angleBetween(b - a, c - a)
                                : 2 * PI - angleBetween(b - a, c - a);
}

/**
 * @brief Computes the signed angle from p to q around point a.
 * @param a Center point.
 * @param p Starting point.
 * @param q Ending point.
 * @return Signed angle in radians (positive if counterclockwise).
 */
T angleTravelled(Point a, Point p, Point q) {
    T ampli = angleBetween(p - a, q - a);
    return orient(a, p, q) > 0 ? ampli : -ampli;
}

/**
 * @brief Checks if a point is in the upper half-plane.
 * @param p Point to check.
 * @return True if p.imag() > 0 or (p.imag() == 0 and p.real() < 0).
 */
bool half(Point p) {
    return p.imag() > 0 || (p.imag() == 0 && p.real() < 0);
}

////////////////////////////////////////// Lines //////////////////////////////////////////
// File: 04_lines.hpp  Requires: 00_core.hpp, 01_basics.hpp

/**
 * @brief Represents a line in 2D space.
 */
struct Line {
    Point v;   ///< Direction vector.
    T c;    ///< Offset (ax + by = c).

    /// @brief Constructs a line from direction vector and offset.
    Line(Point v, T c) : v(v), c(c) {}

    /// @brief Constructs a line from equation ax + by = c.
    Line(T a, T b, T c) : v({b, -a}), c(c) {}

    /// @brief Constructs a line through two points.
    Line(Point p, Point q) : v(q - p), c(cross(v, p)) {}

    /// @brief Computes signed distance to a point (like orient).
    T side(Point p) { return cross(v, p) - c; }

    /// @brief Computes perpendicular distance to a point.
    T dist(Point p) { return abs(side(p)) / abs(v); }

    /// @brief Returns a line perpendicular to this line through point p.
    Line perpThrough(Point p) { return {p, p + perp(v)}; }

    /// @brief Compares projections of two points onto the line.
    bool cmpProj(Point p, Point q) { return dot(v, p) < dot(v, q); }

    /// @brief Translates the line by vector t.
    Line translate(Point t) { return {v, c + cross(v, t)}; }

    /// @brief Shifts the line left by a distance.
    Line shiftLeft(T dist) { return {v, c + dist * abs(v)}; }

    /// @brief Projects a point onto the line.
    Point proj(Point p) { return p - perp(v) * side(p) / getVectorMagnitudeSquared(v); }

    /// @brief Reflects a point over the line.
    Point refl(Point p) { return p - perp(v) * T(2.0) * side(p) / getVectorMagnitudeSquared(v); }
};

/**
 * @brief Computes the slope of a line defined by two points.
 * @param p1 First point.
 * @param p2 Second point.
 * @return Slope (dy/dx). Returns DBL_MAX for vertical lines.
 */
T getLineSlope(Point p1, Point p2) {
    Point d = p2 - p1;
    if (fabs(d.real()) < EPS) return numeric_limits<T>::max(); // Vertical line
    return d.imag() / d.real();
}

/**
 * @brief Finds the intersection of two lines.
 * @param l1 First line.
 * @param l2 Second line.
 * @param out Intersection point (if exists).
 * @return True if lines intersect, false if parallel.
 */
bool inter(Line l1, Line l2, Point &out) {
    T d = cross(l1.v, l2.v);
    if (fabs(d) <= EPS) return false;
    out = (l2.v * l1.c - l1.v * l2.c) / d;
    return true;
}

/**
 * @brief Computes the angle bisector of two lines through a point.
 * @param p1 Common point.
 * @param p2 Point on first line.
 * @param p3 Point on second line.
 * @param interior True for interior bisector, false for exterior.
 * @return Line representing the bisector.
 */
Line bisector(Point p1, Point p2, Point p3, bool interior) {
    Point u = (p2 - p1) / abs(p2 - p1);
    Point w = (p3 - p1) / abs(p3 - p1);
    Point dir = interior ? u + w : u - w;

    if (abs(dir) < EPS) dir = {u.imag(), -u.real()};

    T A = dir.imag();
    T B =
    -dir.real();
    T C = -(A * p1.real() + B * p1.imag());

    if (fabs(A) < EPS) A = 0;
    if (fabs(B) < EPS) B = 0;
    if (fabs(C) < EPS) C = 0;

    return Line(A, B, C);
}

/**
 * @brief Computes distance from a point to a line segment.
 * @param a First endpoint of segment.
 * @param b Second endpoint of segment.
 * @param c Point to measure distance to.
 * @return Perpendicular distance to line ab or distance to endpoints.
 */
T distPointLine(Point a, Point b, Point c) {
    return fabs(cross(b - a, c - a)) / abs(a - b);
}

/**
 * @brief Finds two points on a line given by equation ax + by + c = 0.
 * @param a Coefficient of x.
 * @param b Coefficient of y.
 * @param c Constant term.
 * @param p1 First output point.
 * @param p2 Second output point.
 */
void getTwoPointsFromEquation(T a, T b, T c, Point &p1, Point &p2)
    { if (fabs(b) > EPS) {
        p1 = {0.0, -c / b};
        p2 = {1.0, -(a + c) / b};
    } else {
        T x = -c / a;
        p1 = {x, 0.0};
        p2 = {x, 1.0};
    }
}

/**
 * @brief Finds two points on a line given by direction vector and offset.
 * @param l Input line.
 * @param p1 First output point.
 * @param p2 Second output point.
 */
void getTwoPointsFromDirectionVector(Line l, Point &p1, Point &p2) {
    Point origin = {0, 0};
    T magnitude =
    abs(l.v);
    Point pointOnLine = origin + perp(l.v) * (l.c / (magnitude * magnitude));
    Point dir = l.v / magnitude;
    p1 = pointOnLine + dir * T(1.0);
    p2 = pointOnLine - dir * T(1.0);
}

/**
 * @brief Computes distance from a point to a ray.
 * @param A Ray origin.
 * @param B Point defining ray direction.
 * @param P Point to measure distance to.
 * @return Distance to ray (endpoint A if projection is before A).
 */
T distPointRay(Point A, Point B, Point P) {
    Point v = B - A;
    T t = dot(P - A, v) / getVectorMagnitudeSquared(v);
    return t < 0 ? abs(P - A) : abs(cross(v, P - A)) / abs(v);
}

/**
 * @brief Computes minimum distance between two rays.
 * @param A Origin of first ray.
 * @param B Direction point of first ray.
 * @param C Origin of second ray.
 * @param D Direction point of second ray.
 * @return Minimum distance between rays.
 */
T distRayRay(Point A, Point B, Point C, Point D) {
    Point u = B - A, v = D - C, w0 = A - C;
    T a = dot(u, u), b = dot(u, v), c = dot(v, v);
    T d = dot(u, w0), e = dot(v, w0);
    T denom = a * c - b * b;
    T best = numeric_limits<T>::max();

    if (fabs(denom) > EPS) {
        T t0 = (b * e - c * d) / denom;
        T s0 = (a * e - b * d) / denom;
        if (t0 >= 0 && s0 >= 0) {
            Point dp = w0 + u * t0 - v * s0;
            best = min(best, abs(dp));
        }
    }

    T t = dot(A - C, v) / getVectorMagnitudeSquared(v);
    best = min(best, t < 0 ? abs(A - C) : abs(cross(v, A - C)) / abs(v));

    t = dot(C - A, u) / getVectorMagnitudeSquared(u);
    best = min(best, t < 0 ? abs(C - A) : abs(cross(u, C - A)) / abs(u));

    return best;
}

/**
 * @brief Represents a ray in 2D space.
 */
struct Ray {
    Point origin;    ///< Starting point of the ray.
    Point direction; ///< Unit-length direction vector.
};

/**
 * @brief Creates a ray from two points.
 * @param a Origin point.
 * @param b Point defining direction.
 * @return Ray with unit-length direction.
 */
Ray make_ray(Point a, Point b) {
    Point d = b - a;
    return {a, d / abs(d)};
}

/**
 * @brief Finds the intersection of two rays.
 * @param r1 First ray.
 * @param r2 Second ray.
 * @param out Intersection point (if exists).
 * @return True if rays intersect with non-negative parameters.
 */
bool intersect_ray(Ray r1, Ray r2, Point &out) {
    Point dp = r2.origin - r1.origin;
    T denom = cross(r1.direction,
    r2.direction); if (fabs(denom) < EPS) return
    false;
    T t1 = cross(dp, r2.direction) / denom;
    T t2 = cross(dp, r1.direction) / denom;
    if (t1 < -EPS || t2 < -EPS) return false;
    out = r1.origin + r1.direction * t1;
    return true;
}

/**
 * @brief Determines if a point is left, right, or on a line.
 * @param p1 First point on the line.
 * @param p2 Second point on the line.
 * @param p3 Point to check.
 * @return -1 if right, 1 if left, 0 if on the line.
 */
int pointLineSide(Point p1, Point p2, Point p3) {
    T c = cross(p2 - p1, p3 - p1);
    return sign(c);
}

/**
 * @brief Given 1 ray and 1 point which is supposed to be the bisector, recover the other ray
 * @param U Known ray
 * @param B Bisector
 * @return A ray
 */
Point recover_other_ray(
    Point U,  // known ray (not zero)
    Point B   // bisector (not zero)
) {
    T nu = abs(U), nb = abs(B);
    auto u = U/nu, b = B/nb;
    T dot = real(conj(b)*u);
// reflect & rescale to original length
    return (2*dot*b - u) * nu;
 }

////////////////////////////////////////// Segments //////////////////////////////////////////
// File: 05_segments.hpp  Requires: 00_core.hpp, 01_basics.hpp, 03_angles.hpp, 04_lines.hpp

/**
 * @brief Checks if a point lies within the disk(the region in a plane bounded by a circle) defined by segment endpoints.
 * @param a First endpoint.
 * @param b Second endpoint.
 * @param p Point to check.
 * @return True if dot(a-p, b-p) <= EPS.
 */
bool inDisk(Point a, Point b, Point p) {
    return dot(a - p, b - p) <= EPS;
}

/**
 * @brief Checks if a point lies on a line segment.
 * @param a First endpoint.
 * @param b Second endpoint.
 * @param p Point to check.
 * @return True if p is on segment ab (within EPS).
 */
bool onSegment(Point a, Point b, Point p) {
    return fabs(orient(a, b, p)) <= EPS && inDisk(a, b, p);
}

/**
 * @brief Finds proper intersection of two line segments.
 * @param a First endpoint of first segment.
 * @param b Second endpoint of first segment.
 * @param c First endpoint of second segment.
 * @param d Second endpoint of second segment.
 * @param out Intersection point (if exists).
 * @return True if segments intersect properly (not at endpoints).
 */
bool properInter(Point a, Point b, Point c, Point d, Point &out) {
    T oa = orient(c, d, a), ob = orient(c, d, b);
    T oc = orient(a, b, c), od = orient(a, b, d);
    if (sign(oa) * sign(ob) < 0 && sign(oc) * sign(od) < 0) {
        out = (a * ob - b * oa) / (ob - oa);
        return true;
    }
    return false;
}

/**
 * @brief Finds all intersection points of two line segments, including overlaps.
 * @param s_p First endpoint of first segment.
 * @param s_q Second endpoint of first segment.
 * @param t_p First endpoint of second segment.
 * @param t_q Second endpoint of second segment.
 * @return Vector of intersection points (0, 1, or 2 points for overlapping segments).
 */
vector<Point> intersectSegments(Point s_p, Point s_q, Point t_p, Point t_q) {
    T a = cross(s_q - s_p, t_q - t_p);
    T b = cross(t_p - s_p, t_q - t_p);
    T c = cross(s_q - s_p, s_p - t_p);
    if (a < 0) { a = -a; b = -b; c = -c; }
    if (sign(b) < 0 || sign(a - b) < 0 || sign(c) < 0 || sign(a - c) < 0) return {};
    if (sign(a) != 0) return {s_p + (s_q - s_p) * (b / a)};
    vector<Point> ps;
    auto insert_if_possible = [&](Point p) {
        for (auto q : ps) if (abs(p - q) < EPS) return;
        ps.push_back(p);
    };
    if (dot(s_p - t_p, s_q - t_p) <= EPS) insert_if_possible(t_p);
    if (dot(s_p - t_q, s_q - t_q) <= EPS) insert_if_possible(t_q);
    if (dot(t_p - s_p, t_q - s_p) <= EPS) insert_if_possible(s_p);
    if (dot(t_p - s_q, t_q - s_q) <= EPS) insert_if_possible(s_q);
    return ps;
}

/**
 * @brief Computes distance from a point to a line segment.
 * @param a First endpoint.
 * @param b Second endpoint.
 * @param p Point to measure distance to.
 * @return Minimum distance to segment ab.
 */
T segPoint(Point a, Point b, Point p) {
    if (a != b) {
        Line l(a, b);
        if (l.cmpProj(a, p) && l.cmpProj(p, b)) return l.dist(p);
    }
    return min(abs(p - a), abs(p - b));
}

/**
 * @brief Computes minimum distance between two line segments.
 * @param a First endpoint of first segment.
 * @param b Second endpoint of first segment.
 * @param c First endpoint of second segment.
 * @param d Second endpoint of second segment.
 * @return Minimum distance between segments.
 */
T segSeg(Point a, Point b, Point c, Point d) {
    Point dummy;
    if (properInter(a, b, c, d, dummy)) return 0;
    return min({segPoint(a, b, c), segPoint(a, b,
    d),
                segPoint(c, d, a), segPoint(c, d, b)});
}

/**
 * @brief Intersects an infinite line with a line segment.
 * @param A First point on line.
 * @param B Second point on line.
 * @param U First endpoint of segment.
 * @param V Second endpoint of segment.
 * @return Intersection point.
 */
Point intersect_line_seg(Point A, Point B, Point U, Point V) {
    Point dAB = B - A, dUV = V - U;
    T num = cross(dAB, U - A);
    T denom = cross(dAB, dUV);
    if (fabs(denom) < EPS) return U; // Parallel, return U as a fallback
    return U + dUV * (num / denom);
}

////////////////////////////////////////// Polygons //////////////////////////////////////////
// File: 06_polygons.hpp  Requires: 00_core.hpp, 01_basics.hpp, 03_angles.hpp, 05_segments.hpp

/**
 * @brief Checks if a polygon is convex.
 * @param p Vector of polygon vertices.
 * @return True if all turns are in the same direction (no mixed orientations).
 */
bool isConvex(vector<Point> p) {
    bool hasPos = false, hasNeg = false;
    for (int i = 0, n = p.size(); i < n; i++) {
        int o = sign(orient(p[i], p[(i + 1) % n], p[(i + 2) % n]));
        if (o > 0) hasPos = true;
        if (o < 0) hasNeg = true;
    }
    return !(hasPos && hasNeg);
}

/**
 * @brief Computes the area of a triangle.
 * @param a First vertex.
 * @param b Second vertex.
 * @param c Third vertex.
 * @return Area of triangle abc.
 */
T areaTriangle(Point a, Point b, Point c)
    { return abs(cross(b - a, c - a)) /
    2.0;
}

/**
 * @brief Computes the area of a polygon with integer coordinates.
 * @param p Vector of polygon vertices (integer coordinates).
 * @return Absolute area of the polygon.
 */
T areaPolygonInteger(vector<Point> p) {
    T sum = 0;
    for (int i = 0, n = p.size(); i < n; i++) {
        int j = (i + 1) % n;
        sum += p[i].real() * p[j].imag() - p[j].real() * p[i].imag();
    }
    return abs(sum) / 2.0;
}

/**
 * @brief Computes the area of a polygon.
 * @param p Vector of polygon vertices.
 * @return Absolute area of the polygon.
 */
T areaPolygon(vector<Point> p) {
    T area = 0.0;
    for (int i = 0, n = p.size(); i < n; i++) {
        area += cross(p[i], p[(i + 1) % n]);
    }
    return abs(area) / 2.0;
}

/**
 * @brief Determines if a polygon is clockwise.
 * @param p Vector of polygon vertices.
 * @return True if polygon is clockwise (signed area >= 0).
 */
bool isPolygonClockwise(vector<Point> p) {
    T area = 0.0;
    for (int i = 0, n = p.size(); i < n; i++) {
        area += cross(p[i], p[(i + 1) % n]);
    }
    return area >= 0;
}

/**
 * @brief Checks if point p is at least as high as point a.
 * @param a Reference point.
 * @param p Point to check.
 * @return True if p's y-coordinate is >= a's y-coordinate.
 */
bool above(Point a, Point p) {
    return p.imag() >= a.imag();
}

/**
 * @brief Checks if segment pq crosses a ray from point a.
 * @param a Ray origin.
 * @param p First endpoint of segment.
 * @param q Second endpoint of segment.
 * @return True if segment pq crosses the ray from a.
 */
bool crossesRay(Point a, Point p, Point q) {
    return (above(a, q) - above(a, p)) * orient(a, p, q) > 0;
}

/**
 * @brief Checks if a point is inside a polygon using ray-casting.
 * @param p Vector of polygon vertices.
 * @param a Point to check.
 * @param strict If true, points on boundary are excluded.
 * @return True if a is inside the polygon (odd number of crossings).
 */
bool inPolygon(vector<Point> p, Point a, bool strict = true) {
    int numCrossings = 0;
    for (int i = 0, n = p.size(); i < n; i++) {
        if (onSegment(p[i], p[(i + 1) % n], a)) return !strict;
        numCrossings += crossesRay(a, p[i], p[(i + 1) % n]);
    }
    return numCrossings & 1;
}

/**
 * @brief Checks if a point is inside a polygon using winding number.
 * @param p Vector of polygon vertices.
 * @param q Point to check.
 * @return True if q is inside the polygon (non-zero winding number).
 */
bool pointInPolygonWinding(const vector<Point>& p, const Point& q) {
    bool in = false;
    int n = p.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((p[i].imag() > q.imag()) != (p[j].imag() > q.imag())) &&
            (q.real() < (p[j].real() - p[i].real()) * (q.imag() - p[i].imag()) / (p[j].imag() - p[i].imag()) + p[i].real()))
            in = !in;
    }
    return in;
}

/**
 * @brief Determines if a point is inside, outside, or on a polygon.
 * @param poly Vector of polygon vertices (integer coordinates).
 * @param p Point to check.
 * @return String: "INSIDE", "OUTSIDE", or "BOUNDARY".
 */
string pointPolygonLocation(const vector<Point>& poly, Point p) {
    int cnt = 0;
    for (int i = 0, n = poly.size(); i < n; i++) {
        Point A = poly[i];
        Point B = poly[(i + 1) % n];
        if (imag(conj(B - A) * (p - A)) == 0 && real(conj(p - A) * (p - B)) <= 0) {
            return "BOUNDARY";
        }
        if ((A.imag() > p.imag()) != (B.imag() > p.imag())) {
            T o = (B.real() - A.real()) * (p.imag() - A.imag()) - (B.imag() - A.imag()) * (p.real() - A.real());
            if ((B.imag() > A.imag()) == (o > 0)) {
                cnt++;
            }
        }
    }
    return (cnt % 2 == 1) ? "INSIDE" : "OUTSIDE";
}

T gcdf(T a, T b)
{
    if (a < b)
        return gcdf(b, a);

    // base case
    if (fabs(b) < 0.00000000001)
        return a;

    else
        return (gcdf(b, a - floor(a / b) * b));

}

/**
 * @brief Computes the number of interior and boundary lattice points in a polygon.
 * @param p Vector of polygon vertices (integer coordinates).
 * @return Pair of (interior points, boundary points) using Pick's theorem.
 */
pair<T, T> latticePoints(const vector<Point>& p) {
    T A = 0;
    for (int i = 0, n = p.size(); i < n; i++) {
        int j = (i + 1) % n;
        A += p[i].real() * p[j].imag() - p[j].real() * p[i].imag();
    }
    A = abs(A) / 2;
    T B = 0;
    for (int i = 0, n = p.size(); i < n; i++) {
        int j = (i + 1) % n;
        B += gcdf(abs(p[i].real() - p[j].real()), abs(p[i].imag() - p[j].imag()));
    }
    T I = A - B / 2 + 1;
    return {I, B};
}

/**
 * @brief Computes the kernel (intersection of interior half-planes) of a convex polygon.
 * @param poly Vector of convex polygon vertices (CW or CCW).
 * @return Vector of kernel vertices in CCW order (may be empty).
 */
vector<Point> kernel_of_convex(const vector<Point>& poly) {
    int N = poly.size();
    vector<Point> output = poly;

    for (int i = 0; i < N && !output.empty(); i++) {
        Point A = poly[i];
        Point B = poly[(i + 1) % N];
        vector<Point> input = move(output);
        output.clear();
        int M = input.size();
        for (int j = 0; j < M; j++) {
            Point U = input[j];
            Point V = input[(j + 1) % M];
            bool inU = cross(B - A, U - A) >= 0.0;
            bool inV = cross(B - A, V - A) >= 0.0;
            if (inU) output.push_back(U);
            if (inU ^ inV) output.push_back(intersect_line_seg(A, B, U, V));
        }
    }
    return output;
}

////////////////////////////////////////// Circles //////////////////////////////////////////
// File: 07_circles.hpp  Requires: 00_core.hpp, 01_basics.hpp, 03_angles.hpp, 04_lines.hpp

/**
 * @brief Computes the circumcircle of a triangle.
 * @param a First vertex.
 * @param b Second vertex.
 * @param c Third vertex.
 * @return Pair of center point and radius. Assumes points are not collinear.
 */
pair<Point, T> circumCircle(Point a, Point b, Point c) {
    if(abs(cross(b - a, c - a)) == 0)
        return {0, 0};

    Point rel_b = b - a, rel_c = c - a;
    // NOTE: the right-hand end of the next line was cut off in the PDF; the
    // denominator below is the standard circumcenter formula (KACTL).
    Point center = a + perp(rel_b * getVectorMagnitudeSquared(rel_c) - rel_c * getVectorMagnitudeSquared(rel_b)) / cross(rel_b, rel_c) / T(2);
    return {center, abs(center - a)};
}

/**
 * @brief Finds intersection points of a line and a circle.
 * @param o Circle center.
 * @param r Circle radius.
 * @param l Line to intersect.
 * @param out Pair of intersection points (if they exist).
 * @return Number of intersections (0, 1, or 2).
 */
int circleLine(Point o, T r, Line l, pair<Point, Point> &out) {
    T h2 = r * r - l.dist(o) * l.dist(o);
    if (h2 >= 0) {
        Point p = l.proj(o);
        Point h = l.v * (sqrt(h2) / abs(l.v));
        out = {p - h, p + h};
    }
    return 1 + sign(h2);
}

/**
 * @brief Finds intersection points of two circles.
 * @param a Center of first circle.
 * @param r Radius of first circle.
 * @param b Center of second circle.
 * @param R Radius of second circle.
 * @param sol Array to store intersection points (up to 2).
 * @return Number of intersections (-1 if infinite, 0, 1, or 2).
 */
int circleIntersect(Point a, T r, Point b, T R, Point sol[]) {
    T d = abs(a - b);
    if (sign(d) == 0 && sign(r - R) == 0) return -1; // infinite
    if (d > r + R || d < fabs(r - R)) return 0;        // none
    T x = (d * d - R * R + r * r) / (2 * d);
    T y = sqrt(max(T(0), r * r - x * x));
    Point v = (b - a) / d;
    sol[0] = a + v * x + Point(-v.imag(), v.real()) * y;
    sol[1] = a + v * x - Point(-v.imag(), v.real()) * y;
    return (sign(y) == 0 ? 1 : 2);
}

/**
 * @brief Computes arc length between two points on a circle.
 * @param p1 First point on circle.
 * @param p2 Second point on circle.
 * @param r Circle radius.
 * @return Arc length.
 */
T arcLength(Point p1, Point p2, T r) {
    return r * angleBetween(p1, p2);
}

/**
 * @brief Finds intersection points of two circles.
 * @param o1 Center of first circle.
 * @param r1 Radius of first circle.
 * @param o2 Center of second circle.
 * @param r2 Radius of second circle.
 * @param out Pair of intersection points (if they exist).
 * @return Number of intersections (-10 if concentric, 0, 1, or 2).
 */
int circleCircle(Point o1, T r1, Point o2, T r2, pair<Point, Point> &out) {
    Point d = o2 - o1;
    T d2 = getVectorMagnitudeSquared(d);
    if (d2 == 0) return -10;
    T pd = (d2 + r1 * r1 - r2 * r2) / 2;
    T h2 = r1 * r1 - pd * pd / d2;
    if (h2 >= 0) {
        Point p = o1 + d * pd / d2;
        Point h = perp(d) * sqrt(h2 / d2);
        out = {p - h, p + h};
    }
    return 1 + sign(h2);
}

/**
 * @brief Computes the area of intersection of two circles.
 * @param r1 Radius of first circle.
 * @param r2 Radius of second circle.
 * @param d Distance between centers.
 * @return Intersection area.
 */
T circleIntersectionArea(T r1, T r2, T d)
    { if (d >= r1 + r2) return 0.0;
    if (d <= fabs(r1 - r2)) return PI * min(r1, r2) * min(r1, r2);
    T alpha = acos((d * d + r1 * r1 - r2 * r2) / (2 * d * r1));
    T beta = acos((d * d + r2 * r2 - r1 * r1) / (2 * d *
    r2)); T part1 = r1 * r1 * alpha;
    T part2 = r2 * r2 * beta;
    T part3 = 0.5 * sqrt(max(T(0), (r1 + r2 - d) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2)));
    return part1 + part2 - part3;
}

/**
 * @brief Finds tangent lines between two circles.
 * @param o1 Center of first circle.
 * @param r1 Radius of first circle.
 * @param o2 Center of second circle.
 * @param r2 Radius of second circle.
 * @param inner True for inner tangents, false for outer.
 * @param out Vector of tangent line endpoints (pairs of points).
 * @return Number of tangents (-10 if same center, 0, 1, or 2).
 */
int tangents(Point o1, T r1, Point o2, T r2, bool inner, vector<pair<Point, Point>> &out)
    { if (inner) r2 = -r2;
    Point d = o2 - o1;
    T dr = r1 - r2, d2 = getVectorMagnitudeSquared(d), h2 = d2 - dr *
    dr; if (d2 == 0 || h2 < 0) return -10;
    for (T sign : {-1, 1}) {
        Point v = (d * dr + perp(d) * sqrt(h2) * sign) / d2;
        out.push_back({o1 + v * r1, o2 + v * r2});
    }
    return 1 + (h2 > 0);
}

/**
 * @brief Computes a point on a circle at a given angle.
 * @param O Circle center.
 * @param r Circle radius.
 * @param theta Angle in radians.
 * @return Point on circle.
 */
Point pointAtAngle(Point O, T r, T theta)
    { return O + polar(r, theta);
}

/**
 * @brief Projects a point onto a circle.
 * @param c Circle center.
 * @param r Circle radius.
 * @param p Point to project.
 * @return Closest point on circle to p.
 */
Point projectOntoCircle(Point c, T r, Point p) {
    Point v = p - c;
    T dist = abs(v);
    if (dist < EPS) return c + Point(r, 0);
    return c + v * (r / dist);
}

/**
 * @brief Projects a point along a ray to a circle, choosing the point closest to a target.
 * @param C Circle center.
 * @param r Circle radius.
 * @param p Starting point of ray.
 * @param a Target point defining ray direction.
 * @param out Intersection point on circle.
 * @return True if intersection exists.
 */
bool projectAlongToCircleClosestToA(Point C, T r, Point p, Point a, Point &out) {
    Point d = a - p;
    T d2 = norm(d);
    if (d2 < EPS) {
        Point v = p - C;
        T dist = abs(v);
        out = dist < EPS ? C + Point(r, 0) : C + v * (r / dist);
        return true;
    }

    Point f = p - C;
    T fd = dot(f,
    d);
    T ff_r2 = norm(f) - r * r;
    T disc = fd * fd - d2 * ff_r2;
    if (disc < 0) return false;

    T sqrt_disc = sqrt(disc);
    T t1 = (-fd + sqrt_disc) / d2;
    T t2 = (-fd - sqrt_disc) / d2;
    Point I1 = p + d * t1;
    if (disc == 0) {
        out = I1;
        return true;
    }
    Point I2 = p + d * t2;
    out = abs(I1 - a) < abs(I2 - a) ? I1 : I2;
    return true;
}

/**
 * @brief Computes the unique circle tangent to two lines.
 * @param L1 First line.
 * @param L2 Second line.
 * @param center Output circle center.
 * @param radius Output circle radius.
 * @return True if circle exists.
 */
bool circleBetweenLines(Line &L1, Line &L2, Point &center, T &radius) {
    if (fabs(cross(L1.v, L2.v)) < EPS) {
        T d = fabs(L2.c - L1.c) / abs(L1.v);
        radius = d * 0.5;
        Line M{L1.v, (L1.c + L2.c) * 0.5};
        center = M.proj({0, 0});
        return true;
    }

    Point P;
    inter(L1, L2,
    P);
    Point on1 = P + perp(L1.v) / abs(L1.v);
    Point on2 = P + perp(L2.v) / abs(L2.v);
    Line bis = bisector(P, on1, on2, true);
    center = bis.proj({0, 0});
    radius = L1.dist(center);
    return true;
}

/**
 * @brief Computes the largest circle with diameter ab inside a rectangle.
 * @param a First endpoint of diameter.
 * @param b Second endpoint of diameter.
 * @param P1 Rectangle bottom-left corner.
 * @param P2 Rectangle top-right corner.
 * @return Pair of circle center and radius.
 */
pair<Point, T> constrainedCircleFromDiameter(Point a, Point b, Point P1, Point P2) {
    Point mid = (a + b) * T(0.5);
    T raw_r = abs(b - a) * 0.5;
    T max_r_left = mid.real() - P1.real();
    T max_r_right = P2.real() - mid.real();
    T max_r_bottom = mid.imag() - P1.imag();
    T max_r_top = P2.imag() - mid.imag();
    T fit_r = min({raw_r, max_r_left, max_r_right, max_r_bottom, max_r_top});
    return {mid, fit_r};
}

/**
 * @brief Computes a circle from 0, 1, or 2 boundary points.
 * @param R Vector of boundary points (0, 1, or 2).
 * @return Pair of circle center and radius.
 */
pair<Point, T> circleFrom(const vector<Point> &R) {
    if (R.empty()) return {{0, 0}, 0};
    if (R.size() == 1) return {R[0], 0};
    if (R.size() == 2) return {(R[0] + R[1]) / T(2.0), abs(R[0] - R[1]) / 2.0};
    return circumCircle(R[0], R[1], R[2]);
}

/**
 * @brief Helper for Welzl's minimum enclosing circle algorithm.
 * @param n Number of points to consider.
 * @param P Vector of points.
 * @param R Vector of boundary points.
 * @return Pair of circle center and radius.
 */
pair<Point, T> mecWelzl(int n, vector<Point> &P, vector<Point> &R, mt19937 &rng) {
    if (n == 0 || R.size() == 3) return circleFrom(R);

    int idx = uniform_int_distribution<>(0, n - 1)(rng);

    swap(P[idx], P[n - 1]);

    auto D = mecWelzl(n - 1, P, R, rng);

    if (fabs(P[n - 1] - D.first) <= D.second + EPS) return D;

    R.push_back(P[n - 1]);

    auto D2 = mecWelzl(n - 1, P, R, rng);

    R.pop_back();

    return D2;
}

/**
 * @brief Computes the minimum enclosing circle of a point set (Welzl's algorithm).
 * @param P Vector of points.
 * @return Pair of circle center and radius.
 */
pair<Point, T> minimumEnclosingCircle(vector<Point> P) {
    mt19937 rng(42); // Fixed seed for determinism
    vector<Point> R;
    return mecWelzl(P.size(), P, R, rng);
}

// incenter of the the circle in the triangle
Point incenter(Point A, Point B, Point C) {
    auto dist = [](Point p1, Point p2) {
         return sqrt((p1.real() - p2.real()) * (p1.real() - p2.real()) +
                     (p1.imag() - p2.imag()) * (p1.imag() - p2.imag()));
    };

    T a = dist(B, C);
    T b = dist(A, C);
    T c = dist(A, B);

    T px = (a * A.real() + b * B.real() + c * C.real()) / (a + b +
    c); T py = (a * A.imag() + b * B.imag() + c * C.imag()) / (a + b
    + c);

    return {px, py};
}

////////////////////////////////////////// Convex Hull //////////////////////////////////////////
// File: 08_convex_hull.hpp  Requires: 00_core.hpp, 01_basics.hpp, 03_angles.hpp, 06_polygons.hpp

struct PP{
    Point po;
    int idx;
};

/**
 * @brief Computes the convex hull of a point set using Andrew's monotone chain.
 * @param pts Vector of points (sorted and otherwise modified in place).
 * @return Vector of convex hull vertices in CCW order, starting at leftmost point.
 */
vector<PP> convexHull(vector<PP>& pts) {
    int n = pts.size(), k = 0;
    if (n <= 1) return pts;

    sort(pts.begin(), pts.end(), [](const PP &a, const PP &b) {
        if (fabs(a.po.real() - b.po.real()) > EPS) return a.po.real() < b.po.real();
        return a.po.imag() < b.po.imag();
    });

    vector<PP> H(2 * n);
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross(H[k - 1].po - H[k - 2].po, pts[i].po - H[k - 2].po) < -EPS) k--;
        H[k++] = pts[i];
    }
    for (int i = n - 2, t = k + 1; i >= 0; --i) {
        while (k >= t && cross(H[k - 1].po - H[k - 2].po, pts[i].po - H[k - 2].po) < -EPS) k--;
        H[k++] = pts[i];
    }
    if (k > 0)
        H.resize(k - 1);
    else
        H.clear();
    return H;
}


/**
 * @brief Computes the diameter (farthest pair distance) of a point set via
 * rotating calipers on its convex hull. O(n log n) total.
 * @param pts Vector of points (any order, need not be a hull already).
 * @return Maximum distance between any two points in pts.
 */
T polygonDiameter(vector<Point> pts) {
    int n = pts.size();
    if (n < 2) return 0;
    if (n == 2) return abs(pts[0] - pts[1]);

    vector<PP> tagged(n);
    for (int i = 0; i < n; i++) tagged[i] = {pts[i], i};
    vector<PP> hullTagged = convexHull(tagged);
    vector<Point> hull;
    for (auto &pp : hullTagged) hull.push_back(pp.po);

    int m = hull.size();
    if (m < 2) return 0;
    if (m == 2) return abs(hull[0] - hull[1]);

    T best = 0;
    int j = 1;
    for (int i = 0; i < m; i++) {
        Point edge = hull[(i + 1) % m] - hull[i];
        while (cross(edge, hull[(j + 1) % m] - hull[i]) > cross(edge, hull[j] - hull[i])) {
            j = (j + 1) % m;
        }
        best = max(best, norm(hull[i] - hull[j]));
        best = max(best, norm(hull[(i + 1) % m] - hull[j]));
    }
    return sqrt(best);
}

/**
 * @brief Reorders a polygon to start with the lowest point.
 * @param P Vector of polygon vertices (modified in place).
 */
void reorder_polygon(vector<Point> &P) {
    size_t pos = 0;
    for (size_t i = 1; i < P.size(); i++) {
        if (P[i].imag() < P[pos].imag() || (P[i].imag() == P[pos].imag() && P[i].real() < P[pos].real()))
            pos = i;
    }
    rotate(P.begin(), P.begin() + pos, P.end());
}

/**
 * @brief Computes the Minkowski sum of two convex polygons.
 * @param P First convex polygon (CCW).
 * @param Q Second convex polygon (CCW).
 * @return Vector of vertices of the Minkowski sum.
 */
vector<Point> minkowski(vector<Point> P, vector<Point> Q) {
    reorder_polygon(P);
    reorder_polygon(Q);
    P.push_back(P[0]); P.push_back(P[1]);
    Q.push_back(Q[0]); Q.push_back(Q[1]);
    vector<Point> result;
    size_t i = 0, j = 0;
    while (i < P.size() - 2 || j < Q.size() - 2) {
        result.push_back(P[i] + Q[j]);
        auto c = cross(P[i + 1] - P[i], Q[j + 1] - Q[j]);
        if (c >= 0 && i < P.size() - 2) ++i;
        if (c <= 0 && j < Q.size() - 2) ++j;
    }
    return result;
}

/**
 * @brief Computes the maximum distance between two convex polygons.
 * @param u First convex polygon.
 * @param v Second convex polygon.
 * @return Maximum distance between any pair of points.
 */
T maximum_dist_from_polygon_to_polygon(vector<Point> &u, vector<Point> &v) {
    int n = u.size(), m = v.size();
    T ans = 0;
    if (n < 3 || m < 3) {
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                ans = max(ans, norm(u[i] - v[j]));
        return sqrt(ans);
    }
    if (u[0].real() > v[0].real()) swap(n, m), swap(u, v);
    int i = 0, j = 0, step = n + m + 10;
    while (j + 1 < m && v[j].real() < v[j + 1].real()) j++;
    while (step--) {
        if (cross(u[(i + 1) % n] - u[i], v[(j + 1) % m] - v[j]) >= 0) j = (j + 1) % m;
        else i = (i + 1) % n;
        ans = max(ans, norm(u[i] - v[j]));
    }
    return sqrt(ans);
}

/**
 * @brief Checks if a point is inside a convex polygon using binary search.
 * @param conv Convex polygon vertices (CCW).
 * @param p Point to check.
 * @param isStrict If true, points on boundary are excluded.
 * @return True if p is inside or on boundary (if !isStrict).
 */
bool inConvex(vector<Point> conv, Point p, bool isStrict) {
    if (conv.size() < 3) return false;
    int n = conv.size();
    int l = 1, r = n - 2, ans = 1;
    while (l <= r) {
        int mid = (l + r) / 2;
        if (sign(orient(conv[0], conv[mid], p)) > 0) {
            ans = mid;
            l = mid + 1;
        } else r = mid - 1;
    }
    return inPolygon({conv[0], conv[ans], conv[ans + 1]}, p, isStrict);
}

////////////////////////////////////////// Half-planes //////////////////////////////////////////
// File: 09_halfplanes.hpp  Requires: 00_core.hpp, 01_basics.hpp

/**
 * @brief Represents a half-plane defined by a line.
 */
struct Halfplane {
    Point p, pq;   ///< p: point on line, pq: direction vector.
    T angle;       ///< Angle of the line (for sorting).

    Halfplane() {}
    Halfplane(Point a, Point b) : p(a), pq(b - a) {
        angle = atan2(pq.imag(), pq.real());
    }

    /// @brief Checks if a point is outside the half-plane.
    bool out(Point r) {
        return cross(pq, r - p) < -EPS;
    }

    /// @brief Comparator for sorting half-planes by angle.
    bool operator<(const Halfplane &e) const {
        return angle < e.angle;
    }

    /// @brief Computes intersection point of two half-plane lines.
    friend Point inter(const Halfplane &s, const Halfplane &t)
        { T alpha = cross(t.p - s.p, t.pq) / cross(s.pq, t.pq);
        return s.p + (s.pq * alpha);
    }
};

/**
 * @brief Computes the intersection of a set of half-planes.
 * @param H Vector of half-planes.
 * @return Vector of vertices of the intersection polygon (or empty if none).
 */
vector<Point> hp_intersect(vector<Halfplane> &H) {
    const int inf = 1e9;
    Point box[4] = {{inf, inf}, {-inf, inf}, {-inf, -inf}, {inf, -inf}};
    for (int i = 0; i < 4; i++) H.push_back(Halfplane(box[i], box[(i + 1) %
    4])); sort(H.begin(), H.end());
    deque<Halfplane> dq;
    int len = 0;
    for (int i = 0; i < (int)H.size(); i++) {
        while (len > 1 && H[i].out(inter(dq[len - 1], dq[len - 2]))) {
            dq.pop_back();
            --len;
        }
        while (len > 1 && H[i].out(inter(dq[0], dq[1]))) {
            dq.pop_front();
            --len;
        }
        if (len > 0 && fabs(cross(H[i].pq, dq[len - 1].pq)) < EPS) {
            if (dot(H[i].pq, dq[len - 1].pq) < 0.0) return vector<Point>();
            if (H[i].out(dq[len - 1].p)) {
                dq.pop_back();
                --len;
            } else continue;
        }
        dq.push_back(H[i]);
        ++len;
    }
    while (len > 2 && dq[0].out(inter(dq[len - 1], dq[len - 2]))) {
        dq.pop_back();
        --len;
    }
    while (len > 2 && dq[len - 1].out(inter(dq[0], dq[1]))) {
        dq.pop_front();
        --len;
    }
    if (len < 3) return vector<Point>();
    vector<Point> ret(len);
    for (int i = 0; i + 1 < len; i++) ret[i] = inter(dq[i], dq[i + 1]);
    ret.back() = inter(dq[len - 1], dq[0]);
    return ret;
}

////////////////////////////////////////// Polygon Orientation //////////////////////////////////////////
// File: 10_polygon_orientation.hpp  Requires: 00_core.hpp

/**
 * @brief Computes twice the signed area of a polygon.
 * @param poly Vector of polygon vertices.
 * @return Signed area (positive for CCW, negative for CW).
 */
T signed_area2(const vector<Point> &poly) {
    T A = 0;
    for (int i = 0, n = poly.size(); i < n; i++) {
        A += imag(conj(poly[i]) * poly[(i + 1) % n]);
    }
    return A;
}

/**
 * @brief Ensures a polygon is in counterclockwise (CCW) order.
 * @param poly Vector of polygon vertices (modified in place).
 */
void make_ccw(vector<Point> &poly) {
    if (signed_area2(poly) < 0) reverse(poly.begin(), poly.end());
}

/**
 * @brief Ensures a polygon is in clockwise (CW) order.
 * @param poly Vector of polygon vertices (modified in place).
 */
void make_cw(vector<Point> &poly) {
    if (signed_area2(poly) > 0) reverse(poly.begin(), poly.end());
}

////////////////////////////////////////// Polygon Union //////////////////////////////////////////
// File: 11_polygon_union.hpp  Requires: 00_core.hpp

/**
 * @brief Represents an event for the segment Facetree-based polygon union algorithm.
 */
struct Event {
    int x, y1, y2, type; ///< x: x-coordinate, y1/y2: y-range, type: +1 for start, -1 for end.
    bool operator<(const Event &o) const { return x < o.x; }
};

/**
 * @brief Segment tree for computing the area of union of rectangles.
 */
struct SegTree {
    struct Node { int cover; T length; }; ///< cover: number of active rectangles, length: covered length.
    vector<Node> st;
    vector<int> ys;
    int N;

    SegTree(const vector<int>& _ys) : ys(_ys) {
        N = ys.size() - 1;
        st.assign(4 * N, {0, 0});
    }

    /// @brief Updates the segment tree for a y-range with a value.
    void update(int p, int l, int r, int ql, int qr, int val) {
        if (qr <= l || r <= ql) return;
        if (ql <= l && r <= qr) {
            st[p].cover += val;
        } else {
            int m = (l + r) / 2;
            update(p << 1, l, m, ql, qr, val);
            update(p << 1 | 1, m, r, ql, qr,
            val);
        }
        if (st[p].cover > 0) {
            st[p].length = static_cast<T>(ys[r] - ys[l]);
        } else if (l + 1 == r) {
            st[p].length = 0;
        } else {
            st[p].length = st[p << 1].length + st[p << 1 | 1].length;
        }
    }

    /// @brief Updates the segment tree for a y-range.
    void update(int y1, int y2, int v) {
        int l = lower_bound(ys.begin(), ys.end(), y1) - ys.begin();
        int r = lower_bound(ys.begin(), ys.end(), y2) - ys.begin();
        update(1, 0, N, l, r, v);
    }

    /// @brief Returns the total covered length.
    T covered_length() const { return st[1].length; }
};

/**
 * @brief Computes the area of the union of rectangles.
 * @param rectangles Vector of rectangles as {x1, y1, x2, y2}.
 * @return Area of the union of rectangles.
 */
T areaUnionRectangles(const vector<array<int, 4>>& rectangles) {
    vector<Event> events;
    vector<int> ys;
    events.reserve(2 * rectangles.size());
    ys.reserve(2 * rectangles.size());

    for (const auto& r : rectangles) {
        int x1 = r[0], y1 = r[1], x2 = r[2], y2 = r[3];
        events.push_back({x1, y1, y2, +1});
        events.push_back({x2, y1, y2, -1});
        ys.push_back(y1);
        ys.push_back(y2);
    }
    if (events.empty()) return 0;
    sort(ys.begin(), ys.end());
    ys.erase(unique(ys.begin(), ys.end()), ys.end());

    SegTree st(ys);
    sort(events.begin(),
    events.end());

    T area = 0;
    int prev_x = events[0].x;
    for (const auto& e : events) {
        int dx = e.x - prev_x;
        area += st.covered_length() * dx;
        st.update(e.y1, e.y2, e.type);
        prev_x = e.x;
    }
    return area;
}

////////////////////////////////////////// Line Sweeping //////////////////////////////////////////
// File: 12_line_sweep.hpp  Requires: 00_core.hpp

/**
 * @brief Finds the closest pair of points using a line-sweeping algorithm.
 * @param pts Vector of points.
 * @return Squared distance of the closest pair.
 */
T closestPair(vector<Point> pts) {
    sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        return a.real() < b.real();
    });
    set<array<T, 2>> window;
    T ans = numeric_limits<T>::max();
    T sq = sqrt(ans);
    int j = 0;
    for (int i = 0; i < (int)pts.size(); i++) {
        while (j < (int)pts.size() && pts[i].real() - pts[j].real() > sq) {
            window.erase({pts[j].imag(), pts[j].real()});
            j++;
        }
        auto start = window.lower_bound({pts[i].imag() - sq,
        -numeric_limits<T>::max()}); auto end = window.upper_bound({pts[i].imag() + sq,
        numeric_limits<T>::max()}); for (auto it = start; it != end; ++it) {
            Point cur = {(*it)[1], (*it)[0]};
            T d = norm(pts[i] - cur);
            if (d < ans) {
                ans = d;
                sq = sqrt(d);
            }
        }
        window.insert({pts[i].imag(), pts[i].real()});
    }
    return ans;
}

////////////////////////////////////////// Delaunay Triangulation //////////////////////////////////////////
// File: 13_delaunay.hpp  Requires: 00_core.hpp, 01_basics.hpp

/**
 * @brief Represents a triangle with circumcircle information for Bowyer-Watson algorithm.
 */
struct Triangle {
    Point a, b, c;     ///< Triangle vertices.
    Point circum;      ///< Circumcenter.
    T radius2;         ///< Squared circumradius.

    Triangle(Point a, Point b, Point c) : a(a), b(b), c(c) {
        T d = 2 * cross(b - a, c - a);
        if (fabs(d) < EPS) {
            circum = {0, 0};
            radius2 = -1;
        } else {
            Point u = b - a, v = c - a;
            Point num = perp(u) * norm(v) - perp(v) * norm(u);
            circum = a + num / d;
            radius2 = norm(circum - a);
        }
    }

    /// @brief Checks if a point lies within the triangle's circumcircle.
    bool containsInCircum(const Point& p) const {
        if (radius2 < 0) return false;
        return norm(p - circum) <= radius2 + EPS;
    }
};

/**
 * @brief Represents an edge for Delaunay triangulation.
 */
struct Edge {
    Point u, v;
    bool operator<(const Edge& o) const {
        if (abs(u.real() - o.u.real()) > EPS) return u.real() < o.u.real();
        if (abs(u.imag() - o.u.imag()) > EPS) return u.imag() < o.u.imag();
        if (abs(v.real() - o.v.real()) > EPS) return v.real() < o.v.real();
        return v.imag() < o.v.imag();
    }
};

/**
 * @brief Computes the largest circle in a rectangle avoiding obstacles using Bowyer-Watson.
 * @param n Rectangle width.
 * @param m Rectangle height.
 * @param pts Vector of obstacle points.
 * @return Radius of the largest circle.
 */
T largestCircleInRectangle(T n, T m, vector<Point> pts) {
    T mx = max(n, m) * 10;
    Point A(0, -mx), B(mx * 2, 0), C(0, mx * 2);
    vector<Triangle> tris{Triangle(A, B, C)};

    // Bowyer-Watson insertion
    for (const auto& pt : pts) {
        vector<Edge> boundary;
        vector<Triangle> new_tris;
        for (const auto& tri : tris) {
            if (tri.containsInCircum(pt)) {
                boundary.emplace_back(Edge{tri.a, tri.b});
                boundary.emplace_back(Edge{tri.b, tri.c});
                boundary.emplace_back(Edge{tri.c, tri.a});
            } else {
                new_tris.push_back(tri);
            }
        }
        map<Edge, int> cnt;
        for (const auto& e : boundary) cnt[e]++;
        boundary.clear();
        for (const auto& kv : cnt) if (kv.second == 1) boundary.push_back(kv.first);
        for (const auto& e : boundary) {
            new_tris.emplace_back(e.u, e.v, pt);
        }
        tris.swap(new_tris);
    }

    // Remove triangles with super-triangle vertices
    vector<Triangle> final_tris;
    for (const auto& tri : tris) {
        auto same = [&](Point u, Point v) { return abs(u - v) < EPS; };
        if (same(tri.a, A) || same(tri.a, B) || same(tri.a, C) ||
            same(tri.b, A) || same(tri.b, B) || same(tri.b, C) ||
            same(tri.c, A) || same(tri.c, B) || same(tri.c, C)) continue;
        final_tris.push_back(tri);
    }

    // Collect candidate centers
    vector<Point> candidates;
    candidates.reserve(pts.size() * 5 + final_tris.size());
    candidates.emplace_back(0, 0);
    candidates.emplace_back(n, 0);
    candidates.emplace_back(0, m);
    candidates.emplace_back(n, m);
    for (const auto& o : pts) {
        T x = o.real(), y = o.imag();
        if (y >= 0 && y <= m) {
            candidates.emplace_back(0, y);
            candidates.emplace_back(n, y);
        }
        if (x >= 0 && x <= n) {
            candidates.emplace_back(x, 0);
            candidates.emplace_back(x, m);
        }
    }
   for (const auto& tri : final_tris) {
        candidates.push_back(tri.circum);
    }

    // Evaluate best radius
    T best = 0;
    for (const auto& C : candidates) {
        T cx = C.real(), cy = C.imag();
        if (cx < 0 || cx > n || cy < 0 || cy > m) continue;
        T d_obs = numeric_limits<T>::max();
        for (const auto& o : pts) d_obs = min(d_obs, abs(C - o));
        T d_wall = min({cx, n - cx, cy, m - cy});
        best = max(best, min(d_obs, d_wall));
    }
    return best;
}

struct Segment {
    Point a, b;
};

// Compute intersection point of ray (origin → dir) with segment (p1-p2)
bool raySegmentIntersection(Point origin, Point dir, Segment seg, Point& out) {
    Point p = origin;
    Point r = dir;
    Point q = seg.a;
    Point s = seg.b - seg.a;

    T rxs = cross(r, s);
    if (abs(rxs) < EPS) return false; // Parallel

    T t = cross((q - p), s) / rxs;
    T u = cross((q - p), r) / rxs;

    if (u < -EPS || u > 1+EPS) return false;
    out = p + r * t;
    return true;
}

Point reflect(const Point& p, const Point& c) {
    return T(2.0) * c - p;
}

void getPoint(Point &p){
    T x, y; cin >> x >> y;

    p = {x, y};
}

bool segSegIntersect(const Point& A, const Point& B,
                     const Point& C, const Point& D,
                     Point& out) {
    Point r = B - A;
    Point s = D - C;
    T denom = cross(r, s);

    // If denom == 0, lines are parallel or collinear
    if (std::fabs(denom) < EPS) {
        return false;
    }

    Point AC = C - A;
    T t = cross(AC, s) / denom;
    T u = cross(AC, r) / denom;

    // Check if 0 ≤ t ≤ 1 and 0 ≤ u ≤ 1 → intersection lies within both segments
    if (t < -EPS || t > 1.0 + EPS || u < -EPS || u > 1.0 + EPS)
        return false;

    // Compute intersection point
    out = A + r * t;
    return true;
}

static vector<Point> clipAgainstLine(const vector<Point>& poly,
                                     const Point& A, const Point& B) {
    vector<Point> out;
    int n = poly.size();
    for(int i = 0; i < n; i++){
        Point U = poly[i];
        Point V = poly[(i+1)%n];
        T cU = cross(B - A, U - A);
        T cV = cross(B - A, V - A);
        bool inU = cU >= -EPS;
        bool inV = cV >= -EPS;
        if(inU)
            out.push_back(U);
        if(inU ^ inV) {
            // find t so that U + t*(V-U) lies on line A→B
            Point dir = V - U;
            T num = -cross(B - A, U - A);
            T den = cross(B - A, dir);
            if(fabs(den) > EPS) {
                T t = num / den;
                out.push_back(U + dir *
                t);
            }
        }
    }
    return out;
}

/// Compute the *kernel* of a (simple) polygon `poly` in CCW order.
/// Returns an empty vector if the kernel is empty.
vector<Point> polygonKernel(const vector<Point>& poly) {
    vector<Point> kernel = poly;
    int n = poly.size();
    for(int i = 0; i < n && !kernel.empty(); i++){
        kernel = clipAgainstLine(kernel,
                                 poly[i],
                                 poly[(i+1)%n]);
    }
    return kernel;
}

////////////////////////////////////////// Rectangles //////////////////////////////////////////
// File: 14_rectangles.hpp  Requires: 00_core.hpp, 01_basics.hpp

/**
 * @brief EPS-aware comparison of two doubles, like a 3-way compare.
 * @return -1 if a < b, +1 if a > b, 0 if equal within EPS.
 */
int dcmp(T a, T b) { return sign(a - b); }

/**
 * @brief An axis-aligned rectangle, stored canonicalized (x1<=x2, y1<=y2).
 */
struct Rect {
    T x1, y1, x2, y2; // left, bottom, right, top

    Rect() {}
    Rect(T x1, T y1, T x2, T y2) : x1(x1), y1(y1), x2(x2), y2(y2) { canonicalize(); }

    void canonicalize() {
        if (dcmp(x1, x2) > 0) swap(x1, x2);
        if (dcmp(y1, y2) > 0) swap(y1, y2);
    }
};

/**
 * @brief Prints the overlap of two rectangles to stdout, or "No Overlap".
 * Solution-style helper (has I/O side effects) -- for pure computation use
 * intersectRectangles() instead.
 */
void printRectIntersection(Rect a, Rect b) {
    if (dcmp(b.x2, a.x1) <= 0 || dcmp(b.x1, a.x2) >= 0 ||
        dcmp(b.y2, a.y1) <= 0 || dcmp(b.y1, a.y2) >= 0) {
        cout << "No Overlap\n";
        return;
    }
    if (dcmp(b.x1, a.x1) > 0) a.x1 = b.x1;
    if (dcmp(b.x2, a.x2) < 0) a.x2 = b.x2;
    if (dcmp(b.y1, a.y1) > 0) a.y1 = b.y1;
    if (dcmp(b.y2, a.y2) < 0) a.y2 = b.y2;
    cout << a.x1 << " " << a.y1 << " " << a.x2 << " " << a.y2 << "\n";
}

/**
 * @brief Splits rectangle a by rectangle b and returns the overlapping piece.
 * @param out Output array (size 4) receiving the parts of a that lie outside b.
 * @param nout -2: no intersection (out untouched). -1: a is entirely inside b
 *   (out untouched). >=0: number of pieces written to out, and the returned
 *   Rect is the overlap of a and b.
 * @return The overlapping rectangle (only meaningful when nout >= -1).
 */
Rect intersectRectangles(Rect a, Rect b, Rect out[4], int &nout) {
    nout = -2;
    if (dcmp(b.x2, a.x1) < 0 || dcmp(b.x1, a.x2) > 0 ||
        dcmp(b.y2, a.y1) < 0 || dcmp(b.y1, a.y2) > 0)
        return Rect(0, 0, 0, 0); // no intersection

    nout = -1;
    if (dcmp(b.x1, a.x1) <= 0 && dcmp(b.x2, a.x2) >= 0 &&
        dcmp(b.y1, a.y1) <= 0 && dcmp(b.y2, a.y2) >= 0)
        return a; // a entirely inside b

    Rect t;
    nout = 0;
    if (dcmp(b.x1, a.x1) > 0) { t = a; t.x2 = b.x1; a.x1 = b.x1; out[nout++] = t; } // left
    if (dcmp(b.x2, a.x2) < 0) { t = a; t.x1 = b.x2; a.x2 = b.x2; out[nout++] = t; } // right
    if (dcmp(b.y1, a.y1) > 0) { t = a; t.y2 = b.y1; a.y1 = b.y1; out[nout++] = t; } // bottom
    if (dcmp(b.y2, a.y2) < 0) { t = a; t.y1 = b.y2; a.y2 = b.y2; out[nout++] = t; } // top
    return a; // a now represents the overlap
}

////////////////////////////////////////// Segment Intersection Sweep //////////////////////////////////////////
// File: 15_segment_sweep.hpp  Requires: 00_core.hpp, 01_basics.hpp, 05_segments.hpp

/**
 * @brief A segment tagged for the sweep-line status structure.
 */
struct SweepSeg {
    Point p, q; ///< Endpoints, with p.real() <= q.real().
    int id;

    /// @brief y-coordinate of this segment at a given x (vertical segments
    /// fall back to their lower endpoint -- only ever queried at an x where
    /// both compared segments are simultaneously active).
    T at(T x) const {
        if (fabs(p.real() - q.real()) < EPS) return min(p.imag(), q.imag());
        return p.imag() + (q.imag() - p.imag()) * (x - p.real()) / (q.real() - p.real());
    }
};

/**
 * @brief Orders two active segments by height at the x where both are
 * simultaneously defined (the later of their two left endpoints). This
 * avoids needing a mutable "current sweep x" global.
 */
bool operator<(const SweepSeg &a, const SweepSeg &b) {
    T x = max(min(a.p.real(), a.q.real()), min(b.p.real(),
    b.q.real())); T ya = a.at(x), yb = b.at(x);
    if (fabs(ya - yb) > EPS) return ya < yb;
    return a.id < b.id;
}

/**
 * @brief Sweep-line event: a segment's left endpoint (type +1) or right
 * endpoint (type -1).
 */
struct SweepEvt {
    T x;
    int type, id;
    bool operator<(const SweepEvt &e) const {
        if (fabs(x - e.x) > EPS) return x < e.x;
        return type > e.type; // process opens before closes at the same x
    }
};

/**
 * @brief Determines whether any two of n segments intersect or touch, in
 * O(n log n), by sweeping left to right and only testing segments that
 * become adjacent in the sweep-line status structure.
 * @param P Segment start points.
 * @param Q Segment end points (parallel array to P).
 * @param out Set to a pair of indices of an intersecting/touching pair, if found.
 * @return True if some pair of segments intersects or touches.
 */
bool anySegmentsIntersect(vector<Point> P, vector<Point> Q, pair<int, int> &out) {
    int n = P.size();
    vector<SweepSeg> segs(n);
    vector<SweepEvt> events;
    events.reserve(2 * n);
    for (int i = 0; i < n; i++) {
        Point p = P[i], q = Q[i];
        if (p.real() > q.real()) swap(p, q);
        segs[i] = {p, q, i};
        events.push_back({p.real(), +1, i});
        events.push_back({q.real(), -1, i});
    }
    sort(events.begin(), events.end());

    set<SweepSeg> active;
    vector<set<SweepSeg>::iterator> where(n);

    auto touches = [&](int i, int j) {
        if (segSeg(P[i], Q[i], P[j], Q[j]) <= EPS) { out = {i, j}; return true; }
        return false;
    };

    for (auto &e : events) {
        int id = e.id;
        if (e.type == 1) {
            auto it = active.insert(segs[id]).first;
            where[id] = it;
            auto nxt = next(it);
            if (nxt != active.end() && touches(id, nxt->id)) return true;
            if (it != active.begin() && touches(id, prev(it)->id)) return true;
        } else {
            auto it = where[id];
            auto nxt = next(it);
            bool hasPrev = it != active.begin();
            if (hasPrev && nxt != active.end()) {
                auto prv = prev(it);
                if (touches(prv->id, nxt->id)) { active.erase(it); return true; }
            }
            active.erase(it);
        }
    }
    return false;
}

/**
 * @brief Reports every pairwise intersection point among n segments.
 * Straightforward O(n^2) using intersectSegments() -- robust and simple,
 * unlike a full reporting-Bentley-Ottmann sweep which is substantially more
 * intricate to get right. Use anySegmentsIntersect() first if you only need
 * a yes/no answer on larger n; use this when n is small/moderate or you
 * specifically need every intersection point (including overlaps).
 * @param P Segment start points.
 * @param Q Segment end points (parallel array to P).
 * @return Vector of (i, j, point) for every pair (i<j) that intersects, one
 * entry per intersection point (2 entries for collinear overlaps).
 */
vector<tuple<int, int, Point>> allSegmentIntersections(const vector<Point> &P, const vector<Point> &Q) {
    vector<tuple<int, int, Point>> res;
    int n = P.size();
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) {
            for (auto pt : intersectSegments(P[i], Q[i], P[j],
                Q[j])) res.emplace_back(i, j, pt);
        }
    return res;
}

////////////////////////////////////////// Convex Polygon Clipping //////////////////////////////////////////
// File: 16_convex_clip.hpp  Requires: 00_core.hpp, 01_basics.hpp, 06_polygons.hpp

/**
 * @brief Clips a (possibly non-convex) polygon against a single convex
 * clip polygon's edge, keeping the side left of A->B. Building block for
 * clipConvexPolygon(); reused from the polygon-kernel routine.
 * @param poly Polygon to clip (CCW).
 * @param A First point of the clip edge.
 * @param B Second point of the clip edge.
 * @return The clipped polygon (may be empty).
 */
vector<Point> clipEdge(const vector<Point> &poly, const Point &A, const Point &B) {
    vector<Point> out;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        Point U = poly[i], V = poly[(i + 1) % n];
        T cU = cross(B - A, U - A), cV = cross(B - A, V - A);
        bool inU = cU >= -EPS, inV = cV >= -EPS;
        if (inU) out.push_back(U);
        if (inU ^ inV) {
            Point dir = V - U;
            T num = -cross(B - A, U - A), den = cross(B - A, dir);
            if (fabs(den) > EPS) out.push_back(U + dir * (num / den));
        }
    }
    return out;
}

/**
 * @brief Clips subject against a convex clip polygon (Sutherland-Hodgman).
 * @param subject Polygon to clip (any simple polygon, CCW).
 * @param clipPoly Convex clip polygon, CCW order.
 * @return The part of subject inside clipPoly (may be empty).
 */
vector<Point> clipConvexPolygon(vector<Point> subject, const vector<Point> &clipPoly) {
    int n = clipPoly.size();
    for (int i = 0; i < n && !subject.empty(); i++) {
        subject = clipEdge(subject, clipPoly[i], clipPoly[(i + 1) % n]);
    }
    return subject;
}

/**
 * @brief Computes the area of intersection of two convex polygons.
 * @param A First convex polygon, CCW order.
 * @param B Second convex polygon, CCW order.
 * @return Intersection area (0 if they don't overlap).
 */
T convexPolygonIntersectionArea(const vector<Point> &A, const vector<Point> &B) {
    vector<Point> clipped = clipConvexPolygon(A, B);
    if (clipped.size() < 3) return 0;
    return areaPolygon(clipped);
}

////////////////////////////////////////// Rotating Calipers //////////////////////////////////////////
// File: 17_calipers.hpp  Requires: 00_core.hpp, 01_basics.hpp, 08_convex_hull.hpp

/**
 * @brief The minimum-area bounding rectangle of a point set, not
 * necessarily axis-aligned.
 */
struct MinRect {
    Point corner[4]; ///< The 4 corners, in order.
    T area;          ///< Its area.
};

/**
 * @brief Computes the minimum-area bounding rectangle of a point set via
 * rotating calipers over the convex hull for the width, and a direct O(n)
 * scan for the along-edge extent (kept simple/robust rather than a second
 * pair of rotating pointers). O(n^2) worst case in the hull size.
 * @param pts Vector of points (any order).
 * @return The minimum-area bounding rectangle.
 */
MinRect minBoundingRectangle(vector<Point> pts) {
    int n0 = pts.size();
    if (n0 == 0) return {{{0, 0}, {0, 0}, {0, 0}, {0, 0}}, 0};
    vector<PP> tagged(n0);
    for (int i = 0; i < n0; i++) tagged[i] = {pts[i], i};
    auto hullTagged = convexHull(tagged);
    vector<Point> H;
    for (auto &pp : hullTagged) H.push_back(pp.po);
    int n = H.size();
    if (n == 1) return {{H[0], H[0], H[0], H[0]}, 0};
    if (n == 2) return {{H[0], H[1], H[1], H[0]}, 0};

    T bestArea = numeric_limits<T>::max();
    MinRect best;
    int j = 1;
    for (int i = 0; i < n; i++) {
        Point base = H[(i + 1) % n] - H[i];
        Point edge = base / abs(base);
        while (cross(edge, H[(j + 1) % n] - H[i]) > cross(edge, H[j] - H[i]) + EPS) j = (j + 1) % n;
        T width = cross(edge, H[j] - H[i]);

        T high = -numeric_limits<T>::max(), low = numeric_limits<T>::max();
        for (int t = 0; t < n; t++) {
            T proj = dot(edge, H[t] - H[i]);
            high = max(high, proj);
            low = min(low, proj);
        }

        T area = width * (high - low);
        if (area < bestArea) {
            bestArea = area;
            Point normal = perp(edge);
            Point c1 = H[i] + edge * low;
            Point c2 = H[i] + edge * high;
            Point c3 = c2 + normal * width;
            Point c4 = c1 + normal * width;
            best = {{c1, c2, c3, c4}, area};
        }
    }
    return best;
}

/**
 * @brief Computes the minimum width (smallest extent over all directions)
 * of a point set, via rotating calipers over the convex hull.
 * @param pts Vector of points (any order).
 * @return The minimum width (0 if fewer than 2 distinct points).
 */
T convexMinWidth(vector<Point> pts) {
    int n0 = pts.size();
    if (n0 < 2) return 0;
    vector<PP>
    tagged(n0);
    for (int i = 0; i < n0; i++) tagged[i] = {pts[i], i};
    auto hullTagged = convexHull(tagged);
    vector<Point> H;
    for (auto &pp : hullTagged) H.push_back(pp.po);
    int n = H.size();
    if (n < 3) return 0; // collinear point set has width 0

    T best = numeric_limits<T>::max();
    int j = 1;
    for (int i = 0; i < n; i++) {
        Point base = H[(i + 1) % n] - H[i];
        Point edge = base / abs(base);
        while (cross(edge, H[(j + 1) % n] - H[i]) > cross(edge, H[j] - H[i]) + EPS) j = (j + 1) % n;
        T width = cross(edge, H[j] - H[i]);
        best = min(best, width);
    }
    return best;
}

////////////////////////////////////////// Polygon Triangulation //////////////////////////////////////////
// File: 18_triangulation.hpp  Requires: 00_core.hpp, 01_basics.hpp, 03_angles.hpp

/**
 * @brief Triangulates a simple polygon by ear clipping, O(n^2).
 * @param poly Simple polygon vertices, CCW order (use make_ccw() first if unsure).
 * @return Vector of n-2 triangles (each an array of 3 points) whose union is
 * the polygon and whose total area equals the polygon's area.
 */
vector<array<Point, 3>> triangulate(const vector<Point> &poly) {
    int n = poly.size();
    vector<int> idx(n);
    iota(idx.begin(), idx.end(), 0);
    vector<array<Point, 3>> tris;

    while ((int)idx.size() > 3) {
        int m = idx.size();
        bool found = false;
        for (int k = 0; k < m; k++) {
            int iprev = idx[(k - 1 + m) % m], icur = idx[k], inext = idx[(k + 1) % m];
            Point a = poly[iprev], b = poly[icur], c = poly[inext];
            if (orient(a, b, c) <= EPS) continue; // reflex or degenerate vertex, not an ear tip

            bool anyInside = false;
            for (int t = 0; t < m; t++) {
                int j = idx[t];
                if (j == iprev || j == icur || j == inext) continue;
                if (sign(orient(a, b, poly[j])) >= 0 && sign(orient(b, c, poly[j])) >= 0 &&
                    sign(orient(c, a, poly[j])) >= 0) {
                    anyInside = true;
                    break;
                }
            }
            if (!anyInside) {
                tris.push_back({a, b, c});
                idx.erase(idx.begin() +
                k); found = true;
                break;
            }
        }
        if (!found) break; // shouldn't happen for a valid simple polygon
    }
    if (idx.size() == 3) tris.push_back({poly[idx[0]], poly[idx[1]], poly[idx[2]]});
    return tris;
}

////////////////////////////////////////// 3D Geometry Basics //////////////////////////////////////////
// File: 19_geometry3d.hpp  Requires: 00_core.hpp
// Independent of the 2D Point (complex<T>) used everywhere above -- 3D needs
// its own point type since complex<T> only has 2 components.

/**
 * @brief A 3D point / vector.
 */
struct Point3D {
    T x, y, z;
    Point3D operator+(const Point3D &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Point3D operator-(const Point3D &o) const { return {x - o.x, y - o.y, z - o.z}; }
    Point3D operator*(T k) const { return {x * k, y * k, z * k}; }
    Point3D operator/(T k) const { return {x / k, y / k, z / k}; }
};

/// @brief Dot product of two 3D vectors.
T dot3(Point3D a, Point3D b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// @brief Cross product of two 3D vectors.
Point3D cross3(Point3D a, Point3D b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// @brief Squared magnitude of a 3D vector.
T norm3(Point3D a) { return dot3(a, a); }

/// @brief Magnitude (length) of a 3D vector.
T abs3(Point3D a) { return sqrt(norm3(a)); }

/// @brief Euclidean distance between two 3D points.
T distance3(Point3D a, Point3D b) { return abs3(a - b); }

/// @brief Angle between two 3D vectors, in radians [0, PI].
T angleBetween3(Point3D a, Point3D b) {
    return acos(clamp(dot3(a, b) / (abs3(a) * abs3(b)), T(-1), T(1)));
}

/**
 * @brief A plane, stored as a normal n and offset d such that a point p is
 * on the plane iff dot3(n, p) == d.
 */
struct Plane3D {
    Point3D n;
    T d;
};

/// @brief Builds the plane through 3 non-collinear points.
Plane3D planeFromPoints(Point3D a, Point3D b, Point3D c) {
    Point3D n = cross3(b - a, c - a);
    return {n, dot3(n, a)};
}

/// @brief Signed distance from a point to a plane (positive on the normal's side).
T distPointPlane3(Point3D p, Plane3D pl) { return (dot3(pl.n, p) - pl.d) / abs3(pl.n); }

/// @brief Projects a point onto a plane.
Point3D projectOntoPlane3(Point3D p, Plane3D pl)
    { T t = (dot3(pl.n, p) - pl.d) / norm3(pl.n);
    return p - pl.n * t;
}

/// @brief Volume of the tetrahedron with the given 4 vertices.
T volumeTetrahedron(Point3D a, Point3D b, Point3D c, Point3D d) {
    return fabs(dot3(cross3(b - a, c - a), d - a)) / 6.0;
}

/// @brief Area of the triangle with the given 3 vertices, in 3D.
T areaTriangle3D(Point3D a, Point3D b, Point3D c) { return abs3(cross3(b - a, c - a)) / 2.0; }


// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN
// MORE DOWN

















////////////////////////////////////////// Appendix: Worked Examples //////////////////////////////////////////
// These are complete, self-contained contest solutions (their own Point/globals/solve()), not reusable library
// snippets. Read them for the technique, don't paste them alongside the numbered categories above.
//
// Disabled with #if 0 because they redefine Point/cross/dist/etc. and would clash
// with the library above. Change to #if 1 only in a separate file / standalone build.
#if 0

// Points visible from a point (radial sweep over directions)
//
// Given n points as seen from the origin, find which ones are hidden behind a closer point lying on the exact same
// ray through the origin. Groups points by direction (x/gcd, y/gcd), keeps only the closest point per direction, then
// does a randomized edge sweep to test occlusion between remaining candidates.

const double pi = acos(-1);
const double eps = 1e-9;
const long long oo = 1e18;
typedef complex<long long> point;
#define X real()
#define Y imag()

long long cross(const point &a, const point &b) { return a.X * b.Y - a.Y * b.X; }

long long ccw(point &a, point &b, point &c) { return cross(c - a, c - b); }

long long dist(point &a) { return a.X * a.X + a.Y * a.Y; }

vector<point> arr;

bool sorting(pair<point, int> &a, pair<point, int> &b) {
    if (a.first.Y * b.first.X > a.first.X * b.first.Y) return true;
    if (a.first.Y * b.first.X == a.first.X * b.first.Y) {
        if (dist(a.first) > dist(b.first)) return true;
        else if (dist(a.first) == dist(b.first)) return a.second > b.second;
    }
    return false;
}
mt19937
rng(chrono::steady_clock::now().time_since_epoch().count()); void
solve() {
    int n;
    cin >> n;
    arr = vector<point>(n);
    map<pair<long long, long long>, vector<pair<long long, long long>>> mp;
    set<int> st;
    for (int i = 0; i < n; i++) st.insert(i);
    for (int i = 0; i < n; i++) {
        long long x, y;
        cin >> x >> y;
        long long d = __gcd(x, y);
        arr[i] = point(x, y);
        mp[{x / d, y / d}].push_back({dist(arr[i]), i});
    }

    vector<pair<point, int>> a(n);
    for (int i = 0; i < n; i++) a[i] = {arr[i], i};
    sort(a.begin(), a.end(), sorting);
    reverse(a.begin(), a.end());

    vector<int> rev(n);
    for (int i = 0; i < n; i++) rev[a[i].second] = i;

// keep only the closest point per direction
    for (auto x : mp) {
        vector<pair<long long, long long>> t = x.second;
        sort(t.begin(), t.end());
        for (size_t i = 1; i < t.size(); i++) st.erase(rev[t[i].second]);
    }

    vector<pair<int, int>> e(n);
    for (int i = 0; i < n; i++) e[i] = {i, (i + 1) % n};
    shuffle(e.begin(), e.end(), rng);

    for (int i = 0; i < n; i++) {
        int ind1 = rev[e[i].first], ind2 =
        rev[e[i].second]; if (ind1 > ind2) swap(ind1,
        ind2);
        auto it = st.upper_bound(ind1);
        vector<int> rem;
        for (; it != st.end() && *it < ind2; it++) {
            int j = *it;
            if (ccw(a[ind1].first, a[ind2].first, a[j].first) < 0) rem.push_back(j);
        }
        for (auto x : rem) st.erase(x);
    }

    cout << st.size() << "\n";
    vector<int> res;
    for (auto x : st) res.push_back(a[x].second + 1);
    sort(res.begin(), res.end());
    for (auto x : res) cout << x << " ";
    cout << "\n";
}

#endif
