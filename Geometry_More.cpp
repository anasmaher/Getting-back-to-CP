// Returns the two tangent points from p to circle(o, r).
// Precondition: |p - o| >= r.
pair<Point, Point> tangentPointToCircle(Point p, Point o, T r) {
    Point v = o - p;
    T d2 = norm(v), d = sqrt(d2);
    T a = r * r / d2;              // projection factor
    T b = r * sqrt(d2 - r * r) / d2;
    Point mid = p + v * a;
    Point perp_v = perp(v) * b;
    return {mid + perp_v, mid - perp_v};
}

// Returns 0, 1, or 2 intersection points of segment [a,b] with circle(o,r).
vector<Point> segmentCircle(Point a, Point b, Point o, T r) {
    Line l(a, b);
    pair<Point, Point> pts; int k = circleLine(o, r, l, pts);
    vector<Point> res;
    auto on = [&](Point p) { return dot(p - a, p - b) <= EPS; };
    if (k >= 1 && on(pts.first))  res.push_back(pts.first);
    if (k >= 2 && on(pts.second)) res.push_back(pts.second);
    return res;
}

// p inside or on triangle abc (order-independent, robust).
bool inTriangle(Point a, Point b, Point c, Point p, bool strict = false) {
    T o1 = orient(a, b, p), o2 = orient(b, c, p), o3 = orient(c, a, p);
    bool has_neg = (o1 < -EPS) || (o2 < -EPS) || (o3 < -EPS);
    bool has_pos = (o1 >  EPS) || (o2 >  EPS) || (o3 >  EPS);
    if (strict) return !(has_neg || has_pos);
    return !(has_neg && has_pos);
}

// Center of mass of a polygon (assuming uniform density).
Point centroid(const vector<Point>& p) {
    T A = 0; Point c{0, 0};
    for (int i = 0, n = p.size(); i < n; i++) {
        T cross_v = cross(p[i], p[(i+1) % n]);
        A += cross_v;
        c += (p[i] + p[(i+1) % n]) * cross_v;
    }
    return c / (3 * A);
}

void polarSort(vector<Point>& v) {
    sort(v.begin(), v.end(), [](Point a, Point b) {
        if (half(a) != half(b)) return half(a) > half(b);
        return cross(a, b) > 0;
    });
}

T rectArea(Rect r) { return (r.x2 - r.x1) * (r.y2 - r.y1); }
bool rectsOverlap(Rect a, Rect b) {
    return dcmp(a.x2, b.x1) > 0 && dcmp(b.x2, a.x1) > 0
        && dcmp(a.y2, b.y1) > 0 && dcmp(b.y2, a.y1) > 0;
}
bool rectContains(Rect r, Point p) {
    return r.x1 <= p.real() && p.real() <= r.x2
        && r.y1 <= p.imag() && p.imag() <= r.y2;
}
bool pointOnRectBoundary(Rect r, Point p) { /* ... */ }

bool lineTangentToCircle(Line l, Point o, T r) {
    return fabs(l.dist(o) - r) < EPS;
}
