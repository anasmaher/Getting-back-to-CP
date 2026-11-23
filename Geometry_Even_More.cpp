// (x, y) -> (x + y, x - y)
// Manhattan dist between p, q = Chebyshev dist between transformed
// Useful: Manhattan minimum enclosing circle, Manhattan MST, etc.
Point toChebyshev(Point p) { return {p.real() + p.imag(), p.real() - p.imag()}; }
Point fromChebyshev(Point p) {
    return {(p.real() + p.imag()) / 2, (p.real() - p.imag()) / 2};
}

Point closestOnSegment(Point a, Point b, Point p) {
    Point d = b - a;
    T t = dot(p - a, d) / norm(d);
    t = max(T(0), min(T(1), t));
    return a + d * t;
}

T distPointConvexPolygon(const vector<Point>& cvx, Point p) {
    // assumes cvx is CCW
    int n = cvx.size();
    // find the two adjacent hull vertices that bracket the direction to p
    // (or just check all edges — O(n), usually fine)
    T best = 1e100;
    for (int i = 0; i < n; i++)
        best = min(best, segPoint(cvx[i], cvx[(i+1) % n], p));
    return best;
}

// Sort points by angle around c, use two-pointer to find max in any half-plane
int maxPointsInHalfPlane(vector<Point> pts, Point c) {
    vector<pair<T, int>> ang;
    for (auto& p : pts) ang.push_back({atan2(p.imag()-c.imag(), p.real()-c.real()), 1});
    sort(ang.begin(), ang.end());
    int m = ang.size(), best = 0, r = 0, cur = 0;
    for (int i = 0; i < m; i++) {
        while (r < i + m && (r < m ? ang[r].first : ang[r-m].first + 2*PI) - ang[i].first <= PI + EPS) {
            cur += (r < m ? ang[r].second : ang[r-m].second); r++;
        }
        best = max(best, cur);
        cur -= ang[i].second;
    }
    return best;
}
