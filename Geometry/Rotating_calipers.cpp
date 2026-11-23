using pii = pair<int, int>;

// Antipodal pairs of a STRICTLY CONVEX polygon p (CCW or CW, either works).
// Returns all (edge-endpoint, opposite-vertex) pairs.
// Time: O(n).  Precondition: n >= 3, p strictly convex.
vector<pii> all_antipodal(int n, vector<Point>& p) {
    if (n < 3) return {};
    auto nx = [&](int i) { return (i + 1) % n; };
    auto pv = [&](int i) { return (i - 1 + n) % n; };

    vector<pii> result;
    vector<char> vis(n, false);
    int p1 = 0, p2 = 0;

    for (; p1 < n; p1++) {
        Point base = p[nx(p1)] - p[p1];
        int guard = 0;
        while ((p2 == p1 || p2 == nx(p1) ||
               sign(cross(base, p[nx(p2)] - p[p2])) ==
               sign(cross(base, p[p2]  - p[pv(p2)])))
               && ++guard <= 2 * n)
        {
            p2 = nx(p2);
        }
        if (vis[p1]) continue;
        vis[p1] = true;

        result.push_back({p1, p2});
        result.push_back({nx(p1), p2});

        if (cross(base, p[nx(p2)] - p[p2]) == 0) {   // parallel edges
            result.push_back({p1, nx(p2)});
            result.push_back({nx(p1), nx(p2)});
            vis[p2] = true;
        }
    }
    return result;
}
