struct IPoint { long long x, y; };
using i128 = __int128;

i128 crossI(IPoint a, IPoint b) {
    return (i128)a.x * b.y - (i128)a.y * b.x;
}
i128 orientI(IPoint a, IPoint b, IPoint c) {
    return crossI({b.x - a.x, b.y - a.y}, {c.x - a.x, c.y - a.y});
}
int signI(i128 v) { return (v > 0) - (v < 0); }

vector<IPoint> convexHullI(vector<IPoint> p) {
    sort(p.begin(), p.end(), [](auto& a, auto& b) {
        return tie(a.x, a.y) < tie(b.x, b.y);
    });
    if (p.size() < 2) return p;
    vector<IPoint> h(2 * p.size());
    int k = 0;
    for (auto& pt : p) {
        while (k >= 2 && signI(orientI(h[k-2], h[k-1], pt)) <= 0) k--;
        h[k++] = pt;
    }
    for (int i = p.size() - 2, t = k + 1; i >= 0; i--) {
        while (k >= t && signI(orientI(h[k-2], h[k-1], p[i])) <= 0) k--;
        h[k++] = p[i];
    }
    h.resize(k - 1);
    return h;
}
