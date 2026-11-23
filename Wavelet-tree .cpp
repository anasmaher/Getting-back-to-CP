struct WaveletTree {
private:
    int n, sigma;
    std::vector<int> vals;                         // sorted unique values
    std::vector<std::vector<int>> pref;            // pref[node][i] = # of [0,i) that go left

    void build(int node, int lo, int hi, std::vector<int> cur) {
        pref[node].assign(cur.size() + 1, 0);
        int mid = (lo + hi) / 2;
        for (int i = 0; i < (int)cur.size(); ++i)
            pref[node][i + 1] = pref[node][i] + (cur[i] < mid);
        if (hi - lo == 1) return;

        int left_total = pref[node].back();
        std::vector<int> left, right;
        left.reserve(left_total);
        right.reserve(cur.size() - left_total);
        for (int v : cur) (v < mid ? left : right).push_back(v);
        build(2 * node + 1, lo, mid, std::move(left));
        build(2 * node + 2, mid, hi, std::move(right));
    }

    // # of elements with value-index < v in positions [l, r) of this node
    int count_lt_rec(int node, int lo, int hi, int l, int r, int v) {
        if (r <= l || v <= lo) return 0;
        if (hi <= v)           return r - l;
        int mid = (lo + hi) / 2;
        int left_l = pref[node][l], left_r = pref[node][r];
        if (v <= mid)
            return count_lt_rec(2 * node + 1, lo, mid, left_l, left_r, v);
        int left_total = pref[node].back();
        return (left_r - left_l)
             + count_lt_rec(2 * node + 2, mid, hi,
                            left_total + l - left_l,
                            left_total + r - left_r, v);
    }

    // k-th smallest (1-indexed) value-index in [l, r) of this node
    int kth_rec(int node, int lo, int hi, int l, int r, int k) {
        if (hi - lo == 1) return lo;
        int mid = (lo + hi) / 2;
        int left_l = pref[node][l], left_r = pref[node][r];
        int left_count = left_r - left_l;
        if (k <= left_count)
            return kth_rec(2 * node + 1, lo, mid, left_l, left_r, k);
        int left_total = pref[node].back();
        return kth_rec(2 * node + 2, mid, hi,
                       left_total + l - left_l,
                       left_total + r - left_r, k - left_count);
    }

public:
    explicit WaveletTree(const std::vector<int>& a) : n(a.size()) {
        vals = a;
        std::sort(vals.begin(), vals.end());
        vals.erase(std::unique(vals.begin(), vals.end()), vals.end());
        sigma = vals.size();
        if (sigma == 0) return;

        std::vector<int> cur(n);
        for (int i = 0; i < n; ++i)
            cur[i] = std::lower_bound(vals.begin(), vals.end(), a[i]) - vals.begin();

        pref.resize(4 * sigma);
        build(0, 0, sigma, std::move(cur));
    }

    int count_lt(int l, int r, int x) {          // a[i] <  x
        if (sigma == 0) return 0;
        int v = std::lower_bound(vals.begin(), vals.end(), x) - vals.begin();
        return count_lt_rec(0, 0, sigma, l, r + 1, v);
    }
    int count_le(int l, int r, int x) {          // a[i] <= x
        if (sigma == 0) return 0;
        int v = std::upper_bound(vals.begin(), vals.end(), x) - vals.begin();
        return count_lt_rec(0, 0, sigma, l, r + 1, v);
    }
    int count   (int l, int r, int x) { return count_le(l, r, x) - count_lt(l, r, x); }
    int count_ge(int l, int r, int x) { return (r - l + 1) - count_lt(l, r, x); }
    int count_gt(int l, int r, int x) { return (r - l + 1) - count_le(l, r, x); }

    int kth(int l, int r, int k) {               // 1-indexed
        assert(1 <= k && k <= r - l + 1 && sigma > 0);
        return vals[kth_rec(0, 0, sigma, l, r + 1, k)];
    }
    int range_min(int l, int r) { return kth(l, r, 1); }
    int range_max(int l, int r) { return kth(l, r, r - l + 1); }
};

// WaveletTree wt(a);
