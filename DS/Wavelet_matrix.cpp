struct WaveletMatrix {
private:
    // Handles values in [-2^30, 2^30).
    // Bump LOG (and switch int -> int64_t inside) if you need a wider range.
    static constexpr int LOG    = 62;
    static constexpr int OFFSET = 1ll << 61;   // 2^30

    int n;
    std::vector<int> zeros;
    std::vector<std::vector<int>> pref;

    int rank1(int k, int i) const { return pref[k][i]; }
    int rank0(int k, int i) const { return i - pref[k][i]; }

public:
    explicit WaveletMatrix(const std::vector<int>& a) : n(a.size()) {
        zeros.assign(LOG, 0);
        pref.assign(LOG, std::vector<int>(n + 1, 0));

        // Shift into [0, 2^31) — safe in int32 because max is exactly INT_MAX.
        std::vector<int> cur(n), nxt(n);
        for (int i = 0; i < n; ++i) cur[i] = a[i] + OFFSET;

        for (int k = LOG - 1; k >= 0; --k) {
            for (int i = 0; i < n; ++i)
                pref[k][i + 1] = pref[k][i] + ((cur[i] >> k) & 1);
            zeros[k] = n - pref[k][n];

            int lo = 0, hi = zeros[k];
            for (int v : cur)
                if ((v >> k) & 1) nxt[hi++] = v;
                else             nxt[lo++] = v;
            cur.swap(nxt);
        }
    }

    // # of elements in a[l..r] with value < x, where x is in the *original* domain.
    int count_lt(int l, int r, long long x) const {
        long long xs = x + OFFSET;
        if (xs <= 0)             return 0;                 // x < min possible
        if (xs >= (1LL << LOG))  return r - l + 1;         // x > max possible

        int res = 0;
        ++r;
        for (int k = LOG - 1; k >= 0 && l < r; --k) {
            if ((xs >> k) & 1) {
                res += rank0(k, r) - rank0(k, l);
                l = zeros[k] + rank1(k, l);
                r = zeros[k] + rank1(k, r);
            } else {
                l = rank0(k, l);
                r = rank0(k, r);
            }
        }
        return res;
    }
    int count_le(int l, int r, long long x) const { return count_lt(l, r, x + 1); }
    int count   (int l, int r, int      x) const { return count_le(l, r, x) - count_lt(l, r, x); }
    int count_gt(int l, int r, int      x) const { return (r - l + 1) - count_le(l, r, x); }
    int count_ge(int l, int r, int      x) const { return (r - l + 1) - count_lt(l, r, x); }

    int kth(int l, int r, int k) const {
        long long val = 0;
        ++r;
        for (int b = LOG - 1; b >= 0; --b) {
            int ones_l = rank1(b, l), ones_r = rank1(b, r);
            int zeros_in = (r - l) - (ones_r - ones_l);
            if (k <= zeros_in) {
                l = rank0(b, l);
                r = rank0(b, r);
            } else {
                k -= zeros_in;
                val |= (1LL << b);
                l = zeros[b] + ones_l;
                r = zeros[b] + ones_r;
            }
        }
        return int(val - OFFSET);   // unshift back to signed domain
    }
    int range_min(int l, int r) const { return kth(l, r, 1); }
    int range_max(int l, int r) const { return kth(l, r, r - l + 1); }
};
