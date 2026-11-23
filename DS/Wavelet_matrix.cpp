struct WaveletMatrix {
private:
    static constexpr int LOG = 30;   // values must be in [0, 2^30). Adjust if larger.
    int n;
    std::vector<int> zeros;                 // zeros[k] = total 0-bits at level k
    std::vector<std::vector<int>> pref;     // pref[k][i] = # of 1-bits in [0, i) at level k

    int rank1(int k, int i) const { return pref[k][i]; }
    int rank0(int k, int i) const { return i - pref[k][i]; }

public:
    explicit WaveletMatrix(const std::vector<int>& a) : n(a.size()) {
        zeros.assign(LOG, 0);
        pref.assign(LOG, std::vector<int>(n + 1, 0));

        std::vector<int> cur = a, nxt(n);
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

    // # of elements in a[l..r] with value < x
    int count_lt(int l, int r, int x) const {
        if (x <= 0)               return 0;
        if (x >= (1 << LOG))      return r - l + 1;
        int res = 0;
        ++r;                                     // half-open [l, r)
        for (int k = LOG - 1; k >= 0 && l < r; --k) {
            if ((x >> k) & 1) {
                res += rank0(k, r) - rank0(k, l);            // all 0s qualify
                l = zeros[k] + rank1(k, l);
                r = zeros[k] + rank1(k, r);
            } else {
                l = rank0(k, l);
                r = rank0(k, r);
            }
        }
        return res;
    }
    int count_le(int l, int r, int x) const { return count_lt(l, r, x + 1); }
    int count   (int l, int r, int x) const { return count_le(l, r, x) - count_lt(l, r, x); }
    int count_gt(int l, int r, int x) const { return (r - l + 1) - count_le(l, r, x); }
    int count_ge(int l, int r, int x) const { return (r - l + 1) - count_lt(l, r, x); }

    // k-th smallest (1-indexed) in a[l..r]
    int kth(int l, int r, int k) const {
        int val = 0;
        ++r;
        for (int b = LOG - 1; b >= 0; --b) {
            int ones_l = rank1(b, l), ones_r = rank1(b, r);
            int zeros_in = (r - l) - (ones_r - ones_l);
            if (k <= zeros_in) {
                l = rank0(b, l);
                r = rank0(b, r);
            } else {
                k -= zeros_in;
                val |= (1 << b);
                l = zeros[b] + ones_l;
                r = zeros[b] + ones_r;
            }
        }
        return val;
    }
    int range_min(int l, int r) const { return kth(l, r, 1); }
    int range_max(int l, int r) const { return kth(l, r, r - l + 1); }
};

// WaveletMatrix wm(a);
