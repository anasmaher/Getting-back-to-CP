#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

enum class Op { Max, Min, Sum, Mul, Gcd, Lcm, And, Or, Xor };

template <class T, Op O>
struct SegmentTree {
private:
    static constexpr T identity() {
        if constexpr      (O == Op::Max) return std::numeric_limits<T>::lowest();
        else if constexpr (O == Op::Min) return std::numeric_limits<T>::max();
        else if constexpr (O == Op::And) return ~T{};
        else if constexpr (O == Op::Mul) return T{1};
        else if constexpr (O == Op::Lcm) return T{1};
        else                             return T{};
    }

    static T apply(T a, T b) {
        if constexpr      (O == Op::Max) return std::max(a, b);
        else if constexpr (O == Op::Min) return std::min(a, b);
        else if constexpr (O == Op::Sum) return a + b;
        else if constexpr (O == Op::Mul) return a * b;
        else if constexpr (O == Op::Gcd) return std::gcd(a, b);
        else if constexpr (O == Op::Lcm) return std::lcm(a, b);
        else if constexpr (O == Op::And) return a & b;
        else if constexpr (O == Op::Or)  return a | b;
        else                             return a ^ b;
    }

    int n;
    std::vector<T> seg;

    void build(int node, int l, int r, const std::vector<T>& arr) {
        if (l == r) { seg[node] = arr[l]; return; }
        int mid = (l + r) / 2;
        build(2 * node + 1, l, mid, arr);
        build(2 * node + 2, mid + 1, r, arr);
        seg[node] = apply(seg[2 * node + 1], seg[2 * node + 2]);
    }

    void update(int node, int l, int r, int idx, T val) {
        if (l == r) { seg[node] = val; return; }
        int mid = (l + r) / 2;
        if (idx <= mid) update(2 * node + 1, l, mid, idx, val);
        else            update(2 * node + 2, mid + 1, r, idx, val);
        seg[node] = apply(seg[2 * node + 1], seg[2 * node + 2]);
    }

    T query(int node, int l, int r, int ql, int qr) {
        if (r < ql || l > qr)   return identity();
        if (l >= ql && r <= qr) return seg[node];
        int mid = (l + r) / 2;
        return apply(query(2 * node + 1, l, mid, ql, qr),
                     query(2 * node + 2, mid + 1, r, ql, qr));
    }

public:
    explicit SegmentTree(const std::vector<T>& arr) : n(arr.size()) {
        seg.resize(4 * n);
        if (n) build(0, 0, n - 1, arr);
    }

    void update(int idx, T val) { update(0, 0, n - 1, idx, val); }
    T    query (int l, int r)   { return query(0, 0, n - 1, l, r); }
    T    point (int i)          { return query(i, i); }
};

template <Op O, class T>
auto makeSeg(const std::vector<T>& a) { return SegmentTree<T, O>(a); }

// auto mx = makeSeg<Op::Max>(a);
