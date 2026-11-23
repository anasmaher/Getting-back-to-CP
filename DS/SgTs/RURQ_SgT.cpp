enum class LazyOp { Add, Assign };

template <class T, Op O, LazyOp L = LazyOp::Add>
struct LazySegTree {
    static_assert(O == Op::Sum || O == Op::Min || O == Op::Max,
                  "supports Sum, Min, Max only");

private:
    int n;
    std::vector<T> seg, lazy;
    std::vector<char> has;

    static T identity_q() {
        if constexpr (O == Op::Max) return std::numeric_limits<T>::lowest();
        if constexpr (O == Op::Min) return std::numeric_limits<T>::max();
        return T{};
    }

    static T combine(T a, T b) {
        if constexpr (O == Op::Max) return std::max(a, b);
        if constexpr (O == Op::Min) return std::min(a, b);
        return a + b;
    }

    void apply_node(int node, int l, int r, T v) {
        if constexpr (L == LazyOp::Add) {
            if constexpr (O == Op::Sum) seg[node] += v * (r - l + 1);
            else                        seg[node] += v;
            lazy[node] += v;
        } else {
            if constexpr (O == Op::Sum) seg[node] = v * (r - l + 1);
            else                        seg[node] = v;
            lazy[node] = v;
        }
        has[node] = 1;
    }

    void push(int node, int l, int r) {
        if (!has[node]) return;
        int m = (l + r) / 2;
        apply_node(2 * node + 1, l, m, lazy[node]);
        apply_node(2 * node + 2, m + 1, r, lazy[node]);
        has[node] = 0;
        lazy[node] = T{};
    }

    void build(int node, int l, int r, const std::vector<T>& a) {
        if (l == r) { seg[node] = a[l]; return; }
        int m = (l + r) / 2;
        build(2 * node + 1, l, m, a);
        build(2 * node + 2, m + 1, r, a);
        seg[node] = combine(seg[2 * node + 1], seg[2 * node + 2]);
    }

    void update(int node, int l, int r, int ql, int qr, T v) {
        if (r < ql || l > qr) return;
        if (ql <= l && r <= qr) { apply_node(node, l, r, v); return; }
        push(node, l, r);
        int m = (l + r) / 2;
        update(2 * node + 1, l, m, ql, qr, v);
        update(2 * node + 2, m + 1, r, ql, qr, v);
        seg[node] = combine(seg[2 * node + 1], seg[2 * node + 2]);
    }

    T query(int node, int l, int r, int ql, int qr) {
        if (r < ql || l > qr)   return identity_q();
        if (ql <= l && r <= qr) return seg[node];
        push(node, l, r);
        int m = (l + r) / 2;
        return combine(query(2 * node + 1, l, m, ql, qr),
                       query(2 * node + 2, m + 1, r, ql, qr));
    }

public:
    explicit LazySegTree(const std::vector<T>& a)
        : n(a.size()), seg(4 * n), lazy(4 * n), has(4 * n, 0)
    {
        if (n) build(0, 0, n - 1, a);
    }

    void update(int l, int r, T v) { update(0, 0, n - 1, l, r, v); }
    T    query (int l, int r)      { return query(0, 0, n - 1, l, r); }
};

template <Op O, LazyOp L = LazyOp::Add, class T>
auto makeLazy(const std::vector<T>& a) { return LazySegTree<T, O, L>(a); }
// auto sm = makeLazy<Op::Sum>(a); 
// auto mx = makeLazy<LazyOp::Assign>(a);
