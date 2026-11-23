template <class T, class Policy>
struct CustomSegTree {
private:
    int n;
    T id;
    std::vector<T> seg;

    void build(int node, int l, int r, const std::vector<T>& a) {
        if (l == r) { seg[node] = a[l]; return; }
        int m = (l + r) / 2;
        build(2*node+1, l, m, a);
        build(2*node+2, m+1, r, a);
        seg[node] = Policy::merge(seg[2*node+1], seg[2*node+2]);
    }

    void update(int node, int l, int r, int idx, T v) {
        if (l == r) { seg[node] = v; return; }
        int m = (l + r) / 2;
        if (idx <= m) update(2*node+1, l, m, idx, v);
        else          update(2*node+2, m+1, r, idx, v);
        seg[node] = Policy::merge(seg[2*node+1], seg[2*node+2]);
    }

    T query(int node, int l, int r, int ql, int qr) {
        if (r < ql || l > qr)   return id;
        if (ql <= l && r <= qr) return seg[node];
        int m = (l + r) / 2;
        return Policy::merge(query(2*node+1, l, m, ql, qr),
                             query(2*node+2, m+1, r, ql, qr));
    }

public:
    explicit CustomSegTree(const std::vector<T>& a)
        : n(a.size()), id(Policy::id()), seg(4 * n)
    {
        if (n) build(0, 0, n - 1, a);
    }

    void update(int idx, T v) { update(0, 0, n - 1, idx, v); }
    T    query (int l, int r) { return query(0, 0, n - 1, l, r); }
    T    point (int i)        { return query(i, i); }
};


template <class T, class Policy>
struct CustomLazySegTree {
    using Tag = typename Policy::Tag;

private:
    int n;
    T id;
    std::vector<T>    seg;
    std::vector<Tag>  lazy;
    std::vector<char> has;

    void apply_node(int node, int l, int r, Tag v) {
        seg[node] = Policy::apply(seg[node], v, r - l + 1);
        if (has[node]) lazy[node] = Policy::compose(lazy[node], v);
        else         { lazy[node] = v; has[node] = 1; }
    }

    void push(int node, int l, int r) {
        if (!has[node]) return;
        int m = (l + r) / 2;
        apply_node(2*node+1, l, m, lazy[node]);
        apply_node(2*node+2, m+1, r, lazy[node]);
        has[node] = 0;
    }

    void build(int node, int l, int r, const std::vector<T>& a) {
        if (l == r) { seg[node] = a[l]; return; }
        int m = (l + r) / 2;
        build(2*node+1, l, m, a);
        build(2*node+2, m+1, r, a);
        seg[node] = Policy::merge(seg[2*node+1], seg[2*node+2]);
    }

    void update(int node, int l, int r, int ql, int qr, Tag v) {
        if (r < ql || l > qr) return;
        if (ql <= l && r <= qr) { apply_node(node, l, r, v); return; }
        push(node, l, r);
        int m = (l + r) / 2;
        update(2*node+1, l, m, ql, qr, v);
        update(2*node+2, m+1, r, ql, qr, v);
        seg[node] = Policy::merge(seg[2*node+1], seg[2*node+2]);
    }

    T query(int node, int l, int r, int ql, int qr) {
        if (r < ql || l > qr)   return id;
        if (ql <= l && r <= qr) return seg[node];
        push(node, l, r);
        int m = (l + r) / 2;
        return Policy::merge(query(2*node+1, l, m, ql, qr),
                             query(2*node+2, m+1, r, ql, qr));
    }

public:
    explicit CustomLazySegTree(const std::vector<T>& a)
        : n(a.size()), id(Policy::id()),
          seg(4 * n), lazy(4 * n), has(4 * n, 0)
    {
        if (n) build(0, 0, n - 1, a);
    }

    void update(int l, int r, Tag v) { update(0, 0, n - 1, l, r, v); }
    T    query (int l, int r)        { return query(0, 0, n - 1, l, r); }
};

struct Node{
};


//Non-lazy
struct CustomMerge{
    static Node id() { return {0, NEG, NEG, NEG}; }
    static Node merge(Node L, Node R) {
        return {
        };
    }
};

//Lazy
struct CustomMerge{
    using Tag = int;
    static Node id() { return {0, NEG, NEG, NEG}; }
    static Node merge(Node L, Node R) {
        return {
        };
    }
    static Node apply(Node x, int v, int len) {
        return {};
    }
    static int compose(int existing, int incoming) {
        return existing + incoming;
    }
};

