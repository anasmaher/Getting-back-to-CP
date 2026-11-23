template <class T, class Merge>
struct CustomSegTree {
private:
    int n;
    T id;
    Merge merge;
    std::vector<T> seg;

    void build(int node, int l, int r, const std::vector<T>& arr) {
        if (l == r) { seg[node] = arr[l]; return; }
        int mid = (l + r) / 2;
        build(2 * node + 1, l, mid, arr);
        build(2 * node + 2, mid + 1, r, arr);
        seg[node] = merge(seg[2 * node + 1], seg[2 * node + 2]);
    }

    void update(int node, int l, int r, int idx, T val) {
        if (l == r) { seg[node] = val; return; }
        int mid = (l + r) / 2;
        if (idx <= mid) update(2 * node + 1, l, mid, idx, val);
        else            update(2 * node + 2, mid + 1, r, idx, val);
        seg[node] = merge(seg[2 * node + 1], seg[2 * node + 2]);
    }

    T query(int node, int l, int r, int ql, int qr) {
        if (r < ql || l > qr)   return id;
        if (l >= ql && r <= qr) return seg[node];
        int mid = (l + r) / 2;
        return merge(query(2 * node + 1, l, mid, ql, qr),
                     query(2 * node + 2, mid + 1, r, ql, qr));
    }

public:
    CustomSegTree(const std::vector<T>& arr, T identity, Merge merge)
        : n(arr.size()), id(identity), merge(merge)
    {
        seg.resize(4 * n);
        if (n) build(0, 0, n - 1, arr);
    }

    void update(int idx, T val) { update(0, 0, n - 1, idx, val); }
    T    query (int l, int r)   { return query(0, 0, n - 1, l, r); }
    T    point (int i)          { return query(i, i); }
};

template <class T, class Merge>
auto makeCustomSeg(const std::vector<T>& a, T identity, Merge m) {
    return CustomSegTree<T, Merge>(a, identity, m);
}

struct Node {
};

Node mergeNode(Node L, Node R) {
    return {
    };
}

// const Node Ident{};
// auto st = makeCustomSeg(a, Ident, mergeNode);
