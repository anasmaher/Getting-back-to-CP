vector<vector<int>> seg;
 
void build(int node, int l, int r, vector<int>& a) {
    if (l == r) {
        seg[node] = {a[l]};
        return;
    }
    int m = (l + r) / 2;
    build(2 * node, l, m, a);
    build(2 * node + 1, m + 1, r, a);
    merge(seg[2 * node].begin(), seg[2 * node].end(),
          seg[2 * node + 1].begin(), seg[2 * node + 1].end(),
          back_inserter(seg[node]));
}
 
 
int query_eq(int node, int l, int r, int ql, int qr, int x) {
    if (r < ql || l > qr) return 0;
    if (ql <= l && r <= qr) {
        return upper_bound(seg[node].begin(), seg[node].end(), x) -
               lower_bound(seg[node].begin(), seg[node].end(), x);
    }
    int m = (l + r) / 2;
    return query_eq(2 * node, l, m, ql, qr, x) +
           query_eq(2 * node + 1, m + 1, r, ql, qr, x);
}
 
int query_lt(int node, int l, int r, int ql, int qr, int x) {
    if (r < ql || l > qr) return 0;
    if (ql <= l && r <= qr) {
        return lower_bound(seg[node].begin(), seg[node].end(), x) - seg[node].begin();
    }
    int m = (l + r) / 2;
    return query_lt(2 * node, l, m, ql, qr, x) +
           query_lt(2 * node + 1, m + 1, r, ql, qr, x);
}
 
int query_gt(int node, int l, int r, int ql, int qr, int x) {
    if (r < ql || l > qr) return 0;
    if (ql <= l && r <= qr) {
        return seg[node].end() - upper_bound(seg[node].begin(), seg[node].end(), x);
    }
    int m = (l + r) / 2;
    return query_gt(2 * node, l, m, ql, qr, x) +
           query_gt(2 * node + 1, m + 1, r, ql, qr, x);
}

/*
use
seg.assign(4 * n, {});
build(1, 0, n - 1, ans);

example
query_gt(1, 0, n - 1, i + 1, n - 1, ans[i])
*/
