struct Prefix2D {
    int n, m;
    std::vector<std::vector<long long>> pre;   // pre[i][j] = sum of a[0..i-1][0..j-1]

    Prefix2D(const std::vector<std::vector<int>>& a)
        : n(a.size()), m(a.empty() ? 0 : a[0].size()),
          pre(n + 1, std::vector<long long>(m + 1, 0))
    {
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                pre[i+1][j+1] = a[i][j] + pre[i][j+1] + pre[i+1][j] - pre[i][j];
    }

    // inclusive, 0-indexed
    long long query(int r1, int c1, int r2, int c2) {
        return pre[r2+1][c2+1] - pre[r1][c2+1] - pre[r2+1][c1] + pre[r1][c1];
    }
};
// Prefix2D P(a);
// P.query(0, 2, 2, 2)

struct Fenwick2D {
    int n, m;
    std::vector<std::vector<long long>> bit;

    Fenwick2D(int n, int m) : n(n), m(m), bit(n + 1, std::vector<long long>(m + 1, 0)) {}

    Fenwick2D(const std::vector<std::vector<int>>& a) : Fenwick2D(a.size(), a[0].size()) {
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                add(i, j, a[i][j]);
    }

    void add(int x, int y, long long v) {                 // a[x][y] += v
        for (int i = x + 1; i <= n; i += i & -i)
            for (int j = y + 1; j <= m; j += j & -j)
                bit[i][j] += v;
    }

    long long sum(int x, int y) {                          // prefix sum [0..x][0..y]
        long long r = 0;
        for (int i = x + 1; i > 0; i -= i & -i)
            for (int j = y + 1; j > 0; j -= j & -j)
                r += bit[i][j];
        return r;
    }

    long long query(int r1, int c1, int r2, int c2) {      // rect sum, inclusive
        return sum(r2, c2) - sum(r1-1, c2) - sum(r2, c1-1) + sum(r1-1, c1-1);
    }
};
// Fenwick2D F(a);
// add(x, y, v) is delta, not assignment. To set a[x][y] = v, do add(x, y, v - current).

struct SparseTable2D {
    int n, m;
    std::vector<int> lg;
    // st[kr][kc][i][j] = max over [i, i+2^kr) rows × [j, j+2^kc) cols
    std::vector<std::vector<std::vector<std::vector<int>>>> st;

    SparseTable2D(const std::vector<std::vector<int>>& a) : n(a.size()), m(a[0].size()) {
        int LN = 1, LM = 1;
        while ((1 << LN) <= n) ++LN;
        while ((1 << LM) <= m) ++LM;

        lg.assign(std::max(n, m) + 1, 0);
        for (int i = 2; i <= std::max(n, m); i++) lg[i] = lg[i / 2] + 1;

        st.assign(LN, std::vector<std::vector<std::vector<int>>>(
                      LM, std::vector<std::vector<int>>(n, std::vector<int>(m))));

        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                st[0][0][i][j] = a[i][j];

        // expand columns first (1D along cols for each row)
        for (int kc = 1; kc < LM; kc++)
            for (int i = 0; i < n; i++)
                for (int j = 0; j + (1 << kc) <= m; j++)
                    st[0][kc][i][j] = std::max(st[0][kc-1][i][j],
                                               st[0][kc-1][i][j + (1 << (kc-1))]);

        // then expand rows
        for (int kr = 1; kr < LN; kr++)
            for (int kc = 0; kc < LM; kc++)
                for (int i = 0; i + (1 << kr) <= n; i++)
                    for (int j = 0; j + (1 << kc) <= m; j++)
                        st[kr][kc][i][j] = std::max(st[kr-1][kc][i][j],
                                                    st[kr-1][kc][i + (1 << (kr-1))][j]);
    }

    int query(int r1, int c1, int r2, int c2) {            // inclusive
        int kr = lg[r2 - r1 + 1], kc = lg[c2 - c1 + 1];
        int dr = (1 << kr), dc = (1 << kc);
        return std::max({
            st[kr][kc][r1][c1],
            st[kr][kc][r2 - dr + 1][c1],
            st[kr][kc][r1][c2 - dc + 1],
            st[kr][kc][r2 - dr + 1][c2 - dc + 1]
        });
    }
};
// SparseTable2D S(a);

struct SegTree2D {
    int n, m;
    std::vector<std::vector<int>> arr;    // arr[node][j] = merged value at col j
    std::vector<std::vector<int>> tree;   // tree[node] = 1D segtree over arr[node]
    const int ID = std::numeric_limits<int>::lowest();

    SegTree2D(const std::vector<std::vector<int>>& a)
        : n(a.size()), m(a.empty() ? 0 : a[0].size())
    {
        if (n == 0 || m == 0) return;
        arr.assign(4 * n, std::vector<int>(m, ID));
        tree.assign(4 * n, std::vector<int>(4 * m, ID));
        build(1, 0, n - 1, a);
    }

    void build(int node, int l, int r, const std::vector<std::vector<int>>& a) {
        if (l == r) {
            for (int j = 0; j < m; j++) arr[node][j] = a[l][j];
        } else {
            int mid = (l + r) / 2;
            build(2*node, l, mid, a);
            build(2*node+1, mid+1, r, a);
            for (int j = 0; j < m; j++)
                arr[node][j] = std::max(arr[2*node][j], arr[2*node+1][j]);
        }
        build_inner(node, 1, 0, m - 1);
    }

    void build_inner(int node, int inode, int cl, int cr) {
        if (cl == cr) { tree[node][inode] = arr[node][cl]; return; }
        int cm = (cl + cr) / 2;
        build_inner(node, 2*inode, cl, cm);
        build_inner(node, 2*inode+1, cm+1, cr);
        tree[node][inode] = std::max(tree[node][2*inode], tree[node][2*inode+1]);
    }

    void update_inner(int node, int inode, int cl, int cr, int col, int val) {
        if (cl == cr) { tree[node][inode] = val; return; }
        int cm = (cl + cr) / 2;
        if (col <= cm) update_inner(node, 2*inode, cl, cm, col, val);
        else           update_inner(node, 2*inode+1, cm+1, cr, col, val);
        tree[node][inode] = std::max(tree[node][2*inode], tree[node][2*inode+1]);
    }

    void update(int node, int l, int r, int row, int col, int val) {
        if (l == r) {
            arr[node][col] = val;
            update_inner(node, 1, 0, m - 1, col, val);
            return;
        }
        int mid = (l + r) / 2;
        if (row <= mid) update(2*node, l, mid, row, col, val);
        else            update(2*node+1, mid+1, r, row, col, val);
        arr[node][col] = std::max(arr[2*node][col], arr[2*node+1][col]);
        update_inner(node, 1, 0, m - 1, col, arr[node][col]);
    }

    int query_inner(int node, int inode, int cl, int cr, int ql, int qr) {
        if (qr < cl || cr < ql) return ID;
        if (ql <= cl && cr <= qr) return tree[node][inode];
        int cm = (cl + cr) / 2;
        return std::max(query_inner(node, 2*inode, cl, cm, ql, qr),
                        query_inner(node, 2*inode+1, cm+1, cr, ql, qr));
    }

    int query(int node, int l, int r, int r1, int r2, int c1, int c2) {
        if (r2 < l || r < r1) return ID;
        if (r1 <= l && r <= r2) return query_inner(node, 1, 0, m - 1, c1, c2);
        int mid = (l + r) / 2;
        return std::max(query(2*node, l, mid, r1, r2, c1, c2),
                        query(2*node+1, mid+1, r, r1, r2, c1, c2));
    }

    void update(int row, int col, int val) { update(1, 0, n-1, row, col, val); }
    int  query (int r1, int c1, int r2, int c2) { return query(1, 0, n-1, r1, r2, c1, c2); }
};

struct Count2D {
    std::map<int, std::vector<std::vector<int>>> pre;   // pre[value] = 2D prefix of indicator
    int n, m;

    Count2D(const std::vector<std::vector<int>>& a) : n(a.size()), m(a[0].size()) {
        std::set<int> vals;
        for (auto& row : a) for (int x : row) vals.insert(x);

        for (int v : vals) {
            auto& P = pre[v];
            P.assign(n + 1, std::vector<int>(m + 1, 0));
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++)
                    P[i+1][j+1] = (a[i][j] == v)
                                + P[i][j+1] + P[i+1][j] - P[i][j];
        }
    }

    int query(int r1, int c1, int r2, int c2, int v) {
        auto it = pre.find(v);
        if (it == pre.end()) return 0;
        const auto& P = it->second;
        return P[r2+1][c2+1] - P[r1][c2+1] - P[r2+1][c1] + P[r1][c1];
    }
};
// SegTree2D T(a);
// update(row, col, val) is absolute assignment, not delta.

struct Count2D {
    std::map<int, std::vector<std::vector<int>>> pre;   // pre[value] = 2D prefix of indicator
    int n, m;

    Count2D(const std::vector<std::vector<int>>& a) : n(a.size()), m(a[0].size()) {
        std::set<int> vals;
        for (auto& row : a) for (int x : row) vals.insert(x);

        for (int v : vals) {
            auto& P = pre[v];
            P.assign(n + 1, std::vector<int>(m + 1, 0));
            for (int i = 0; i < n; i++)
                for (int j = 0; j < m; j++)
                    P[i+1][j+1] = (a[i][j] == v)
                                + P[i][j+1] + P[i+1][j] - P[i][j];
        }
    }

    int query(int r1, int c1, int r2, int c2, int v) {
        auto it = pre.find(v);
        if (it == pre.end()) return 0;
        const auto& P = it->second;
        return P[r2+1][c2+1] - P[r1][c2+1] - P[r2+1][c1] + P[r1][c1];
    }
};
// Count2D C(b);
// The value v is the 4th argument. There's one prefix matrix per distinct value, built once in the constructor.

/*
Structure	Build	Memory	Point update	Rect query	Ops
Prefix2D	O(nm)	O(nm)	✗	            O(1)	Sum, Xor
Fenwick2D	O(nm log²)	    O(nm)	O(log²)	O(log²)	Sum, Xor
SparseTable2D	O(nm log²)	O(nm log²)	✗	O(1)	Max, Min, Gcd, And, Or
SegTree2D	O(nm)	O(nm)	O(log²)	O(log²)	Any monoid
Count2D	O(k nm)	O(k nm)	✗	O(log k)	Count-of-value
*/
