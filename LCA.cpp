struct LCA {
    int n, LOG;
    std::vector<int> depth;
    std::vector<std::vector<int>> up;   // up[k][v] = 2^k-th ancestor of v

    LCA(const std::vector<std::vector<int>>& adj, int root = 0) {
        n = adj.size();
        LOG = 1; while ((1 << LOG) < n) ++LOG; ++LOG;

        depth.assign(n, 0);
        up.assign(LOG, std::vector<int>(n, root));

        std::vector<int> order = {root};
        std::vector<char> seen(n, 0);
        seen[root] = 1;
        for (int i = 0; i < (int)order.size(); ++i) {
            int v = order[i];
            for (int u : adj[v]) if (!seen[u]) {
                seen[u] = 1;
                depth[u] = depth[v] + 1;
                up[0][u] = v;
                order.push_back(u);
            }
        }
        for (int k = 1; k < LOG; ++k)
            for (int v = 0; v < n; ++v)
                up[k][v] = up[k - 1][up[k - 1][v]];
    }

    int lift(int v, int d) {
        for (int k = 0; k < LOG; ++k)
            if (d & (1 << k)) v = up[k][v];
        return v;
    }

    int lca(int u, int v) {
        if (depth[u] < depth[v]) std::swap(u, v);
        u = lift(u, depth[u] - depth[v]);
        if (u == v) return u;
        for (int k = LOG - 1; k >= 0; --k)
            if (up[k][u] != up[k][v]) { u = up[k][u]; v = up[k][v]; }
        return up[0][u];
    }

    int dist(int u, int v) {
        return depth[u] + depth[v] - 2 * depth[lca(u, v)];
    }
};
