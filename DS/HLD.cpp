struct HLD {
    int n;
    std::vector<int> parent, depth, sz, heavy, head, pos;
    std::vector<int> base;   // flattened values, indexed by pos

    HLD(const std::vector<std::vector<int>>& adj, int root,
        const std::vector<int>& val)
    {
        n = adj.size();
        parent.assign(n, -1);
        depth.assign(n, 0);
        sz.assign(n, 1);
        heavy.assign(n, -1);
        head.assign(n, -1);
        pos.assign(n, -1);
        base.assign(n, 0);

        // Pass 1: BFS → parent/depth; reverse → sz, heavy
        std::vector<int> order = {root};
        parent[root] = root;
        for (int i = 0; i < n; ++i) {
            int v = order[i];
            for (int u : adj[v]) if (u != parent[v]) {
                parent[u] = v;
                depth[u] = depth[v] + 1;
                order.push_back(u);
            }
        }
        for (int i = n - 1; i >= 0; --i) {
            int v = order[i];
            if (v == root) continue;
            sz[parent[v]] += sz[v];
            if (heavy[parent[v]] == -1 || sz[v] > sz[heavy[parent[v]]])
                heavy[parent[v]] = v;
        }

        // Pass 2: heavy-first DFS → pos, head, base
        int cur = 0;
        std::vector<int> stk = {root};
        while (!stk.empty()) {
            int h = stk.back(); stk.pop_back();
            for (int v = h; v != -1; v = heavy[v]) {
                head[v] = h;
                pos[v] = cur;
                base[cur++] = val[v];
                for (int u : adj[v])
                    if (u != parent[v] && u != heavy[v]) stk.push_back(u);
            }
        }
    }

    // Calls f(l, r) on each chain segment of the path u→v (in base-array indices)
    template <class F>
    void path(int u, int v, F f) {
        while (head[u] != head[v]) {
            if (depth[head[u]] < depth[head[v]]) std::swap(u, v);
            f(pos[head[u]], pos[u]);
            u = parent[head[u]];
        }
        if (depth[u] > depth[v]) std::swap(u, v);
        f(pos[u], pos[v]);
    }

    // Subtree of v is [pos[v], pos[v] + sz[v] - 1] in base array
    int subtree_l(int v) { return pos[v]; }
    int subtree_r(int v) { return pos[v] + sz[v] - 1; }

    int lca(int u, int v) {
        while (head[u] != head[v]) {
            if (depth[head[u]] < depth[head[v]]) std::swap(u, v);
            u = parent[head[u]];
        }
        return depth[u] < depth[v] ? u : v;
    }
};

// adj
// vals
// HLD hld(adj, 0, vals);
// hld.base
// hld.path(3, 6, [&](int l, int r) {});
