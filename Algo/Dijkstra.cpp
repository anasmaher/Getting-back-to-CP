template <class W>
vector<W> dijkstra(int n, const vector<vector<pair<int, W>>>& adj, int src){
    const W INF = numeric_limits<W>::max();

    vector<W> dist(n, INF);
    using P = pair<W, int>;
    priority_queue<P, vector<P>, greater<P>> pq;

    dist[src] = W{};
    pq.push({W{}, src});

    while(!pq.empty()){
        auto [d, v] = pq.top(); pq.pop();

        if(d > dist[v]) continue;                 // stale entry
        
        for(auto [u, w] : adj[v]){
            if(dist[v] + w >= dist[u])
                continue;

            dist[u] = dist[v] + w;
            pq.push({dist[u], u});
        }
    }

    return dist;
}
