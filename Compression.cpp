// Compresses a in place to ranks in [start, start + distinct - 1].
// Returns the number of distinct values.
template <class T>
int compress(std::vector<T>& a, int start = 0) {
    int n = (int)a.size();
    std::vector<std::pair<T, int>> pairs(n);
    for (int i = 0; i < n; ++i) pairs[i] = {a[i], i};
    std::sort(pairs.begin(), pairs.end());

    int nxt = start;
    for (int i = 0; i < n; ++i) {
        if (i > 0 && pairs[i - 1].first != pairs[i].first) ++nxt;
        a[pairs[i].second] = nxt;
    }
    return nxt - start + (n > 0);
}
