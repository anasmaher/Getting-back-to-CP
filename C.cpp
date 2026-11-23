// ── 1. LOOP  (single query, small k) ─────────────────────────────────
Mint C(int n, int k) {
    if (k < 0 || k > n) return 0;
    k = std::min(k, n - k);
    Mint num = 1, den = 1;
    for (int i = 1; i <= k; ++i) {
        num *= (n - k + i);
        den *= i;
    }
    return num / den;               // 1 inverse total
}
// Requires: M prime, M > k.  O(k) time, O(1) memory.

// ── 2. PRECOMPUTE  (many queries, n ≤ MAXN) ──────────────────────────
constexpr int MAXN = 1'000'005;
Mint fact[MAXN], inv_fact[MAXN];

void initComb() {
    fact[0] = 1;
    for (int i = 1; i < MAXN; ++i) fact[i] = fact[i-1] * i;
    inv_fact[MAXN-1] = fact[MAXN-1].inv();       // 1 inverse total
    for (int i = MAXN-2; i >= 0; --i) inv_fact[i] = inv_fact[i+1] * (i+1);
}

Mint C(int n, int k) {
    if (k < 0 || k > n) return 0;
    return fact[n] * inv_fact[k] * inv_fact[n-k];
}
// Requires: M prime, MAXN ≤ M.  O(MAXN) build, O(1) per query.

// ── 3. LUCAS  (small M, huge n) ──────────────────────────────────────
Mint C_small(int n, int k) {                     // same as #1, digit-safe
    if (k < 0 || k > n) return 0;
    k = std::min(k, n - k);
    Mint num = 1, den = 1;
    for (int i = 1; i <= k; ++i) { num *= (n - k + i); den *= i; }
    return num / den;
}

Mint C_lucas(long long n, long long k) {
    if (k < 0 || k > n) return 0;
    Mint r = 1;
    while (n > 0 || k > 0) {
        r *= C_small(int(n % M), int(k % M));
        n /= M; k /= M;
    }
    return r;
}
// Requires: M prime (any size).  O(M + log_M(n)) if using #2 internally,
// or O(M · log_M(n)) with the loop.  Use when n ≥ M.
