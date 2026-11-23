//get them 1e12
std::vector<long long> divisors(long long n) {
    if (n <= 0) return {};
    std::vector<long long> lo, hi;
    for (long long d = 1; d * d <= n; ++d) {
        if (n % d == 0) {
            lo.push_back(d);
            if (d != n / d) hi.push_back(n / d);
        }
    }
    lo.insert(lo.end(), hi.rbegin(), hi.rend());
    return lo;
}

//count them 1e18
static bool is_prime(long long n) {
    if (n < 2) return false;
    for (long long p : {2LL, 3LL, 5LL, 7LL, 11LL, 13LL, 17LL, 19LL,
                        23LL, 29LL, 31LL, 37LL}) {
        if (n % p == 0) return n == p;
    }
    long long d = n - 1; int s = 0;
    while (!(d & 1)) { d >>= 1; ++s; }
    for (long long a : {2LL, 3LL, 5LL, 7LL, 11LL, 13LL, 17LL, 19LL,
                        23LL, 29LL, 31LL, 37LL}) {
        __int128 x = 1, b = a % n;
        for (long long e = d; e; e >>= 1, b = b * b % n)
            if (e & 1) x = x * b % n;
        if (x == 1 || x == n - 1) continue;
        bool ok = false;
        for (int r = 1; r < s; ++r) {
            x = x * x % n;
            if (x == n - 1) { ok = true; break; }
        }
        if (!ok) return false;
    }
    return true;
}

long long numberOfDivisors(long long n) {
    if (n <= 0) return 0;
    long long ans = 1;
    for (long long p = 2; p * p * p <= n; ++p) {
        if (n % p == 0) {
            int c = 0;
            while (n % p == 0) { n /= p; ++c; }
            ans *= (c + 1);
        }
    }
    if (n == 1) return ans;
    long long r = std::llround(std::sqrtl((long double)n));
    while ((__int128)r * r > n)      --r;
    while ((__int128)(r + 1) * (r + 1) <= n) ++r;
    if ((__int128)r * r == n) return ans * 3;   // n = p²
    if (is_prime(n))          return ans * 2;   // n = p
    return ans * 4;                              // n = p·q
}
