bool perfectCube(long long n) {
    long long x = std::llround(std::cbrtl((long double)n));
    while ((__int128)x * x * x > n)                    --x;
    while ((__int128)(x + 1) * (x + 1) * (x + 1) <= n) ++x;
    return (__int128)x * x * x == n;
}
