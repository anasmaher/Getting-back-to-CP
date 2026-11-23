std::mt19937_64 rng(
    std::chrono::steady_clock::now().time_since_epoch().count());

long long rnd(long long lo, long long hi) {   // inclusive
    return std::uniform_int_distribution<long long>(lo, hi)(rng);
}
