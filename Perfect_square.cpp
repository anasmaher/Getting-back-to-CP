bool isPerfectSquare(long long x) {
    if (x < 0) return false;
    long long y = (long long)std::sqrtl((long double)x);
    while (y > 0 && y > x / y) --y
    while (y + 1 <= x / (y + 1)) ++y
    return y * y == x;
}
