int powmod(int a, int b) {
    int r = 1 % mod;
    a = ((a % mod) + m) % mod;
    while (b > 0) {
        if (b & 1) r = (__int128)r * a % mod;
        a = (__int128)a * a % mod;
        b >>= 1;
    }
    return r;
}
