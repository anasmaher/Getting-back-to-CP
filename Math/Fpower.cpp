int64_t powmod(int64_t a, int64_t b){
    int64_t r = 1 % mod;
    a = ((a % mod) + mod) % mod;
    while (b > 0){
        if (b & 1) r = (int64_t)r * a % mod;
        a = (int64_t)a * a % mod;
        b >>= 1;
    }
    return r;
}
