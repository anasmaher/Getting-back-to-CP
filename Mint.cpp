template <int M>
struct ModInt {
    int v;
    static constexpr int mod() { return M; }

    constexpr ModInt() : v(0) {}
    constexpr ModInt(long long x) : v(int((x % M + M) % M)) {}

    constexpr ModInt& operator+=(ModInt o) { v += o.v; if (v >= M) v -= M; return *this; }
    constexpr ModInt& operator-=(ModInt o) { v -= o.v; if (v <  0) v += M; return *this; }
    constexpr ModInt& operator*=(ModInt o) { v = int(1LL * v * o.v % M); return *this; }
    ModInt& operator/=(ModInt o) { return *this *= o.inv(); }

    constexpr ModInt operator+(ModInt o) const { return ModInt(*this) += o; }
    constexpr ModInt operator-(ModInt o) const { return ModInt(*this) -= o; }
    constexpr ModInt operator*(ModInt o) const { return ModInt(*this) *= o; }
    ModInt operator/(ModInt o) const { return ModInt(*this) /= o; }
    constexpr ModInt operator-() const { return v ? ModInt(M - v) : ModInt(); }

    constexpr bool operator==(ModInt o) const { return v == o.v; }
    constexpr bool operator!=(ModInt o) const { return v != o.v; }

    constexpr ModInt& operator++() { return *this += 1; }
    constexpr ModInt& operator--() { return *this -= 1; }
    constexpr ModInt  operator++(int) { ModInt t = *this; ++*this; return t; }
    constexpr ModInt  operator--(int) { ModInt t = *this; --*this; return t; }

    ModInt pow(long long e) const {
        ModInt r = 1, a = *this;
        for (; e > 0; e >>= 1, a *= a)
            if (e & 1) r *= a;
        return r;
    }

    ModInt inv() const {
        // extended Euclid — works for any M with gcd(v, M) == 1
        long long a = v, m = M, u = 1, w = 0;
        while (a) { long long t = m / a; m -= t * a; std::swap(a, m);
                                           u -= t * w; std::swap(u, w); }
        return ModInt(u);
    }

    friend std::ostream& operator<<(std::ostream& os, ModInt x) { return os << x.v; }
    friend std::istream& operator>>(std::istream& is, ModInt& x) {
        long long t; is >> t; x = ModInt(t); return is;
    }
};

using Mint = ModInt<1'000'000'007>;
