string toBin(int n, int width = -1) {
    string s;
    if (n == 0) s = "0";
    while (n > 0) { s += char('0' + (n & 1)); n >>= 1; }
    reverse(s.begin(), s.end());
    if (width > (int)s.size()) s = string(width - s.size(), '0') + s;
    return s;
}
