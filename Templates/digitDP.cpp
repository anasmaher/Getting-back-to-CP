//https://github.com/AbdoSleem552/competitive-programming-template

#include <iostream>
#include <string>
#include <vector>
#include <cstring>

using namespace std;

/*
 * Single-Pass Digit DP Template
 * 
 * Computes the answer for [L, R] directly. 
 * We pad the string L with leading zeros so it matches the length of R.
 * 
 * The two flags:
 * 1. f1: Are we constrained by the lower bound (L)?
 * 2. f2: Are we constrained by the upper bound (R)?
 */

string L, R;
ll dp[20][2][2][2]; // [idx][f1][f2][st]

ll solve(int idx, bool f1, bool f2, bool st) {
    // Base Case: We've successfully built a valid number of length R.size()
    if (idx == R.size()) {
        return 1; // Return 1 if property is satisfied, else 0
    }
    

    if (dp[idx][f1][f2][st] != -1) {
        return dp[idx][f1][f2][st];
    }
    
    // The "two flags" dictate our loop boundaries (down and up)
    int down = f1 ? (L[idx] - '0') : 0;
    int up = f2 ? (R[idx] - '0') : 9;
    
    ll ans = 0;
    
    for (int d = down; d <= up; d++) {
        
        // Update flags for the next recursive step
        bool new_f1 = f1 && (d == down);
        bool new_f2 = f2 && (d == up);
        bool new_st = st && (d == 0);
        
        // Example: Track properties here
        // if (new_st) { ... } else { ... }
        
        ans += solve(idx + 1, new_f1, new_f2, new_st);
    }
    
    return dp[idx][f1][f2][st] = ans;
}

ll query(ll left, ll right) {
    L = to_string(left);
    R = to_string(right);
    
    // Pad L with leading zeros so it matches the length of R
    // Example: L="7", R="120" -> L="007", R="120"
    string pad = "";
    for (int i = 0; i < R.size() - L.size(); i++) {
        pad += '0';
    }
    L = pad + L;
    
    // Initialize dp table with -1
    memset(dp, -1, sizeof(dp));
    
    // Start recursion:
    // idx = 0
    // f1 = true (we cannot pick digits below L's first digit)
    // f2 = true (we cannot pick digits above R's first digit)
    // st = true (we haven't placed any non-zero digits yet)
    return solve(0, true, true, true);
}
