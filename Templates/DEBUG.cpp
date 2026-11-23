/*
Basic values, Multiple values at once, Containers, Pairs, tuples, nesting, Inside loops == debug() or debug( , , ... , )
C-arrays == debugArr() or debugArr2() for 2D

make sure to adjust file name and location etc in snippets too
*/

#pragma once
#include <bits/stdc++.h>
using namespace std;

// ── to_string overloads: what each type looks like when printed ─────────
string to_string(const string& s)   { return '"' + s + '"'; }
string to_string(const char* s)     { return to_string((string) s); }
string to_string(bool b)            { return (b ? "true" : "false"); }
string to_string(char c)            { return string(1, c); }

// vector<bool> is special — operator[] returns a proxy, not bool&
string to_string(vector<bool> v) {
    bool first = true;
    string res = "{";
    for (int i = 0; i < (int)v.size(); i++) {
        if (!first) res += ", ";
        first = false;
        res += to_string(static_cast<bool>(v[i]));
    }
    return res + "}";
}

template <size_t N>
string to_string(bitset<N> v) {
    string res;
    for (size_t i = 0; i < N; i++) res += static_cast<char>('0' + v[i]);
    return res;
}

// any container with begin()/end() (vector, set, map, list, deque, ...)
template <typename A>
string to_string(A v) {
    bool first = true;
    string res = "{";
    for (const auto& x : v) {
        if (!first) res += ", ";
        first = false;
        res += to_string(x);
    }
    return res + "}";
}

template <typename A, typename B>
string to_string(pair<A, B> p) {
    return "(" + to_string(p.first) + ", " + to_string(p.second) + ")";
}

template <typename A, typename B, typename C>
string to_string(tuple<A, B, C> p) {
    return "(" + to_string(get<0>(p)) + ", "
                + to_string(get<1>(p)) + ", "
                + to_string(get<2>(p)) + ")";
}

// ── printers ───────────────────────────────────────────────────────────
void debug_out() { cerr << endl; }

template <typename Head, typename... Tail>
void debug_out(Head H, Tail... T) {
    cerr << " " << to_string(H);
    debug_out(T...);
}

template <typename T>
void debug_arr_1d(T* arr, int n, const string& name) {
    cerr << "[" << name << "]: {";
    for (int i = 0; i < n; i++) {
        if (i) cerr << ", ";
        cerr << to_string(arr[i]);
    }
    cerr << "}" << endl;
}

template <typename T, int M>
void debug_arr_2d(T (*arr)[M], int n, const string& name) {
    cerr << "[" << name << "]: {" << endl;
    for (int i = 0; i < n; i++) {
        cerr << "  {";
        for (int j = 0; j < M; j++) {
            if (j) cerr << ", ";
            cerr << to_string(arr[i][j]);
        }
        cerr << "}" << endl;
    }
    cerr << "}" << endl;
}

// ── macros ─────────────────────────────────────────────────────────────
#define debug(...)           cerr << "[" << #__VA_ARGS__ << "]:", debug_out(__VA_ARGS__)
#define debugArr(arr, n)     debug_arr_1d(arr, n, #arr)
#define debugArr2(arr, n)    debug_arr_2d(arr, n, #arr)
