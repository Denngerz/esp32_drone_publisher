#include "../include/BallisticTable.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>

namespace
{
    // Linear interpolation of a Result (both fields in parallel).
    BallisticTable::Result lerp(const BallisticTable::Result& a,
                                const BallisticTable::Result& b, float t)
    {
        return {
            a.t     + (b.t     - a.t)     * t,
            a.hDist + (b.hDist - a.hDist) * t
        };
    }

    // Lower index + fraction [0..1] for a value on a sorted axis.
    struct Interp { int lo; float frac; };

    Interp findInterp(float val, const std::vector<float>& axis)
    {
        if (axis.size() < 2)               return { 0, 0.0f };
        if (val <= axis.front())           return { 0, 0.0f };
        if (val >= axis.back())            return { (int)axis.size() - 2, 1.0f };

        auto it = std::lower_bound(axis.begin(), axis.end(), val);
        int i = (int)(it - axis.begin()) - 1;
        if (i < 0) i = 0;

        float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
        return { i, frac };
    }
}

bool BallisticTable::load(const char* path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "Cannot open table " << path << '\n';
        return false;
    }

    int nZ, nV, nM, nD, nL;
    f >> nZ >> nV >> nM >> nD >> nL;

    axisZ0.resize(nZ); for (auto& v : axisZ0) f >> v;
    axisV0.resize(nV); for (auto& v : axisV0) f >> v;
    axisM.resize(nM);  for (auto& v : axisM)  f >> v;
    axisD.resize(nD);  for (auto& v : axisD)  f >> v;
    axisL.resize(nL);  for (auto& v : axisL)  f >> v;

    std::size_t total = (std::size_t)nZ * nV * nM * nD * nL;
    data.resize(total);

    // Order: Z0 -> V0 -> m -> d -> l (outermost -> innermost).
    for (std::size_t i = 0; i < total; ++i)
        f >> data[i].t >> data[i].hDist;

    return f.good() || f.eof();
}

bool BallisticTable::save(const char* path) const
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        std::cerr << "Cannot write table " << path << '\n';
        return false;
    }

    f << axisZ0.size() << ' ' << axisV0.size() << ' ' << axisM.size() << ' '
      << axisD.size()  << ' ' << axisL.size()  << '\n';

    auto writeAxis = [&](const std::vector<float>& a) {
        for (float v : a) f << v << ' ';
        f << '\n';
    };
    writeAxis(axisZ0);
    writeAxis(axisV0);
    writeAxis(axisM);
    writeAxis(axisD);
    writeAxis(axisL);

    for (const auto& r : data)
        f << r.t << ' ' << r.hDist << '\n';

    return f.good();
}

BallisticTable::Result BallisticTable::lookup(
    float Z0, float V0, float m, float d, float l) const
{
    Interp iz = findInterp(Z0, axisZ0);
    Interp iv = findInterp(V0, axisV0);
    Interp im = findInterp(m,  axisM);
    Interp id = findInterp(d,  axisD);
    Interp il = findInterp(l,  axisL);

    // Collapse the 2^5 = 32 hypercube corners one axis at a time:
    // 32 -> 16 -> 8 -> 4 -> 2 -> 1.

    // l: 32 -> 16
    Result v[16];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       for (int e = 0; e < 2; ++e)
       {
           const auto& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
           const auto& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
           v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
       }

    // d: 16 -> 8
    Result w[8];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       w[a * 4 + b * 2 + c] =
           lerp(v[a * 8 + b * 4 + c * 2], v[a * 8 + b * 4 + c * 2 + 1], id.frac);

    // m: 8 -> 4
    Result u[4];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);

    // V0: 4 -> 2
    Result s[2];
    for (int a = 0; a < 2; ++a)
        s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);

    // Z0: 2 -> 1
    return lerp(s[0], s[1], iz.frac);
}
