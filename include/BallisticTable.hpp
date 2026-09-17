#pragma once
#include <vector>
#include <cstddef>

// 5-dimensional ballistic lookup table with non-uniform axes.
// Each node stores a pre-computed flight time and horizontal distance;
// queries between nodes use multi-dimensional linear interpolation, and
// queries outside the grid clamp to the edge values.
struct BallisticTable
{
    std::vector<float> axisZ0;  // altitude
    std::vector<float> axisV0;  // speed
    std::vector<float> axisM;   // mass
    std::vector<float> axisD;   // drag
    std::vector<float> axisL;   // lift

    struct Result
    {
        float t     = 0.0f;  // flight time
        float hDist = 0.0f;  // horizontal distance
    };

    // Flat array sized |Z0| * |V0| * |M| * |D| * |L|.
    std::vector<Result> data;

    // Flat index for node [iZ0][iV0][iM][iD][iL].
    std::size_t index(int iz, int iv, int im, int id, int il) const
    {
        return ((((std::size_t)iz * axisV0.size() + iv)
                                 * axisM.size()  + im)
                                 * axisD.size()  + id)
                                 * axisL.size()  + il;
    }

    const Result& at(int iz, int iv, int im, int id, int il) const
    {
        return data[index(iz, iv, im, id, il)];
    }

    // Load the grid from a whitespace-delimited text file.
    bool load(const char* path);

    // Save the grid to a whitespace-delimited text file.
    bool save(const char* path) const;

    // Interpolated lookup; clamps to the table edges for out-of-range input.
    Result lookup(float Z0, float V0, float m, float d, float l) const;
};
