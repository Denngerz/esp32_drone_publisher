#include "../include/JsonTargetProvider.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include "nlohmann/json.hpp"
 
using json = nlohmann::json;
 
// constructor and destructor removed — std::vector manages its own memory
 
bool JsonTargetProvider::loadFromFile(const char* path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "Cannot open " << path << '\n';
        return false;
    }
 
    json j;
    f >> j;
 
    const int targetCount = j["targetCount"];
    const int timeSteps   = j["timeSteps"];
    arrayTimeStep_ = j.value("targetArrayTimeStep", 1.0f);
 
    // was: positions_ = new Coord*[targetCount]; + inner new Coord[timeSteps]
    positions_.assign(targetCount, std::vector<Coord>(timeSteps));
 
    for (int i = 0; i < targetCount; ++i)
    {
        for (int k = 0; k < timeSteps; ++k)
        {
            positions_[i][k].x = j["targets"][i]["positions"][k]["x"];
            positions_[i][k].y = j["targets"][i]["positions"][k]["y"];
        }
    }
 
    // was: current_ = new Coord[targetCount];
    current_.resize(targetCount);
    for (int i = 0; i < targetCount; ++i)
    {
        current_[i] = positions_[i][0];
    }
 
    return true;
}
 
int JsonTargetProvider::getTargetCount()
{
    return static_cast<int>(current_.size());  // was: return targetCount_;
}
 
Coord JsonTargetProvider::getTarget(int index)
{
    return current_[index];
}
 
Coord JsonTargetProvider::getTargetAt(int index, float time)
{
    const int timeSteps = static_cast<int>(positions_[index].size());  // was: timeSteps_
    int idx = static_cast<int>(std::floor(time / arrayTimeStep_)) % timeSteps;
    if (idx < 0) idx += timeSteps;
 
    int next = (idx + 1) % timeSteps;
    float frac = (time - idx * arrayTimeStep_) / arrayTimeStep_;
 
    Coord a = positions_[index][idx];
    Coord b = positions_[index][next];
    return a + (b - a) * frac;
}
 
void JsonTargetProvider::interpolateAll(float time)
{
    // was: for (int i = 0; i < targetCount_; ++i)
    for (int i = 0; i < static_cast<int>(current_.size()); ++i)
    {
        current_[i] = getTargetAt(i, time);
    }
}