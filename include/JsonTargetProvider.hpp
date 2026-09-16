#pragma once
#include "interfaces/ITargetProvider.hpp"
#include <vector>

class JsonTargetProvider : public ITargetProvider
{
public:
    bool  loadFromFile(const char* path);
    int   getTargetCount() override;
    Coord getTarget(int index) override;
    Coord getTargetAt(int index, float time) override;
    void  interpolateAll(float time) override;
 
private:
    std::vector<std::vector<Coord>> positions_;
    std::vector<Coord>              current_; 
    float                           arrayTimeStep_ = 1.0f;
};