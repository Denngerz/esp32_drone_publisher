#include "../../include/mt/ThreadSafeTargetProvider.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(float arrayTimeStep,
                                                   float targetTimeStep,
                                                   float timeScale)
    : arrayTimeStep_(arrayTimeStep)
    , targetTimeStep_(targetTimeStep)
    , timeScale_(timeScale)
{
}

bool ThreadSafeTargetProvider::loadFromFile(const char* path)
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

    trajectories_.assign(targetCount, std::vector<Coord>(timeSteps));
    for (int i = 0; i < targetCount; ++i)
        for (int k = 0; k < timeSteps; ++k)
        {
            trajectories_[i][k].x = j["targets"][i]["positions"][k]["x"];
            trajectories_[i][k].y = j["targets"][i]["positions"][k]["y"];
        }

    count_ = targetCount;
    current_.assign(targetCount, Target{});
    advance(0.0f);  // initial snapshot at t = 0
    return true;
}

// Interpolate one target's position along its looping trajectory and derive
// its velocity from the finite difference of neighbouring nodes.
Target ThreadSafeTargetProvider::sample(int index, float simTime) const
{
    const auto& traj = trajectories_[index];
    const int   n    = static_cast<int>(traj.size());
    Target out;
    if (n == 0) return out;
    if (n == 1) { out.pos = traj[0]; return out; }

    int idx = static_cast<int>(std::floor(simTime / arrayTimeStep_)) % n;
    if (idx < 0) idx += n;
    int next = (idx + 1) % n;

    float frac = (simTime - std::floor(simTime / arrayTimeStep_) * arrayTimeStep_)
               / arrayTimeStep_;

    Coord a = traj[idx];
    Coord b = traj[next];
    out.pos      = a + (b - a) * frac;
    out.velocity = (b - a) / arrayTimeStep_;
    return out;
}

void ThreadSafeTargetProvider::advance(float simTime)
{
    std::vector<Target> snap(count_);
    for (int i = 0; i < count_; ++i)
        snap[i] = sample(i, simTime);

    std::lock_guard<std::mutex> lock(mutex_);
    current_ = std::move(snap);
}

void ThreadSafeTargetProvider::start()  { started_.store(true); }
void ThreadSafeTargetProvider::stop()   { running_.store(false); }

int ThreadSafeTargetProvider::getTargetCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
}

Target ThreadSafeTargetProvider::getTarget(int index) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < 0 || index >= static_cast<int>(current_.size()))
        return Target{};
    return current_[index];
}

void ThreadSafeTargetProvider::run()
{
    ready_.store(true);

    const float scale = timeScale_ > 0.0f ? timeScale_ : 1.0f;
    float simTime = 0.0f;

    while (running_.load())
    {
        if (started_.load())
        {
            simTime += targetTimeStep_;
            advance(simTime);
        }
        std::this_thread::sleep_for(
            std::chrono::duration<float>(targetTimeStep_ / scale));
    }
}
