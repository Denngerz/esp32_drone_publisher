#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include "../dto/Coord.hpp"
#include "../dto/Target.hpp"
#include "../interfaces/ITargetSource.hpp"

// Moves targets along their (private) trajectories on its own thread and
// publishes only current position + velocity snapshots under a mutex.
class ThreadSafeTargetProvider : public ITargetSource
{
public:
    explicit ThreadSafeTargetProvider(float arrayTimeStep = 1.0f,
                                      float targetTimeStep = 0.05f,
                                      float timeScale = 10.0f);

    bool loadFromFile(const char* path);

    // Thread lifecycle.
    void run() override;              // thread body
    bool isThreadReady() const override { return ready_.load(); }
    void start() override;            // begin moving the targets
    void stop() override;             // signal stop (owner joins the thread)
    bool running() const { return running_.load(); }

    // Snapshots (copy under mutex, no references to internal data).
    int    getTargetCount() const override;
    Target getTarget(int index) const override;

    // A local trajectory file cannot go silent.
    bool healthy() const override { return true; }

private:
    void   advance(float simTime);  // recompute snapshots for a given sim time
    Target sample(int index, float simTime) const;

    std::vector<std::vector<Coord>> trajectories_;  // private node data
    float arrayTimeStep_;
    float targetTimeStep_;
    float timeScale_;

    mutable std::mutex  mutex_;
    std::vector<Target> current_;   // published snapshots
    int                 count_ = 0;

    std::atomic<bool> ready_{ false };
    std::atomic<bool> started_{ false };
    std::atomic<bool> running_{ true };
};
