#include "../../include/mt/MissionRunner.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

MissionRunner::MissionRunner(const DroneConfig& cfg,
                             const AmmoParams& ammo,
                             std::unique_ptr<IBallisticSolver> solver,
                             DronePhysics* physics,
                             ThreadSafeTargetProvider* targets)
    : cfg_(cfg)
    , ammo_(ammo)
    , solver_(std::move(solver))
    , physics_(physics)
    , targets_(targets)
{
    ctx_.cfg = cfg_;
    state_   = std::make_unique<MtStopped>();
}

void MissionRunner::start() { started_.store(true); }
void MissionRunner::stop()  { running_.store(false); }

void MissionRunner::planAndCommand()
{
    // Pull the latest telemetry snapshot — the only drone state we read.
    DroneTelemetry tel = physics_->getTelemetry();
    ctx_.position  = tel.pos;
    ctx_.direction = tel.direction;
    ctx_.speed     = length(tel.speed);

    // Plan the intercept / drop point for the current target.
    Target tgt = targets_->getTarget(currentIdx_);

    Ballistics bl = solver_->solve(cfg_.altitude, cfg_.attackSpeed, ammo_);
    double hLength    = bl.hLength;
    double flightTime = bl.flightTime;
    if (hLength < 0.0 || flightTime < 0.0) { hLength = 0.0; flightTime = 0.0; }

    Coord predicted = tgt.pos + tgt.velocity * (float)flightTime;
    Coord dir       = normalize(predicted - ctx_.position);
    Coord dropPt    = predicted - dir * (float)hLength;

    ctx_.goal = dropPt;

    // Advance the state machine and command physics.
    auto next = state_->execute(ctx_);
    if (next) state_ = std::move(next);
    physics_->sendCommand(ctx_.command);

    // Log this planning step (state label comes from telemetry's mode).
    {
        Step s;
        s.position          = tel.pos;
        s.direction         = tel.direction;
        s.state             = toString(tel.state);
        s.targetIndex       = currentIdx_;
        s.dropPoint         = dropPt;
        s.aimPoint          = predicted;
        s.predictedTarget   = predicted;
        s.timeSecSinceStart = tel.timeSecSinceStart;

        std::lock_guard<std::mutex> lock(logMutex_);
        log_.push_back(s);
    }

    // Reached the drop point?
    if (length(ctx_.goal - ctx_.position) <= cfg_.hitRadius)
    {
        std::cout << "Drop on target " << currentIdx_
                  << " at (" << ctx_.position.x << ", " << ctx_.position.y << ")"
                  << " t=" << tel.timeSecSinceStart << '\n';
        ++currentIdx_;
        if (currentIdx_ >= targets_->getTargetCount())
            done_.store(true);
    }
}

void MissionRunner::run()
{
    ready_.store(true);

    const float scale = cfg_.timeScale > 0.0f ? cfg_.timeScale : 1.0f;
    const float dt    = cfg_.simTimeStep;
    int steps = 0;

    while (running_.load() && !done_.load())
    {
        if (started_.load())
        {
            planAndCommand();
            if (++steps >= MAX_STEPS) break;
        }
        std::this_thread::sleep_for(std::chrono::duration<float>(dt / scale));
    }

    // Make sure the drone is commanded to stop once the mission is over.
    physics_->sendCommand({ DroneState::STOPPED, 0.0f });
}

bool MissionRunner::writeLog(const char* path) const
{
    std::lock_guard<std::mutex> lock(logMutex_);

    std::ofstream f(path);
    if (!f.is_open())
    {
        std::cerr << "Cannot write " << path << '\n';
        return false;
    }

    auto coord = [](std::ostream& o, const Coord& c) {
        o << "{ \"x\": " << c.x << ", \"y\": " << c.y << " }";
    };

    f << "{\n  \"steps\": [\n";
    for (std::size_t i = 0; i < log_.size(); ++i)
    {
        const Step& s = log_[i];
        f << "    {\n";
        f << "      \"position\": ";        coord(f, s.position);        f << ",\n";
        f << "      \"direction\": " << s.direction << ",\n";
        f << "      \"state\": \"" << s.state << "\",\n";
        f << "      \"targetIndex\": " << s.targetIndex << ",\n";
        f << "      \"dropPoint\": ";       coord(f, s.dropPoint);       f << ",\n";
        f << "      \"aimPoint\": ";        coord(f, s.aimPoint);        f << ",\n";
        f << "      \"predictedTarget\": "; coord(f, s.predictedTarget); f << ",\n";
        f << "      \"timeSecSinceStart\": " << s.timeSecSinceStart << "\n";
        f << "    }" << (i + 1 < log_.size() ? "," : "") << "\n";
    }
    f << "  ]\n}\n";
    return f.good();
}
