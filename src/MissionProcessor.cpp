#include "../include/MissionProcessor.hpp"
#include "../include/JsonTargetProvider.hpp"
#include "../include/states/States.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> solver,
                                   std::unique_ptr<ITargetProvider>  targets,
                                   std::unique_ptr<IConfigLoader>     loader)
    : solver_(std::move(solver))
    , targets_(std::move(targets))
    , loader_(std::move(loader))
{
}

bool MissionProcessor::init(const char* configPath, const char* ammoPath,
                            const char* targetsPath)
{
    if (!loader_->load(configPath, ammoPath))
        return false;

    config_ = loader_->getConfig();
    ammo_   = loader_->getAmmoParams(config_.ammoName.c_str());

    // The JSON provider needs its trajectory file loaded.
    if (auto* jsonProvider = dynamic_cast<JsonTargetProvider*>(targets_.get()))
    {
        if (!jsonProvider->loadFromFile(targetsPath))
            return false;
    }

    currentIdx_  = 0;
    currentTime_ = 0.0f;
    stepCount_   = 0;
    log_.clear();

    // Seed the shared context and start in the Stopped state.
    ctx_.cfg       = config_;
    ctx_.position  = config_.startPos;
    ctx_.direction = config_.initialDir;
    ctx_.speed     = 0.0f;
    ctx_.dt        = config_.simTimeStep;
    ctx_.hasGoal   = true;
    state_         = std::make_unique<StateStopped>();

    targets_->interpolateAll(currentTime_);
    return true;
}

bool MissionProcessor::hasNext()
{
    return currentIdx_ < targets_->getTargetCount() && stepCount_ < MAX_STEPS;
}

DropResult MissionProcessor::planCurrentTarget()
{
    Coord targetNow = targets_->getTarget(currentIdx_);

    Ballistics bl = solver_->solve(config_.altitude, config_.attackSpeed, ammo_);
    double hLength    = bl.hLength;
    double flightTime = bl.flightTime;
    if (hLength < 0.0 || flightTime < 0.0)  // invalid model: aim straight at it
    {
        hLength    = 0.0;
        flightTime = 0.0;
    }

    // Where the target will be once the round lands -> aim there.
    Coord predicted = targets_->getTargetAt(
        currentIdx_, currentTime_ + (float)flightTime);

    Coord dir    = normalize(predicted - ctx_.position);
    Coord dropPt = predicted - dir * (float)hLength;

    DropResult r;
    r.dropPoint  = dropPt;
    r.targetPos  = predicted;
    r.hLength    = hLength;
    r.flightTime = flightTime;
    return r;
}

DropResult MissionProcessor::step()
{
    DropResult plan = planCurrentTarget();

    ctx_.goal    = plan.dropPoint;
    ctx_.hasGoal = true;
    ctx_.dt      = config_.simTimeStep;

    // Advance the state machine one tick.
    auto next = state_->execute(ctx_);
    if (next) state_ = std::move(next);

    // Log this step.
    LoggedStep ls;
    ls.position        = ctx_.position;
    ls.direction       = ctx_.direction;
    ls.state           = state_->name();
    ls.targetIndex     = currentIdx_;
    ls.dropPoint       = plan.dropPoint;
    ls.aimPoint        = plan.targetPos;
    ls.predictedTarget = plan.targetPos;
    log_.push_back(ls);

    // Reached the drop point for this target?
    if (length(ctx_.goal - ctx_.position) <= config_.hitRadius)
    {
        std::cout << "Drop on target " << currentIdx_
                  << " at (" << ctx_.position.x << ", " << ctx_.position.y << ")"
                  << " hLen=" << plan.hLength
                  << " t=" << plan.flightTime << '\n';
        ++currentIdx_;
        if (currentIdx_ >= targets_->getTargetCount())
            ctx_.hasGoal = false;
    }

    // Advance time and move the targets along their trajectories.
    currentTime_ += config_.simTimeStep;
    ++stepCount_;
    targets_->interpolateAll(currentTime_);

    return plan;
}

void MissionProcessor::reset()
{
    currentIdx_  = 0;
    currentTime_ = 0.0f;
    stepCount_   = 0;
    log_.clear();
    ctx_.position  = config_.startPos;
    ctx_.direction = config_.initialDir;
    ctx_.speed     = 0.0f;
    state_         = std::make_unique<StateStopped>();
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
    solver_ = std::move(solver);
}

bool MissionProcessor::writeLog(const char* path) const
{
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
        const LoggedStep& s = log_[i];
        f << "    {\n";
        f << "      \"position\": ";        coord(f, s.position);        f << ",\n";
        f << "      \"direction\": " << s.direction << ",\n";
        f << "      \"state\": \"" << s.state << "\",\n";
        f << "      \"targetIndex\": " << s.targetIndex << ",\n";
        f << "      \"dropPoint\": ";       coord(f, s.dropPoint);       f << ",\n";
        f << "      \"aimPoint\": ";        coord(f, s.aimPoint);        f << ",\n";
        f << "      \"predictedTarget\": "; coord(f, s.predictedTarget); f << "\n";
        f << "    }" << (i + 1 < log_.size() ? "," : "") << "\n";
    }
    f << "  ]\n}\n";
    return f.good();
}
