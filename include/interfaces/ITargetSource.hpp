#pragma once
// ITargetSource.hpp — where the mission gets its targets from.
//
// The mission needs two things: how many targets exist, and where one of them
// is right now. It has no business knowing whether that comes from a
// trajectory file replayed locally or from a seeker on the other end of a
// serial link, which is exactly the substitution this thesis makes.
//
// The lifecycle calls are part of the interface because every source owns a
// thread: one walks trajectories, the other blocks on a UART. Keeping them
// here lets the program start and stop either without knowing which it has.

#include "../dto/Target.hpp"

class ITargetSource
{
public:
    virtual ~ITargetSource() = default;

    // --- data ---
    // Both return snapshots by value: nothing hands out a reference into
    // state that another thread is writing.
    virtual int    getTargetCount() const = 0;
    virtual Target getTarget(int index) const = 0;

    // --- thread lifecycle ---
    virtual void run() = 0;                  // thread body
    virtual bool isThreadReady() const = 0;  // true once run() is up
    virtual void start() = 0;                // begin producing
    virtual void stop() = 0;                 // ask run() to return
};
