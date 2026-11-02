#pragma once

namespace traffic {

class Simulation;

class Object {
public:
    virtual ~Object();
    virtual void act(Simulation& simulation, double dt) = 0;
};

} // namespace traffic
