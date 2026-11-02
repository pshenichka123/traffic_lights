#pragma once

#include "../Object/object.h"
namespace traffic {

class Car final : public Object {
public:
    Car(int id, int laneId, double maxSpeed = 13.9, double length = 4.5,
        double acceleration = 2.0, double initialSpeed = 0);
    void act(Simulation& simulation, double dt) override;

    int id() const { return id_; }
    int laneId() const { return laneId_; }
    double speed() const { return speed_; }
    double maxSpeed() const { return maxSpeed_; }
    double acceleration() const { return acceleration_; }
    double length() const { return length_; }
    double position() const { return position_; }
    bool finished() const { return finished_; }
    void enterLane(int laneId, double speed);
    void waitAtEnd();

private:
    int id_;
    int laneId_;
    double speed_ = 0;
    double maxSpeed_;
    double acceleration_;
    double length_;
    double position_ = 0;
    bool finished_ = false;
};

} // namespace traffic
