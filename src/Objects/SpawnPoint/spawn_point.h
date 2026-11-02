#pragma once

#include "../Object/object.h"
#include "../Road/road.h"

namespace traffic {

class SpawnPoint final : public Object {
public:
    SpawnPoint(int id, Road& road, Direction direction, int lane,
               double carsPerSecond);
    void act(Simulation& simulation, double dt) override;

    int id() const { return id_; }
    Road& road() const { return *road_; }
    Direction direction() const { return direction_; }
    int lane() const { return lane_; }
    int laneId() const { return laneId_; }
    double carsPerSecond() const { return carsPerSecond_; }
    double minInitialSpeed() const { return minInitialSpeed_; }
    double maxInitialSpeed() const { return maxInitialSpeed_; }
    void setCarsPerSecond(double value);
    void setInitialSpeedRange(double minimum, double maximum);
    void reset() { pending_ = 0; }

private:
    int id_;
    Road* road_;
    Direction direction_;
    int lane_;
    int laneId_;
    double carsPerSecond_;
    double minInitialSpeed_ = 0;
    double maxInitialSpeed_ = 0;
    double pending_ = 0;
};

} // namespace traffic
