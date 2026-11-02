#include "spawn_point.h"
#include "../Simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {

SpawnPoint::SpawnPoint(int id, Road& road, Direction direction, int lane,
                       double carsPerSecond)
    : id_(id), road_(&road), direction_(direction), lane_(lane),
      laneId_(road.laneId(direction, lane)),
      carsPerSecond_(carsPerSecond) {
    if (lane < 0 || lane >= road.laneCount(direction))
        throw std::invalid_argument("Invalid spawn lane");
    setCarsPerSecond(carsPerSecond);
}

void SpawnPoint::setInitialSpeedRange(double minimum, double maximum) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
        minimum < 0 || maximum < minimum ||
        maximum > std::min(13.9, road_->speedLimit()))
        throw std::invalid_argument("Invalid initial speed range");
    minInitialSpeed_ = minimum;
    maxInitialSpeed_ = maximum;
}

void SpawnPoint::setCarsPerSecond(double value) {
    if (!std::isfinite(value) || value < 0 || value > 10)
        throw std::invalid_argument("Spawn rate must be between 0 and 10");
    carsPerSecond_ = value;
    if (value == 0) pending_ = 0;
}

void SpawnPoint::act(Simulation& simulation, double dt) {
    if (simulation.hasIncomingConnection(*road_, direction_)) {
        pending_ = 0;
        return;
    }
    pending_ = std::min(20.0, pending_ + carsPerSecond_ * dt);
    while (pending_ + 1e-9 >= 1.0) {
        const double speed = simulation.randomInitialSpeed(
            minInitialSpeed_, maxInitialSpeed_);
        if (!simulation.trySpawn(*road_, direction_, lane_, speed)) break;
        pending_ = std::max(0.0, pending_ - 1.0);
    }
}

} // namespace traffic
