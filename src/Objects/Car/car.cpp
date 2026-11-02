#include "car.h"
#include "../../config.h"
#include "../Road/road.h"
#include "../Simulation/simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {

Car::Car(int id, int laneId, double maxSpeed, double length,
         double acceleration, double initialSpeed)
    : id_(id), laneId_(laneId), speed_(initialSpeed), maxSpeed_(maxSpeed),
      acceleration_(acceleration), length_(length) {
    if (!std::isfinite(maxSpeed) || !std::isfinite(length) ||
        !std::isfinite(acceleration) || !std::isfinite(initialSpeed) ||
        laneId <= 0 || maxSpeed < 0 || length <= 0 || acceleration < 0 ||
        initialSpeed < 0 || initialSpeed > maxSpeed)
        throw std::invalid_argument("Invalid car");
}

void Car::enterLane(int laneId, double speed) {
    if (laneId <= 0 || !std::isfinite(speed) || speed < 0)
        throw std::invalid_argument("Invalid destination lane");
    laneId_ = laneId;
    position_ = 0;
    speed_ = std::min(speed, maxSpeed_);
    finished_ = false;
}

void Car::waitAtEnd() {
    speed_ = 0;
    finished_ = false;
}

void Car::act(Simulation& simulation, double dt) {
    if (finished_ || dt <= 0) return;

    const Road& road = simulation.roadForLane(laneId_);
    double nextSpeed = std::min({maxSpeed_, road.speedLimit(),
                                 speed_ + acceleration_ * dt});
    if (const Car* leader = simulation.carAhead(*this)) {
        const double distance = leader->position() - position_ - length_
                                - minimumCarGap;
        const double relativeSpeed = speed_ - leader->speed();
        if (distance <= 0) {
            nextSpeed = std::min(nextSpeed, leader->speed());
        } else if (relativeSpeed > 0) {
            // With the default brake, will relative speed reach zero before 1 m?
            const double remainingRelativeSpeed = std::sqrt(std::max(
                0.0, relativeSpeed * relativeSpeed -
                2 * Default_brake_acceleration * distance));
            if (remainingRelativeSpeed > 0) {
                const double requiredBrake = relativeSpeed * relativeSpeed /
                                             (2 * distance);
                nextSpeed = std::min(nextSpeed,
                    std::max(leader->speed(), speed_ - requiredBrake * dt));
            }
        }
    }
    const double desiredDistance = (speed_ + nextSpeed) * 0.5 * dt;
    const double allowedPosition = simulation.availablePosition(*this);
    const double newPosition = std::min(position_ + desiredDistance, allowedPosition);
    const double travelled = std::max(0.0, newPosition - position_);
    speed_ = travelled + 1e-9 < desiredDistance ? travelled / dt : nextSpeed;
    position_ = newPosition;
    finished_ = position_ >= road.length();
}

} // namespace traffic
