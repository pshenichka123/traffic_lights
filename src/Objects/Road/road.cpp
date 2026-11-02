#include "road.h"
#include "../../config.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {

namespace {
constexpr double minRoadLength = 6.5;
}

Road::Road(int id, Point start, Point end, int forwardLanes,
           int backwardLanes, double speedLimit)
    : id_(id), start_(start), end_(end),
      length_(std::hypot(end.x - start.x, end.y - start.y) * metresPerPixel),
      forwardLanes_(forwardLanes), backwardLanes_(backwardLanes),
      speedLimit_(speedLimit) {
    if (length_ < minRoadLength)
        throw std::invalid_argument("Road is too short");
    if (forwardLanes < 0 || forwardLanes > 4 || backwardLanes < 0 ||
        backwardLanes > 4 || (forwardLanes == 0 && backwardLanes == 0) ||
        !std::isfinite(speedLimit) ||
        speedLimit <= 0 || speedLimit > 60)
        throw std::invalid_argument("Invalid road parameters");
}

void Road::setEndpoints(Point start, Point end) {
    const double length = std::hypot(end.x - start.x, end.y - start.y) * metresPerPixel;
    if (!std::isfinite(length) || length < minRoadLength)
        throw std::invalid_argument("Road is too short or has invalid endpoints");
    start_ = start;
    end_ = end;
    length_ = length;
}

void Road::act(Simulation&, double) {}

int Road::laneCount(Direction direction) const {
    return direction == Direction::forward ? forwardLanes_ : backwardLanes_;
}

const std::vector<int>& Road::laneIds(Direction direction) const {
    return direction == Direction::forward ? forwardLaneIds_ : backwardLaneIds_;
}

int Road::laneId(Direction direction, int index) const {
    return laneIds(direction).at(static_cast<std::size_t>(index));
}

void Road::attachLane(int id, Direction direction) {
    auto& ids = direction == Direction::forward ? forwardLaneIds_ : backwardLaneIds_;
    if (static_cast<int>(ids.size()) >= laneCount(direction))
        throw std::logic_error("Too many lanes on road");
    ids.push_back(id);
}

Point Road::pointAt(double metres) const {
    const double part = std::clamp(metres / length_, 0.0, 1.0);
    return {start_.x + (end_.x - start_.x) * part,
            start_.y + (end_.y - start_.y) * part};
}

Point Road::lanePointAt(double metres, Direction direction, int lane) const {
    if (lane < 0 || lane >= laneCount(direction))
        throw std::out_of_range("Invalid lane");
    const double centreDistance = direction == Direction::forward
        ? metres : length_ - metres;
    Point point = pointAt(centreDistance);
    const double pixels = length_ / metresPerPixel;
    const double nx = -(end_.y - start_.y) / pixels;
    const double ny = (end_.x - start_.x) / pixels;
    const double offset = (lane + 0.5) * laneWidthPixels *
        (direction == Direction::forward ? 1.0 : -1.0);
    return {point.x + nx * offset, point.y + ny * offset};
}

} // namespace traffic
