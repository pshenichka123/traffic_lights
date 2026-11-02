#pragma once

#include "../Object/object.h"
#include "../Point/point.h"

#include <vector>

namespace traffic {

enum class Direction { forward, backward };

class Road final : public Object {
public:
    Road(int id, Point start, Point end, int forwardLanes = 1,
         int backwardLanes = 1, double speedLimit = 13.9);
    void act(Simulation&, double) override;

    int id() const { return id_; }
    Point start() const { return start_; }
    Point end() const { return end_; }
    double length() const { return length_; }
    void setEndpoints(Point start, Point end);
    int laneCount(Direction direction) const;
    const std::vector<int>& laneIds(Direction direction) const;
    int laneId(Direction direction, int index) const;
    void attachLane(int id, Direction direction);
    double speedLimit() const { return speedLimit_; }
    Point pointAt(double metres) const;
    Point lanePointAt(double metres, Direction direction, int lane) const;

private:
    int id_;
    Point start_;
    Point end_;
    double length_;
    int forwardLanes_;
    int backwardLanes_;
    double speedLimit_;
    std::vector<int> forwardLaneIds_;
    std::vector<int> backwardLaneIds_;
};

} // namespace traffic
