#include "simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {

namespace {
constexpr double defaultCarLength = 4.5;
constexpr double defaultCarMaxSpeed = 13.9;
}

Road& Simulation::addRoad(Point start, Point end, int forwardLanes,
                          int backwardLanes, double speedLimit) {
    auto road = std::make_unique<Road>(nextId_, start, end, forwardLanes,
                                       backwardLanes, speedLimit);
    ++nextId_;
    Road& result = *road;
    roads_.push_back(std::move(road));
    for (Direction direction : {Direction::forward, Direction::backward}) {
        for (int index = 0; index < result.laneCount(direction); ++index) {
            const int laneId = nextId_++;
            lanes_.emplace(laneId, std::make_unique<Lane>(
                laneId, result.id(), direction, index));
            result.attachLane(laneId, direction);
        }
    }
    return result;
}

const Lane& Simulation::laneById(int id) const {
    auto it = lanes_.find(id);
    if (it == lanes_.end()) throw std::out_of_range("Lane not found");
    return *it->second;
}

Road& Simulation::roadForLane(int id) const {
    const int roadId = laneById(id).roadId();
    for (const auto& road : roads_)
        if (road->id() == roadId) return *road;
    throw std::out_of_range("Road for lane not found");
}

const Car* Simulation::carAhead(const Car& car) const {
    const Car* nearest = nullptr;
    for (const auto& candidate : cars_) {
        if (candidate.get() != &car && candidate->laneId() == car.laneId() &&
            !candidate->finished() && candidate->position() > car.position() &&
            (!nearest || candidate->position() < nearest->position()))
            nearest = candidate.get();
    }
    return nearest;
}

bool Simulation::isConnectedEnd(const Road& road, bool start) const {
    const Point endpoint = start ? road.start() : road.end();
    for (const auto& other : roads_) {
        if (other.get() == &road) continue;
        if (std::hypot(endpoint.x - other->start().x,
                       endpoint.y - other->start().y) < 1e-6 ||
            std::hypot(endpoint.x - other->end().x,
                       endpoint.y - other->end().y) < 1e-6)
            return true;
    }
    return false;
}

bool Simulation::hasIncomingConnection(const Road& road, Direction direction) const {
    return isConnectedEnd(road, direction == Direction::forward);
}

int Simulation::continuationLaneId(const Lane& lane) const {
    const Road& current = roadForLane(lane.id());
    const bool forward = lane.direction() == Direction::forward;
    const Point endpoint = forward ? current.end() : current.start();
    const double inX = (forward ? current.end().x - current.start().x
                                : current.start().x - current.end().x);
    const double inY = (forward ? current.end().y - current.start().y
                                : current.start().y - current.end().y);
    const double inLength = std::hypot(inX, inY);
    double bestAlignment = -2;
    int bestRoadId = 0;
    int bestLaneId = 0;
    for (const auto& candidate : roads_) {
        if (candidate.get() == &current) continue;
        for (Direction direction : {Direction::forward, Direction::backward}) {
            if (candidate->laneCount(direction) == 0) continue;
            const Point entry = direction == Direction::forward
                ? candidate->start() : candidate->end();
            if (std::hypot(endpoint.x - entry.x, endpoint.y - entry.y) >= 1e-6)
                continue;
            const Point exit = direction == Direction::forward
                ? candidate->end() : candidate->start();
            const double outX = exit.x - entry.x, outY = exit.y - entry.y;
            const double alignment = (inX * outX + inY * outY) /
                (inLength * std::hypot(outX, outY));
            if (alignment > bestAlignment + 1e-9 ||
                (std::abs(alignment - bestAlignment) <= 1e-9 &&
                 (bestRoadId == 0 || candidate->id() < bestRoadId))) {
                bestAlignment = alignment;
                bestRoadId = candidate->id();
                bestLaneId = candidate->laneId(direction,
                    std::min(lane.index(), candidate->laneCount(direction) - 1));
            }
        }
    }
    return bestLaneId;
}

bool Simulation::destinationFree(int laneId, double carLength) const {
    for (const auto& car : cars_)
        if (car->laneId() == laneId && !car->finished() &&
            car->position() < carLength + minimumCarGap)
            return false;
    return true;
}

SpawnPoint& Simulation::addSpawnPoint(Road& road, Direction direction, int lane,
                                      double carsPerSecond) {
    if (std::none_of(roads_.begin(), roads_.end(),
        [&road](const auto& owned) { return owned.get() == &road; }))
        throw std::invalid_argument("Road does not belong to simulation");
    if (std::any_of(spawnPoints_.begin(), spawnPoints_.end(),
        [&](const auto& point) { return &point->road() == &road &&
            point->direction() == direction && point->lane() == lane; }))
        throw std::invalid_argument("Spawn point already exists on lane");
    auto point = std::make_unique<SpawnPoint>(nextId_, road, direction,
                                              lane, carsPerSecond);
    ++nextId_;
    SpawnPoint& result = *point;
    spawnPoints_.push_back(std::move(point));
    return result;
}

bool Simulation::trySpawn(Road& road, Direction direction, int lane,
                          double initialSpeed) {
    if (std::none_of(roads_.begin(), roads_.end(),
        [&road](const auto& owned) { return owned.get() == &road; }))
        throw std::invalid_argument("Road does not belong to simulation");
    if (lane < 0 || lane >= road.laneCount(direction))
        throw std::invalid_argument("Invalid spawn lane");
    if (hasIncomingConnection(road, direction)) return false;
    if (!std::isfinite(initialSpeed) || initialSpeed < 0 ||
        initialSpeed > std::min(defaultCarMaxSpeed, road.speedLimit()))
        throw std::invalid_argument("Invalid initial speed");
    const int laneId = road.laneId(direction, lane);
    for (const auto& car : cars_) {
        if (car->laneId() == laneId &&
            car->position() < defaultCarLength + minimumCarGap)
            return false;
    }
    cars_.push_back(std::make_unique<Car>(nextId_++, laneId,
        defaultCarMaxSpeed, defaultCarLength, 2.0, initialSpeed));
    ++spawnedCars_;
    return true;
}

double Simulation::randomInitialSpeed(double minimum, double maximum) {
    if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
        minimum < 0 || maximum < minimum)
        throw std::invalid_argument("Invalid speed range");
    if (minimum == maximum) return minimum;
    return std::uniform_real_distribution<double>(minimum, maximum)(random_);
}

double Simulation::availablePosition(const Car& car) const {
    double limit = roadForLane(car.laneId()).length();
    if (const Car* leader = carAhead(car))
        limit = std::min(limit, leader->position() - car.length() - minimumCarGap);
    return std::max(car.position(), limit);
}

void Simulation::tick() {
    // A snapshot ensures newly spawned cars first move on the following tick.
    std::vector<Object*> objects;
    objects.reserve(roads_.size() + lanes_.size() +
                    spawnPoints_.size() + cars_.size());
    for (auto& road : roads_) objects.push_back(road.get());
    for (auto& lane : lanes_) objects.push_back(lane.second.get());
    for (auto& point : spawnPoints_) objects.push_back(point.get());
    for (auto& car : cars_) objects.push_back(car.get());
    for (Object* object : objects) object->act(*this, tickSeconds);

    for (auto& car : cars_) {
        if (!car->finished()) continue;
        const Lane& source = laneById(car->laneId());
        const int nextLaneId = continuationLaneId(source);
        if (nextLaneId == 0) continue;
        if (destinationFree(nextLaneId, car->length())) {
            const double speedLimit = roadForLane(nextLaneId).speedLimit();
            car->enterLane(nextLaneId, std::min(car->speed(), speedLimit));
        } else {
            car->waitAtEnd();
        }
    }

    const auto oldSize = cars_.size();
    cars_.erase(std::remove_if(cars_.begin(), cars_.end(),
                [](const auto& car) { return car->finished(); }), cars_.end());
    exitedCars_ += oldSize - cars_.size();
    time_ += tickSeconds;
}

void Simulation::reset() {
    cars_.clear();
    for (auto& point : spawnPoints_) point->reset();
    time_ = 0;
    exitedCars_ = 0;
    spawnedCars_ = 0;
    random_.seed(0x5eedULL);
}

void Simulation::clear() {
    reset();
    spawnPoints_.clear();
    lanes_.clear();
    roads_.clear();
    nextId_ = 1;
}

bool Simulation::removeRoad(int id) {
    auto it = std::find_if(roads_.begin(), roads_.end(),
                           [id](const auto& road) { return road->id() == id; });
    if (it == roads_.end()) return false;
    Road* road = it->get();
    cars_.erase(std::remove_if(cars_.begin(), cars_.end(),
                [this, road](const auto& car) {
                    return &roadForLane(car->laneId()) == road;
                }), cars_.end());
    spawnPoints_.erase(std::remove_if(spawnPoints_.begin(), spawnPoints_.end(),
                [road](const auto& point) { return &point->road() == road; }), spawnPoints_.end());
    for (Direction direction : {Direction::forward, Direction::backward})
        for (int laneId : road->laneIds(direction)) lanes_.erase(laneId);
    roads_.erase(it);
    return true;
}

bool Simulation::removeSpawnPoint(int id) {
    auto it = std::find_if(spawnPoints_.begin(), spawnPoints_.end(),
                           [id](const auto& point) { return point->id() == id; });
    if (it == spawnPoints_.end()) return false;
    spawnPoints_.erase(it);
    return true;
}

} // namespace traffic
