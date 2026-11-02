#pragma once

#include "../Car/car.h"
#include "../Lane/lane.h"
#include "../../config.h"
#include "../Road/road.h"
#include "../SpawnPoint/spawn_point.h"

#include <cstddef>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

namespace traffic {

class Simulation {
public:
    Road& addRoad(Point start, Point end, int forwardLanes = 1,
                  int backwardLanes = 1, double speedLimit = 13.9);
    SpawnPoint& addSpawnPoint(Road& road, Direction direction, int lane,
                              double carsPerSecond = 0.5);
    bool trySpawn(Road& road, Direction direction, int lane,
                  double initialSpeed = 0);
    double randomInitialSpeed(double minimum, double maximum);
    void tick();
    void reset();
    void clear();
    bool removeRoad(int id);
    bool removeSpawnPoint(int id);

    const std::vector<std::unique_ptr<Road>>& roads() const { return roads_; }
    const std::vector<std::unique_ptr<SpawnPoint>>& spawnPoints() const { return spawnPoints_; }
    const std::vector<std::unique_ptr<Car>>& cars() const { return cars_; }
    const std::unordered_map<int, std::unique_ptr<Lane>>& lanes() const { return lanes_; }
    const Lane& laneById(int id) const;
    Road& roadForLane(int id) const;
    const Car* carAhead(const Car& car) const;
    bool isConnectedEnd(const Road& road, bool start) const;
    bool hasIncomingConnection(const Road& road, Direction direction) const;
    double time() const { return time_; }
    std::size_t exitedCars() const { return exitedCars_; }
    std::size_t spawnedCars() const { return spawnedCars_; }
    double availablePosition(const Car& car) const;

private:
    std::vector<std::unique_ptr<Road>> roads_;
    std::unordered_map<int, std::unique_ptr<Lane>> lanes_;
    std::vector<std::unique_ptr<SpawnPoint>> spawnPoints_;
    std::vector<std::unique_ptr<Car>> cars_;
    int nextId_ = 1;
    double time_ = 0;
    std::size_t exitedCars_ = 0;
    std::size_t spawnedCars_ = 0;
    std::mt19937_64 random_{0x5eedULL};
    int continuationLaneId(const Lane& lane) const;
    bool destinationFree(int laneId, double carLength) const;
};

} // namespace traffic
