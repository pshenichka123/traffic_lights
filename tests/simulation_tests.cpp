#include "Objects/Simulation/simulation.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

using namespace traffic;

int main() {
    {
        Simulation oneWay;
        Road& forward = oneWay.addRoad({0, 0}, {500, 0}, 2, 0, 10.0);
        Road& backward = oneWay.addRoad({0, 50}, {500, 50}, 0, 3, 10.0);
        assert(forward.laneCount(Direction::backward) == 0);
        assert(forward.laneIds(Direction::backward).empty());
        assert(backward.laneCount(Direction::forward) == 0);
        assert(backward.laneIds(Direction::forward).empty());
        assert(oneWay.lanes().size() == 5);
        assert(oneWay.trySpawn(forward, Direction::forward, 0));
        assert(oneWay.trySpawn(backward, Direction::backward, 2));
        oneWay.tick();
        assert(oneWay.cars().size() == 2);

        bool rejectedEmptyRoad = false;
        try {
            oneWay.addRoad({0, 100}, {500, 100}, 0, 0, 10.0);
        } catch (const std::invalid_argument&) {
            rejectedEmptyRoad = true;
        }
        assert(rejectedEmptyRoad);
    }

    {
        Simulation empty;
        Road& testRoad = empty.addRoad({0, 0}, {1000, 0});
        Car car(100, testRoad.laneId(Direction::forward, 0), 10.0, 4.5, 2.0);
        assert(car.speed() == 0);
        assert(car.maxSpeed() == 10.0);
        assert(car.acceleration() == 2.0);
        for (int i = 0; i < 20; ++i) car.act(empty, tickSeconds);
        assert(std::abs(car.speed() - 2.0) < 1e-9);
        assert(std::abs(car.position() - 1.0) < 1e-9);
        for (int i = 0; i < 80; ++i) car.act(empty, tickSeconds);
        assert(std::abs(car.speed() - 10.0) < 1e-9);
        assert(std::abs(car.position() - 25.0) < 1e-9);
        car.act(empty, tickSeconds);
        assert(std::abs(car.speed() - 10.0) < 1e-9);
        assert(std::abs(car.position() - 25.5) < 1e-9);
        assert(empty.lanes().size() == 2);
        assert(empty.laneById(car.laneId()).roadId() == testRoad.id());
    }

    {
        Simulation lanes;
        Road& testRoad = lanes.addRoad({0, 0}, {500, 0}, 2, 2, 5.0);
        assert(lanes.trySpawn(testRoad, Direction::forward, 0));
        assert(!lanes.trySpawn(testRoad, Direction::forward, 0));
        assert(lanes.trySpawn(testRoad, Direction::forward, 1));
        assert(lanes.trySpawn(testRoad, Direction::backward, 0));
        assert(lanes.cars().size() == 3);
        for (int i = 0; i < 200; ++i) lanes.tick();
        for (const auto& car : lanes.cars())
            assert(car->speed() <= testRoad.speedLimit() + 1e-9);
    }

    Simulation simulation;
    Road& road = simulation.addRoad({0, 0}, {1000, 0}, 2, 2, 8.0);
    assert(std::abs(road.length() - 200.0) < 1e-9);
    assert(road.laneCount(Direction::forward) == 2);
    assert(road.laneCount(Direction::backward) == 2);
    assert(road.lanePointAt(0, Direction::forward, 0).x == 0);
    assert(road.lanePointAt(0, Direction::backward, 0).x == 1000);
    assert(road.lanePointAt(0, Direction::forward, 0).y > 0);
    assert(road.lanePointAt(0, Direction::backward, 0).y < 0);

    SpawnPoint& spawn = simulation.addSpawnPoint(road, Direction::forward, 0, 2.0);
    simulation.addSpawnPoint(road, Direction::backward, 1, 1.0);
    simulation.addSpawnPoint(road, Direction::forward, 1, 1.0);
    for (int i = 0; i < 80; ++i) simulation.tick();
    assert(std::abs(simulation.time() - 4.0) < 1e-9);
    assert(simulation.spawnedCars() >= 4);
    assert(simulation.cars().size() >= 4);
    assert(simulation.cars()[0]->position() > 0);
    assert(simulation.cars()[0]->speed() < simulation.cars()[0]->maxSpeed());
    assert(&simulation.roadForLane(simulation.cars()[0]->laneId()) == &road);
    bool hasReverse = false;
    bool hasOtherLane = false;
    for (const auto& car : simulation.cars()) {
        assert(car->speed() <= road.speedLimit() + 1e-9);
        const Lane& lane = simulation.laneById(car->laneId());
        hasReverse |= lane.direction() == Direction::backward;
        hasOtherLane |= lane.direction() == Direction::forward && lane.index() == 1;
    }
    assert(hasReverse && hasOtherLane);

    for (int i = 0; i < 1200; ++i) simulation.tick();
    assert(simulation.exitedCars() > 0);
    for (const auto& car : simulation.cars()) {
        assert(car->position() >= 0);
        assert(car->position() < road.length());
    }

    spawn.setCarsPerSecond(0);
    for (const auto& point : simulation.spawnPoints())
        point->setCarsPerSecond(0);
    simulation.reset();
    assert(simulation.time() == 0);
    assert(simulation.cars().empty());
    assert(simulation.spawnedCars() == 0);
    for (int i = 0; i < 40; ++i) simulation.tick();
    assert(simulation.cars().empty());

    assert(simulation.removeRoad(road.id()));
    assert(simulation.roads().empty());
    assert(simulation.lanes().empty());
    assert(simulation.spawnPoints().empty());

    {
        Simulation speeds;
        Road& roadWithSpawn = speeds.addRoad({0, 0}, {1000, 0});
        SpawnPoint& point = speeds.addSpawnPoint(
            roadWithSpawn, Direction::forward, 0, 1.0);
        point.setInitialSpeedRange(1.0, 3.0);
        for (int i = 0; i < 20; ++i) speeds.tick();
        assert(speeds.cars().size() == 1);
        assert(speeds.cars()[0]->speed() >= 1.0);
        assert(speeds.cars()[0]->speed() <= 3.0);
        assert(speeds.cars()[0]->laneId() == point.laneId());
        double sum = 0;
        for (int i = 0; i < 10000; ++i) {
            const double sample = speeds.randomInitialSpeed(1.0, 3.0);
            assert(sample >= 1.0 && sample <= 3.0);
            sum += sample;
        }
        assert(std::abs(sum / 10000 - 2.0) < 0.05);
    }

    {
        Simulation following;
        Road& testRoad = following.addRoad({0, 0}, {1000, 0});
        assert(Default_brake_acceleration == 3.0);
        assert(following.trySpawn(testRoad, Direction::forward, 0));
        for (int i = 0; i < 60; ++i) following.tick();
        assert(following.trySpawn(testRoad, Direction::forward, 0, 13.9));
        const double initialFollowerSpeed = following.cars()[1]->speed();
        following.tick();
        assert(following.cars()[1]->speed() < initialFollowerSpeed);
        for (int i = 0; i < 120; ++i) {
            following.tick();
            if (following.cars().size() == 2) {
                const Car& front = *following.cars()[0];
                const Car& back = *following.cars()[1];
                assert(front.position() - back.position() - back.length() >=
                       minimumCarGap - 1e-8);
            }
        }
    }

    {
        Simulation connected;
        Road& first = connected.addRoad({0, 0}, {150, 0}, 2, 2);
        Road& bend = connected.addRoad({150, 0}, {150, 150}, 2, 2);
        assert(connected.isConnectedEnd(first, false));
        assert(connected.isConnectedEnd(bend, true));
        assert(connected.hasIncomingConnection(bend, Direction::forward));
        assert(!connected.trySpawn(bend, Direction::forward, 0));
        assert(connected.trySpawn(first, Direction::forward, 1, 10.0));
        for (int i = 0; i < 70; ++i) connected.tick();
        assert(connected.cars().size() == 1);
        assert(connected.cars()[0]->laneId() == bend.laneId(Direction::forward, 1));
        assert(connected.cars()[0]->position() > 0);
        assert(connected.exitedCars() == 0);
        for (int i = 0; i < 100; ++i) connected.tick();
        assert(connected.cars().empty());
        assert(connected.exitedCars() == 1);

        const double oldLength = bend.length();
        bend.setEndpoints({160, 0}, {160, 200});
        assert(bend.length() > oldLength);
        assert(!connected.isConnectedEnd(first, false));
        assert(connected.trySpawn(bend, Direction::forward, 0));
        bool rejected = false;
        try { bend.setEndpoints({160, 0}, {161, 0}); }
        catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
        assert(bend.length() > oldLength);
    }

    {
        Simulation reverse;
        Road& first = reverse.addRoad({0, 0}, {150, 0});
        Road& second = reverse.addRoad({0, 0}, {-150, 0});
        assert(reverse.trySpawn(first, Direction::backward, 0, 10.0));
        for (int i = 0; i < 70; ++i) reverse.tick();
        assert(reverse.cars().size() == 1);
        assert(reverse.cars()[0]->laneId() == second.laneId(Direction::forward, 0));
    }

    {
        Simulation branches;
        Road& first = branches.addRoad({0, 0}, {150, 0});
        branches.addRoad({150, 0}, {150, 150});
        Road& straight = branches.addRoad({150, 0}, {300, 0});
        assert(branches.trySpawn(first, Direction::forward, 0, 10.0));
        for (int i = 0; i < 70; ++i) branches.tick();
        assert(branches.cars().size() == 1);
        assert(branches.cars()[0]->laneId() ==
               straight.laneId(Direction::forward, 0));
    }

    {
        Simulation entry;
        entry.addRoad({0, 0}, {150, 0});
        Road& second = entry.addRoad({160, 0}, {310, 0});
        entry.addSpawnPoint(second, Direction::forward, 0, 2.0);
        second.setEndpoints({150, 0}, {300, 0});
        for (int i = 0; i < 40; ++i) entry.tick();
        assert(entry.spawnedCars() == 0);
        second.setEndpoints({170, 0}, {320, 0});
        for (int i = 0; i < 40; ++i) entry.tick();
        assert(entry.spawnedCars() > 0);
    }

    {
        Simulation blocked;
        Road& first = blocked.addRoad({0, 0}, {35, 0});
        Road& second = blocked.addRoad({40, 0}, {540, 0});
        assert(blocked.trySpawn(second, Direction::forward, 0));
        second.setEndpoints({35, 0}, {535, 0});
        assert(blocked.trySpawn(first, Direction::forward, 0, 13.9));
        bool waited = false;
        bool transferred = false;
        for (int i = 0; i < 100; ++i) {
            blocked.tick();
            const Car& incoming = *blocked.cars()[1];
            if (incoming.laneId() == first.laneId(Direction::forward, 0) &&
                std::abs(incoming.position() - first.length()) < 1e-8)
                waited = true;
            if (incoming.laneId() == second.laneId(Direction::forward, 0)) {
                transferred = true;
                const Car& leader = *blocked.cars()[0];
                assert(leader.position() - incoming.position() - incoming.length() >=
                       minimumCarGap - 1e-8);
                break;
            }
        }
        assert(waited && transferred);
    }
}
