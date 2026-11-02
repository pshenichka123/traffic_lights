#include "lane.h"

#include <stdexcept>

namespace traffic {

Lane::Lane(int id, int roadId, Direction direction, int index)
    : id_(id), roadId_(roadId), direction_(direction), index_(index) {
    if (id <= 0 || roadId <= 0 || index < 0)
        throw std::invalid_argument("Invalid lane");
}

void Lane::act(Simulation&, double) {}

} // namespace traffic
