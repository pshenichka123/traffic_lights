#pragma once

#include "../Object/object.h"
#include "../Road/road.h"

namespace traffic {

class Lane final : public Object {
public:
    Lane(int id, int roadId, Direction direction, int index);
    void act(Simulation&, double) override;

    int id() const { return id_; }
    int roadId() const { return roadId_; }
    Direction direction() const { return direction_; }
    int index() const { return index_; }

private:
    int id_;
    int roadId_;
    Direction direction_;
    int index_;
};

} // namespace traffic
