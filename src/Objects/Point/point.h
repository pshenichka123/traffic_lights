#pragma once

namespace traffic {

struct Point {
    Point() = default;
    Point(double x, double y);

    double x = 0;
    double y = 0;
};

} // namespace traffic
