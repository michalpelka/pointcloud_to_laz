#pragma once
#include <string>
#include <array>
#include <vector>
namespace mandeye
{
    struct Point{
        std::array<float,3> xyz;
        double timestamp;
        uint8_t line_id;
        uint8_t tag;
        float reflectivity;
    };
bool saveLaz(const std::string& filename, const std::vector<Point>& buffer);
}