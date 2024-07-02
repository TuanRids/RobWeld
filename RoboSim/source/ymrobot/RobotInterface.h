#pragma once
#include <vector>
#include "pch.h"
namespace nymrobot {
    class RobotInterface {
    public:
        virtual ~RobotInterface() = default;
        virtual void setter_get6pos(const std::vector<std::vector<float>>& get6pos_) = 0;
        virtual void set_limitangle(const std::vector<std::vector<float>>& getlim) = 0;
    };
}