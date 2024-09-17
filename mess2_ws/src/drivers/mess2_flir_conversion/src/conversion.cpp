
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class FLIRConversion : public rclcpp::Node
{
public:
    explicit FLIRConversion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("flir_conversion", options)
    {
        this->declare_parameter("dir_logs", "");
        this->declare_parameter("dir_sub", "");
        this->declare_parameter("mode_gain", "high");
        this->declare_parameter("name_actors", (""));
        this->declare_parameter("units", "C");

        this->get_parameter("dir_logs", dir_logs_);
        this->get_parameter("dir_sub", dir_sub_);
        this->get_parameter("mode_gain", mode_gain_);
        this->get_parameter("name_actors", name_actors_);
        this->get_parameter("units", units_);
    }
private:
    void conversion()
    {
        double c_gain;
        if (mode_gain_ == "high") {
            c_gain = 0.04;
        } else if (mode_gain_ == "low") {
            c_gain = 0.4;
        } else {
            RCLCPP_ERROR(this->get_logger(), "invalid gain mode (must be high or low)");
        }

        double c_units;
        if (units_ == "C") {
            c_units = -273.15;
        } else if (units_ == "K") {
            c_units = 0.0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "invalid units (must be C or K)");
        }

        for (const auto& name_actor : name_actors_) {
            // path_meta = 
        }
    }

    std::string dir_logs_;
    std::string dir_sub_;
    std::string mode_gain_;
    std::vector<std::string> name_actors_;
    std::string units_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FLIRConversion>();
    rclcpp::shutdown();
    return 0;
}
