#include "rclcpp/rclcpp.hpp"

#include "../include/pfh_publisher/HistogramPublisher.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Start processing data from the node 
    rclcpp::spin(std::make_shared<HistogramPublisher>());

    rclcpp::shutdown();
    return 0;
}
