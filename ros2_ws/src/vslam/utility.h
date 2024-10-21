#ifndef UTILITY_H
#define UTILITY_H

#include "rclcpp/rclcpp.hpp"

class Utility
{
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp)
  {
    double seconds = stamp.sec + (stamp.nanosec * pow(10,-9));
    return seconds;
  }
};

#endif // UTILITY_H
