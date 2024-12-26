#ifndef COMMON_VALUES_HPP_
#define COMMON_VALUES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace ros2_cmv
{
    inline rclcpp::Logger logger = rclcpp::get_logger("plugin_generator");

    class GlobalValues
    {
    private:
        GlobalValues() = default;
        std_msgs::msg::Header header;

        // Delete copy constructor and assignment operator to prevent copying
        GlobalValues(const GlobalValues &) = delete;
        GlobalValues &operator=(const GlobalValues &) = delete;

    public:
        static GlobalValues &getInstance()
        {
            static GlobalValues instance;
            return instance;
        }

        void setHeader(const std_msgs::msg::Header &newHeader)
        {
            header = newHeader;
        }

        const std_msgs::msg::Header &getHeader() const
        {
            return header;
        }
    };

    inline GlobalValues& globalValues = GlobalValues::getInstance();
};

#endif