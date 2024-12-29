#ifndef CUSTOM_MSG_PROCESS_HPP_
#define CUSTOM_MSG_PROCESS_HPP_

#include "ros2_cmv/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

#include "ros2_cmv/exposed_displays.hpp"

namespace ros2_cmv
{
    void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<IExposedDisplay>> &enabledInstances);
}

#endif