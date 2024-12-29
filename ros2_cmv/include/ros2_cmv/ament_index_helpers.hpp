#ifndef AMENT_INDEX_HELPERS_
#define AMENT_INDEX_HELPERS_


#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "ros2_cmv/cmv_common.hpp"

namespace ros2_cmv
{
    struct Interface
    {
        std::string package;
        std::string base_name;
    };

    std::vector<Interface> getMsgInterfaces();

    std::string getMsgPath(std::string &package_name, std::string &msg_name);

    std::string getPackagePrefix(std::string package_name);
};

#endif