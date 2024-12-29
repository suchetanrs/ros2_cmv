#ifndef HPP_CPP_GEENERATOR_HPP_
#define HPP_CPP_GEENERATOR_HPP_

#include "ros2_cmv/cmv_common.hpp"
#include "ros2_cmv/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

namespace ros2_cmv
{
    // Function to parse a .msg file and return a vector of Message structs
    std::vector<Message> parseMsgFile(const std::string &msg_file_path, std::vector<int> &valid_line_idx);

    // Function to generate custom_msg_metadata.hpp file.
    void generateMetadataHeader(std::vector<Message> &messages, const std::string &output_file,
                                const std::string &CUSTOM_MESSAGE_HEADER, const std::string &CUSTOM_MESSAGE_TYPE);

    void generateProcessMsgFile(std::vector<Message> &messages, const std::string &output_file);

    // Function to copy a file from input_path to output_path
    void copyFile(const std::string &input_path, const std::string &output_path);
};

#endif