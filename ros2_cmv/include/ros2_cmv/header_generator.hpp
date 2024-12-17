#ifndef HEADER_GENERATOR_HPP_
#define HEADER_GENERATOR_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <filesystem>

struct Message
{
    std::string type;
    std::string name;
};

// Function to trim whitespace from both ends of a string
std::string trim(const std::string &s);

// Function to split a string by whitespace
std::vector<std::string> split(const std::string &s);

std::vector<Message> parse_msg_file(const std::string &msg_file_path);

std::string convert_ros_type_to_cpp(const std::string &ros_type);

void generate_cpp_header(const std::string &MSG_FILE, const std::string &output_file,
                         const std::string &CUSTOM_MESSAGE_HEADER, const std::string &CUSTOM_MESSAGE_TYPE);

#endif