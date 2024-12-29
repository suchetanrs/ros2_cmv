#include "ros2_cmv/generators/generator.hpp"

namespace ros2_cmv
{
    bool checkMessageValidity(const std::string &msg_file_path, std::vector<int> &valid_line_idx)
    {
        bool headerExists = false;
        std::ifstream file(msg_file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: Unable to open " << msg_file_path);
            throw std::runtime_error("Error: Unable to open " + msg_file_path);
        }
        std::string line;
        int currIdx = -1;
        while (std::getline(file, line))
        {
            currIdx++;
            // find the first # and remove everything after it
            size_t comment_pos = line.find('#');
            if (comment_pos != std::string::npos)
            {
                line = line.substr(0, comment_pos);
            }

            // Trim whitespace
            line = trim(line);
            if (line.empty())
            {
                continue; // Skip empty lines
            }

            // Split line into parts using whitespace
            std::vector<std::string> parts = split(line);
            if (parts.size() == 3 || parts.size() == 2)
            {
                std::string msg_type = parts[0];
                std::string var_name = parts[1];
                // std::string default_value = parts[2]; // Not used in C++ generation
                if (msg_type == "std_msgs/Header")
                {
                    valid_line_idx.push_back(currIdx);
                    headerExists = true;
                    continue;
                }
                if (std::find(variableTypes.begin(), variableTypes.end(), msg_type) != variableTypes.end())
                {
                    valid_line_idx.push_back(currIdx);
                }
            }
        }
        file.close();
        if (!headerExists || valid_line_idx.size() == 0)
            return false;
        return true;
    }

    void generateFiles(std::string &msgFilePath, std::string &outputPackageDir,
                       std::string &selectedMessage, std::string &projectName,
                       bool newPackage)
    {
        std::vector<std::string> directories = {
            outputPackageDir + "/src/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/",
            outputPackageDir + "/include/" + projectName + "/message_specific/" +  + STRINGIFY(MESSAGE_NAME) + "/"};

        // Iterate through each directory and create it if it doesn't exist
        for (const auto &directory : directories)
        {
            try
            {
                if (std::filesystem::create_directories(directory))
                {
                    RCLCPP_INFO_STREAM(globalValues.getLogger(), "Directory created: " << directory);
                }
                else
                {
                    RCLCPP_INFO_STREAM(globalValues.getLogger(), "Directory already exists: " << directory);
                }
            }
            catch (const std::filesystem::filesystem_error &e)
            {
                throw std::runtime_error("Failed to create directory: " + directory);
            }
        }

        std::vector<int> validIdx;
        if (!checkMessageValidity(msgFilePath, validIdx))
        {
            throw std::runtime_error("Message" + selectedMessage + "is not valid");
        }

        // Parse the .msg file
        std::vector<Message> messages = parseMsgFile(msgFilePath, validIdx);
        if (messages.empty())
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "No valid messages found in the .msg file.");
            // throw std::runtime_error("No valid messages found in the .msg file.");
        }

        RCLCPP_INFO_STREAM(globalValues.getLogger(), "===================================================================");
        copyFile(getPackagePrefix("ros2_cmv") + "/include/ros2_cmv/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_display.hpp", outputPackageDir + "/include/" + projectName + "/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_display.hpp");
        RCLCPP_INFO_STREAM(globalValues.getLogger(), "===================================================================");
        generateMetadataHeader(messages, outputPackageDir + "/include/" + projectName + "/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_metadata.hpp", convertToIncludePath(selectedMessage), convertRosTypeToCpp(selectedMessage));
        generateProcessMsgFile(messages, outputPackageDir + "/src/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_process.cpp");
        RCLCPP_INFO_STREAM(globalValues.getLogger(), "===================================================================");
        copyFile(getPackagePrefix("ros2_cmv") + "/share/ros2_cmv/base_files/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_display.cpp", outputPackageDir + "/src/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_display.cpp");
        copyFile(getPackagePrefix("ros2_cmv") + "/include/ros2_cmv/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_process.hpp", outputPackageDir + "/include/" + projectName + "/message_specific/" + STRINGIFY(MESSAGE_NAME) + "/custom_msg_process.hpp");
        RCLCPP_INFO_STREAM(globalValues.getLogger(), "===================================================================");
        generatePluginXML(projectName, projectName, outputPackageDir + "/plugin.xml", projectName);
        if (newPackage)
        {
            RCLCPP_INFO_STREAM(globalValues.getLogger(), "===================================================================");
            generateCMakeLists(projectName, getPackagePrefix("ros2_cmv") + "/share/ros2_cmv/base_files/base_cmakelists.txt", outputPackageDir + "/CMakeLists.txt", convertToPackageName(selectedMessage));
            RCLCPP_INFO_STREAM(globalValues.getLogger(), "===================================================================");
            generatePackageXML(projectName, getPackagePrefix("ros2_cmv") + "/share/ros2_cmv/base_files/base_package.xml", outputPackageDir + "/package.xml", convertToPackageName(selectedMessage));
        }
    }
}