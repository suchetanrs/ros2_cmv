#include <ros2_cmv/generators/header_generator.hpp>

namespace ros2_cmv
{
    // Function to trim whitespace from both ends of a string
    std::string trim(const std::string &s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
            return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        return s.substr(start, end - start + 1);
    }

    // Function to split a string by whitespace
    std::vector<std::string> split(const std::string &s)
    {
        std::vector<std::string> tokens;
        std::istringstream iss(s);
        std::string token;
        while (iss >> token)
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    std::vector<Message> parse_msg_file(const std::string &msg_file_path)
    {
        std::vector<Message> messages;
        std::ifstream file(msg_file_path);
        if (!file.is_open())
        {
            std::cerr << "Error: Unable to open " << msg_file_path << std::endl;
            return messages;
        }

        std::string line;
        while (std::getline(file, line))
        {
            // Remove comments
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

            // Split line into parts
            std::vector<std::string> parts = split(line);
            if (parts.size() == 3)
            {
                std::string msg_type = parts[0];
                std::string var_name = parts[1];
                // std::string default_value = parts[2]; // Not used in C++ generation
                if (msg_type != "std_msgs/Header")
                {
                    messages.push_back(Message{msg_type, var_name});
                }
            }
            else if (parts.size() == 2)
            {
                std::string msg_type = parts[0];
                std::string var_name = parts[1];
                if (msg_type != "std_msgs/Header")
                {
                    messages.push_back(Message{msg_type, var_name});
                }
            }
            else
            {
                std::cerr << "Skipping invalid line: " << line << std::endl;
            }
        }

        file.close();
        return messages;
    }

    std::string convert_ros_type_to_cpp(const std::string &ros_type)
    {
        size_t slash_pos = ros_type.find('/');
        if (slash_pos == std::string::npos)
        {
            std::cerr << "Invalid ROS type format: " << ros_type << std::endl;
            return ros_type; // Return as is
        }
        std::string package = ros_type.substr(0, slash_pos);
        std::string message = ros_type.substr(slash_pos + 1);
        return package + "::msg::" + message;
    }

    std::string convertCamelCaseToSnakeCase(const std::string &input)
    {
        std::string result = input;

        // First substitution
        std::regex first_pattern(R"((.)([A-Z][a-z]+))");
        result = std::regex_replace(result, first_pattern, "$1_$2");

        // Second substitution
        std::regex second_pattern(R"(([a-z0-9])([A-Z]))");
        result = std::regex_replace(result, second_pattern, "$1_$2");

        // Convert to lowercase
        std::transform(result.begin(), result.end(), result.begin(), ::tolower);

        return result;
    }

    void generate_cpp_header(const std::string &MSG_FILE, const std::string &output_file,
                             const std::string &CUSTOM_MESSAGE_HEADER, const std::string &CUSTOM_MESSAGE_TYPE)
    {
        // Parse the .msg file
        std::vector<Message> messages = parse_msg_file(MSG_FILE);
        if (messages.empty())
        {
            std::cerr << "No valid messages found in the .msg file." << std::endl;
            return;
        }

        std::ofstream ofs(output_file);
        if (!ofs.is_open())
        {
            std::cerr << "Error: Unable to open " << output_file << " for writing." << std::endl;
            return;
        }

        // Header Guard
        ofs << "#ifndef CUSTOM_MSG_TEMPLATES_\n";
        ofs << "#define CUSTOM_MSG_TEMPLATES_\n\n";

        // Includes
        ofs << "#include <string>\n";
        ofs << "#include <vector>\n";
        ofs << "#include <unordered_map>\n";
        ofs << "#include <memory>\n\n";

        ofs << "#include \"" << CUSTOM_MESSAGE_HEADER << "\"\n";
        ofs << "#include \"" << "ros2_cmv/exposed_displays.hpp" << "\"\n\n";

        // Using declaration
        ofs << "using CustomMessage = " << CUSTOM_MESSAGE_TYPE << ";\n\n";

        // variableTypes Vector
        ofs << "static const std::vector<std::string> variableTypes = {\n";
        for (const auto &msg : messages)
        {
            ofs << "    \"" << msg.type << "\",\n";
        }
        ofs << "};\n\n";

        // variableNames Vector
        ofs << "static const std::vector<std::string> variableNames = {\n";
        for (const auto &msg : messages)
        {
            ofs << "    \"" << msg.name << "\",\n";
        }
        ofs << "};\n\n";

        // processCustomMessage Function
        ofs << "inline void processCustomMessage(const CustomMessage::ConstSharedPtr &msg, std::unordered_map<std::string, std::shared_ptr<ros2_cmv::IExposedDisplay>>& enabledInstances)\n";
        ofs << "{\n";
        for (const auto &msg : messages)
        {
            std::string cpp_type = convert_ros_type_to_cpp(msg.type);
            std::string var_name = msg.name;
            ofs << "    if(enabledInstances.find(\"" << var_name << "\") != enabledInstances.end()) {\n";
            ofs << "        enabledInstances[\"" << var_name << "\"]->processMessage(std::make_shared<const " << cpp_type << ">(msg->" << var_name << "));\n";
            ofs << "    }\n";
        }
        ofs << "}\n\n";

        // End Header Guard
        ofs << "#endif // CUSTOM_MSG_TEMPLATES_\n";

        ofs.close();
        std::cout << "Generated " << output_file << std::endl;
    }

    void copyFile(const std::string &input_path, const std::string &output_path)
    {
        // Open the input file for reading
        std::ifstream input_file(input_path, std::ios::binary);
        if (!input_file.is_open())
        {
            std::cerr << "Error: Unable to open " << input_path << " for reading." << std::endl;
            return;
        }

        // Open the output file for writing
        std::ofstream output_file(output_path, std::ios::binary);
        if (!output_file.is_open())
        {
            std::cerr << "Error: Unable to open " << output_path << " for writing." << std::endl;
            input_file.close();
            return;
        }

        // Copy the file content
        output_file << input_file.rdbuf();

        // Close the files
        input_file.close();
        output_file.close();

        std::cout << "File copied from " << input_path << " to " << output_path << std::endl;
    }

    std::string convertToIncludePath(const std::string &input)
    {
        // Find the position of the last '/'
        size_t pos = input.rfind('/');
        if (pos == std::string::npos)
        {
            throw std::runtime_error("Invalid input: No '/' found in the input.");
        }

        // Split the input into prefix and class name
        std::string prefix = input.substr(0, pos);
        std::string className = input.substr(pos + 1);

        // Convert the class name to snake_case and append ".hpp"
        std::string snakeCaseName = convertCamelCaseToSnakeCase(className) + ".hpp";

        // Construct the include path
        return prefix + "/msg/" + snakeCaseName;
    }

    std::string convertToNamespace(const std::string &input)
    {
        // Find the position of the last '/'
        size_t pos = input.rfind('/');
        if (pos == std::string::npos)
        {
            throw std::runtime_error("Invalid input: No '/' found in the input.");
        }

        // Split the input into prefix and class name
        std::string prefix = input.substr(0, pos);
        std::string className = input.substr(pos + 1);

        // Construct the namespace string
        return prefix + "::msg::" + className;
    }

    // Function to extract the package name from the given input
    std::string convertToPackageName(const std::string &input)
    {
        // Find the position of the last '/'
        size_t pos = input.rfind('/');

        // Extract and return the package name (substring before the last '/')
        return input.substr(0, pos);
    }

    std::string convertToRvizPluginName(const std::string &input)
    {
        // Find the slash
        std::size_t slashPos = input.find('/');
        if (slashPos == std::string::npos)
        {
            throw std::runtime_error("Invalid input: No '/' found in the input.");
        }

        // Extract the parts before and after the slash
        std::string packagePart = input.substr(0, slashPos);
        std::string classPart = input.substr(slashPos + 1);

        // Convert the classPart to lowercase
        std::string classPartLower = classPart;
        std::transform(
            classPartLower.begin(),
            classPartLower.end(),
            classPartLower.begin(),
            ::tolower);

        // Construct the final string
        // e.g. "ros2_cmv_example_example_rviz_plugin"
        return "rviz_plugin_" + packagePart + "_" + classPartLower;
        // return packagePart + "_" + classPartLower + "_rviz_plugin";
    }
}