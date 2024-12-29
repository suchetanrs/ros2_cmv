#include <ros2_cmv/generators/cmakelists_generator.hpp>

namespace ros2_cmv
{
    void generateCMakeLists(const std::string &project_name, const std::string &input_file,
                            const std::string &output_file, std::string additional_package)
    {
        // Read the content of the input file
        std::ifstream ifs(input_file);
        if (!ifs.is_open())
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: Unable to open " << input_file << " for reading.");
            throw std::runtime_error("Unable to open " + input_file + " for reading.");
        }

        std::ostringstream content_stream;
        content_stream << ifs.rdbuf();
        std::string content = content_stream.str();
        ifs.close();

        // 1. Replace the project name in the content
        size_t pos = content.find("project(");
        if (pos != std::string::npos)
        {
            size_t start = pos + 8; // Skip "project("
            size_t end = content.find(")", start);
            if (end != std::string::npos)
            {
                content.replace(start, end - start, project_name);
            }
        }

        // 2. Add find_package for the additional package as REQUIRED
        std::string find_package_entry = "find_package(" + additional_package + " REQUIRED)";
        size_t find_pkg_pos = content.find("find_package(" + additional_package);
        if (find_pkg_pos == std::string::npos)
        {
            // Insert the find_package entry after the last existing find_package
            size_t last_find_pkg = content.rfind("find_package(");
            if (last_find_pkg != std::string::npos)
            {
                size_t insert_pos = content.find("\n", last_find_pkg);
                if (insert_pos != std::string::npos)
                {
                    content.insert(insert_pos + 1, find_package_entry + "\n");
                    RCLCPP_INFO_STREAM(globalValues.getLogger(), "Added find_package entry for '" << additional_package << "'.");
                }
                else
                {
                    // If no newline found after last find_package, append at the end
                    content += "\n" + find_package_entry + "\n";
                    RCLCPP_INFO_STREAM(globalValues.getLogger(), "Appended find_package entry for '" << additional_package << "' at the end.");
                }
            }
            else
            {
                throw std::runtime_error("Could not find any find_package entries in the base CMakeLists.txt.");
            }
        }
        else
        {
            RCLCPP_WARN_STREAM(globalValues.getLogger(), "find_package for '" << additional_package << "' already exists.");
        }

        // 3. Add the additional package to the dependencies set
        std::string dependencies_start = "set(dependencies";
        size_t dep_pos = content.find(dependencies_start);
        if (dep_pos != std::string::npos)
        {
            size_t open_paren = content.find("(", dep_pos);
            size_t close_paren = content.find(")", open_paren);
            if (open_paren != std::string::npos && close_paren != std::string::npos)
            {
                std::string dependencies_block = content.substr(open_paren + 1, close_paren - open_paren - 1);
                if (dependencies_block.find(additional_package) == std::string::npos)
                {
                    // Insert the additional package before the closing parenthesis
                    content.insert(close_paren, additional_package);
                    RCLCPP_INFO_STREAM(globalValues.getLogger(), "Added '" << additional_package << "' to dependencies.");
                }
                else
                {
                    RCLCPP_WARN_STREAM(globalValues.getLogger(), "Dependency '" << additional_package << "' already exists.");
                }
            }
            else
            {
                RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Warning: Malformed dependencies set. Unable to add dependency.");
                throw std::runtime_error("Malformed dependencies set in the base CMakeLists.txt.");
            }
        }
        else
        {
            throw std::runtime_error("Could not find dependencies set in the base CMakeLists.txt.");
        }

        // Write the modified content to the output file
        std::ofstream ofs(output_file);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: Unable to open " << output_file << " for writing.");
            throw std::runtime_error("Unable to open " + output_file + " for writing.");
        }

        ofs << content;
        ofs.close();

        RCLCPP_INFO_STREAM(globalValues.getLogger(), "Generated " << output_file << " with project name: " << project_name);
    }
}