#include <ros2_cmv/generators/xml_generator.hpp>

namespace ros2_cmv
{
    void generatePluginXML(const std::string &libpath, const std::string &pluginName, const std::string &output_file,
                           const std::string &project_name)
    {
        (void)libpath;
        try
        {
            // Open the output file
            std::ofstream ofs(output_file);
            if (!ofs.is_open())
            {
                RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: Unable to open " << output_file << " for writing.");
                throw std::runtime_error("Unable to open " + output_file + " for writing.");
            }

            // Write the XML content
            ofs << "<?xml version=\"1.0\"?>\n";
            ofs << "<library path=\"" << project_name << "_" << STRINGIFY(MESSAGE_NAME) << "_rviz_plugin\">\n";
            // ofs << "<library path=\"" << libpath << "\">\n";
            ofs << "  <class name=\"" << pluginName << "\" type=\"" << project_name << "::" << "CustomMessageDisplay" << "\" base_class_type=\"rviz_common::Display\">\n";
            ofs << "    <description>Custom message display for RViz</description>\n";
            ofs << "  </class>\n";
            ofs << "</library>\n";

            // Close the file
            ofs.close();
            RCLCPP_INFO_STREAM(globalValues.getLogger(), "Generated " << output_file);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: " << e.what());
            throw std::runtime_error("Error generating " + output_file + ": " + e.what());
        }
    }

    void generatePackageXML(const std::string &new_name, const std::string &input_file, const std::string &output_file,
                            std::string additional_package)
    {
        try
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

            // 1. Replace the <name>...</name> tag
            size_t start_pos = content.find("<name>");
            size_t end_pos = content.find("</name>", start_pos);
            if (start_pos != std::string::npos && end_pos != std::string::npos)
            {
                start_pos += 6; // Move past the "<name>" tag
                content.replace(start_pos, end_pos - start_pos, new_name);
            }
            else
            {
                RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: <name> tag not found in " << input_file);
                throw std::runtime_error("Error: <name> tag not found in " + input_file);
            }

            // 2. Replace the <depend>...</depend> tag
            size_t last_depend_end_pos = content.rfind("</depend>");
            if (last_depend_end_pos != std::string::npos)
            {
                // Check if the dependency already exists
                const std::string dependency_tag = "<depend>" + additional_package + "</depend>";
                if (content.find(dependency_tag) == std::string::npos)
                {
                    // Dependency does not exist; insert it
                    last_depend_end_pos += 9; // Move past the last "</depend>"
                    const std::string new_line = "\n  " + dependency_tag;
                    content.insert(last_depend_end_pos, new_line);
                }
                else
                {
                    RCLCPP_WARN_STREAM(globalValues.getLogger(), "Dependency '" << additional_package << "' already exists in the package.xml file.");
                }
            }
            else
            {
                throw std::runtime_error("Error: No <depend> lines found in " + input_file);
            }

            // Write the modified content to the output file
            std::ofstream ofs(output_file);
            if (!ofs.is_open())
            {
                RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: Unable to open " << output_file << " for writing.");
                return;
            }

            ofs << content;
            ofs.close();

            RCLCPP_INFO_STREAM(globalValues.getLogger(), "Generated " << output_file << " with package name: " << new_name);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(globalValues.getLogger(), "Error: " << e.what());
            throw std::runtime_error("Error generating " + output_file + ": " + e.what());
        }
    }
}