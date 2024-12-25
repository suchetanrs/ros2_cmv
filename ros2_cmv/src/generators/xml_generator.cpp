#include <ros2_cmv/generators/xml_generator.hpp>

namespace ros2_cmv
{
    void generatePluginXML(const std::string &libpath, const std::string &pluginName, const std::string &output_file)
    {
        try
        {
            // Open the output file
            std::ofstream ofs(output_file);
            if (!ofs.is_open())
            {
                std::cerr << "Error: Unable to open " << output_file << " for writing." << std::endl;
                return;
            }

            // Write the XML content
            ofs << "<?xml version=\"1.0\"?>\n";
            ofs << "<library path=\"" << libpath << "\">\n";
            ofs << "  <class name=\"" << pluginName << "\" type=\"ros2_cmv::" << "CustomMessageDisplay" << "\" base_class_type=\"rviz_common::Display\">\n";
            ofs << "    <description>Custom message display for RViz</description>\n";
            ofs << "  </class>\n";
            ofs << "</library>\n";

            // Close the file
            ofs.close();
            std::cout << "Generated " << output_file << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    void generatePackageXML(const std::string &new_name, const std::string &input_file, const std::string &output_file)
    {
        try
        {
            // Read the content of the input file
            std::ifstream ifs(input_file);
            if (!ifs.is_open())
            {
                std::cerr << "Error: Unable to open " << input_file << " for reading." << std::endl;
                return;
            }

            std::ostringstream content_stream;
            content_stream << ifs.rdbuf();
            std::string content = content_stream.str();
            ifs.close();

            // Replace the <name>...</name> tag
            size_t start_pos = content.find("<name>");
            size_t end_pos = content.find("</name>", start_pos);
            if (start_pos != std::string::npos && end_pos != std::string::npos)
            {
                start_pos += 6; // Move past the "<name>" tag
                content.replace(start_pos, end_pos - start_pos, new_name);
            }
            else
            {
                std::cerr << "Error: <name> tag not found in " << input_file << std::endl;
                return;
            }

            // Write the modified content to the output file
            std::ofstream ofs(output_file);
            if (!ofs.is_open())
            {
                std::cerr << "Error: Unable to open " << output_file << " for writing." << std::endl;
                return;
            }

            ofs << content;
            ofs.close();

            std::cout << "Generated " << output_file << " with package name: " << new_name << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
}