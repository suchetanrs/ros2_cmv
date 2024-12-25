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
            std::cerr << "Error: Unable to open " << input_file << " for reading." << std::endl;
            return;
        }

        std::ostringstream content_stream;
        content_stream << ifs.rdbuf();
        std::string content = content_stream.str();
        ifs.close();

        // Replace the project name in the content
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
                    std::cout << "Added find_package entry for '" << additional_package << "'." << std::endl;
                }
                else
                {
                    // If no newline found after last find_package, append at the end
                    content += "\n" + find_package_entry + "\n";
                    std::cout << "Appended find_package entry for '" << additional_package << "' at the end." << std::endl;
                }
            }
            else
            {
                // If no find_package exists, insert after project()
                size_t project_end = content.find(")", pos);
                if (project_end != std::string::npos)
                {
                    content.insert(project_end + 1, "\n" + find_package_entry + "\n");
                    std::cout << "Inserted find_package entry for '" << additional_package << "' after project()." << std::endl;
                }
                else
                {
                    // If project() is also missing, prepend at the beginning
                    content = find_package_entry + "\n" + content;
                    std::cout << "Inserted find_package entry for '" << additional_package << "' at the beginning." << std::endl;
                }
            }
        }
        else
        {
            std::cout << "find_package for '" << additional_package << "' already exists." << std::endl;
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
                    std::cout << "Added '" << additional_package << "' to dependencies." << std::endl;
                }
                else
                {
                    std::cout << "Dependency '" << additional_package << "' already exists." << std::endl;
                }
            }
            else
            {
                std::cerr << "Warning: Malformed dependencies set. Unable to add dependency." << std::endl;
            }
        }
        else
        {
            std::cerr << "Warning: 'set(dependencies ...)' block not found. Inserting dependencies set." << std::endl;
            // Insert the dependencies set after project()
            size_t project_end = content.find(")", pos);
            if (project_end != std::string::npos)
            {
                std::string dependencies_set = "\nset(dependencies\n  " + additional_package + "\n)\n";
                content.insert(project_end + 1, dependencies_set);
                std::cout << "Inserted dependencies set with '" << additional_package << "'." << std::endl;
            }
            else
            {
                // If project() is also missing, append at the end
                std::string dependencies_set = "\nset(dependencies\n  " + additional_package + "\n)\n";
                content += dependencies_set;
                std::cout << "Appended dependencies set with '" << additional_package << "' at the end." << std::endl;
            }
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

        std::cout << "Generated " << output_file << " with project name: " << project_name << std::endl;
    }
}