#include "ros2_cmv/pluginGeneratorApp.hpp"

namespace ros2_cmv
{
    // Constructor
    PluginGeneratorApp::PluginGeneratorApp(QWidget *parent) : QWidget(parent)
    {
        // Main vertical layout
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        // --- Section 0: Title and Legend ---
        // Create a legend as a QLabel
        QLabel *legend = new QLabel(this);
        legend->setText(
            "<b>Legend:</b><br>"
            "<span style='color:green;'><b>------</b></span> Visualizable message fields<br>"
            "<span style='color:red;'><b>------</b></span> Non-visualizable message fields");
        legend->setAlignment(Qt::AlignLeft);

        // Add the legend to the layout
        mainLayout->addWidget(legend);

        // --- Section 1: Content Display ---
        contentDisplay = new QTextEdit(this);
        contentDisplay->setReadOnly(true);
        mainLayout->addWidget(contentDisplay);

        // --- Section 2: Interface Dropdown ---
        QHBoxLayout *dropdownLayout = new QHBoxLayout();

        QLabel *dropdownLabel = new QLabel("Select Interface:", this);
        interfaceComboBox = new QComboBox(this);

        dropdownLayout->addWidget(dropdownLabel);
        dropdownLayout->addWidget(interfaceComboBox);

        mainLayout->addLayout(dropdownLayout);

        // Populate the dropdown with interfaces
        populateInterfaceDropdown();

        // --- Section 3: Input Fields ---
        QFormLayout *formLayout = new QFormLayout();

        // Initialize input fields with labels
        customPluginPath = new QLineEdit(this);
        // input2 = new QLineEdit(this);
        // input3 = new QLineEdit(this);
        // input4 = new QLineEdit(this);
        // input5 = new QLineEdit(this);

        formLayout->addRow(new QLabel("Plugin output path:"), customPluginPath);
        // formLayout->addRow(new QLabel("Input 2:"), input2);
        // formLayout->addRow(new QLabel("Input 3:"), input3);
        // formLayout->addRow(new QLabel("Input 4:"), input4);
        // formLayout->addRow(new QLabel("Input 5:"), input5);

        mainLayout->addLayout(formLayout);

        // --- Section 4: Buttons ---
        QHBoxLayout *buttonLayout = new QHBoxLayout();

        // Process button (75% width)
        processButton = new QPushButton("Process Inputs", this);
        buttonLayout->addWidget(processButton, 3); // Stretch factor 3

        // Save button (25% width)
        saveButton = new QPushButton("Save Settings", this);
        buttonLayout->addWidget(saveButton, 1); // Stretch factor 1

        // Add the horizontal button layout to the main layout
        mainLayout->addLayout(buttonLayout);

        // Connect button signals to their respective slots
        connect(processButton, &QPushButton::clicked, this, &PluginGeneratorApp::processInput);
        connect(saveButton, &QPushButton::clicked, this, &PluginGeneratorApp::saveSettings);

        // Connect to QApplication's aboutToQuit signal
        connect(QApplication::instance(), &QApplication::aboutToQuit, this, &PluginGeneratorApp::saveSettings);
        QSettings settings("ROS2_CMV", "PluginGeneratorApp");

        // Restore plugin path
        QString savedPath = settings.value("customPluginPath", "").toString();
        customPluginPath->setText(savedPath);

        // Restore selected interface
        QString savedInterface = settings.value("selectedInterface", "").toString();
        int index = interfaceComboBox->findText(savedInterface);
        if (index != -1)
        {
            interfaceComboBox->setCurrentIndex(index);
        }
        else
        {
            interfaceComboBox->setCurrentIndex(0); // Default to the first item
        }
    }

    PluginGeneratorApp::~PluginGeneratorApp()
    {
        RCLCPP_INFO_STREAM(logger, "Closed cleanly.");
    }

    void PluginGeneratorApp::saveSettings()
    {
        RCLCPP_INFO_STREAM(logger, "Saving settings");
        QSettings settings("ROS2_CMV", "PluginGeneratorApp");
        settings.setValue("customPluginPath", customPluginPath->text());
        settings.setValue("selectedInterface", interfaceComboBox->currentText());
    }

    // Function to populate the dropdown with interface names
    void PluginGeneratorApp::populateInterfaceDropdown()
    {
        std::vector<Interface> interfaces = listInterfaces();

        // Clear existing items
        interfaceComboBox->clear();

        auto displayDefault = QString::fromStdString("Click to select a ROS 2 custom message !!");
        interfaceComboBox->addItem(displayDefault);

        for (const auto &iface : interfaces)
        {
            QString displayName = QString::fromStdString(iface.package) + "/" + QString::fromStdString(iface.base_name);
            interfaceComboBox->addItem(displayName);
        }

        if (interfaces.empty())
        {
            interfaceComboBox->addItem("No interfaces found");
        }

        connect(interfaceComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &PluginGeneratorApp::printSelectedInterface);
    }

    void PluginGeneratorApp::printSelectedInterface()
    {
        QString selectedText = interfaceComboBox->currentText();
        std::string selectedTextString = selectedText.toStdString();
        auto slashPose = selectedTextString.find('/');
        std::string package_name = selectedTextString.substr(0, slashPose);
        std::string message_name = selectedTextString.substr(slashPose + 1);
        msgFilePath = getMsgPath(package_name, message_name);
        // RCLCPP_INFO_STREAM(logger, "Selected Interface: " << selectedText.toStdString());
        // RCLCPP_INFO_STREAM(logger, "Path: " << filePath);
        loadMsgFile(msgFilePath);
    }

    // Slot to load and display a .msg file
    void PluginGeneratorApp::loadMsgFile(std::string &filePathStr)
    {
        auto filePath = QString::fromStdString(filePathStr);
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QMessageBox::critical(this, "Error", QString("Unable to open the file: %1").arg(file.errorString()));
            return;
        }

        QTextStream in(&file);
        QString fileContent = in.readAll();
        // contentDisplay->setText(fileContent);

        // Split the file content into lines
        QStringList lines = fileContent.split('\n');

        validIdx.clear();
        if (!check_message_validity(filePathStr, validIdx))
        {
            QMessageBox::critical(this, "Error", QString("Message has either 0 valid lines or does not have header."));
        }

        QMap<int, bool> lineColors;
        for (auto &idx : validIdx)
        {
            lineColors[idx] = true; // Example: line 0 -> light green
        }

        // Clear and prepare the text edit for colored lines
        contentDisplay->clear();

        // Use a QTextCursor to insert lines with formatting
        QTextCursor cursor = contentDisplay->textCursor();
        for (int i = 0; i < lines.size(); ++i)
        {
            QTextCharFormat format;
            if (lineColors.contains(i))
            {
                format.setForeground(QColor("green"));
            }
            else
            {
                format.setForeground(QColor("red"));
            }
            cursor.insertText(lines[i], format);
            cursor.insertBlock();
        }
    }

    // Slot to process input data
    void PluginGeneratorApp::processInput()
    {
        auto selectedText = interfaceComboBox->currentText().toStdString();
        std::string packageName = convertToRvizPluginName(selectedText);

        QString outputPath = customPluginPath->text();
        auto outputStrPath = outputPath.toStdString();
        if (outputPath.isEmpty())
        {
            QMessageBox::warning(this, "Warning", "Output path is empty. Please provide a valid path.");
            return;
        }
        if (outputStrPath.back() == '/')
        {
            outputStrPath.pop_back();
        }
        RCLCPP_INFO_STREAM(logger, "Selected output path: " << outputStrPath);
        auto outputCorePath = outputStrPath + "/src/" + packageName;

        std::vector<std::string> directories = {
            outputCorePath + "/src/",
            outputCorePath + "/include/" + packageName + "/"};

        // Iterate through each directory and create it if it doesn't exist
        for (const auto &directory : directories)
        {
            try
            {
                if (std::filesystem::create_directories(directory))
                {
                    RCLCPP_INFO_STREAM(logger, "Directory created: " << directory);
                }
                else
                {
                    RCLCPP_INFO_STREAM(logger, "Directory already exists: " << directory);
                }
            }
            catch (const std::filesystem::filesystem_error &e)
            {
                QMessageBox::critical(this, "Error", QString("Failed to create directory: %1").arg(directory.c_str()));
            }
        }

        RCLCPP_INFO_STREAM(logger, "===================================================================");
        copyFile(getPackagePrefix("ros2_cmv") + "/include/ros2_cmv/custom_msg_display.hpp", outputCorePath + "/include/" + packageName + "/custom_msg_display.hpp");
        RCLCPP_INFO_STREAM(logger, "===================================================================");
        generate_cpp_header(msgFilePath, outputCorePath + "/include/" + packageName + "/custom_msg_metadata.hpp", convertToIncludePath(selectedText), convertToNamespace(selectedText), validIdx);
        RCLCPP_INFO_STREAM(logger, "===================================================================");
        copyFile(getPackagePrefix("ros2_cmv") + "/share/ros2_cmv/base_files/custom_msg_display.cpp", outputCorePath + "/src/custom_msg_display.cpp");
        RCLCPP_INFO_STREAM(logger, "===================================================================");
        generatePluginXML(packageName, packageName, outputCorePath + "/plugin.xml", packageName);
        RCLCPP_INFO_STREAM(logger, "===================================================================");
        generateCMakeLists(packageName, getPackagePrefix("ros2_cmv") + "/share/ros2_cmv/base_files/base_cmakelists.txt", outputCorePath + "/CMakeLists.txt", convertToPackageName(selectedText));
        RCLCPP_INFO_STREAM(logger, "===================================================================");
        generatePackageXML(packageName, getPackagePrefix("ros2_cmv") + "/share/ros2_cmv/base_files/base_package.xml", outputCorePath + "/package.xml", convertToPackageName(selectedText));

        // QString value2 = input2->text();
        // QString value3 = input3->text();
        // QString value4 = input4->text();
        // QString value5 = input5->text();

        // // Example processing: Display the input values in a message box
        // QString message = QString("Selected Interface: %1\nInput 1: %2\nInput 2: %3\nInput 3: %4\nInput 4: %5\nInput 5: %6")
        //                   .arg(selectedInterface)
        //                   .arg(value1)
        //                   .arg(value2)
        //                   .arg(value3)
        //                   .arg(value4)
        //                   .arg(value5);

        // QMessageBox::information(this, "Input Values", message);
    }
};

// Main function
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    auto mainWindow = new ros2_cmv::PluginGeneratorApp();
    mainWindow->setWindowTitle("RViz Plugin Generator");
    mainWindow->resize(900, 700);
    mainWindow->show();

    return app.exec();
}

#include "pluginGeneratorApp.moc"