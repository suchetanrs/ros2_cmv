/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#include "custom_msg_visualizer/plugin_generator_app.hpp"

namespace custom_msg_visualizer
{
    // Constructor
    PluginGeneratorApp::PluginGeneratorApp(QWidget *parent) : QWidget(parent)
    {
        // Main vertical layout
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        // ======================================= Section 0: Title and Legend =======================================
        // Create a horizontal layout for the legend and image
        QHBoxLayout *legendLayout = new QHBoxLayout();

        // Create a legend as a QLabel
        QLabel *legend = new QLabel(this);
        legend->setText(
            "<b>Legend:</b><br>"
            "<span style='color:green;'><b>------</b></span> Visualizable message fields<br>"
            "<span style='color:red;'><b>------</b></span> Non-visualizable message fields");
        legend->setAlignment(Qt::AlignLeft);

        // Add the legend to the horizontal layout
        legendLayout->addWidget(legend);

        // Create an image label
        QLabel *imageLabel = new QLabel(this);
        QPixmap image(QString::fromStdString(getPackagePrefix("custom_msg_visualizer") + "/share/custom_msg_visualizer/base_files/custom_msg_visualizer_logo.png"));
        imageLabel->setPixmap(image.scaled(150, 150, Qt::KeepAspectRatio));
        imageLabel->setAlignment(Qt::AlignRight);
        legendLayout->addWidget(imageLabel);
        mainLayout->addLayout(legendLayout);

        // ======================================= Section 1: Content Display =======================================
        contentDisplay = new QTextEdit(this);
        contentDisplay->setReadOnly(true);
        mainLayout->addWidget(contentDisplay);

        // ======================================= Section 2: Interface Dropdown =======================================
        QHBoxLayout *dropdownLayout = new QHBoxLayout();

        QLabel *dropdownLabel = new QLabel("Select Interface:", this);
        interfaceComboBox = new QComboBox(this);

        dropdownLayout->addWidget(dropdownLabel);
        dropdownLayout->addWidget(interfaceComboBox);

        mainLayout->addLayout(dropdownLayout);

        // Populate the dropdown with interfaces
        populateInterfaceDropdown();

        // ======================================= Section 3: Input fields =======================================
        QFormLayout *formLayout = new QFormLayout();

        // Initialize input fields with labels
        customPluginPath = new QLineEdit(this);

        formLayout->addRow(new QLabel("Plugin output path:"), customPluginPath);

        mainLayout->addLayout(formLayout);

        // ======================================= Section 4: Buttons =======================================
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

        // ======================================= Load previous settings =======================================
        QSettings settings("custom_msg_visualizer", "PluginGeneratorApp");

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
        std::cout << "Closed cleanly." << std::endl;
    }

    void PluginGeneratorApp::saveSettings()
    {
        std::cout << "Saving settings" << std::endl;
        QSettings settings("custom_msg_visualizer", "PluginGeneratorApp");
        settings.setValue("customPluginPath", customPluginPath->text());
        settings.setValue("selectedInterface", interfaceComboBox->currentText());
    }

    void PluginGeneratorApp::populateInterfaceDropdown()
    {
        std::vector<Interface> interfaces = getMsgInterfaces();

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
                this, &PluginGeneratorApp::inspectSelectedInterface);
    }

    void PluginGeneratorApp::inspectSelectedInterface()
    {
        QString selectedText = interfaceComboBox->currentText();
        std::string selectedTextString = selectedText.toStdString();
        auto slashPose = selectedTextString.find('/');
        std::string package_name = selectedTextString.substr(0, slashPose);
        std::string message_name = selectedTextString.substr(slashPose + 1);
        auto msgFilePath = getMsgPath(package_name, message_name);
        loadMsgFile(msgFilePath);
    }

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

        std::vector<int> validIdx;
        bool messageValid = checkMessageValidity(filePathStr, validIdx);

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
        if (!messageValid)
        {
            QMessageBox::critical(this, "Error", "The message either has 0 valid lines or does not contain a header. Plugin generation will not work correctly.");
        }
    }

    // Slot to process input data
    void PluginGeneratorApp::processInput()
    {
        // 1. Get the package name.
        auto selectedText = interfaceComboBox->currentText().toStdString();

        // 2. Get the package name.
        std::string rvizPluginName = convertToRvizPluginName(selectedText);

        // 3. Get the core package path (outputCorePath)
        QString outputWsPath = customPluginPath->text();
        auto outputWsPathStr = outputWsPath.toStdString();
        if (outputWsPath.isEmpty())
        {
            QMessageBox::warning(this, "Warning", "Output path is empty. Please provide a valid path.");
            return;
        }
        if (outputWsPathStr.back() == '/')
        {
            outputWsPathStr.pop_back();
        }
        std::cout << "Selected output path: " << outputWsPathStr << std::endl;
        auto outputCorePath = outputWsPathStr + "/src/" + rvizPluginName;

        // 4. Get the message file path (msgFilePath)
        auto slashPose = selectedText.find('/');
        std::string package_name = selectedText.substr(0, slashPose);
        std::string message_name = selectedText.substr(slashPose + 1);
        auto msgFilePath = getMsgPath(package_name, message_name);

        // 5. Generate the files
        try
        {
            generateFiles(msgFilePath, outputCorePath, selectedText, rvizPluginName, true);
        }
        catch (const std::runtime_error &e)
        {
            QMessageBox::critical(this, "Error", QString("An error occurred: %1").arg(e.what()));
        }
        catch (const std::exception &e)
        {
            // Handle other standard exceptions
            QMessageBox::critical(this, "Error", QString("An error occurred: %1").arg(e.what()));
        }
    }
};

// Main function
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    auto mainWindow = new custom_msg_visualizer::PluginGeneratorApp();
    mainWindow->setWindowTitle("RViz Plugin Generator");
    mainWindow->resize(900, 750);
    mainWindow->show();

    return app.exec();
}

#include "plugin_generator_app.moc"