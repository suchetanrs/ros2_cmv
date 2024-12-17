#include "ros2_cmv/msgLoaderApp.hpp"

#include <iostream>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_resource.hpp"

// Qt includes
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QFileDialog>
#include <QMessageBox>

#include <ros2_cmv/header_generator.hpp>

// Constructor
MsgLoaderApp::MsgLoaderApp(QWidget *parent) : QWidget(parent) {
    // Main vertical layout
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // --- Section 1: Load .msg File ---
    // loadButton = new QPushButton("Load .msg File", this);
    contentDisplay = new QTextEdit(this);
    contentDisplay->setReadOnly(true);

    // mainLayout->addWidget(loadButton);
    mainLayout->addWidget(contentDisplay);

    // connect(loadButton, &QPushButton::clicked, this, &MsgLoaderApp::loadMsgFile);

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

    // --- Section 4: Process Button ---
    processButton = new QPushButton("Process Inputs", this);
    mainLayout->addWidget(processButton);

    connect(processButton, &QPushButton::clicked, this, &MsgLoaderApp::processInput);
}

// Function to populate the dropdown with interface names
void MsgLoaderApp::populateInterfaceDropdown()
{
    std::vector<Interface> interfaces = listInterfaces();

    // Clear existing items
    interfaceComboBox->clear();

    auto displayDefault = QString::fromStdString("Click to select a ROS 2 custom message !!");
    interfaceComboBox->addItem(displayDefault);

    for (const auto & iface : interfaces) {
        QString displayName = QString::fromStdString(iface.package) + "/" + QString::fromStdString(iface.base_name);
        interfaceComboBox->addItem(displayName);
    }

    if (interfaces.empty()) {
        interfaceComboBox->addItem("No interfaces found");
    }

    connect(interfaceComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, &MsgLoaderApp::printSelectedInterface);
}

void MsgLoaderApp::printSelectedInterface() {
    QString selectedText = interfaceComboBox->currentText();
    std::string selectedTextString = selectedText.toStdString();
    auto slashPose = selectedTextString.find('/');
    std::string package_name = selectedTextString.substr(0, slashPose);
    std::string message_name = selectedTextString.substr(slashPose + 1);
    msgFilePath = getMsgPath(package_name, message_name);
    // std::cout << "Selected Interface: " << selectedText.toStdString() << std::endl;
    // std::cout << "Path: " << filePath << std::endl;
    loadMsgFile(msgFilePath);
}

// Slot to load and display a .msg file
void MsgLoaderApp::loadMsgFile(std::string& filePathStr) {
    auto filePath = QString::fromStdString(filePathStr);
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Error", QString("Unable to open the file: %1").arg(file.errorString()));
        return;
    }

    QTextStream in(&file);
    QString fileContent = in.readAll();
    contentDisplay->setText(fileContent);
}

// Slot to process input data
void MsgLoaderApp::processInput() {
    // Retrieve and store the input values
    // QString selectedInterface = interfaceComboBox->currentText();
    QString outputPath = customPluginPath->text();
    auto strPath = outputPath.toStdString();
    std::cout << "Selected output path: " << strPath << std::endl;

    generate_cpp_header(msgFilePath, "custom_gen_msg_templates.hpp", "ros2_cmv_example/msg/example.hpp", "ros2_cmv_example::msg::Example");
    
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

    // TODO: Replace the above with your desired processing logic
}

// Main function
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    MsgLoaderApp mainWindow;
    mainWindow.setWindowTitle("MSG File Loader");
    mainWindow.resize(800, 600);
    mainWindow.show();

    return app.exec();
}

#include "msgLoaderApp.moc"
