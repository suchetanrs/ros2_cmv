#ifndef MSGLOADERAPP_HPP_
#define MSGLOADERAPP_HPP_

#include <QWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTextEdit>
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QFormLayout>
#include <QLabel>
#include <QComboBox>
#include <QHBoxLayout>
#include <QSettings>

#include <iostream>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "ament_index_cpp/get_resource.hpp"

#include <ros2_cmv/generators/header_generator.hpp>
#include <ros2_cmv/generators/xml_generator.hpp>
#include <ros2_cmv/generators/cmakelists_generator.hpp>

#include <ros2_cmv/ament_index_helpers.hpp>

namespace ros2_cmv
{
    class PluginGeneratorApp : public QWidget
    {
        Q_OBJECT

    public:
        explicit PluginGeneratorApp(QWidget *parent = nullptr);
        ~PluginGeneratorApp();
        void saveSettings();

    private slots:
        void loadMsgFile(std::string &filePathStr);
        void processInput();
        void printSelectedInterface();

    private:
        // UI Elements
        QTextEdit *contentDisplay;
        QPushButton *processButton;
        QPushButton *saveButton;

        QLineEdit *customPluginPath;
        // QLineEdit *input2;
        // QLineEdit *input3;
        // QLineEdit *input4;
        // QLineEdit *input5;

        QComboBox *interfaceComboBox;
        std::string msgFilePath;
        std::vector<int> validIdx;

        void populateInterfaceDropdown();
    };
};
#endif // MSGLOADERAPP_HPP_