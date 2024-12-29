#ifndef PLUGIN_GENERATOR_APP_
#define PLUGIN_GENERATOR_APP_

#include <QWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QFormLayout>
#include <QLabel>
#include <QComboBox>
#include <QSettings>

#include "ros2_cmv/generators/generator.hpp"

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
        void processInput(); // for process inputs button

        // displays the interfaces information
        // populates msgFilePath.
        // populates validIdx.
        void inspectSelectedInterface();

    private:
        void populateInterfaceDropdown();
        void loadMsgFile(std::string &filePathStr);

        // ---------------------- UI Elements ----------------------
        QTextEdit *contentDisplay;
        QPushButton *processButton;
        QPushButton *saveButton;
        QLineEdit *customPluginPath;
        QComboBox *interfaceComboBox;
    };
};
#endif // PLUGIN_GENERATOR_APP_