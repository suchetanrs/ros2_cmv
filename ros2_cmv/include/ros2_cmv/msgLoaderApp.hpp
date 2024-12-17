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

#include <ros2_cmv/ament_index_helpers.hpp>

class MsgLoaderApp : public QWidget {
    Q_OBJECT

public:
    explicit MsgLoaderApp(QWidget *parent = nullptr);

private slots:
    void loadMsgFile(std::string& filePathStr);
    void processInput();
    void printSelectedInterface();

private:
    // UI Elements
    // QPushButton *loadButton;
    QTextEdit *contentDisplay;
    QPushButton *processButton;

    QLineEdit *customPluginPath;
    // QLineEdit *input2;
    // QLineEdit *input3;
    // QLineEdit *input4;
    // QLineEdit *input5;

    QComboBox *interfaceComboBox;
    std::string msgFilePath;

    void populateInterfaceDropdown();
};

#endif // MSGLOADERAPP_HPP_