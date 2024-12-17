// mainwindow.cpp
#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <filesystem>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Connect buttons for browsing file paths
    connect(ui->browseMsgFileButton, &QPushButton::clicked, [this]() {
        QString fileName = QFileDialog::getOpenFileName(this, "Select .msg File", "", "Message Files (*.msg);;All Files (*)");
        if (!fileName.isEmpty()) {
            ui->msgFileLineEdit->setText(fileName);
        }
    });

    connect(ui->browseOutputFileButton, &QPushButton::clicked, [this]() {
        QString fileName = QFileDialog::getSaveFileName(this, "Select Output Header File", "", "Header Files (*.hpp);;All Files (*)");
        if (!fileName.isEmpty()) {
            ui->outputFileLineEdit->setText(fileName);
        }
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_generateButton_clicked()
{
    // Retrieve input values
    QString msgFilePath = ui->msgFileLineEdit->text();
    QString outputFile = ui->outputFileLineEdit->text();
    QString customMessageType = ui->customMessageTypeLineEdit->text();
    QString customMessageHeader = ui->customMessageHeaderLineEdit->text();
    QString exposedDisplaysHeader = ui->exposedDisplaysHeaderLineEdit->text();

    // Validate inputs
    if (msgFilePath.isEmpty() || outputFile.isEmpty() || customMessageType.isEmpty() ||
        customMessageHeader.isEmpty() || exposedDisplaysHeader.isEmpty()) {
        QMessageBox::warning(this, "Input Error", "Please fill in all fields.");
        return;
    }

    // Check if msg file exists
    if (!std::filesystem::exists(msgFilePath.toStdString())) {
        QMessageBox::critical(this, "File Error", "The specified .msg file does not exist.");
        return;
    }

    // Parse the .msg file
    std::vector<Message> messages = parse_msg_file(msgFilePath.toStdString());
    if (messages.empty()) {
        QMessageBox::warning(this, "Parsing Error", "No valid messages found in the .msg file.");
        return;
    }

    // Generate the C++ header file
    generate_cpp_header(
        messages,
        outputFile.toStdString(),
        customMessageHeader.toStdString(),
        exposedDisplaysHeader.toStdString(),
        customMessageType.toStdString()
    );

    QMessageBox::information(this, "Success", "Header file generated successfully.");
}