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

#include "custom_msg_visualizer/generators/generator.hpp"

namespace custom_msg_visualizer
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