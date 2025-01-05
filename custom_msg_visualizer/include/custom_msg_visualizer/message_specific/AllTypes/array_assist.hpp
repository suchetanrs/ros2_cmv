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

#ifndef ARRAY_ASSIST_HPP_
#define ARRAY_ASSIST_HPP_

#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

#include "custom_msg_visualizer/exposed_displays.hpp"

namespace MESSAGE_NAME
{
    class ArrayMessageAssist
    {
    public:
        ArrayMessageAssist(rviz_common::DisplayContext *context,
                           std::vector<std::string> variableNamesArrays, std::vector<std::string> variableTypesArrays);
        ~ArrayMessageAssist();

        void initialize();
        void update(float wall_dt, float ros_dt);
        void reset();

        template <typename T>
        void processArray(const std::vector<T> &array, std::string key, std::string type)
        {
            if (displayArrayInstances_.count(key) == 0)
            {
                return;
            }
            auto &displays = displayArrayInstances_[key];

            if (array.size() > displays.size())
            {
                size_t needed = array.size() - displays.size();
                for (size_t i = 0; i < needed; ++i)
                {
                    // std::cout << "Creating new display for: " << type << " " << i << std::endl;
                    auto newInstance = custom_msg_visualizer::DisplayFactory::instance().createDisplay(type, context_);
                    newInstance->onInitialize();
                    newInstance->onEnable();
                    displays.push_back(newInstance);
                }
            }

            for (size_t i = 0; i < array.size(); ++i)
            {
                auto ptr = std::make_shared<const T>(array[i]);
                displays[i]->processMessage(ptr);
            }
        }

    private:
        std::unordered_map<std::string, std::vector<std::shared_ptr<custom_msg_visualizer::IExposedDisplay>>> displayArrayInstances_;
        rviz_common::DisplayContext *context_;
        std::vector<std::string> variableNamesArrays_;
        std::vector<std::string> variableTypesArrays_;
    };

}
#endif