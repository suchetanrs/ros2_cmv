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

#ifndef EXPOSED_DISPLAY_CORE_HPP_
#define EXPOSED_DISPLAY_CORE_HPP_

#include <any>
#include <rviz_common/display_context.hpp>

namespace ros2_cmv
{
    template <typename MessageType>
    const MessageType &castMessage(std::any &param)
    {
        if (!param.has_value())
        {
            throw std::runtime_error("No parameter provided for castMessage");
        }
        // Try to extract MessageType
        try
        {
            const MessageType &actualParam = std::any_cast<const MessageType &>(param);
            return actualParam;
        }
        catch (const std::bad_any_cast &)
        {
            throw std::runtime_error("Invalid parameter type for this class: " + std::string(typeid(MessageType).name()));
        }
    }

    // polymorphism: Use this as common interface for all displays
    class IExposedDisplay
    {
    public:
        virtual ~IExposedDisplay() = default;
        virtual void onInitialize() = 0;
        virtual void onEnable() = 0;
        virtual void processMessage(std::any param) = 0;
        virtual void reset() = 0;
        virtual void update(float wall_dt, float ros_dt) = 0;
    };

    template <typename BaseDisplay, typename MessageType>
    class ExposedDisplay : public BaseDisplay, public IExposedDisplay
    {
    public:
        ExposedDisplay(rviz_common::DisplayContext *context) : BaseDisplay()
        {
            this->context_ = context;
            this->scene_manager_ = this->context_->getSceneManager();
            this->scene_node_ = this->scene_manager_->getRootSceneNode()->createChildSceneNode();
            this->fixed_frame_ = this->context_->getFixedFrame();
        }

        virtual ~ExposedDisplay() = default;

        void onInitialize() override { BaseDisplay::onInitialize(); }

        void onEnable() override { BaseDisplay::onEnable(); }

        void reset() override { BaseDisplay::reset(); }

        void update(float wall_dt, float ros_dt) override
        {
            BaseDisplay::update(wall_dt, ros_dt);
        }

        void processMessage(std::any param) override
        {
            auto actualParam = castMessage<MessageType>(param);
            BaseDisplay::processMessage(actualParam);
        }
    };

    // Base Factory Class
    class DisplayFactory
    {
    public:
        using Creator = std::function<std::shared_ptr<IExposedDisplay>(rviz_common::DisplayContext *)>;

        static DisplayFactory &instance()
        {
            static DisplayFactory inst;
            return inst;
        }

        // Register a class with its string name
        bool registerDisplay(const std::string &name, Creator creator)
        {
            return creators.emplace(name, creator).second;
        }

        // Create an instance of the requested class
        std::shared_ptr<IExposedDisplay> createDisplay(
            const std::string &name, rviz_common::DisplayContext *context)
        {
            auto it = creators.find(name);
            if (it != creators.end())
            {
                return it->second(context);
            }
            else
            {
                throw std::runtime_error("Unknown display type encountered while creating display: " + name +
                                         " !! Check exposed_displays.hpp for supported displays for custom plugin generation.");
            }
        }

        std::unordered_map<std::string, Creator> getCreators()
        {
            return creators;
        }

    private:
        std::unordered_map<std::string, Creator> creators;
    };

// Macro for registering classes
#define REGISTER_DISPLAY_CLASS(NAME, TYPE)                                                                                                                                                  \
    namespace                                                                                                                                                                               \
    {                                                                                                                                                                                       \
        struct StaticReg##TYPE                                                                                                                                                              \
        {                                                                                                                                                                                   \
            StaticReg##TYPE()                                                                                                                                                               \
            {                                                                                                                                                                               \
                DisplayFactory::instance().registerDisplay(NAME, [](rviz_common::DisplayContext *context) -> std::shared_ptr<IExposedDisplay> { return std::make_shared<TYPE>(context); }); \
            }                                                                                                                                                                               \
        } staticReg##TYPE;                                                                                                                                                                  \
    }

};
#endif // EXPOSED_DISPLAY_CORE_HPP_