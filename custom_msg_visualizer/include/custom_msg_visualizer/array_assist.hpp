#ifndef ARRAY_ASSIST_HPP_
#define ARRAY_ASSIST_HPP_

#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

#include "custom_msg_visualizer/exposed_displays.hpp"
#include "custom_msg_visualizer/cmv_macros.hpp"
#include INCLUDE_PROJECT_HEADER(custom_msg_metadata.hpp)

namespace custom_msg_visualizer
{
    class ArrayMessageAssist
    {
    public:
        ArrayMessageAssist(rviz_common::DisplayContext *context);
        ~ArrayMessageAssist();

        void initialize();
        void update(float wall_dt, float ros_dt);
        void reset();

        template <typename T>
        void processArray(const std::vector<T> &array, std::string key, std::string type)
        {
            if (displayArrayInstances_.count(key) == 0)
            {
                throw std::runtime_error("Array not initialized:" + key);
            }
            auto &displays = displayArrayInstances_[key];

            if (array.size() > displays.size())
            {
                size_t needed = array.size() - displays.size();
                for (size_t i = 0; i < needed; ++i)
                {
                    std::cout << "Creating new display for: " << type << " " << i << std::endl;
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
        std::unordered_map<std::string, std::vector<std::shared_ptr<IExposedDisplay>>> displayArrayInstances_;
        rviz_common::DisplayContext *context_;
    };

}
#endif