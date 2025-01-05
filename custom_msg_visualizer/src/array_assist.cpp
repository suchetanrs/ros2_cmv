#include "custom_msg_visualizer/array_assist.hpp"

namespace custom_msg_visualizer
{

    ArrayMessageAssist::ArrayMessageAssist(rviz_common::DisplayContext *context)
    {
        context_ = context;
    }

    ArrayMessageAssist::~ArrayMessageAssist() {}

    void ArrayMessageAssist::initialize()
    {
        for (size_t i = 0; i < variableNamesArrays.size(); ++i)
        {
            displayArrayInstances_[variableNamesArrays[i]] = std::vector<std::shared_ptr<IExposedDisplay>>();
        }
    }

    void ArrayMessageAssist::update(float wall_dt, float ros_dt)
    {
        for (const auto &pair : displayArrayInstances_)
        {
            for (const auto &instance : pair.second)
            {
                instance->update(wall_dt, ros_dt);
            }
        }
    }

    void ArrayMessageAssist::reset()
    {
        for (const auto &pair : displayArrayInstances_)
        {
            {
                for (const auto &instance : pair.second)
                {
                    instance->reset();
                }
            }
        }
    }

    // void ArrayMessageAssist::processPoseArray(const std::vector<geometry_msgs::msg::Pose> &poses)
    // {
    //     {

    //         rviz_common::ViewController *view_controller = context_->getViewManager()->getCurrent();
    //         if (!view_controller)
    //         {
    //             return;
    //         }
    //         Ogre::Camera *camera = view_controller->getCamera();
    //         if (!camera)
    //         {
    //             return;
    //         }

    //         // Construct view-proj matrix once per frame
    //         Ogre::Matrix4 view_mat = camera->getViewMatrix();
    //         Ogre::Matrix4 proj_mat = camera->getProjectionMatrix();
    //         Ogre::Matrix4 viewproj_mat = proj_mat * view_mat;
    //         // std::cout << "ViewProj: " << viewproj_mat << std::endl;

    //         for (const auto &pose : poses)
    //         {
    //             Ogre::Vector4 clip_pos = viewproj_mat * Ogre::Vector4(
    //                                                         pose.position.x,
    //                                                         pose.position.y,
    //                                                         pose.position.z,
    //                                                         1.0f);
    //             std::cout << "Pose: x: " << pose.position.x << " y: " << pose.position.y << " z: " << pose.position.z << std::endl;
    //             std::cout << "ClipPos: " << clip_pos << std::endl;
    //         }
    //     }
    // }

} // namespace custom_msg_visualizer
