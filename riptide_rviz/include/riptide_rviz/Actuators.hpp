#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/action/arm_torpedo_dropper.hpp>
#include <riptide_msgs2/action/change_claw_state.hpp>
#include <riptide_msgs2/action/actuate_torpedos.hpp>
#include <riptide_msgs2/action/actuate_droppers.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_Actuators.h"
#include <QTimer>

namespace riptide_rviz
{
    using ArmTorpedoDropper = riptide_msgs2::action::ArmTorpedoDropper;
    using GHArmTorpedoDropper = rclcpp_action::ClientGoalHandle<ArmTorpedoDropper>;
    using ChangeClawState = riptide_msgs2::action::ChangeClawState;
    using GHChangeClawState = rclcpp_action::ClientGoalHandle<ChangeClawState>;
    using ActuateTorpedos = riptide_msgs2::action::ActuateTorpedos;
    using GHActuateTorpedos = rclcpp_action::ClientGoalHandle<ActuateTorpedos>;
    using ActuateDroppers = riptide_msgs2::action::ActuateDroppers;
    using GHActuateDropper = rclcpp_action::ClientGoalHandle<ActuateDroppers>;

    class Actuators : public rviz_common::Panel
    {
        Q_OBJECT public : Actuators(QWidget *parent = 0);
        ~Actuators();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

    protected Q_SLOTS:
        // QT slots (function callbacks)
        

    protected:
        bool event(QEvent *event);
        
        //For ArmTorpedoDropper
        void taskStartCbATD(const GHArmTorpedoDropper::SharedPtr & goalHandle);
        void cancelAcceptATD(const action_msgs::srv::CancelGoal::Response::SharedPtr );
        void taskCompleteCbATD(const GHArmTorpedoDropper::WrappedResult & result);
        void taskFeedbackCbATD(GHArmTorpedoDropper::SharedPtr goalHandle,
                            ArmTorpedoDropper::Feedback::ConstSharedPtr feedback);
        //For ChangeClawState
        void taskStartCbCCS(const GHChangeClawState::SharedPtr & goalHandle);
        void cancelAcceptCCS(const action_msgs::srv::CancelGoal::Response::SharedPtr );
        void taskCompleteCbCCS(const GHChangeClawState::WrappedResult & result);
        void taskFeedbackCbCCS(GHChangeClawState::SharedPtr goalHandle,
                            ChangeClawState::Feedback::ConstSharedPtr feedback);
        //For ActuateTorpedos
        void taskStartCbAT(const GHActuateTorpedos::SharedPtr & goalHandle);
        void cancelAcceptAT(const action_msgs::srv::CancelGoal::Response::SharedPtr );
        void taskCompleteCbAT(const GHActuateTorpedos::WrappedResult & result);
        void taskFeedbackCbAT(GHActuateTorpedos::SharedPtr goalHandle,
                            ActuateTorpedos::Feedback::ConstSharedPtr feedback);
        //For ActuateDropper
        void taskStartCbAD(const GHActuateDropper::SharedPtr & goalHandle);
        void cancelAcceptAD(const action_msgs::srv::CancelGoal::Response::SharedPtr );
        void taskCompleteCbAD(const GHActuateDropper::WrappedResult & result);
        void taskFeedbackCbAD(GHActuateDropper::SharedPtr goalHandle,
                            ActuateDroppers::Feedback::ConstSharedPtr feedback);

    private:
        // UI Panel instance
        Ui_Actuators *uiPanel;

        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;
    };

} // namespace riptide_rviz