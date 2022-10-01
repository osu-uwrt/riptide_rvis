#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_ControlPanel.h"
#include <QTimer>



namespace riptide_rviz
{

    class ControlPanel : public rviz_common::Panel
    {
        Q_OBJECT public : ControlPanel(QWidget *parent = 0);
        ~ControlPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

        // ROS Subscriber callbacks
        void odomCallback(const nav_msgs::msg::Odometry & msg);

        // ROS timer callbacks
        void sendKillMsgTimer();


    protected Q_SLOTS:
        // QT slots (function callbacks)
        // slots for handling mode setting of the controller
        void handleEnable();
        void handleDisable(); // pressing disable asserts kill and clears command
        void switchMode(uint8_t mode);

        // slots for controlling the UI
        void toggleDegrees();

        // slots for sending commands to the vehicle
        void handleLocalDive();
        void handleCurrent();
        void handleCommand();

    protected:
        bool event(QEvent *event);

    private:
        // UI Panel instance
        Ui_ControlPanel *uiPanel;

        // message for sending commands to the controller
        riptide_msgs2::msg::ControllerCommand::SharedPtr controllerCommand;

        // internal flags 
        bool vehicleEnabled = false;
        bool degreeReadout = true;

        // core stuff for creating and managing a ROS node
        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;

        // publishers
        rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr ctrlCmdPub;
        rclcpp::Publisher<riptide_msgs2::msg::KillSwitchReport>::SharedPtr killStatePub;

        // ROS Timers
        rclcpp::TimerBase::SharedPtr killPubTimer;

        // ROS Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    };

} // namespace riptide_rviz