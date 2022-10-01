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

#define ODOM_TIMEOUT 25s
#define MAX_IN_PLACE_DEPTH 0.5 // distance in M

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
        void refreshUI();

        // slots for sending commands to the vehicle
        void handleLocalDive();
        void handleCurrent();
        void handleCommand();

    protected:
        bool event(QEvent *event);

    private:
        // UI Panel instance
        Ui_ControlPanel *uiPanel;

        // mode for sending commands to the controller
        uint8_t ctrlMode;

        // last time we have recieved odom
        builtin_interfaces::msg::Time odomTime;

        // internal flags 
        bool vehicleEnabled = false;
        bool degreeReadout = true;

        // core stuff for creating and managing a ROS node
        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;

        // QT ui timer for handling data freshness
        QTimer * uiTimer;

        // publishers
        rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr ctrlCmdLinPub, ctrlCmdAngPub;
        rclcpp::Publisher<riptide_msgs2::msg::KillSwitchReport>::SharedPtr killStatePub;

        // ROS Timers
        rclcpp::TimerBase::SharedPtr killPubTimer;

        // ROS Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

    };

} // namespace riptide_rviz