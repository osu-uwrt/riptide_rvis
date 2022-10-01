#include "riptide_rviz/ControlPanel.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <algorithm>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define ROBOT_NS "/tempest"

namespace riptide_rviz
{
    ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_ControlPanel();
        uiPanel->setupUi(this);

        // create the RCLCPP client
        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_control", options);

        // create the default message
        ctrlMode = riptide_msgs2::msg::ControllerCommand::DISABLED;
    }

    void ControlPanel::onInitialize()
    {
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        uiTimer = new QTimer(this);
        connect(uiTimer, &QTimer::timeout, [this](void)
                { refreshUI(); });
        uiTimer->start(100);

        // make publishers
        killStatePub = clientNode->create_publisher<riptide_msgs2::msg::KillSwitchReport>(ROBOT_NS + std::string("/control/software_kill"), rclcpp::SystemDefaultsQoS());
        ctrlCmdLinPub = clientNode->create_publisher<riptide_msgs2::msg::ControllerCommand>(ROBOT_NS + std::string("/controller/linear"), rclcpp::SystemDefaultsQoS());
        ctrlCmdAngPub = clientNode->create_publisher<riptide_msgs2::msg::ControllerCommand>(ROBOT_NS + std::string("/controller/angular"), rclcpp::SystemDefaultsQoS());

        // make ROS timers
        killPubTimer = clientNode->create_wall_timer(50ms, std::bind(&ControlPanel::sendKillMsgTimer, this));

        // make ROS Subscribers
        odomSub = clientNode->create_subscription<nav_msgs::msg::Odometry>(
            ROBOT_NS + std::string("/odom"), rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::odomCallback, this, _1));
        steadySub = clientNode->create_subscription<std_msgs::msg::Bool>(
            ROBOT_NS + std::string("/controller/steady"), rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::steadyCallback, this, _1));

        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->ctrlEnable, &QPushButton::clicked, [this](void)
                { handleEnable(); });
        connect(uiPanel->ctrlDisable, &QPushButton::clicked, [this](void)
                { handleDisable(); });
        connect(uiPanel->ctrlDegOrRad, &QPushButton::clicked, [this](void)
                { toggleDegrees(); });

        // mode seting buttons
        connect(uiPanel->ctrlModePos, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::POSITION); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::VELOCITY); });
        connect(uiPanel->ctrlModeFFD, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::FEEDFORWARD); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(22); });

        // command sending buttons
        connect(uiPanel->ctrlDiveInPlace, &QPushButton::clicked, [this](void)
                { handleLocalDive(); });
        connect(uiPanel->ctrlFwdCurrent, &QPushButton::clicked, [this](void)
                { handleCurrent(); });
        connect(uiPanel->CtrlSendCmd, &QPushButton::clicked, [this](void)
                { handleCommand(); });
    }

    void ControlPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void ControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    bool ControlPanel::event(QEvent *event)
    {
    }

    ControlPanel::~ControlPanel()
    {
        // master window control removal
        delete uiPanel;

        // remove the timers
        delete spinTimer, uiTimer;

        rclcpp::shutdown();
    }

    // slots for handling mode setting of the controller
    void ControlPanel::handleEnable()
    {
        vehicleEnabled = true;
        uiPanel->ctrlEnable->setEnabled(false);
        uiPanel->ctrlDisable->setEnabled(true);
    }

    void ControlPanel::handleDisable()
    {
        vehicleEnabled = false;
        uiPanel->ctrlEnable->setEnabled(true);
        uiPanel->ctrlDisable->setEnabled(false);

        // clear the controller command mode
        ctrlMode = riptide_msgs2::msg::ControllerCommand::DISABLED;
    }

    void ControlPanel::switchMode(uint8_t mode)
    {
        ctrlMode = mode;
        switch (ctrlMode)
        {
        case riptide_msgs2::msg::ControllerCommand::POSITION:
            uiPanel->ctrlModeFFD->setEnabled(true);
            uiPanel->ctrlModeVel->setEnabled(true);
            uiPanel->ctrlModePos->setEnabled(false);
            uiPanel->ctrlModeTele->setEnabled(true);

            break;
        case riptide_msgs2::msg::ControllerCommand::VELOCITY:
            uiPanel->ctrlModeFFD->setEnabled(true);
            uiPanel->ctrlModeVel->setEnabled(false);
            uiPanel->ctrlModePos->setEnabled(true);
            uiPanel->ctrlModeTele->setEnabled(true);

            break;
        case riptide_msgs2::msg::ControllerCommand::FEEDFORWARD:
            uiPanel->ctrlModeFFD->setEnabled(false);
            uiPanel->ctrlModeVel->setEnabled(true);
            uiPanel->ctrlModePos->setEnabled(true);
            uiPanel->ctrlModeTele->setEnabled(true);
            break;
        default:
            std::cerr << "Button not yet operable" << std::endl;
            break;
        }
    }

    void ControlPanel::refreshUI()
    {
        // handle timing out the UI buttons if odom gets too stale
        auto now = clientNode->get_clock()->now();
        if (now - odomTime > ODOM_TIMEOUT || !vehicleEnabled)
        {
            // the odom has timed out
            if (uiPanel->CtrlSendCmd->isEnabled())
                std::cerr << "Odom timed out. disabling local control buttons" << std::endl;

            uiPanel->ctrlDiveInPlace->setEnabled(false);
            uiPanel->CtrlSendCmd->setEnabled(false);
        }
        else
        {
            uiPanel->CtrlSendCmd->setEnabled(true);

            // check the current depth. if we are below 0.5m, disable the submerge in place button
            bool convOk;
            double z = uiPanel->cmdCurrZ->text().toDouble(&convOk);
            if (convOk && z < -MAX_IN_PLACE_DEPTH)
            {
                uiPanel->ctrlDiveInPlace->setEnabled(false);
            }
            else
            {
                uiPanel->ctrlDiveInPlace->setEnabled(true);
            }
        }
    }

    // slots for sending commands to the vehicle
    void ControlPanel::handleLocalDive()
    {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double x, y, yaw;
        bool convOk[3];

        // make sure that the conversion goes okay as well
        x = uiPanel->cmdCurrX->text().toDouble(&convOk[0]);
        y = uiPanel->cmdCurrY->text().toDouble(&convOk[1]);
        yaw = uiPanel->cmdCurrX->text().toDouble(&convOk[2]);

        if (std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; }))
        {
            std::cerr << "Failed to convert current position to floating point" << std::endl;
            return;
        }

        // build the linear control message
        // auto override the control mode to position
        auto linear = geometry_msgs::msg::Vector3();
        linear.x = x;
        linear.y = y;
        linear.z = 0.75; // automatically go to 0.75m below surface
        auto linCmd = riptide_msgs2::msg::ControllerCommand();
        linCmd.setpoint_vect = linear;
        linCmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;

        // convert RPY to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);

        // build the angular message
        auto angular = tf2::toMsg(quat);
        auto angCmd = riptide_msgs2::msg::ControllerCommand();
        angCmd.setpoint_quat = angular;
        angCmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;

        // send the control messages
        ctrlCmdLinPub->publish(linCmd);
        ctrlCmdAngPub->publish(angCmd);
    }

    void ControlPanel::toggleDegrees()
    {
        degreeReadout = !degreeReadout;
        if (degreeReadout)
        {
            uiPanel->ctrlDegOrRad->setText("Degrees");
        }
        else
        {
            uiPanel->ctrlDegOrRad->setText("Radians");
        }
    }

    void ControlPanel::handleCurrent()
    {
        // take the values from the readouts
        QString x, y, z, roll, pitch, yaw;
        x = uiPanel->cmdCurrX->text();
        y = uiPanel->cmdCurrY->text();
        z = uiPanel->cmdCurrZ->text();
        roll = uiPanel->cmdCurrX->text();
        pitch = uiPanel->cmdCurrX->text();
        yaw = uiPanel->cmdCurrX->text();

        // take the values and propagate them into the requested values
        uiPanel->cmdReqX->setText(x);
        uiPanel->cmdReqY->setText(y);
        uiPanel->cmdReqZ->setText(z);
        uiPanel->cmdReqR->setText(roll);
        uiPanel->cmdReqP->setText(pitch);
        uiPanel->cmdReqYaw->setText(yaw);
    }

    void ControlPanel::handleCommand()
    {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double x, y, z, roll, pitch, yaw;
        bool convOk[6];

        // make sure that the conversion goes okay as well
        x = uiPanel->cmdReqX->text().toDouble(&convOk[0]);
        y = uiPanel->cmdReqY->text().toDouble(&convOk[1]);
        z = uiPanel->cmdReqZ->text().toDouble(&convOk[2]);
        roll = uiPanel->cmdReqR->text().toDouble(&convOk[3]);
        pitch = uiPanel->cmdReqP->text().toDouble(&convOk[4]);
        yaw = uiPanel->cmdReqYaw->text().toDouble(&convOk[5]);

        if (std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; }))
        {
            std::cerr << "Failed to convert requested position to floating point" << std::endl;
            // set the red stylesheet
            uiPanel->CtrlSendCmd->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->CtrlSendCmd->setStyleSheet(""); });
            return;
        }

        // now we can build the command and send it
        // build the linear control message
        auto linear = geometry_msgs::msg::Vector3();
        linear.x = x;
        linear.y = y;
        linear.z = z;
        auto linCmd = riptide_msgs2::msg::ControllerCommand();
        linCmd.setpoint_vect = linear;
        linCmd.mode = ctrlMode;

        // if we are in position, we use quat, otherwise use the vector
        auto angCmd = riptide_msgs2::msg::ControllerCommand();
        if (ctrlMode == riptide_msgs2::msg::ControllerCommand::POSITION)
        {
            // convert RPY to quaternion
            tf2::Quaternion quat;
            quat.setRPY(0, 0, yaw);

            // build the angular quat message
            auto angular = tf2::toMsg(quat);
            angCmd.setpoint_quat = angular;
        } else {
            // build the vector
            auto angular = geometry_msgs::msg::Vector3();
            linear.x = roll;
            linear.y = pitch;
            linear.z = yaw;
            angCmd.setpoint_vect = angular;
        }

        // add the current control mode
        angCmd.mode = ctrlMode;

        // send the control messages
        ctrlCmdLinPub->publish(linCmd);
        ctrlCmdAngPub->publish(angCmd);
    }

    void ControlPanel::odomCallback(const nav_msgs::msg::Odometry &msg)
    {
        // parse the quaternion to RPY
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // save the header timestamp
        odomTime = msg.header.stamp;

        // convert to degrees if its what we're showing
        if (degreeReadout)
        {
            roll *= 180.0 / M_PI;
            pitch *= 180.0 / M_PI;
            yaw *= 180.0 / M_PI;
        }

        // show it to the user
        uiPanel->cmdCurrR->setText(QString::number(roll, 'f', 2));
        uiPanel->cmdCurrP->setText(QString::number(pitch, 'f', 2));
        uiPanel->cmdCurrYaw->setText(QString::number(yaw, 'f', 2));

        uiPanel->cmdCurrX->setText(QString::number(msg.pose.pose.position.x, 'f', 2));
        uiPanel->cmdCurrY->setText(QString::number(msg.pose.pose.position.y, 'f', 2));
        uiPanel->cmdCurrZ->setText(QString::number(msg.pose.pose.position.z, 'f', 2));
    }

    void ControlPanel::steadyCallback(const std_msgs::msg::Bool &msg)
    {
        uiPanel->cmdSteady->setEnabled(msg.data);
    }

    // ROS timer callbacks
    void ControlPanel::sendKillMsgTimer()
    {
        auto killMsg = riptide_msgs2::msg::KillSwitchReport();
        killMsg.kill_switch_id = riptide_msgs2::msg::KillSwitchReport::KILL_SWITCH_RQT_CONTROLLER;
        killMsg.sender_id = "/riptide_rviz_control";
        killMsg.switch_asserting_kill = !vehicleEnabled;
        killMsg.switch_needs_update = uiPanel->ctrlRequireKill->isChecked();

        killStatePub->publish(killMsg);
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);