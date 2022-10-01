#include "riptide_rviz/ControlPanel.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

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
        controllerCommand = std::make_shared<riptide_msgs2::msg::ControllerCommand>();
        controllerCommand->mode = riptide_msgs2::msg::ControllerCommand::DISABLED;
    }

    void ControlPanel::onInitialize()
    {
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
            { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // make publishers 
        killStatePub = clientNode->create_publisher<riptide_msgs2::msg::KillSwitchReport>
            (ROBOT_NS + std::string("/control/software_kill"), rclcpp::SystemDefaultsQoS());

        // make ROS timers
        killPubTimer = clientNode->create_wall_timer(50ms, std::bind(&ControlPanel::sendKillMsgTimer, this));

        // make ROS Subscribers
        odomSub = clientNode->create_subscription<nav_msgs::msg::Odometry>(
            ROBOT_NS + std::string("/odom"), rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::odomCallback, this, _1)
        );


        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->ctrlEnable, &QPushButton::clicked, [this](void){ handleEnable(); });
        connect(uiPanel->ctrlDisable, &QPushButton::clicked, [this](void){ handleDisable(); });
        connect(uiPanel->ctrlDegOrRad, &QPushButton::clicked, [this](void){ toggleDegrees(); });

        // mode seting buttons
        connect(uiPanel->ctrlModePos, &QPushButton::clicked, 
            [this](void){ switchMode(riptide_msgs2::msg::ControllerCommand::POSITION); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked, 
            [this](void){ switchMode(riptide_msgs2::msg::ControllerCommand::VELOCITY); });
        connect(uiPanel->ctrlModeFFD, &QPushButton::clicked, 
            [this](void){ switchMode(riptide_msgs2::msg::ControllerCommand::FEEDFORWARD); });
        // connect(uiPanel->ctrlModeVel, &QPushButton::clicked, 
        //     [this](void){ switchMode(riptide_msgs2::msg::ControllerCommand::VELOCITY); });

        // command sending buttons
        connect(uiPanel->ctrlDiveInPlace, &QPushButton::clicked, [this](void){ handleLocalDive(); });
        connect(uiPanel->ctrlFwdCurrent, &QPushButton::clicked, [this](void){ handleCurrent(); });
        connect(uiPanel->CtrlSendCmd, &QPushButton::clicked, [this](void){ handleCommand(); });
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
        delete spinTimer;

        rclcpp::shutdown();
    }

    // slots for handling mode setting of the controller
    void ControlPanel::handleEnable(){

    }

    void ControlPanel::handleDisable(){

    }

    void ControlPanel::switchMode(uint8_t mode){

    }

    // slots for sending commands to the vehicle
    void ControlPanel::handleLocalDive(){

    }

    void ControlPanel::toggleDegrees(){
        degreeReadout = ! degreeReadout;
        if(degreeReadout){
            uiPanel->ctrlDegOrRad->setText("Radians");
        } else {
            uiPanel->ctrlDegOrRad->setText("Degrees");
        }
    }

    void ControlPanel::handleCurrent(){
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

    void ControlPanel::handleCommand(){

    }

    void ControlPanel::odomCallback(const nav_msgs::msg::Odometry & msg){
        // parse the quaternion to RPY
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // convert to degrees if its what we're showing
        if(degreeReadout){
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
    
    // ROS timer callbacks
    void ControlPanel::sendKillMsgTimer(){
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