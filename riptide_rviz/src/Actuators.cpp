#include "riptide_rviz/Actuators.hpp"
#include <chrono>


using namespace std::chrono_literals;

namespace riptide_rviz
{
    Actuators::Actuators(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_Actuators();
        uiPanel->setupUi(this);

        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_actuators", options);

        //strings below are placeholders
        armTorpedoDropper = rclcpp::create_client<ArmTorpedoDropper>(clientNode, "/tempest/autonomy/run_tree");
        changeClawState = rclcpp::create_client<ChangeClawState>(clientNode, "/tempest/autonomy/run_tree");
        actuateTorpedos = rclcpp::create_client<ActuateTorpedos>(clientNode, "/tempest/autonomy/run_tree");
        actuateDroppers = rclcpp::create_client<ActuateDroppers>(clientNode, "/tempest/autonomy/run_tree");
    }

    void Actuators::onInitialize()
    {
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->actDrop1, &QPushButton::clicked, [this](void)
                { handleDroppers(1); });
        connect(uiPanel->actDrop2, &QPushButton::clicked, [this](void)
                { handleDroppers(2); });
        connect(uiPanel->actTorp1, &QPushButton::clicked, [this](void)
                { handleTorpedos(1); });
        connect(uiPanel->actTorp2, &QPushButton::clicked, [this](void)
                { handleTorpedos(2); });
        connect(uiPanel->actDropArm, &QPushButton::clicked, [this](void)
                { handleArming(false); });
        connect(uiPanel->actTorpArm, &QPushButton::clicked, [this](void)
                { handleArming(true); });
        connect(uiPanel->actClawClose, &QPushButton::clicked, [this](void)
                { handleClaw(true); });
        connect(uiPanel->actClawOpen, &QPushButton::clicked, [this](void)
                { handleClaw(false); });
    }

    void Actuators::handleDroppers(int dropper_id)
    {
       
        if (!uiPanel->actDropArm->isEnabled())
        {
            auto goal = ActuateDroppers::Goal();
            goal.dropper_id = dropper_id;

            // create the goal callbacks to bind to
            auto sendGoalOptions = rclcpp_action::Client<ActuateDroppers>::SendGoalOptions();
            sendGoalOptions.goal_response_callback =
                std::bind(&Actuators::dropperTaskStartCb, this, _1);
            sendGoalOptions.feedback_callback =
                std::bind(&Actuators::dropperTaskFeedbackCb, this, _1, _2);
            sendGoalOptions.result_callback =
                std::bind(&Actuators::dropperTaskCompleteCb, this, _1);

            // send the goal with the callbacks configured
            actuateDroppers->async_send_goal(goal, sendGoalOptions);
            
        }
        
    }

    void Actuators::dropperTaskStartCbD(const GHActuateDroppers::SharedPtr &goalHandle)
    { 
            uiPanel->actDrop1->setEnabled(false);
            uiPanel->actDrop2->setEnabled(false);
            
    }

    void Actuators::dropperTaskCompleteCbDrops(const GHActuateDroppers::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->actDropArm->setEnabled(true);
    }

    void Actuators::dropperTaskFeedbackCb(GHActuateDroppers::SharedPtr goalHandle,
                                      ActuateDroppers::Feedback::ConstSharedPtr feedback)
    {
        //TODO figure out how to render the stack
    }

    void Actuators::handleTorpedos(int torpedo_id)
    {
       
        if (!uiPanel->actTorpArm->isEnabled())
        {
            auto goal = ActuateTorpedos::Goal();
            goal.torpedo_id = torpedo_id;

            // create the goal callbacks to bind to
            auto sendGoalOptions = rclcpp_action::Client<ActuateTorpedos>::SendGoalOptions();
            sendGoalOptions.goal_response_callback =
                std::bind(&Actuators::torpedoTaskStartCb, this, _1);
            sendGoalOptions.feedback_callback =
                std::bind(&Actuators::torpedoTaskFeedbackCb, this, _1, _2);
            sendGoalOptions.result_callback =
                std::bind(&Actuators::torpedoTaskCompleteCb, this, _1);

            // send the goal with the callbacks configured
            actuateTorpedos->async_send_goal(goal, sendGoalOptions);
            
        }
        
    }

    void Actuators::torpedoTaskStartCb(const GHActuateTorpedos::SharedPtr &goalHandle)
    { 
            uiPanel->actTorp1->setEnabled(false);
            uiPanel->actTorp2->setEnabled(false);
            
    }

    void Actuators::torpedoTaskCompleteCb(const GHActuateTorpedos::WrappedResult &result)
    {
        // we can re-enable the staret button and disable the stop button
        uiPanel->actTorpArm->setEnabled(true);
    }

    void Actuators::torpedoTaskFeedbackCb(GHActuateTorpedos::SharedPtr goalHandle,
                                      ActuateTorpedos::Feedback::ConstSharedPtr feedback)
    {
        //TODO figure out how to render the stack
    }


    void Actuators::handleTorpedoArming()
    {
        
        auto start = clientNode->get_clock()->now();
        while (!armTorpedoDropper->wait_for_action_server(100ms))
            if (clientNode->get_clock()->now() - start > 1s || !rclcpp::ok())
            {
                // we have timed out waiting, re-enable the button
                uiPanel->btStart->setEnabled(true);
                return;
            }
        auto goal = ArmTorpedoDropper::Goal();
    }

    void Actuators::handleDropperArming()
    {

    }

    void Actuators::handleArming(bool arm)
    {
        if (arm)
        {
            handleTorpedoArming()
        }else
        {
            handleDropperArming()
        }  
    }

    void Actuators::handleClaw(bool clawopen)
    {

    }

    void Actuators::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void Actuators::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    bool Actuators::event(QEvent *event)
    {
    }
    
    void Actuators::arm()
    
    Actuators::~Actuators()
    {
        // master window control removal
        delete uiPanel;

        // remove the timers
        delete spinTimer;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Actuators, rviz_common::Panel);