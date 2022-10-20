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
        
        armTorpedos = rclcpp::create_client<ArmTorpedoDropper>(clientNode, "/tempest/autonomy/run_tree");
        changeClawState = rclcpp::create_client<ChangeClawState>(clientNode, "/tempest/autonomy/run_tree");
        actuateTorpedos = rclcpp::create_client<ActuateTorpedos>(clientNode, "/tempest/autonomy/run_tree");
        actuateDropper = rclcpp::create_client<ActuateDropper>(clientNode, "/tempest/autonomy/run_tree");
    }

    void Actuators::onInitialize()
    {
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // Connect UI signals for controlling the riptide vehicle
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