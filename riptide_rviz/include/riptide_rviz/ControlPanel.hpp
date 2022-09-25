#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <launch_msgs/srv/start_launch.hpp>
#include <launch_msgs/srv/list_launch.hpp>
#include <launch_msgs/srv/stop_launch.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_ControlPanel.h"

#define BRINGUP_PKG "riptide_bringup2"
#define BRINGUP_POLLING_RATE 1s

namespace riptide_rviz
{
    class ControlPanel : public rviz_common::Panel
    {
        Q_OBJECT public : ControlPanel(QWidget *parent = 0);
        ~ControlPanel();

        virtual void load(const rviz_common::Config &config);
        virtual void save(rviz_common::Config config) const;

    protected Q_SLOTS:
        // QT slots (function callbacks)
        void bringupListRefresh();
        void handleBringupHost(int selection);
        void startBringup();
        void checkBringupStatus();

    protected:
        bool event(QEvent *event);

    private:
        Ui_ControlPanel *uiPanel;

        // ros node for the panel. we have to spawn our own.
        rclcpp::Node::SharedPtr nodeHandle;

        rclcpp::Client<launch_msgs::srv::StartLaunch>::SharedPtr bringupStartClient;
        rclcpp::Client<launch_msgs::srv::ListLaunch>::SharedPtr bringupListClient;
        rclcpp::Client<launch_msgs::srv::StopLaunch>::SharedPtr bringupStopClient;
        rclcpp::TimerBase::SharedPtr bringupCheckTimer;
        int bringupID = -1;

        // void discover_ns();
    };

} // namespace riptide_rviz