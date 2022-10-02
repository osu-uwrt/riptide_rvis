#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/srv/list_trees.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>

#include "ui_MissionPanel.h"

namespace riptide_rviz
{
    class MissionPanel : public rviz_common::Panel
    {
        Q_OBJECT public : MissionPanel(QWidget *parent = 0);
        ~MissionPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override; 
    protected Q_SLOTS:
        //QT slots (function callbacks)
        void refresh();

    protected:
        bool event(QEvent *event);

    private:
        Ui_MissionPanel *uiPanel;
        rclcpp::Node::SharedPtr clientNode;
        QTimer * spinTimer;
        rclcpp::Client<riptide_msgs2::srv::ListTrees>::SharedPtr refreshClient;
        // void discover_ns();
    };

} // namespace riptide_rviz