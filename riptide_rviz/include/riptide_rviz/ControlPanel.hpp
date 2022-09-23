#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_ControlPanel.h"

namespace riptide_rviz
{
    class ControlPanel : public rviz_common::Panel
    {
        Q_OBJECT public : ControlPanel(QWidget *parent = 0);
        ~ControlPanel();

        virtual void load(const rviz_common::Config &config);
        virtual void save(rviz_common::Config config) const;

        // protected Q_SLOTS:
        // QT slots (function callbacks)
        // void trigger_service(bool msg, std::string service_name);

    protected:
        bool event(QEvent *event);

    private:
        Ui_ControlPanel *uiPanel;
        // void discover_ns();
    };

} // namespace riptide_rviz