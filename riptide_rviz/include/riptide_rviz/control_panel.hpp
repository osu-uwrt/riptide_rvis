#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include <QLabel>

namespace riptide_rviz
{
    class MissionPanel : public rviz_common::Panel
    {
        Q_OBJECT public : MissionPanel(QWidget *parent = 0);
        ~MissionPanel();

        virtual void load(const rviz_common::Config &config);
        virtual void save(rviz_common::Config config) const;

        // protected Q_SLOTS:
        // QT slots (function callbacks)
        // void trigger_service(bool msg, std::string service_name);

    protected:
        bool event(QEvent *event);

    private:
        QLabel *label;
        // void discover_ns();
    };

} // namespace riptide_rviz