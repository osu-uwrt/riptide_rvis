#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/srv/list_trees.hpp>
#include <riptide_msgs2/action/execute_tree.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <QTimer>

#include "ui_MissionPanel.h"

namespace riptide_rviz
{
    using ExecuteTree = riptide_msgs2::action::ExecuteTree;
    using GHExecuteTree = rclcpp_action::ClientGoalHandle<ExecuteTree>;

    class MissionPanel : public rviz_common::Panel
    {
        Q_OBJECT public : MissionPanel(QWidget *parent = 0);
        ~MissionPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;
        void onInitialize() override;
    protected Q_SLOTS:
        // QT slots (function callbacks)
        void refresh();
        void handleSelectTree(int);

        void startTask();
        void taskStartCb(const GHExecuteTree::SharedPtr & goalHandle);
        void cancelTask();
        void cancelAccept(const action_msgs::srv::CancelGoal::Response::SharedPtr );
        void taskCompleteCb(const GHExecuteTree::WrappedResult & result);
        void taskFeedbackCb(GHExecuteTree::SharedPtr goalHandle,
                            ExecuteTree::Feedback::ConstSharedPtr feedback);

    protected:
        bool event(QEvent *event);

    private:
        Ui_MissionPanel *uiPanel;
        rclcpp::Node::SharedPtr clientNode;
        QTimer *spinTimer;

        std::vector<std::string> treeList;

        rclcpp::Client<riptide_msgs2::srv::ListTrees>::SharedPtr refreshClient;
        rclcpp_action::Client<ExecuteTree>::SharedPtr actionServer;
        // void discover_ns();
    };

} // namespace riptide_rviz