#include "riptide_rviz/MissionPanel.hpp"

namespace riptide_rviz
{
    MissionPanel::MissionPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_MissionPanel();
        uiPanel->setupUi(this);

        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_mission", options);

    }

    void MissionPanel::onInitialize()
    {
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // uiTimer = new QTimer(this);
        // connect(uiTimer, &QTimer::timeout, [this](void)
        //         { refreshUI(); });
        // uiTimer->start(100);

        // Connect UI signals for controlling the riptide vehicle
        // connect(uiPanel->btStart, &QPushButton::clicked, [this](void)
        //         { handleEnable(); });
        // connect(uiPanel->btStop, &QPushButton::clicked, [this](void)
        //         { handleDisable(); });
        connect(uiPanel->btRefresh, &QPushButton::clicked, [this](void)
                { refresh(); });
    }

    void MissionPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load( config );
    }

    void MissionPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save( config );
    }

    bool MissionPanel::event(QEvent *event)
    {
    }

    MissionPanel::~MissionPanel(){
        // master window control removal
        delete uiPanel;
        delete spinTimer;
    }

    void MissionPanel::refresh()
    {
        refreshClient = clientNode->create_client<riptide_msgs::srv::ListTrees>("/tempest/autonomy/list_trees");
        
        riptide_msgs::srv::ListTrees::Request::SharedPtr startReq = std::make_shared<riptide_msgs::srv::ListTrees::Request>();

        while (!refreshClient->wait_for_service(100ms))
                if (!rclcpp::ok())
                    return;
        
        auto refreshFuture = refreshClient->async_send_request();
        if (rclcpp::spin_until_future_complete(clientNode, refreshFuture) == rclcpp::FutureReturnCode::SUCCESS)
        {

            auto response = refreshFuture.get();
            
            uiPanel->btSelect->clear();

            uiPanel->btSelect->addItem("None Selected");
            
            for (auto tree : response->trees)
            {   
                // push these into the combo box
                uiPanel->btSelect->addItem(QString::fromStdString(tree));
            }
        }
    }   

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MissionPanel, rviz_common::Panel);