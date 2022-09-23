#include "riptide_rviz/MissionPanel.hpp"

namespace riptide_rviz
{
    MissionPanel::MissionPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_MissionPanel();
        uiPanel->setupUi(this);
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
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MissionPanel, rviz_common::Panel);