#include "riptide_rviz/ControlPanel.hpp"
#include <iostream>

namespace riptide_rviz
{
    ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_ControlPanel();
        uiPanel->setupUi(this);
    }

    void ControlPanel::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load( config );
    }

    void ControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save( config );
    }

    bool ControlPanel::event(QEvent *event)
    {
    }

    ControlPanel::~ControlPanel(){
        // master window control removal
        delete uiPanel;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);