#include "riptide_rviz/control_panel.hpp"
#include <iostream>

namespace riptide_rviz
{
    MissionPanel::MissionPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        // std::cout << "hi from riptide rviz control panel" << std::endl;
        setFocusPolicy(Qt::ClickFocus);

        label = new QLabel(this);
        label->setFrameStyle(QFrame::Panel | QFrame::Sunken);
        label->setText("jbsdfjkndf jkn");
        label->setAlignment(Qt::AlignBottom | Qt::AlignRight);
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
        delete label;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MissionPanel, rviz_common::Panel);