#include "riptide_rviz/control_panel.hpp"
#include <iostream>

namespace riptide_rviz
{
    MissionPanel::MissionPanel(QWidget *parent)
    {
        std::cout << "hi form riptide rviz control panel" << std::endl;
        label = new QLabel(parent);
        label->setFrameStyle(QFrame::Panel | QFrame::Sunken);
        label->setText("first line\nsecond line");
        label->setAlignment(Qt::AlignBottom | Qt::AlignRight);
    }

    void MissionPanel::load(const rviz_common::Config &config)
    {
    }

    void MissionPanel::save(rviz_common::Config config) const
    {
    }

    bool MissionPanel::event(QEvent *event)
    {
    }

    MissionPanel::~MissionPanel(){
        delete label;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::MissionPanel, rviz_common::Panel)