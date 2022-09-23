#include "riptide_rviz/control_panel.hpp"
#include <iostream>

namespace riptide_rviz
{
    ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        // std::cout << "hi from riptide rviz control panel" << std::endl;
        setFocusPolicy(Qt::ClickFocus);

        // label = new QLabel(this);
        // label->setFrameStyle(QFrame::Panel | QFrame::Sunken);
        // label->setText("jbsdfjkndf jkn");
        // label->setAlignment(Qt::AlignBottom | Qt::AlignRight);

        // overall panel layout
        layout = new QVBoxLayout();

        // create the bringup dropdown lists
        bringupHost = new QComboBox(this);
        bringupHost->setEditable(false);
        bringupHost->setFrame(true);
        bringupHost->insertItems(0, QStringList(QString("Computer 1")));
        bringupFile = new QComboBox(this);
        bringupFile->setEditable(false);
        bringupFile->setFrame(true);
        bringupFile->insertItems(0, QStringList(QString("bringup.py")));

        // make the bringup buttons
        bringupStart = new QPushButton("Start Code", this);
        bringupStart->setStyleSheet("QPushButton {background-color: green;}");
        bringupStop = new QPushButton("Stop Code", this);
        bringupStop->setStyleSheet("QPushButton {background-color: red;}");
        bringupStop->setDisabled(true);

        // define the layout of the bringup section
        bringup = new QHBoxLayout();
        QVBoxLayout *bringupSelectors = new QVBoxLayout();
        bringupSelectors->addWidget(bringupHost);
        bringupSelectors->addWidget(bringupFile);
        bringup->addLayout(bringupSelectors);
        QVBoxLayout *bringupButtons = new QVBoxLayout();
        bringupButtons->addWidget(bringupStart);
        bringupButtons->addWidget(bringupStop);
        bringup->addLayout(bringupButtons);

        // handle the controller command layout
        ctrlCmdPos = new QPushButton("Position", this);
        ctrlCmdPos->setStyleSheet("QPushButton {background-color: #A3C1DA;}"); // the active mode
        ctrlCmdVel = new QPushButton("Velocity", this);
        ctrlCmdFFD = new QPushButton("FeedFwd", this);
        ctrlCmdTele = new QPushButton("Teleop", this);

        ctrlEnable = new QPushButton("Enable", this);
        ctrlEnable->setStyleSheet("QPushButton {background-color: green;}");
        ctrlDisable = new QPushButton("Disable", this);
        ctrlDisable->setStyleSheet("QPushButton {background-color: red;}");
        ctrlDisable->setDisabled(true);

        // define control layout
        ctrl = new QHBoxLayout();
        QVBoxLayout *ctrlModeLayout = new QVBoxLayout();
        ctrlModeLayout->addWidget(ctrlCmdPos);
        ctrlModeLayout->addWidget(ctrlCmdVel);
        ctrlModeLayout->addWidget(ctrlCmdFFD);
        ctrlModeLayout->addWidget(ctrlCmdTele);
        ctrl->addLayout(ctrlModeLayout);
        QVBoxLayout *ctrlEnableLayout = new QVBoxLayout();
        ctrlEnableLayout->addWidget(ctrlEnable);
        ctrlEnableLayout->addWidget(ctrlDisable);
        ctrl->addLayout(ctrlEnableLayout);

        



        // register the parent layout to the widget
        layout->addLayout(bringup);
        layout->addLayout(ctrl);
        setLayout(layout);
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
        delete layout, bringup, ctrl;

        // bringup panel destruction
        delete bringupHost, bringupFile, bringupStart, bringupStop;

        // controller panel distruction
        delete ctrlCmdPos, ctrlCmdVel, ctrlCmdFFD, ctrlCmdTele, ctrlEnable, ctrlDisable;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);