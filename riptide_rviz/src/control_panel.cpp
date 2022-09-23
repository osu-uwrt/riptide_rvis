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

        // top slice for startup
        startup = new QHBoxLayout();

        // create the dropdown lists
        bringupHost = new QComboBox(this);
        bringupHost->setEditable(false);
        bringupHost->setFrame(true);
        bringupHost->insertItems(0, QStringList(QString("Computer 1")));

        bringupFile = new QComboBox(this);
        bringupFile->setEditable(false);
        bringupFile->setFrame(true);
        bringupFile->insertItems(0, QStringList(QString("bringup.py")));

        bringupStart = new QPushButton(this);
        bringupStart->setText("Start Code");

        bringupStop = new QPushButton(this);
        bringupStop->setText("Stop Code");


        // define the layout of the bringup section
        QVBoxLayout *bringupSelectors = new QVBoxLayout();
        bringupSelectors->addWidget(bringupHost);
        bringupSelectors->addWidget(bringupFile);
        startup->addLayout(bringupSelectors);
        QVBoxLayout *bringupButtons = new QVBoxLayout();
        bringupButtons->addWidget(bringupStart);
        bringupButtons->addWidget(bringupStop);
        startup->addLayout(bringupButtons);
        



        // register the parent layout to the widget
        layout->addLayout(startup);
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
        delete layout, startup;

        // bringup panel destruction
        delete bringupHost, bringupFile, bringupStart, bringupStop;
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);