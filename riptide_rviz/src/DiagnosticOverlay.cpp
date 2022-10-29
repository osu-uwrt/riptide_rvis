#include "riptide_rviz/DiagnosticOverlay.hpp"

namespace riptide_rviz
{
    DiagnosticOverlay::DiagnosticOverlay(){

    }

    DiagnosticOverlay::~DiagnosticOverlay(){
        
    }

} // namespace riptide_rviz


// Since this is a parent class this has been removed
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(riptide_rviz::DiagnosticOverlay, rviz_common::Display)