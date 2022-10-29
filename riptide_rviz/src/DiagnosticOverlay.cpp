#include "riptide_rviz/DiagnosticOverlay.hpp"

using namespace std::placeholders;

namespace riptide_rviz
{
    DiagnosticOverlay::DiagnosticOverlay(){
        // make the ROS node
        auto options = rclcpp::NodeOptions().arguments({});
        nodeHandle = std::make_shared<rclcpp::Node>("riptide_rviz_overlay", options);
    }

    DiagnosticOverlay::~DiagnosticOverlay(){
    }

    void DiagnosticOverlay::onInitialize(){
        OverlayTextDisplay::onInitialize();

        // make the diagnostic subscriber
        diagSub = nodeHandle->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_agg", rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::diagnosticCallback, this, _1)
        );
    }

    void DiagnosticOverlay::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg){
        // look for specific packets
        for(auto diagnostic : msg.status){
            // handle robot voltage packet
            if(diagnostic.name == "/Robot Diagnostics/Electronics/Voltages and Currents/V+ Rail Voltage"){
                FrameProperties props = {
                    "",
                    300,
                    100,

                    12,
                    0,
                    0,

                    QColor(0, 0, 0, 0),
                    QColor(255, 0, 0, 255),

                    300
                };
                    
                if(diagnostic.message.find("No data") != std::string::npos){
                    // set text to ??? and color to red
                    props.text = "???";
                    props.fg_color_ = QColor(255, 0, 0, 255);
                }
                else{
                    // now we need to look at the status of the device to determine color
                    // ok is green, warn is yellow, error is red
                    if(diagnostic.message == "Error"){
                        props.fg_color_ = QColor(255, 0, 0, 255);
                    } else if (diagnostic.message == "Warn"){
                        props.fg_color_ = QColor(255, 255, 0, 255);
                    } else {
                        props.fg_color_ = QColor(0, 255, 0, 255);
                    }
                    props.text = "33.5V";

                }

                setText(props, true);
            }
        }


    }
    
    void DiagnosticOverlay::onEnable(){
        OverlayTextDisplay::onEnable();
    }
    
    void DiagnosticOverlay::onDisable(){
        OverlayTextDisplay::onDisable();
        
    }
    
    void DiagnosticOverlay::update(float wall_dt, float ros_dt){
        rclcpp::spin_some(nodeHandle);

        OverlayTextDisplay::update(wall_dt, ros_dt);
    }

    void DiagnosticOverlay::reset(){
        OverlayTextDisplay::reset();

    }

} // namespace riptide_rviz


// Since this is a parent class this has been removed
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(riptide_rviz::DiagnosticOverlay, rviz_common::Display)