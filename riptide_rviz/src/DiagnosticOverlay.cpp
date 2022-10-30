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
        OverlayDisplay::onInitialize();

        // make the diagnostic subscriber
        diagSub = nodeHandle->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics_agg", rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::diagnosticCallback, this, _1)
        );

        PaintedTextConfig initText = {
            0, 0, 0, 0, "00.00 V",
            "", false, 2, 12,
            QColor(255, 0, 255, 255)
        };

        std::cerr << "Placing inital text" << std::endl;

        voltageTextId = addText(initText);

        PaintedCircleConfig ledConfig = {
            40, 60, 0, 0, 25, 30,
            QColor(0, 255, 255, 200),
            QColor(0, 0, 0, 200)
        };

        ledConfigId = addCircle(ledConfig);
    }

    void DiagnosticOverlay::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg){
        // look for specific packets
        for(auto diagnostic : msg.status){
            // handle robot voltage packet
            if(diagnostic.name == "/Robot Diagnostics/Electronics/Voltages and Currents/V+ Rail Voltage"){
                // set text to ??? and color to red
                PaintedTextConfig initText = {
                    0, 0, 0, 0, "00.00 V",
                    "", false, 2, 12,
                    QColor(255, 0, 0, 255)
                };
                    
                if(diagnostic.message.find("No data") != std::string::npos){
                    updateText(voltageTextId, initText);
                }
                else{
                    // now we need to look at the status of the voltage to determine color
                    // ok is green, warn is yellow, error is red
                    if(diagnostic.message == "Error"){
                        //
                    } else if (diagnostic.message == "Warn"){
                        //
                    } else {
                        //
                    }
                    

                }

                // edit the text
            }
        }


    }
    
    void DiagnosticOverlay::onEnable(){
        OverlayDisplay::onEnable();
    }
    
    void DiagnosticOverlay::onDisable(){
        OverlayDisplay::onDisable();
        
    }
    
    void DiagnosticOverlay::update(float wall_dt, float ros_dt){
        rclcpp::spin_some(nodeHandle);

        OverlayDisplay::update(wall_dt, ros_dt);
    }

    void DiagnosticOverlay::reset(){
        OverlayDisplay::reset();

    }

} // namespace riptide_rviz


// Since this is a parent class this has been removed
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(riptide_rviz::DiagnosticOverlay, rviz_common::Display)