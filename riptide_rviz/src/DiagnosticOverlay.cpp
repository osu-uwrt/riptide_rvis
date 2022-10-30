#include "riptide_rviz/DiagnosticOverlay.hpp"

#include <QFontDatabase>
#include <rviz_common/logging.hpp>

using namespace std::placeholders;

namespace riptide_rviz
{
    DiagnosticOverlay::DiagnosticOverlay(){
        // make the ROS node
        auto options = rclcpp::NodeOptions().arguments({});
        nodeHandle = std::make_shared<rclcpp::Node>("riptide_rviz_overlay", options);

        // init font parameter
        QFontDatabase database;
        fontFamilies = database.families();
        fontProperty = new rviz_common::properties::EnumProperty("font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
        for (ssize_t i = 0; i < fontFamilies.size(); i++) {
            fontProperty->addOption(fontFamilies[i], (int) i);
        }
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
            12, 0, 0, 0, "00.00 V",
            fontName, false, 2, 12,
            QColor(255, 0, 255, 255)
        };

        std::cerr << "Placing inital text" << std::endl;

        voltageTextId = addText(initText);

        PaintedCircleConfig ledConfig = {
            40, 60, 0, 0, 25, 30,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
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
                    12, 0, 0, 0, "00.00 V",
                    fontName, false, 2, 12,
                    QColor(255, 0, 0, 255)
                };
                    
                if(diagnostic.message.find("No data") == std::string::npos){
                    // now we need to look at the status of the voltage to determine color
                    // ok is green, warn is yellow, error is red
                    if(diagnostic.level == diagnostic.ERROR){
                        initText.text_color_ = QColor(255, 0, 0, 255);
                    } else if (diagnostic.level == diagnostic.WARN){
                        initText.text_color_ = QColor(255, 255, 0, 255);
                    } else {
                        initText.text_color_ = QColor(0, 255, 0, 255);
                    }

                    // find voltage
                    initText.text_ = "20.25 V";
                }

                // edit the text
                updateText(voltageTextId, initText);
            }

            // handle general packet for LED
            else if(diagnostic.name == "/Robot Diagnostics"){
                PaintedCircleConfig ledConfig = {
                    40, 60, 0, 0, 25, 30,
                    QColor(255, 0, 0, 255),
                    QColor(0, 0, 0, 255)
                };

                // Determine the LED color to use
                if(diagnostic.level == diagnostic.ERROR){
                    ledConfig.inner_color_ = QColor(255, 0, 0, 255);
                } else if (diagnostic.level == diagnostic.WARN){
                    ledConfig.inner_color_ = QColor(255, 255, 0, 255);
                } else {
                    ledConfig.inner_color_ = QColor(0, 255, 0, 255);
                }

                updateCircle(ledConfigId, ledConfig);
            }
        }
    }

    void DiagnosticOverlay::updateFont() {
        int font_index = fontProperty->getOptionInt();
        if (font_index < fontFamilies.size()) {
            fontName = fontFamilies[font_index].toStdString();
        } else {
            RVIZ_COMMON_LOG_ERROR_STREAM("Unexpected error at selecting font index " << font_index);
            return;
        }
        

        require_update_texture_ = true;
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