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

        killSub = nodeHandle->create_subscription<riptide_msgs2::msg::RobotState>(
            ROBOT_NS + std::string("/state/firmware"), rclcpp::SystemDefaultsQoS(), std::bind(&DiagnosticOverlay::killCallback, this, _1)
        );

        // std::cerr << "Placing inital overlay" << std::endl;

        // add all of the variable design items
        voltageConfig.text_color_ = QColor(255, 0, 255, 255);
        voltageTextId = addText(voltageConfig);

        diagLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        diagLedConfigId = addCircle(diagLedConfig);

        killLedConfig.inner_color_ = QColor(255, 0, 255, 255);
        killLedConfigId = addCircle(killLedConfig);

        // add the static design items
        PaintedTextConfig diagLedLabel = {
            6, 20, 0, 0, "Diag",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        PaintedTextConfig killLedLabel = {
            50, 20, 0, 0, "Kill",
            fontName, false, 2, 12,
            QColor(255, 255, 255, 255)
        };
        addText(diagLedLabel);
        addText(killLedLabel);
    }

    void DiagnosticOverlay::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg){
        // look for specific packets
        for(auto diagnostic : msg.status){
            // handle robot voltage packet
            if(diagnostic.name == "/Robot Diagnostics/Electronics/Voltages and Currents/V+ Rail Voltage"){
                voltageConfig.text_ = "00.00 V";
                voltageConfig.text_color_ = QColor(255, 0, 0, 255);
                if(diagnostic.message.find("No data") == std::string::npos){
                    // now we need to look at the status of the voltage to determine color
                    // ok is green, warn is yellow, error is red
                    if(diagnostic.level == diagnostic.ERROR){
                        voltageConfig.text_color_ = QColor(255, 0, 0, 255);
                    } else if (diagnostic.level == diagnostic.WARN){
                        voltageConfig.text_color_ = QColor(255, 255, 0, 255);
                    } else {
                        voltageConfig.text_color_ = QColor(0, 255, 0, 255);
                    }

                    // find voltage
                    voltageConfig.text_ = "20.25 V";
                }

                // edit the text
                updateText(voltageTextId, voltageConfig);
            }

            // handle general packet for LED
            else if(diagnostic.name == "/Robot Diagnostics"){
                // Determine the LED color to use
                if(diagnostic.level == diagnostic.ERROR){
                    diagLedConfig.inner_color_ = QColor(255, 0, 0, 255);
                } else if (diagnostic.level == diagnostic.WARN){
                    diagLedConfig.inner_color_ = QColor(255, 255, 0, 255);
                } else {
                    diagLedConfig.inner_color_ = QColor(0, 255, 0, 255);
                }

                updateCircle(diagLedConfigId, diagLedConfig);
            }
        }
    }

    void DiagnosticOverlay::killCallback(const riptide_msgs2::msg::RobotState & msg){
        if(msg.kill_switch_inserted){
            killLedConfig.inner_color_ = QColor(0, 255, 0, 255);
        } else {
            killLedConfig.inner_color_ = QColor(255, 0, 0, 255);
        }
        updateCircle(killLedConfigId, killLedConfig);
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