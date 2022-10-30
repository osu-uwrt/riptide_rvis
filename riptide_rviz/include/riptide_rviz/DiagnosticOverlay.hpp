#pragma once

#include "riptide_rviz/OverlayDisplay.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <riptide_msgs2/msg/robot_state.hpp>

#define ROBOT_NS "/tempest"

namespace riptide_rviz
{
    class DiagnosticOverlay : public OverlayDisplay {
        Q_OBJECT
        public:
        DiagnosticOverlay();

        ~DiagnosticOverlay();

        protected:

        virtual void onInitialize() override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void update(float wall_dt, float ros_dt) override;
        virtual void reset() override;

        void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray & msg);
        void killCallback(const riptide_msgs2::msg::RobotState & msg);

        protected Q_SLOTS:
        void updateFont();

        private:
        // internal node
        rclcpp::Node::SharedPtr nodeHandle;

        // subscription for diagnostics
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;
        rclcpp::Subscription<riptide_msgs2::msg::RobotState>::SharedPtr killSub;

        // ids for rendering items so that we can edit them
        int voltageTextId = -1;
        int diagLedConfigId = -1;
        int killLedConfigId = -1;

        // font configuration info
        QStringList fontFamilies;
        std::string fontName;

        // Addtional RVIZ settings
        rviz_common::properties::EnumProperty *fontProperty;

        // configurations for display items
        PaintedCircleConfig diagLedConfig = {
            20, 50, 0, 0, 7, 9,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
        };
        PaintedCircleConfig killLedConfig = {
            60, 50, 0, 0, 7, 9,
            QColor(255, 0, 255, 255),
            QColor(0, 0, 0, 255)
        };
        PaintedTextConfig voltageConfig = {
            12, 0, 0, 0, "00.00 V",
            fontName, false, 2, 12,
            QColor(255, 0, 0, 255)
        };

    };
} // namespace riptide_rviz
