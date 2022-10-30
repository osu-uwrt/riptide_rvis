#pragma once

#include "riptide_rviz/OverlayDisplay.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

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

        protected Q_SLOTS:
        void updateFont();

        private:
        // internal node
        rclcpp::Node::SharedPtr nodeHandle;

        // subscription for diagnostics
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;

        // ids for rendering items so that we can edit them
        int voltageTextId = -1;
        int ledConfigId = -1;

        // font configuration info
        QStringList fontFamilies;
        std::string fontName;

        // Addtional RVIZ settings
        rviz_common::properties::EnumProperty *fontProperty;

    };
} // namespace riptide_rviz
