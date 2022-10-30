#pragma once

#include "riptide_rviz/OverlayDisplay.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace riptide_rviz
{
    class DiagnosticOverlay : public OverlayDisplay {
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


        private:
        // internal node
        rclcpp::Node::SharedPtr nodeHandle;

        // subscription for diagnostics
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagSub;

        int voltageTextId = -1;
        int ledConfigId = -1;

    };
} // namespace riptide_rviz
