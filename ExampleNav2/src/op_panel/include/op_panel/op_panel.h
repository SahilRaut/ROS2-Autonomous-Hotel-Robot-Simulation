#pragma once

#include <QMainWindow>
#include "rviz_common/panel.hpp"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QRadioButton>
#include <QPushButton>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

namespace op_panel
{
    class OperatorPanelNode : public rclcpp::Node
    {
        public:
            OperatorPanelNode() : Node("op_panel") {}
            ~OperatorPanelNode() {}
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub;

        private:

    };

    class OperatorPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
            explicit OperatorPanel(QWidget *parent=nullptr);
            void onInitialize() override;

		// Public Qt slots.
		public Q_SLOTS:
            
		// Internal Qt slots.
		protected Q_SLOTS:
			void setRobotSpeed(void);	
	        void resetPressed();

        protected:
            QLabel *text;
            QLabel *text_v;
            QLabel *text_w;
            QSlider *slider_v;
            QSlider *slider_w;
            QLabel *lb_slider_v_value;
            QLabel *lb_slider_w_value;
            QPushButton *pb_reset;

        private:
	        void initRobotSpeedSlider(QSlider *slider);
	        void publishRobotSpeed(float forward, float rotational);
            OperatorPanelNode node;

    };
}