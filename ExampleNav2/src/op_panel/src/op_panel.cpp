#include "op_panel/op_panel.h"

namespace op_panel
{
    const float FORWARD_SPEED_MAX = 2.0;  // Metres per second
    const float ROTATIONAL_SPEED_MAX = 2.0; // Radians per second
    const int SPEED_SLIDER_MAX = 100;  // Percent - Maximum for the speed control slider.
    const int SPEED_SLIDER_LARGE_STEP = 10;  // Percent - Large step for the speed control slider.
    const int SPEED_SLIDER_SINGLE_STEP = 1;  // Percent - Small step for the speed control slider.

    OperatorPanel::OperatorPanel(QWidget *parent)
                : rviz_common::Panel(parent)
    {
        auto layout = new QVBoxLayout();
        layout->setContentsMargins(0,0,0,0);

        text = new QLabel("Speed Control");

		// Reset button
		pb_reset = new QPushButton("Reset");
		pb_reset->setEnabled(true);

		// Forward speed control
		QHBoxLayout* linear_layout = new QHBoxLayout();
        text_v = new QLabel("Forward: ");
        slider_v = new QSlider();
        lb_slider_v_value = new QLabel("--");
		initRobotSpeedSlider(slider_v);
        lb_slider_v_value->setFrameShape(QFrame::Panel);
        lb_slider_v_value->setMinimumWidth(50);
		linear_layout->addWidget(text_v);
		linear_layout->addWidget(slider_v);
		linear_layout->addWidget(lb_slider_v_value);
		
//        linear_layout->addStretch();

		// Rotational speed control
		QHBoxLayout* angular_layout = new QHBoxLayout();
        text_w = new QLabel("Angular: ");
        slider_w = new QSlider();
        lb_slider_w_value = new QLabel("--");
		initRobotSpeedSlider(slider_w);
        lb_slider_w_value->setFrameShape(QFrame::Panel);
        lb_slider_w_value->setMinimumWidth(50);
		angular_layout->addWidget(text_w);
		angular_layout->addWidget(slider_w);
		angular_layout->addWidget(lb_slider_w_value);
        
		// Add elements to the vertical layout
        layout->addWidget(text);
		layout->addLayout(linear_layout);
		layout->addLayout(angular_layout);
        layout->addWidget(pb_reset);
		setLayout(layout);

        // Sockets and signals
	    connect(slider_v, SIGNAL(valueChanged(int)), this, SLOT(setRobotSpeed()));
	    connect(slider_w, SIGNAL(valueChanged(int)), this, SLOT(setRobotSpeed()));
		connect(pb_reset, SIGNAL(clicked()), this, SLOT(resetPressed()));

        // Set up publishers
        node.twist_pub = node.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        node.reset_pub = node.create_publisher<std_msgs::msg::Empty>("reset", 10);

		// Initial values
		setRobotSpeed();
    }

    void OperatorPanel::onInitialize()
    {
        // Override function
    }

	/**
	\fn		setRobotSpeed
	\brief	Send a new speed message.
	*/
	void OperatorPanel::setRobotSpeed()
	{
		// Read the sliders and convert to a speed value.
		float forward_speed = FORWARD_SPEED_MAX*static_cast<float>(slider_v->sliderPosition())/static_cast<float>(SPEED_SLIDER_MAX);
		float rotational_speed = ROTATIONAL_SPEED_MAX*static_cast<float>(slider_w->sliderPosition())/static_cast<float>(SPEED_SLIDER_MAX);
    
		// Set text boxes on the display
	    float speed = roundf(forward_speed*100.0f)/100.0f; // Limit to two decimal places.
	    lb_slider_v_value->setNum(speed);
	    speed = roundf(rotational_speed*100.0f)/100.0f; // Limit to two decimal places.
	    lb_slider_w_value->setNum(speed);

		publishRobotSpeed(forward_speed, rotational_speed);
	}

	/**
	\fn		publishRobotSpeed
	\brief	Publish a new speed message.
	*/
	void OperatorPanel::publishRobotSpeed(float forward, float rotational)
	{
		// Publish the new speed control.
		geometry_msgs::msg::Twist msg;
//        msg.header.frame_id = "base_link";
//        msg.header.stamp = node.get_clock()->now();

		msg.linear.x = forward;
		msg.linear.y = 0.;
		msg.linear.z = 0.;

		msg.angular.x = 0.;
		msg.angular.y = 0.;
		msg.angular.z = rotational;

        // Send the message
		node.twist_pub->publish(msg);
	}

	/**
	\fn		initRobotSpeedSlider
	\brief	Initializes a given speed control slider.
	*/
	void OperatorPanel::initRobotSpeedSlider(QSlider *slider)
	{
		slider->setMinimumWidth( 100 );
		slider->setMinimum(-SPEED_SLIDER_MAX);
		slider->setMaximum(SPEED_SLIDER_MAX);
		slider->setOrientation(Qt::Horizontal);
		slider->setTickPosition(QSlider::TicksAbove);
		slider->setSingleStep(SPEED_SLIDER_SINGLE_STEP);  // Small step size (arrow keys).
		slider->setPageStep(SPEED_SLIDER_LARGE_STEP);  // Large step size (Page Up/Down)
		slider->setTracking(false); // Only signal when the slider is released.
    }

	/**
	\fn		resetPressed
	\brief	Reset processing.
	*/
	void OperatorPanel::resetPressed()
	{
		// Reset sliders and value feedback
		slider_v->setValue(0);
		slider_w->setValue(0);

		// Send the new speed
		setRobotSpeed();

		// Send the reset request
		node.reset_pub->publish(std_msgs::msg::Empty());
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(op_panel::OperatorPanel, rviz_common::Panel)