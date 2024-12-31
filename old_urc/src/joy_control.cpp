#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

class JoyStick{
    public:
        JoyStick();
    private:
        void joystick_callback(const sensor_msgs::Joy::ConstPtr& joy);
        ros::NodeHandle nh;

        int linear, angular, strafe;
        double l_scale, a_scale, s_scale;

        ros::Publisher velocity_pub;
        ros::Subscriber joystick_sub;
};

JoyStick::JoyStick():
    linear(1), 
    angular(2), 
    strafe(3)
{
    nh.param("axis_linear", linear, linear);
    nh.param("axis_angular", angular, angular);
    nh.param("axis_strafe", strafe, strafe);
    nh.param("scale_angular", a_scale, a_scale);
    nh.param("scale_linear", l_scale, l_scale);
    nh.param("scale_strafe", s_scale, s_scale);
    // nh.param("acceleration", accel, accel);

    velocity_pub = nh.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    joystick_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyStick::joystick_callback, this);

}

void JoyStick::joystick_callback(const sensor_msgs::Joy::ConstPtr& joy){
    // ROS_INFO("INSIDE JOYSTICK CALLBACK");
    geometry_msgs::Twist twist;
    int pressed = joy->buttons[7];
    int acceleration = 0;
    if (pressed == 1){
        ROS_INFO("Acceleration Pressed");
        acceleration = 10;
    }
    twist.angular.z = a_scale*joy->axes[angular];
    twist.linear.x = acceleration*joy->axes[linear];  
    velocity_pub.publish(twist);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "joystick");
    ROS_INFO("Node Started");
    JoyStick joystick; 
    ros::spin();
}