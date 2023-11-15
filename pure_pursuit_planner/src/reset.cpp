#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "reset");
    
    // 노드 핸들 생성
    ros::NodeHandle nh;
    
    // /cmd_vel 토픽에 퍼블리쉬 할 수 있는 퍼블리셔 객체 생성
    ros::Publisher reset_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    ros::Duration(0.5).sleep(); // 중요!!

    // Gazebo의 /gazebo/set_model_state 서비스에 접근
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    reset_pub.publish(cmd_vel_msg);
        

    ros::spinOnce();

    // 로그 메시지 출력
    ROS_INFO("Stopping the robot.");

    ros::Duration(1.5).sleep();
    
    // Gazebo에서 모델의 상태를 설정
    gazebo_msgs::ModelState model_state;
    model_state.model_name = "turtlebot3_burger";
    model_state.pose.position.x = -2.0;
    model_state.pose.position.y = -0.58;
    model_state.pose.position.z = 0;
    model_state.pose.orientation.z = 0;
    
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = model_state;
    
    if (client.call(srv))
    {
        ROS_INFO("Robot position reset in Gazebo.");
    }
    else
    {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
    }
    
    return 0;
}
