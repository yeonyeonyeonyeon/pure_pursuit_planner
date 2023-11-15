#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdlib>
#include <fstream>
#include <sstream>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// 클래스 정의 및 멤버 변수
class PurePursuit {
private:
    ros::NodeHandle nh;  // ROS 노드 핸들러, ROS와의 인터페이스를 제공
    ros::Subscriber path_subscriber;  // 경로 데이터를 구독
    ros::Subscriber odom_subscriber;  // 오도메트리 정보 구독
    ros::Subscriber goal_sub; 
    ros::Publisher cmd_vel_publisher; // 속도 명령 발행 
    ros::Publisher saved_path_publisher;  // 저장한 경로를 다시 publish
    ros::Publisher lookahead_point_publisher; // Lookahead point 발행
    geometry_msgs::PoseStamped current_pose; // 로봇의 현재 위치 정보
    std::vector<geometry_msgs::PoseStamped> path; // 받아온 경로 정보를 저장    
    geometry_msgs::PoseStamped lookahead_point; // lookahead_point를 저장할 변수 선언
    geometry_msgs::Twist cmd_vel; 
    tf::TransformListener tf_listener;
    std::vector<double> error_data;


    bool receivedPath = false;  // 플래그 추가
    bool begin = false;
    double lookahead_distance;
    double distance_squre;
    double transformed_x;
    double transformed_y;
    double error;
    
 
public:
    
    //생성자 (경로 정보와 오도메트리르 구독, 속도 명령 발행 준비)
    PurePursuit() 
        : path_subscriber(nh.subscribe("/move_base/GlobalPlanner/plan", 1, &PurePursuit::pathCallback, this)),
          odom_subscriber(nh.subscribe("/odom", 1, &PurePursuit::odomCallback, this)),
          goal_sub(nh.subscribe("/move_base_simple/goal", 1, &PurePursuit::goalCallback, this)),
          cmd_vel_publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1)), 
          saved_path_publisher(nh.advertise<nav_msgs::Path>("/saved_path", 1)),
          lookahead_point_publisher(nh.advertise<geometry_msgs::PoseStamped>("lookahead_point", 1)),
          receivedPath(false), // 초기화
          lookahead_distance(0.0), // 초기화
          distance_squre(0.0)
    {
        current_pose.pose.position.x = 0;
        current_pose.pose.position.y = 0;
        current_pose.pose.orientation.w = 1;

        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        path = std::vector<geometry_msgs::PoseStamped>();

        transformed_x = 0.0;
        transformed_y = 0.0;

        error_data = std::vector<double>();
        
    }
    

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        begin = true;
    }

    // dijksta 경로 받아서 'path'벡터에 저장
     void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
        //고정된 경로 사용
        if (!receivedPath && begin) {
            receivedPath = true; 
            const std::string file_name = "saved_path.txt";

            //경로를 바꾸지 않게 할때
            nav_msgs::Path saved_path;
            saved_path.header.stamp = ros::Time::now();
            saved_path.header.frame_id = "map";

            // 파일에서 경로 데이터 읽어오기
            std::ifstream file_in(file_name);
            if (file_in.is_open()) {
                std::string line;
                while (std::getline(file_in, line)) {
                    std::istringstream iss(line);
                    geometry_msgs::PoseStamped pose;
                    char delimiter = ',';
                    if (!(iss >> pose.pose.position.x >> delimiter >> pose.pose.position.y >> delimiter >> pose.pose.position.z >>
                        delimiter >> pose.pose.orientation.x >> delimiter >> pose.pose.orientation.y >> delimiter >> pose.pose.orientation.z >> delimiter >> pose.pose.orientation.w)) {
                        ROS_ERROR("Failed to parse line: %s", line.c_str());
                        continue;
                    }

                    // 경로 메시지에 추가
                    saved_path.poses.push_back(pose);
                    path.push_back(pose);
                }
                file_in.close();
            } else {
                ROS_ERROR("Unable to open file: %s", file_name.c_str());
                return ;  // 파일 열기 실패 시 종료
            }
            saved_path_publisher.publish(saved_path);
        }
    


        //경로를 바꿀때
        // if (!receivedPath && begin) { 
        //     path = path_msg->poses;
        //     receivedPath = true;  // 경로를 받았다는 것을 표시

        //     // 저장한 경로를 다시 publish
        //     nav_msgs::Path saved_path;
        //     saved_path.header.stamp = ros::Time::now();
        //     saved_path.header.frame_id = "map";
        //     saved_path.poses = path;

        //     saved_path_publisher.publish(saved_path);
        //     savePathToFile(saved_path); // 경로 텍스트 파일로 저장

        // }
    }

    // 오도메트리 데이터를 받아 'current_pose'에 저장
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
        current_pose.pose = odom_msg->pose.pose;
    }

    // 거리 계산 함수
    double distance(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
        return std::sqrt(std::pow(b.pose.position.x - a.pose.position.x, 2) + std::pow(b.pose.position.y - a.pose.position.y, 2));
    }

    // 제어 루프
    void controlLoop() {
        // 경로 없으면 종료
        if (path.empty() || !begin) {
            ROS_INFO("no goal point");
            ros::Duration(1).sleep();
            return;
        }

        // 목표 지점에 도달 했는지 확인하고 도달하면 멈추기
        double distance_to_goal = distance(current_pose, path.back());
        if (distance_to_goal < 0.1) {
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_publisher.publish(stop_cmd);
            ROS_INFO("Reached the goal!");
            ros::Duration(1).sleep();
            begin = false;
            //ros::shutdown();  // ROS 노드 종료
            return;
        }

        // 현재 위치 설정
        tf::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 가장 가까운 웨이포인트 찾기
        // double 데이터 타입이 가질 수 있는 최대 값을 반환, 이렇게 하면 이후 최소 거리 값을 쉽게 찾을 수 있다.
        double min_dist = std::numeric_limits<double>::max(); 
       

        // lookahed 거리 설정
        lookahead_distance =0.8; 

        // lookahead 점 계산
        double min_lookahead_distance = std::numeric_limits<double>::max();

        for (const auto& waypoint : path) {
            double d = distance(current_pose, waypoint);

            // waypoint와 로봇 사이의 방향을 계산
            double alpha = atan2(waypoint.pose.position.y - current_pose.pose.position.y,
                                waypoint.pose.position.x - current_pose.pose.position.x);

            // 방향 차이를 계산
            double delta_theta = alpha - yaw;

            // 차이가 -pi와 pi 사이에 오도록 보정
            while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
            while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

            // 차이가 임계값 내에 있는지 확인
            if (std::abs(delta_theta) < M_PI / 4) {  
                if (std::abs(d - lookahead_distance) < min_lookahead_distance) {
                    min_lookahead_distance = std::abs(d - lookahead_distance);
                    lookahead_point = waypoint;
                }
            }
        }

        // lookahead 포인트 퍼블리쉬 
        lookahead_point.header.stamp = ros::Time::now();
        lookahead_point.header.frame_id = "map";
        lookahead_point_publisher.publish(lookahead_point);


        // tf 변환을 위한 StampedTransform 객체
        tf::StampedTransform transform;
        try {
            // 현재 시간 기준으로 "map" 프레임에서 "base_link" 프레임까지의 변환
            tf_listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("Received an exception trying to transform a point from 'map' to 'base_link': %s", ex.what());
            return;
        }
        

        tf::Vector3 point_in_map_frame(lookahead_point.pose.position.x, lookahead_point.pose.position.y, 0);
        tf::Vector3 point_in_base_frame = transform.inverse() * point_in_map_frame;

        transformed_x = point_in_base_frame.x();
        transformed_y = point_in_base_frame.y();
        
        ROS_INFO("Transformed lookahead_point (base_link frame): x = %f, y = %f", transformed_x, transformed_y);

        // 로봇과 lookahead 거리
        distance_squre = distance(current_pose, lookahead_point);
        //ROS_INFO("distance_squre: %f", distance_squre);

        //error 값
        error = transformed_y;
        error_data.push_back(error);
        ROS_INFO("error : %f", error);


        // 속도
        cmd_vel.linear.x = 0.4 * (sqrt(distance_squre));  
        cmd_vel.angular.z = 0.6 * ((2*(transformed_y))/(distance_squre));

        cmd_vel_publisher.publish(cmd_vel);
        ROS_INFO("cmd_vel linear.x: %f, angular.z: %f", cmd_vel.linear.x, cmd_vel.angular.z);
        ROS_INFO("  ");

    }

    void stop() {
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_publisher.publish(stop_cmd);
    }

    
    //Open cv 코드
    void plotErrorGraph() {
        if (!error_data.empty()) {
            int graph_width = 1000;
            int graph_height = 500;
            double max_error = *std::max_element(error_data.begin(), error_data.end());
            double min_error = *std::min_element(error_data.begin(), error_data.end());

            double max_abs_error = std::max(std::abs(max_error), std::abs(min_error));

            cv::Mat graph(graph_height, graph_width, CV_8UC3, cv::Scalar(255, 255, 255));

            for (size_t i = 1; i < error_data.size(); i++) {
                int x1 = (i - 1) * (graph_width / (double)error_data.size());
                int y1 = graph_height/2 - (error_data[i - 1] / max_abs_error) * (graph_height/2);
                
                int x2 = i * (graph_width / (double)error_data.size());
                int y2 = graph_height/2 - (error_data[i] / max_abs_error) * (graph_height/2);

                cv::line(graph, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);
            }

            // X축 및 Y축 그리기
            cv::line(graph, cv::Point(0, graph_height / 2), cv::Point(graph_width, graph_height / 2), cv::Scalar(0, 0, 0), 2); // 중앙 X축
            cv::line(graph, cv::Point(20, 0), cv::Point(20, graph_height), cv::Scalar(0, 0, 0), 2); // 좌측 Y축

            // X축 및 Y축 레이블 추가
            cv::putText(graph, "Time", cv::Point(graph_width - 50, graph_height/2 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::putText(graph, "Error", cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

            cv::imshow("Error Data Graph", graph);
            cv::waitKey(30);
        }
    }

    void savePathToFile(const nav_msgs::Path& path) {
        std::ofstream file("saved_path.txt");

        if (file.is_open()) {
            for (const auto& pose_stamped : path.poses) {
                // 위치 정보 저장
                file << pose_stamped.pose.position.x << ", "
                    << pose_stamped.pose.position.y << ", "
                    << pose_stamped.pose.position.z << ", ";

                // 방향 정보 저장
                file << pose_stamped.pose.orientation.x << ", "
                    << pose_stamped.pose.orientation.y << ", "
                    << pose_stamped.pose.orientation.z << ", "
                    << pose_stamped.pose.orientation.w << "\n";
            }
            file.close();
            ROS_INFO("Path saved to saved_path.txt");
        } 
        
        else {
            ROS_WARN("Unable to open file");
        }
    }

  



};



// 메인 함수
int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit_planner");
    PurePursuit pure_pursuit;

    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        pure_pursuit.controlLoop();
        pure_pursuit.plotErrorGraph(); 
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
