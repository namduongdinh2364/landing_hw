/**
 * 
*/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "landing.h"

using namespace std;
using namespace Eigen;

static mavros_msgs::State current_state;
static geometry_msgs::PoseStamped mavros_local_position_pose, desPose;
static geometry_msgs::TwistStamped desVelocity;
static Matrix3f imu_rotation_matrix;
static bool check_state = true;
static char next_status[20];
double cur_roll, cur_pitch, cur_yaw;


/**
 * @brief Get the state
 */
void state_Callback(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
}

/**
 * @brief Get local position form mavros
 */
void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        mavros_local_position_pose = *msg;
}

/**
 * @brief Get local position form imu
 */
void imuPose_Callback(const sensor_msgs::Imu::ConstPtr& msg) {
        double x, y, z, w;
        Quaternionf quat;

        x = msg->orientation.x;
        y = msg->orientation.y;
        z = msg->orientation.z;
        w = msg->orientation.w;

        /* making a quaternion of position */
        quat = Eigen::Quaternionf(w, x, y, z);

        /* making rotation matrix from quaternion */
        imu_rotation_matrix = quat.toRotationMatrix();
}

/**
 * @brief Get local position form imu
 */
void getNextStatus_Callback(const std_msgs::String::ConstPtr& msg) {
        strcpy(next_status, msg->data.c_str());
}

void getDestinatonPose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        /**  
         * For world frame, we will use ENU (EAST, NORTH and UP)
         *      +Z     +Y
         *       ^    ^
         *       |  /
         *       |/
         *     world------> +X
         */
        desPose.header.stamp = ros::Time::now();
        desPose.pose.position.x = msg->pose.position.x;
        desPose.pose.position.y = msg->pose.position.y;
        desPose.pose.position.z = msg->pose.position.z;

        double xq,yq,zq,wq;
        xq = msg->pose.orientation.x;
        yq = msg->pose.orientation.y;
        zq = msg->pose.orientation.z;
        wq = msg->pose.orientation.w;

        tf2::Quaternion q;
        Quaternionf q_update;
        double m_roll, m_pitch, m_yaw, output_yaw;

        q.setValue(xq, yq, zq, wq);
        tf2::Matrix3x3(q).getRPY(m_roll, m_pitch, m_yaw);

        /**
         * If yaw into range of +- 10 degrees.
         * It should be continually updated orientation
         */
        if (YAW_ANGLE(m_yaw) >= 10 || YAW_ANGLE(m_yaw) <= -10) {
                q_update = AngleAxisf(0, Vector3f::UnitX()) *
                           AngleAxisf(0, Vector3f::UnitY()) *
                           AngleAxisf(cur_yaw - RATODE(10), Vector3f::UnitZ());
        }

        desPose.pose.orientation.x = q_update.x();
        desPose.pose.orientation.y = q_update.y();
        desPose.pose.orientation.z = q_update.z();
        desPose.pose.orientation.w = q_update.w();
        cout<< "Marker2Drone : " << PRECISION(desPose.pose.position.x) <<'\t'
                                 << PRECISION(desPose.pose.position.y) << '\t'
                                 << PRECISION(desPose.pose.position.z) << endl;
}

int main(int argc, char **argv) {
        /* Display console */
        cout<< "______  __   __    ___     _____    _____ " << endl;
        cout<< "| ___ \\ \\ \\ / /   /   |   / ___ \\  | ___ \\" <<endl;
        cout<< "| |_/ /  \\ V /   / /| |   | | | |  | |_/ |" <<endl;
        cout<< "|  __/   /   \\  / /_| |   | | | |  |  __ /" <<endl;
        cout<< "| |     / /^\\ \\ \\___  |   | |_| |  | |_/ \\" <<endl;
        cout<< "\\_|     \\/   \\/     |_/   \\_____/  \\_____/\n\n" <<endl;
        cout << "-------Mode OFFBOARD CONTROL-------"<< endl;
        cout << "======================================="<< endl;
        cout << "\U0001F449 Control follow local position"<< endl;
        cout << "======================================="<< endl;

        /* Init position */
        cout << "\x1B[93m\u262D PLEASE ENTER INIT POSITION Local ENU frame [x y z] = \033[0m";
        cin  >> desPose.pose.position.x >> desPose.pose.position.y >> desPose.pose.position.z;
        cout << "\x1B[93mAxis x\033[0m : " << desPose.pose.position.x << "m" << endl;
        cout << "\x1B[93mAxis y\033[0m : " << desPose.pose.position.y << "m" << endl;
        cout << "\x1B[93mAxis z\033[0m : " << desPose.pose.position.z << "m" << endl;

        ros::init(argc, argv, "offboard_node");
        ros::NodeHandle nh;

        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");
        ros::Subscriber mavros_state_sub = nh.subscribe<mavros_msgs::State>
                ("mavros/state", 10, state_Callback);
        ros::Subscriber mavros_imu_data_sub = nh.subscribe<sensor_msgs::Imu>
                ("/mavros/imu/data", 10, imuPose_Callback);
        ros::Subscriber mavros_local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose", 10, mavrosPose_Callback);
        ros::Subscriber set_desposition_local_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("cmd/set_desposition/local", 10, getDestinatonPose_callback);
        ros::Subscriber set_next_status_sub = nh.subscribe<std_msgs::String>
                ("cmd/set_next_status/type", 10, getNextStatus_Callback);
        ros::Publisher mavros_setpoint_position_local_pub = nh.advertise<geometry_msgs::PoseStamped>
                ("mavros/setpoint_position/local", 10);
        ros::Publisher mavros_setpoint_velocity_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                ("mavros/setpoint_velocity/cmd_vel", 10);
        ros::Rate rate(20.0);

        /* wait for FCU connection */
        while(ros::ok() && !current_state.connected) {
                ros::spinOnce();
                rate.sleep();
        }

        mavros_msgs::SetMode offboard_set_mode, takeoff_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        offboard_set_mode.request.custom_mode = "OFFBOARD";
        takeoff_set_mode.request.custom_mode = "AUTO.LAND";
        arm_cmd.request.value = true;
        ros::Time last_time = ros::Time::now();

        // desPose.pose.orientation.x = 0;
        // desPose.pose.orientation.y = 0;
        // desPose.pose.orientation.z = 0.3826834;
        // desPose.pose.orientation.w = 0.9238795;

        while(ros::ok()) {
#ifdef HITL
                system("echo -n \"\e[4mWaiting for activation mode\e[0m\n\"");
                system("echo -n \"OFFBOARD && ARMED mode...\"");
                while(current_state.mode != "OFFBOARD" || !current_state.armed);
                system("echo -e \"\\r\033[0;32m\xE2\x9C\x94\033[0m OFFBOARD && ARMED ready!!!\"");
#else
                if(check_state) {
                        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_time > ros::Duration(5.0))) {
                                if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                                        ROS_INFO("Offboard enabled");
                                }
                                last_time = ros::Time::now();
                        } else {
                                if(!current_state.armed && (ros::Time::now() - last_time > ros::Duration(5.0))) {
                                        if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                                                ROS_INFO("Vehicle armed");
                                        }
                                        last_time = ros::Time::now();
                                }
                        }
                }
#endif /* HITL */
                if(strcmp(next_status, "LAND") == 0) {
                        last_time = ros::Time::now();

                        if(current_state.mode != "AUTO.LAND") {
                                takeoff_set_mode.request.custom_mode = "AUTO.LAND";
                                if(set_mode_client.call(takeoff_set_mode) && takeoff_set_mode.response.mode_sent) {
                                        ROS_INFO("AUTO LAND enabled");
                                        check_state = false;
                                }
                                last_time = ros::Time::now();
                        }
                }
              
                mavros_setpoint_position_local_pub.publish(desPose);

                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
