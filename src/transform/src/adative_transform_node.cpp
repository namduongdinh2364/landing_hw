/**
 * Input pose of marker to drone
 * Output pose of marker to NEU
*/
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <math.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include "landing.h"

using namespace std;
using namespace Eigen;

static geometry_msgs::PoseStamped mavros_local_position_pose;
static geometry_msgs::PoseStamped desPose, mov2CurPose;
static Matrix3f imu_rotation_matrix, cam2drone_matrix;
static Vector3f drone_postition, cam_postition, marker2neu_postition, point_change, marker2neu_postition_change;

void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        mavros_local_position_pose = *msg;
}

void imuPose_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
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

static void get_info_form_marker_Callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
        /* Marker ----> Drone */
        cam_postition[0] = msg->transforms[0].transform.translation.x;
        cam_postition[1] = msg->transforms[0].transform.translation.y;
        cam_postition[2] = msg->transforms[0].transform.translation.z;
        cam2drone_matrix << 0.0 , -1.0 , 0.0 , -1.0 , 0.0 , 0.0 , 0.0 , 0.0 , -1.0;

        drone_postition = cam2drone_matrix * cam_postition;

        /* Marker ----> NEU */
        marker2neu_postition = imu_rotation_matrix * drone_postition;

        mov2CurPose.pose.position.x = marker2neu_postition[0] + mavros_local_position_pose.pose.position.x;
        mov2CurPose.pose.position.y = marker2neu_postition[1] + mavros_local_position_pose.pose.position.y;
        mov2CurPose.pose.position.z = marker2neu_postition[2] + mavros_local_position_pose.pose.position.z;

#ifdef LOG_INFO
                cout<< "Marker2Drone : " << PRECISION(drone_postition[0]) <<'\t'
                                         << PRECISION(drone_postition[1]) << '\t'
                                         << PRECISION(drone_postition[2]) << endl;
                cout<< "Marker2NEU : " << mov2CurPose.pose.position.x <<'\t'<< mov2CurPose.pose.position.y << '\t' << mov2CurPose.pose.position.z << endl;
                cout << "===================================================" << endl;
#endif /* LOG_INFO */

}

void get_current_setpoint_position_local_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "transform_node");
        ros::NodeHandle transform;

        ros::Subscriber mavros_imu_data_sub = transform.subscribe<sensor_msgs::Imu>
                ("/mavros/imu/data", 10, imuPose_Callback);
        ros::Subscriber mavros_local_position_sub = transform.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose", 10, mavrosPose_Callback);
        ros::Subscriber marker_pose_sub = transform.subscribe<tf2_msgs::TFMessage>
                ("/tf", 10, get_info_form_marker_Callback);
        ros::Rate rate(10.0);

        while(ros::ok()) {
                ros::spinOnce();
                rate.sleep();
        }

        return 0;
}
