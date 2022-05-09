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

time_t time_now = time(0);
tm *ltime = localtime(&time_now);

static geometry_msgs::PoseStamped mavros_local_position_pose;
static geometry_msgs::PoseStamped desPose, mov2CurPose;
static Matrix3f imu_rotation_matrix;
static Vector3f drone_postition, NEU_postition, point_change, NEU_postition_change;
static bool locked_trans = true;
static bool accept_trans = true;
static bool marker_detected = false;
static int stable_loop = 0;
static bool locked_landing = false;
static bool locked_inc_altitude = false;
static bool accept_landing = false;
int detect_failed_repeat = 0;
float minutes = 0;
float seconds = 0;

ros::Time last_time_trans, stable_time_trans, time_increase_repeat_trans;

static double x_ = 0, y_ = 0, z_ = 0;

/**
 * @brief Get local position form mavros
 */
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

/**
 * @brief 
 * 
 * @param 
 *
 * @return 
 */
static void get_info_form_marker_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        /** Reset last transform time */
        last_time_trans = TIME_NOW;
        marker_detected = true;
        /**
         * The detection stabilizes after 3 sec.
         * detect_failed_repeat and locked_inc_altitude wil be reset.
         */
        if(TIME_NOW - stable_time_trans > TIME_DURATION(3.0)) {
                detect_failed_repeat = 0;
                locked_inc_altitude = false;
        }

        if(!locked_landing) {
                if(accept_trans) {
                        cout << "\x1B[93mStart transform\033[0m" <<endl;
                        accept_trans = false;
                }

                /* Marker ----> Drone */
                drone_postition[0] = msg->pose.position.x;
                drone_postition[1] = msg->pose.position.y;
                drone_postition[2] = msg->pose.position.z;

                /* Marker ----> NEU */
                NEU_postition = imu_rotation_matrix * drone_postition;

                if(!locked_trans) {
                        mov2CurPose.pose.position.x = NEU_postition[0] + mavros_local_position_pose.pose.position.x;
                        mov2CurPose.pose.position.y = NEU_postition[1] + mavros_local_position_pose.pose.position.y;
                        mov2CurPose.pose.position.z = NEU_postition[2] + mavros_local_position_pose.pose.position.z;

                        mov2CurPose.pose.position.x = PRECISION(mov2CurPose.pose.position.x);
                        mov2CurPose.pose.position.y = PRECISION(mov2CurPose.pose.position.y);
                        mov2CurPose.pose.position.z = PRECISION(mov2CurPose.pose.position.z);
                        mov2CurPose.pose.orientation.x = msg->pose.orientation.x;
                        mov2CurPose.pose.orientation.y = msg->pose.orientation.y;
                        mov2CurPose.pose.orientation.z = msg->pose.orientation.z;
                        mov2CurPose.pose.orientation.w = msg->pose.orientation.w;
                        locked_trans = true;
                }
                /**
                 *      A *---------------* B
                 *        |   \           |
                 *        |       \       |
                 *        |           \   |
                 *      D *---------------* C (UAV)
                 */
                /* Check error angle */
                float var_AC, var_alpha;
                static float curErr_alpha;
                var_AC = (float)sqrt(pow(drone_postition[0],2) + pow(drone_postition[1],2));
                var_alpha = atan(var_AC / abs(drone_postition[2])) * 180 / PI;

                if(mavros_local_position_pose.pose.position.z >= ALTITUDE_CHANGE_ANGLE) {
                        curErr_alpha = ERROR_ACCEPTANCE_DEGREES_20;
                }
                else {
                        curErr_alpha = ERROR_ACCEPTANCE_DEGREES_10;
                }
                
                if(var_alpha <= curErr_alpha && mavros_local_position_pose.pose.position.z > ALTITUDE_CHANGE_METHOD) {
                        desPose.pose.position.x = mov2CurPose.pose.position.x;
                        desPose.pose.position.y = mov2CurPose.pose.position.y;

                        if (mavros_local_position_pose.pose.position.z >= 0.8) {
                                if (desPose.pose.position.z <= 0.5) {
                                        desPose.pose.position.z = 0.5;
                                } else {
                                        desPose.pose.position.z = mavros_local_position_pose.pose.position.z - 2.0;
                                }
                        }
                        else {
                                accept_landing = true;
                        }
                }
                else if(mavros_local_position_pose.pose.position.z <= (ALTITUDE_CHANGE_METHOD + 0.2)) {
                        if(var_AC <= DISTANCE) {
                                desPose.pose.position.x = mov2CurPose.pose.position.x;
                                desPose.pose.position.y = mov2CurPose.pose.position.y;

                                if (mavros_local_position_pose.pose.position.z >= 0.8) {
                                        if (desPose.pose.position.z <= 0.5) {
                                                desPose.pose.position.z = 0.5;
                                        } else {
                                                desPose.pose.position.z = mavros_local_position_pose.pose.position.z - 2.0;
                                        }
                                }
                                else {
                                        accept_landing = true;
                                }
                        } else {
                                desPose.pose.position.x = mov2CurPose.pose.position.x;
                                desPose.pose.position.y = mov2CurPose.pose.position.y;
                                desPose.pose.position.z = mavros_local_position_pose.pose.position.z;
                                ROS_INFO("Aligning under 5m........!");
                        }
                }
                else {
                        desPose.pose.position.x = mov2CurPose.pose.position.x;
                        desPose.pose.position.y = mov2CurPose.pose.position.y;
                        desPose.pose.position.z = mavros_local_position_pose.pose.position.z;
                        ROS_INFO("Aligning........!");
                }


                desPose.pose.orientation.x = mov2CurPose.pose.orientation.x;
                desPose.pose.orientation.y = mov2CurPose.pose.orientation.y;
                desPose.pose.orientation.z = mov2CurPose.pose.orientation.z;
                desPose.pose.orientation.w = mov2CurPose.pose.orientation.w;
                /**
                 * stable param
                */
                stable_loop ++;
                if(stable_loop == NUM_LOOP) {
                        locked_trans = false;
                        stable_loop = 0;
                }

#ifdef LOG_INFO
                cout<< "Radius: " << var_AC << endl;
                cout<< "Current Alpha: " << var_alpha << endl;
                cout<< "Current Error Alpha: " << curErr_alpha << endl;
                cout<< "Marker2Drone : " << PRECISION(drone_postition[0]) <<'\t'
                                         << PRECISION(drone_postition[1]) << '\t'
                                         << PRECISION(drone_postition[2]) << endl;
                cout<< "Marker2NEU : " << mov2CurPose.pose.position.x <<'\t'<< mov2CurPose.pose.position.y << '\t' << mov2CurPose.pose.position.z << endl;
                cout<< "Drone : " << PRECISION(mavros_local_position_pose.pose.position.x) << "\t"
                                  << PRECISION(mavros_local_position_pose.pose.position.y) << "\t"
                                  << PRECISION(mavros_local_position_pose.pose.position.z) << endl;
                cout << "===================================================" << endl;
#endif /* LOG_INFO */
        }
}

/**
 * @brief
 * 
 * @param
 *
 * @return
 */
void get_current_setpoint_position_local_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        /**
         * Workaround issue haven't destination pose to 
         * public at time the init node.
         */
        if(!marker_detected) {
                desPose.pose.position.x= msg->pose.position.x;
                desPose.pose.position.y= msg->pose.position.y;
                desPose.pose.position.z= msg->pose.position.z;
        }
}

int main(int argc, char **argv) {
        ros::init(argc, argv, "transform_node");
        ros::NodeHandle transform;

        ros::Subscriber mavros_imu_data_sub = transform.subscribe<sensor_msgs::Imu>
                ("/mavros/imu/data", 10, imuPose_Callback);
        ros::Subscriber mavros_local_position_sub = transform.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose", 10, mavrosPose_Callback);
        ros::Subscriber marker_pose_sub = transform.subscribe<geometry_msgs::PoseStamped>
                ("marker/imu_position/pose", 10, get_info_form_marker_Callback);
        ros::Subscriber mavros_setpoint_position_local_sub = transform.subscribe<geometry_msgs::PoseStamped>
                ("mavros/setpoint_position/local", 10, get_current_setpoint_position_local_Callback);
        ros::Publisher set_desposition_local_pub = transform.advertise<geometry_msgs::PoseStamped>
                ("cmd/set_desposition/local", 10);
        ros::Publisher set_next_status_pub = transform.advertise<std_msgs::String>
                ("cmd/set_next_status/type", 10);
        ros::Rate rate(20.0);

        for(int i = 10; ros::ok() && i > 0; --i) {
                locked_trans = false;
                ros::spinOnce();
                rate.sleep();
        }

        last_time_trans = TIME_NOW;
        time_increase_repeat_trans = TIME_NOW;
        int timeout_trans = 2 * MAX_REPEAT_DETECT;

        while(ros::ok()) {
                /**
                 * If can't detect marker during 2 second.
                 * Get detect again failed more than maxium,
                 * Landing will be called.
                 */
                if(TIME_NOW - last_time_trans > TIME_DURATION(2.0)) {
                        static double new_altitude = 0;
                        if(TIME_NOW - time_increase_repeat_trans > TIME_DURATION(2.0)) {
                                detect_failed_repeat ++;
                                time_increase_repeat_trans = TIME_NOW;
                                cout << "Can't detect Marker" << endl;
                        }
                        if(!locked_inc_altitude) {
                                new_altitude = mavros_local_position_pose.pose.position.z + INCREASE_ALTITUDE_NOT_DETECT;
                        }
                        locked_inc_altitude = true;
                        stable_time_trans = TIME_NOW;
                        desPose.pose.position.x = mavros_local_position_pose.pose.position.x;
                        desPose.pose.position.y = mavros_local_position_pose.pose.position.y;
                        desPose.pose.position.z = new_altitude;
                        if((TIME_NOW - last_time_trans > TIME_DURATION(timeout_trans)) || detect_failed_repeat == MAX_REPEAT_DETECT) {
                                cout << "\x1B[31mDetection failed: Timeout\033[0m\t" <<endl;
                                accept_landing = true;
                        }
                }
                /**
                 * If the current pose greater than the maximum altitude,
                 * Landing mode should be enabled.
                 */
                if(MAX_ALTITUDE < mavros_local_position_pose.pose.position.z) {
                        accept_landing = true;
                        desPose.pose.position.z = MAX_ALTITUDE;
                        cout << "\x1B[31mEnable Landing: UAV is approaded the maximum altitude\033[0m\t" <<endl;
                }

                set_desposition_local_pub.publish(desPose);

                if (accept_landing) {
                        locked_landing = true;
                        time_now = time(0);
                        ltime = localtime(&time_now);
                        /* Public message to the status topic for the next status will be required */
                        std_msgs::String msg;
                        stringstream ss;
                        ss << "LAND";
                        msg.data = ss.str();
                        set_next_status_pub.publish(msg);

                        ROS_INFO("AUTO LANDING MODE is required");
                        cout << "\n\x1B[36m========================================\033[0m" << endl;
                        cout << "\x1B[34m|-----------Time AUTO LANDING----------|\033[0m" << endl;
                        cout << "\x1B[36m========================================\033[0m" << endl;
                        cout << "Start: " << minutes <<":"<< seconds;
                        cout << "\tEnd: " << ltime->tm_min <<":"<< ltime->tm_sec << endl;
                        cout << "\x1B[93mTotal: \033[0m" << ltime->tm_min - minutes <<" Min " << ltime->tm_sec - seconds << " Sec" << endl;
                        cout << "========================================"<< endl;
                        cout<< "Drone : x = " << mavros_local_position_pose.pose.position.x << \
                                      " y = " << mavros_local_position_pose.pose.position.y << \
                                      " z = " << mavros_local_position_pose.pose.position.z << endl;
                        exit(0);
                }
                ros::spinOnce();
                rate.sleep();
        }

        return 0;
}
