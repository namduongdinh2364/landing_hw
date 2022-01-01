/**
 * 
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
static geometry_msgs::PoseStamped desPose;
static Matrix3f imu_rotation_matrix;
static bool locked_trans = true;
static bool accept_trans = true;
static bool marker_detected = false;
static int stable_loop = 0;
static bool locked_landing = false;
static bool accept_landing = false;
int detect_failed_repeat = 0;
int max_repeat_dectec = 5;
float minutes = 0;
float seconds = 0;

ros::Time last_time_trans, stable_time_trans, time_increase_repeat_trans;

static double x, y, z;
static double x_ = 0, y_ = 0, z_ = 0;
Matrix3f R, cam2imu_rotation, cam2imu_rotation_april;
Vector3f drone_postition, cam_position, NEU_postition, point_change, NEU_postition_change;

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
static void get_info_form_marker_Callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
        last_time_trans = TIME_NOW;
        marker_detected = true;
        float radius;

        if(TIME_NOW - stable_time_trans > TIME_DURATION(3.0)) {
                detect_failed_repeat = 0;
        }

        if(!locked_landing) {
                if(accept_trans) {
                        time_now = time(0);
                        ltime = localtime(&time_now);
                        cout << "Start transfor:" << ltime->tm_min << ":";
                        cout << ltime->tm_sec << endl;
                        minutes = ltime->tm_min;
                        seconds = ltime->tm_sec;
                        accept_trans = false;
                }

                double xq, yq, zq, wq;
                Quaternionf quat;
                Matrix3f R1;
                cam2imu_rotation << -0.0001, -1, 0, -1, 0, 0, -0.0001, 0, -1;
                /* Marker ----> camera */
                cam_position[0] = msg->transforms[0].transform.translation.x;
                cam_position[1] = msg->transforms[0].transform.translation.y;
                cam_position[2] = msg->transforms[0].transform.translation.z;

                xq = msg->transforms[0].transform.rotation.x;
                yq = msg->transforms[0].transform.rotation.y;
                zq = msg->transforms[0].transform.rotation.z;
                wq = msg->transforms[0].transform.rotation.w;

                /* Marker ----> Drone */
                drone_postition = cam2imu_rotation * cam_position;
                /* Marker ----> NEU */
                NEU_postition = imu_rotation_matrix * drone_postition;
                /* update the position */
                double alpha, OR, A1E, Y2, X2, Z2;
                OR = (float)sqrt(pow(drone_postition[0],2) + pow(drone_postition[1],2));
                alpha = atan (OR/drone_postition[2]) * 180 / PI;
                A1E = (OR *( abs(drone_postition[2]) - HeightChangeAngle )) / abs(drone_postition[2]);
                point_change[1] = (drone_postition[1] * A1E) / OR;
                point_change[0] = (drone_postition[0] * A1E) / OR;
                point_change[2] = drone_postition[2] + HeightChangeAngle;
                NEU_postition_change = R*point_change;
                if(!locked_trans) {
                        x = NEU_postition[0];
                        y = NEU_postition[1];
                        z = NEU_postition[2];
                        /* Convert float round 2 */
                        x = PRECISION(x);
                        y = PRECISION(y);
                        z = PRECISION(z);

                        x_ = NEU_postition_change[0]+mavros_local_position_pose.pose.position.x;
                        y_ = NEU_postition_change[1]+mavros_local_position_pose.pose.position.y;
                        z_ = NEU_postition_change[2]+mavros_local_position_pose.pose.position.z;
                        x_ = PRECISION(x_);
                        y_ = PRECISION(y_);
                        z_ = PRECISION(z_);
                }
                /* Used for control with pose */
                radius = (float)sqrt(pow(NEU_postition[0] - mavros_local_position_pose.pose.position.x,2) + pow(NEU_postition[1] - mavros_local_position_pose.pose.position.y,2));
                if(radius <= DISTANCE) {
                        desPose.pose.position.x = x;
                        desPose.pose.position.y = y;

                        if (mavros_local_position_pose.pose.position.z >= PRECISION(z) + 0.9) {
                                if (desPose.pose.position.z <= PRECISION(z) + 0.5) {
                                        desPose.pose.position.z = PRECISION(z) + 0.5;
                                } else {
                                        desPose.pose.position.z = mavros_local_position_pose.pose.position.z - 2.0;
                                }
                        } else {
                                accept_landing = true;
                        }
                } 

        // if (20 >= abs(alpha) && mavros_local_position_pose.pose.position.z > (HeightChangeAngle + 1)) {
        //     desPose.pose.position.x = x;
        //     desPose.pose.position.y = y;
        //     desPose.pose.position.z = HeightChangeAngle;
        // } else if (10 >= abs(alpha) && mavros_local_position_pose.pose.position.z <= (HeightChangeAngle + 1)) {
        //     desPose.pose.position.x = x;
        //     desPose.pose.position.y = y;

        //     if (mavros_local_position_pose.pose.position.z >= PRECISION(z) + 0.9) {
        //         if (desPose.pose.position.z <= PRECISION(z) + 0.5) {
        //             desPose.pose.position.z = PRECISION(z) + 0.5;
        //         } else {
        //             desPose.pose.position.z = mavros_local_position_pose.pose.position.z - 2.0;
        //         }
        //     } else {
        //         accept_landing = true;
        //     }
        // } 
                else {
                        desPose.pose.position.x = x;
                        desPose.pose.position.y = y;
                        desPose.pose.position.z = mavros_local_position_pose.pose.position.z;
                        ROS_INFO("Aligning........!");
                }

                desPose.pose.orientation.x = xq;
                desPose.pose.orientation.y = yq;
                desPose.pose.orientation.z = zq;
                desPose.pose.orientation.w = wq;
                locked_trans = true;

                /**
                 * stable param
                */
                stable_loop ++;
                if(stable_loop == 3) {
                        locked_trans = true;
                        stable_loop = 0;
                }

#ifdef LOG_INFO
                cout<< "Marker2Cam : " << PRECISION(cam_position[0]) << '\t'
                                       << PRECISION(cam_position[1]) << '\t'
                                       << PRECISION(cam_position[2]) << endl;
                cout<< "Marker2Drone : " << PRECISION(drone_postition[0]) <<'\t'
                                         << PRECISION(drone_postition[1]) << '\t'
                                         << PRECISION(drone_postition[2]) << endl;
                cout<< "Marker2NEU : " << x <<'\t'<< y << '\t' << z << endl;
                cout<< "Drone : " << PRECISION(mavros_local_position_pose.pose.position.x) << "\t"
                                  << PRECISION(mavros_local_position_pose.pose.position.y) << "\t"
                                  << PRECISION(mavros_local_position_pose.pose.position.z) << endl;
                cout << "===================================================" << endl;
#endif
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
        cout << "\x1B[93mYear\033[0m  : "<< 1900 + ltime->tm_year << endl;
        cout << "\x1B[93mMonth\033[0m : "<< 1 + ltime->tm_mon<< endl;
        cout << "\x1B[93mDay\033[0m   : "<< ltime->tm_mday << endl;
        cout << "\x1B[93mTime\033[0m  : "<< ltime->tm_hour << ":";
        cout << ltime->tm_min << ":";
        cout << ltime->tm_sec << endl;

        ros::init(argc, argv, "transform_node");
        ros::NodeHandle transform;

        ros::Subscriber mavros_imu_data_sub = transform.subscribe<sensor_msgs::Imu>
                ("/mavros/imu/data", 10, imuPose_Callback);
        ros::Subscriber mavros_local_position_sub = transform.subscribe<geometry_msgs::PoseStamped>
                ("/mavros/local_position/pose", 10, mavrosPose_Callback);
        ros::Subscriber pose_sub = transform.subscribe
                ("/tf_list", 10, get_info_form_marker_Callback);
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
        int timeout_trans = 2 * max_repeat_dectec;

        while(ros::ok()) {
                /**
                 * If can't detect marker during 2 second.
                 * Get detect again failed more than maxium,
                 * Landing will be called.
                 */
                if(TIME_NOW - last_time_trans > TIME_DURATION(2.0)) {
                        if(TIME_NOW - time_increase_repeat_trans > TIME_DURATION(2.0)) {
                                detect_failed_repeat ++;
                                time_increase_repeat_trans = TIME_NOW;
                                cout << "Can't detect Marker" << endl;
                        }
                        stable_time_trans = TIME_NOW;
                        desPose.pose.position.x = mavros_local_position_pose.pose.position.x;
                        desPose.pose.position.y = mavros_local_position_pose.pose.position.y;
                        desPose.pose.position.z = mavros_local_position_pose.pose.position.z + IncreaseHeightNotDetect;
                        if((TIME_NOW - last_time_trans > TIME_DURATION(timeout_trans)) || detect_failed_repeat == max_repeat_dectec) {
                                cout << "\x1B[31mDetection failed: Timeout\033[0m\t" <<endl;
                        }
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
#ifdef LOG_INFO
                        cout << "\n\x1B[36m========================================\033[0m" << endl;
                        cout << "\x1B[34m|-----------Time AUTO LANDING-----------\033[0m" << endl;
                        cout << "\x1B[36m========================================\033[0m" << endl;
                        cout << "| Start :" << minutes <<" : "<< seconds << endl;
                        cout << "| END   :" << ltime->tm_min <<" : "<< ltime->tm_sec << endl;
                        cout << "| Total :" << ltime->tm_min - minutes <<" Minute " << ltime->tm_sec - seconds << " Second" << endl;
                        cout << "========================================"<< endl;
                        cout<< "Drone : x = " << mavros_local_position_pose.pose.position.x << \
                                      " y = " << mavros_local_position_pose.pose.position.y << \
                                      " z = " << mavros_local_position_pose.pose.position.z << endl;
#endif
                        exit(-1);
                }
                ros::spinOnce();
                rate.sleep();
        }

        return 0;
}
