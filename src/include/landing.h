/**
 * 
*/
#ifndef LANDING_H
#define LANDING_H

#include <ros/ros.h>

#define POINT   1
#define PID     2

#define PRECISION(x)    round(x * 100) / 100
#define CHECK_M         (CHECK_MARK="\033[0;32m\xE2\x9C\x94\033[0m")
#define PRECISION(x)    round(x * 100) / 100
#define PI              3.14159265
#define YAW_ANGLE(x)    (x * (180 / PI))
#define RATODE(x)       (x / (180 / PI))

#define DISTANCE        0.3
#define NSTEP   4
#define HeightChangeAngle       7
#define IncreaseHeightNotDetect 1
#define MaxRepeatDetections     2
#define TIME_NOW                ros::Time::now()
#define TIME_DURATION(x)        ros::Duration(x)
#define LOG_INFO                1


#endif /* LANDING_H */
