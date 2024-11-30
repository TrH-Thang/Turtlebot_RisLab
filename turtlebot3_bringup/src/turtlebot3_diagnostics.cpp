/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <string>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>


ros::Publisher tb3_diagnostics_pub;

diagnostic_msgs::DiagnosticStatus HOKUYO_state;
diagnostic_msgs::DiagnosticStatus T265_state;

bool isT265Connected = false;
bool isCamreraConnected = false;

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name  = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setHOKUYODiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&HOKUYO_state, level, "Lidar Sensor", message, "hokuyo_ust_20lx");
}

void HOKUYOMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setHOKUYODiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

// void setT265Diagnosis(uint8_t level, std::string message)
// {
//   setDiagnosisMsg(&T265_state, level, "Camera", message, "Intel Realsense");
// }

// void T265StateCallback(const std_msgs::String::ConstPtr &msg)
// {
//     using json = nlohmann::json;
//     std::string tmp = msg->data.c_str();
    
//     json parsedJson = json::parse(tmp);
    
//     if(parsedJson["t265_state"] == "1")
//     {
//         isT265Connected = true;
//     }
//     else {
//         isT265Connected = false;
//     }
// }

// void checkCameraConnection()
// {

//   isCamreraConnected = isT265Connected;

//   if(isCamreraConnected)
//   {
//     setT265Diagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
//   }
//   else if(!isCamreraConnected)
//   {
//     setT265Diagnosis(diagnostic_msgs::DiagnosticStatus::ERROR, "Bad Condition");
//   }
// }
  
void msgPub()
{
  diagnostic_msgs::DiagnosticArray tb3_diagnostics;

  tb3_diagnostics.header.stamp = ros::Time::now();

  tb3_diagnostics.status.clear();
  tb3_diagnostics.status.push_back(HOKUYO_state);
  // tb3_diagnostics.status.push_back(T265_state);

  tb3_diagnostics_pub.publish(tb3_diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_diagnostic");
  ros::NodeHandle nh;

  tb3_diagnostics_pub  = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

  ros::Subscriber hokuyo = nh.subscribe("scan", 10, HOKUYOMsgCallback);
  // ros::Subscriber t265_camera  = nh.subscribe("/t265_state", 10, T265StateCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    // if(!isCamreraConnected)
    // {
    //     checkCameraConnection();
    // }
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
