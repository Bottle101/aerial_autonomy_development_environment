#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string sim_name = "unity";
string sensor_name = "lidar";

double realtimeFactor = 1.0;
double windCoeff = 0.05;
double maxRollPitchRate = 20.0;
double rollPitchSmoothRate = 0.1;
double sensorPitch = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 1.0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;
float vehicleVelXG = 0;
float vehicleVelYG = 0;
float vehicleVelZG = 0;

float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleRollCmd = 0;
float vehiclePitchCmd = 0;
float vehicleYawRate = 0;

const int systemDelay = 3;
int systemInitCount = 0;
bool systemInited = false;
ros::Publisher* pubScanPointer = NULL;
ros::Publisher* pubgndPointer = NULL;

std::vector<int> scanInd;
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData_tmp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr scanData_gnd(new pcl::PointCloud<pcl::PointXYZI>());

// pcl::PointCloud<pcl::PointXYZ>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZ>());
// pcl::PointCloud<pcl::PointXYZ>::Ptr scanData_tmp(new pcl::PointCloud<pcl::PointXYZ>());
// pcl::PointCloud<pcl::PointXYZ>::Ptr scanData_gnd(new pcl::PointCloud<pcl::PointXYZ>());

void controlHandler(const geometry_msgs::TwistStamped::ConstPtr& controlIn)
{
  vehicleRollCmd = controlIn->twist.linear.x;
  vehiclePitchCmd = controlIn->twist.linear.y;
  vehicleYawRate = controlIn->twist.angular.z;
  vehicleVelZG = controlIn->twist.linear.z;
}


void scanHandler(const sensor_msgs::PointCloud2::ConstPtr& scanIn)
{
  if (sim_name == "unity") {
    return;
  }

  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
    }
    return;
  }

  double scanTime = scanIn->header.stamp.toSec();

  scanData->clear();
  scanData_tmp->clear();
  pcl::fromROSMsg(*scanIn, *scanData);
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

  // ROS_WARN("scanHandler: %d", scanData->points.size());

  int scanDataSize = scanData->points.size();
  if (sensor_name == "lidar") {
    for (int i = 0; i < scanDataSize; i++)
    {
      float pointX1 = scanData->points[i].x  * cos(vehicleYaw) - scanData->points[i].y * sin(vehicleYaw);
      float pointY1 = scanData->points[i].x  * sin(vehicleYaw) + scanData->points[i].y * cos(vehicleYaw);
      float pointZ1 = scanData->points[i].z;

      float pointX3 = pointX1 + vehicleX;
      float pointY3 = pointY1 + vehicleY;
      float pointZ3 = pointZ1 + vehicleZ;
      scanData->points[i].x = pointX3;
      scanData->points[i].y = pointY3;
      scanData->points[i].z = pointZ3;

      // if (pointZ3 > 0.1) {
      scanData_tmp->push_back(scanData->points[i]);
      // }
    }
  } else {
    for (int i = 0; i < scanDataSize; i++)
    {
    float pointX1 = scanData->points[i].z;
    float pointY1 = - scanData->points[i].x;
    float pointZ1 = - scanData->points[i].y;

    float pointX2 = pointX1;
    float pointY2 = pointY1  * cos(vehicleRoll) - pointZ1 * sin(vehicleRoll);
    float pointZ2 = pointY1  * sin(vehicleRoll) + pointZ1 * cos(vehicleRoll);       

    float pointX3 = pointX2  * cos(vehiclePitch) + pointZ2 * sin(vehiclePitch);
    float pointY3 = pointY2;
    float pointZ3 = - pointX2  * sin(vehiclePitch) + pointZ2 * cos(vehiclePitch);

    float pointX4 = pointX3  * cos(vehicleYaw) - pointY3 * sin(vehicleYaw);
    float pointY4 = pointX3  * sin(vehicleYaw) + pointY3 * cos(vehicleYaw);
    float pointZ4 = pointZ3;

    float pointX5 = pointX4 + vehicleX;
    float pointY5 = pointY4 + vehicleY;
    float pointZ5 = pointZ4 + vehicleZ;
    scanData->points[i].x = pointX5;
    scanData->points[i].y = pointY5;
    scanData->points[i].z = pointZ5;
    }
  }


  // publish 5Hz registered scan messages
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  scanData2.header.stamp = ros::Time().now();
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);
  // scanData_tmp->clear();
  
  // publish 5Hz registered scan messages
  // sensor_msgs::PointCloud2 scanData3;
  // pcl::toROSMsg(*scanData_gnd, scanData3);
  // scanData3.header.stamp = ros::Time().now();
  // scanData3.header.frame_id = "map";
  // pubgndPointer->publish(scanData3);
  // scanData_gnd->clear();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicleSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("realtimeFactor", realtimeFactor);
  nhPrivate.getParam("windCoeff", windCoeff);
  nhPrivate.getParam("maxRollPitchRate", maxRollPitchRate);
  nhPrivate.getParam("rollPitchSmoothRate", rollPitchSmoothRate);
  nhPrivate.getParam("sensorPitch", sensorPitch);
  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  nhPrivate.getParam("vehicleYaw", vehicleYaw);
  nhPrivate.getParam("sim_name", sim_name);
  nhPrivate.getParam("sensor_name", sensor_name);


  ros::Subscriber subControl = nh.subscribe<geometry_msgs::TwistStamped> ("/attitude_control", 5, controlHandler);

  string scanTopic = "/velodyne_points";
  if (sensor_name != "lidar") {
    scanTopic = "/rgbd_camera/depth/points";
  }

  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>(scanTopic, 2, scanHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "vehicle";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "vehicle";

  ros::Publisher pubModelStateUnity = nh.advertise<geometry_msgs::PoseStamped>("/unity_sim/set_model_state", 5);
  geometry_msgs::PoseStamped robotStateUnity;
  robotStateUnity.header.frame_id = "map";

  ros::Publisher pubModelStateGazebo = nh.advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 5);
  gazebo_msgs::ModelState cameraState;
  cameraState.model_name = "rgbd_camera";
  gazebo_msgs::ModelState lidarState;
  lidarState.model_name = "lidar";
  gazebo_msgs::ModelState robotStateGazebo;
  robotStateGazebo.model_name = "robot";

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 5);
  pubScanPointer = &pubScan;
  ros::Publisher pubGND = nh.advertise<sensor_msgs::PointCloud2>("/terrain", 2);
  pubgndPointer = &pubGND;

  printf("\nSimulation started.\n\n");

  ros::Rate rate(200 * realtimeFactor);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;

    if (vehicleRollCmd - vehicleRoll > maxRollPitchRate / 200.0) vehicleRoll += maxRollPitchRate / 200.0;
    else if (vehicleRollCmd - vehicleRoll < -maxRollPitchRate / 200.0) vehicleRoll -= maxRollPitchRate / 200.0;
    else vehicleRoll = vehicleRollCmd;
    vehicleRoll = rollPitchSmoothRate * vehicleRoll + (1.0 - rollPitchSmoothRate) * vehicleRecRoll;

    if (vehiclePitchCmd - vehiclePitch > maxRollPitchRate / 200.0) vehiclePitch += maxRollPitchRate / 200.0;
    else if (vehiclePitchCmd - vehiclePitch < -maxRollPitchRate / 200.0) vehiclePitch -= maxRollPitchRate / 200.0;
    else vehiclePitch = vehiclePitchCmd;
    vehiclePitch = rollPitchSmoothRate * vehiclePitch + (1.0 - rollPitchSmoothRate) * vehicleRecPitch;

    float vehicleAccX = 9.8 * tan(vehiclePitch);
    float vehicleAccY = -9.8 * tan(vehicleRoll) / cos(vehiclePitch);

    if (vehicleVelXG < 0) vehicleVelXG += windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    else vehicleVelXG -= windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    if (vehicleVelYG < 0) vehicleVelYG += windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;
    else vehicleVelYG -= windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;

    vehicleVelXG += (vehicleAccX * cos(vehicleYaw) - vehicleAccY * sin(vehicleYaw)) / 200.0;
    vehicleVelYG += (vehicleAccX * sin(vehicleYaw) + vehicleAccY * cos(vehicleYaw)) / 200.0;

    float velX1 = vehicleVelXG * cos(vehicleYaw) + vehicleVelYG * sin(vehicleYaw);
    float velY1 = -vehicleVelXG * sin(vehicleYaw) + vehicleVelYG * cos(vehicleYaw);
    float velZ1 = vehicleVelZG;

    float velX2 = velX1 * cos(vehiclePitch) - velZ1 * sin(vehiclePitch);
    float velY2 = velY1;
    float velZ2 = velX1 * sin(vehiclePitch) + velZ1 * cos(vehiclePitch);

    vehicleVelX = velX2;
    vehicleVelY = velY2 * cos(vehicleRoll) + velZ2 * sin(vehicleRoll);
    vehicleVelZ = -velY2 * sin(vehicleRoll) + velZ2 * cos(vehicleRoll);

    vehicleX += vehicleVelXG / 200.0;
    vehicleY += vehicleVelYG / 200.0;
    vehicleZ += vehicleVelZG / 200.0;
    vehicleYaw += vehicleYawRate / 200.0;

    ros::Time timeNow = ros::Time::now();

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    // publish 200Hz odometry messages
    odomData.header.stamp = timeNow;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleVelX;
    odomData.twist.twist.linear.y = vehicleVelY;
    odomData.twist.twist.linear.z = vehicleVelZ;
    pubVehicleOdom.publish(odomData);

    odomTrans.stamp_ = timeNow;
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
    tfBroadcaster.sendTransform(odomTrans);

    // robotState.header.stamp = timeNow;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, sensorPitch + vehiclePitch, vehicleYaw);

    geometry_msgs::Quaternion geoQuat_lidar = tf::createQuaternionMsgFromRollPitchYaw(0, 0, vehicleYaw);

    if (sim_name == "unity") {
      robotStateUnity.pose.orientation = geoQuat;
      robotStateUnity.pose.position.x = vehicleX;
      robotStateUnity.pose.position.y = vehicleY;
      robotStateUnity.pose.position.z = vehicleZ;
      pubModelStateUnity.publish(robotStateUnity);      
    } else {
      // publish 200Hz Gazebo model state messages
      cameraState.pose.orientation = geoQuat;
      cameraState.pose.position.x = vehicleX;
      cameraState.pose.position.y = vehicleY;
      cameraState.pose.position.z = vehicleZ;
      pubModelStateGazebo.publish(cameraState);
      
      robotStateGazebo.pose.orientation = geoQuat;
      robotStateGazebo.pose.position.x = vehicleX;
      robotStateGazebo.pose.position.y = vehicleY;
      robotStateGazebo.pose.position.z = vehicleZ;
      pubModelStateGazebo.publish(robotStateGazebo);

      lidarState.pose.orientation = geoQuat_lidar;
      lidarState.pose.position.x = vehicleX;
      lidarState.pose.position.y = vehicleY;
      lidarState.pose.position.z = vehicleZ;
      pubModelStateGazebo.publish(lidarState);      
    }


    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
