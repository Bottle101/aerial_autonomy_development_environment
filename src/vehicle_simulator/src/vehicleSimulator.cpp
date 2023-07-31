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

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

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

void controlHandler(const geometry_msgs::TwistStamped::ConstPtr& controlIn)
{
  vehicleRollCmd = controlIn->twist.linear.x;
  vehiclePitchCmd = controlIn->twist.linear.y;
  vehicleYawRate = controlIn->twist.angular.z;
  vehicleVelZG = controlIn->twist.linear.z;
}


void scanHandler(const sensor_msgs::PointCloud2::ConstPtr& scanIn)
{
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

  int scanDataSize = scanData->points.size();

  for (int i = 0; i < scanDataSize; i++)
  {
    float pointX1 = scanData->points[i].x  * cos(vehicleYaw) - scanData->points[i].y * sin(vehicleYaw);
    float pointY1 = scanData->points[i].x  * sin(vehicleYaw) + scanData->points[i].y * cos(vehicleYaw);
    float pointZ1 = scanData->points[i].z;

    // float pointX2 = pointX1 * cos(vehiclePitch) + pointZ1 * sin(vehiclePitch);
    // float pointY2 = pointY1;
    // float pointZ2 = - pointX1 * sin(vehiclePitch) + pointZ1 * cos(vehiclePitch);

    // pointX2 = pointX2;
    // pointY2 = pointY2 * cos(vehicleRoll) - pointZ2 * sin(vehicleRoll);
    // pointZ2 = pointY2 * sin(vehicleRoll) + pointZ2 * cos(vehicleRoll);

    // float pointX3 = pointX2 + vehicleX;
    // float pointY3 = pointY2 + vehicleY;
    // float pointZ3 = pointZ2 + vehicleZ;

    // float pointX3 = scanData->points[i].x + vehicleX;
    // float pointY3 = scanData->points[i].y + vehicleY;
    // float pointZ3 = scanData->points[i].z + vehicleZ;

    // float pointX2 = pointX1 * cos(vehiclePitch) - pointZ1 * sin(vehiclePitch);
    // float pointY2 = pointY1;
    // float pointZ2 = pointX1 * sin(vehiclePitch) + pointZ1 * cos(vehiclePitch);

    float pointX3 = pointX1 + vehicleX;
    float pointY3 = pointY1 + vehicleY;
    float pointZ3 = pointZ1 + vehicleZ;
    scanData->points[i].x = pointX3;
    scanData->points[i].y = pointY3;
    scanData->points[i].z = pointZ3;
    if (pointZ3 > 0.1) {
      scanData_tmp->push_back(scanData->points[i]);
    }
  }

  // publish 5Hz registered scan messages
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData_tmp, scanData2);
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

  ros::Subscriber subControl = nh.subscribe<geometry_msgs::TwistStamped> ("/attitude_control", 5, controlHandler);
  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, scanHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "vehicle";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "vehicle";

  ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 5);
  gazebo_msgs::ModelState cameraState;
  cameraState.model_name = "rgbd_camera";
  gazebo_msgs::ModelState lidarState;
  lidarState.model_name = "lidar";
  gazebo_msgs::ModelState robotState;
  robotState.model_name = "robot";

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);
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

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, sensorPitch + vehiclePitch, vehicleYaw);

    geometry_msgs::Quaternion geoQuat_lidar = tf::createQuaternionMsgFromRollPitchYaw(0, 0, vehicleYaw);

    // publish 200Hz Gazebo model state messages
    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ;
    pubModelState.publish(cameraState);
    
    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    pubModelState.publish(robotState);

    lidarState.pose.orientation = geoQuat_lidar;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    pubModelState.publish(lidarState);


    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
