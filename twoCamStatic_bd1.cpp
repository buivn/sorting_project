/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


// ROS
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

// Grasp Candidates Generator
#include <gpg/cloud_camera.h>
#include <geometry_msgs/Pose.h>
//#include "kdl_conversions/kdl_msg.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

// Custom
#include "../../include/gpd/grasp_detector.h"
#include "../../include/gpd/sequential_importance_sampling.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGBA> cloudA;


int main(int argc, char* argv[]) 
{
  // initialize ROS
  ros::init(argc, argv, "twoCamStatic_bd1");
  ros::NodeHandle node("~");

  Eigen::Matrix3f camera_matrix;
  camera_matrix << 538.2075, 0.0, 318.3089, 0.0, 538.6995, 231.273, 0.0, 0.0, 1.0;
    

  // Set the position from which the camera sees the point cloud.
  Eigen::Matrix3Xd view_points(3,1);
  view_points << 0.0, 0.0, 0.0;
  std::vector<double> camera_position1;
  node.getParam("camera_position", camera_position1);
  view_points << camera_position1[0], camera_position1[1], camera_position1[2];

  // rotating angle - roll. pitch, yam
  float roll, pitch, yaw, distance_x, distance_y, distance_z;//-M_PI/5;
  node.getParam("roll", roll);
  node.getParam("pitch", pitch);
  node.getParam("yaw", yaw);

  node.getParam("twocam_dis_x", distance_x);
  node.getParam("twocam_dis_y", distance_y);
  node.getParam("twocam_dis_z", distance_z);
  // set up the transformation matrix
  Eigen::Affine3f tranMatrix = Eigen::Affine3f::Identity();
  // Define a translation of 0.45 meters on the x axis.
  tranMatrix.translation() << distance_x, distance_y, distance_z; //-0.40, 0.06, 0.0;
  // The same rotation matrix as before; theta radians around Y axis
  tranMatrix.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
  tranMatrix.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
  tranMatrix.rotate (Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));

  
  // Load point cloud from file.
  std::string filename1, filename2;
  node.param("cloud_filename1", filename1, std::string(""));
  node.param("cloud_filename2", filename2, std::string(""));
  
  // select the object to grasp
  std::vector<int> haipixel;
  //haipixel << 502, 270;
  //haipixel << 269, 320;
  //haipixel << 140, 325;
  //haipixel << 350, 360;
  node.getParam("selected_2Dpixel", haipixel);  
  
  Eigen::Vector2i selected_2Dpixel;
  selected_2Dpixel << haipixel[0], haipixel[1];
  // parameter used in planar filter function
  double basedObjectNumberFiltPara;
  node.param("based_object_number_filterPara", basedObjectNumberFiltPara, 0.2);
  
  CloudCamera cloud_cam(filename1, filename2, view_points, selected_2Dpixel, basedObjectNumberFiltPara, tranMatrix);

  if (cloud_cam.getCloudOriginal()->size() == 0)
  {
    std::cout << "Input point cloud is empty or does not exist!\n";
    return (-1);
  }
  // Detect grasp poses.
  bool use_importance_sampling;
  node.param("use_importance_sampling", use_importance_sampling, false);


  geometry_msgs::Pose handPose; 

  // this node publish a message on currentThetaValues topic
  ros::Publisher graspPub = node.advertise<geometry_msgs::Pose>("selectedGrasp", 1);
  ros::Rate rate(0.1);
  while (ros::ok())
  {
    GraspDetector detector(node);
    // Preprocess the point cloud (voxelize, workspace, etc.).
    detector.preprocessPointCloud(cloud_cam);

    // Detect grasps in the point cloud.
    std::vector<Grasp> grasps = detector.detectGrasps(cloud_cam);
    //Eigen::Vector3d surface_check;
    //6DOF handPose1;

    Eigen::Vector3d bottom;
    bottom = grasps[0].getGraspBottom();
    handPose.position.x = bottom(0);
    handPose.position.y = bottom(1);
    handPose.position.z = bottom(2);
    //handPose.orie_ = grasps[0].getFrame();

    Eigen::Vector3d angles; // -> the unit here is radian
    tf::Matrix3x3 matrix2;
    Eigen::Matrix3d matrix1;
    
    matrix1 = grasps[0].getFrame();
    //tf::transformEigenToTF(matrix1, matrix2);
    //tf::matrixEigenToTF(matrix1, matrix2);

    matrix2.setValue(matrix1(0,0),matrix1(0,1),matrix1(0,2),
      matrix1(1,0),matrix1(1,1),matrix1(1,2),matrix1(2,0),matrix1(2,1),matrix1(2,2));
    
    matrix2.getRPY(angles(0), angles(1), angles(2), 1);
    
    std::cout << "the frame after selected" << std::endl;
    std::cout << grasps[0].getFrame() << std::endl;

    std::cout << "Angles for each axis: \n"; // -> the unit here is radian
    std::cout << angles << std::endl;

    std::cout << "Position of the handPose: \n"; // -> the unit here is radian
    std::cout << "Position X = " << handPose.position.x << std::endl;
    std::cout << "Position Y = " << handPose.position.y << std::endl;
    std::cout << "Position Z = " << handPose.position.z << std::endl;

    graspPub.publish(handPose);
  

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
