

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
#include <sensor_msgs/PointCloud2.h>
//#include "kdl_conversions/kdl_msg.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom
#include "../../include/gpd/grasp_detector.h"
#include "../../include/gpd/sequential_importance_sampling.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> cloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> cloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGBA> cloudA;

bool has_cloud_ = false;
CloudCamera* cloud_camera_;
Eigen::Matrix3Xd view_points(3,1);
Eigen::Vector2i selected_2Dpixel;

bool doOneTime = true;
// parameter used in planar filter function
double basedObjectNumberFiltPara;

void cloud_callback(const sensor_msgs::PointCloud2& msg)
{
  if ((!has_cloud_)&(doOneTime))
  {
    cloudA::Ptr cloud(new cloudA);
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp4(new pcl::PointCloud<pcl::PointXYZ>); 

    pcl::fromROSMsg(msg, *cloud);
    cloud_camera_ = new CloudCamera(cloud, selected_2Dpixel, 0, view_points, basedObjectNumberFiltPara); // bui's constructor
    //cloud_camera_->setPlanarFilterPara(object_number_filtPara);
    //std::cout << "The value of the filter parameter is in callback function: " << object_number_filtPara << std::endl;


    //cloud_camera_header_ = msg.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");

    has_cloud_ = true;
    doOneTime = false;
    //frame_ = msg.header.frame_id;
  }
}

int main(int argc, char* argv[]) 
{
  // initialize ROS
  ros::init(argc, argv, "classify_grasp_candidates_bd3");
  ros::NodeHandle node("~");

  Eigen::Matrix3f camera_matrix;
  camera_matrix << 538.2075, 0.0, 318.3089, 0.0, 538.6995, 231.273, 0.0, 0.0, 1.0;
    

  // Set the position from which the camera sees the point cloud.
  //view_points << 0.0, 0.0, 0.0;
  std::vector<double> camera_position1;
  node.getParam("camera_position", camera_position1);
  view_points << camera_position1[0], camera_position1[1], camera_position1[2];
  
  // select the object to grasp
  std::vector<int> haipixel;
  node.getParam("selected_2Dpixel", haipixel);  
  selected_2Dpixel << haipixel[0], haipixel[1];
  
  // parameter used in planar filter function
  //double object_number_filtPara;
  node.param("based_object_number_filterPara", basedObjectNumberFiltPara, 0.2);
  
  std::string cloud_topic;
  node.param("cloud_topic", cloud_topic, std::string("/camera_remote/depth_registered/points"));
  ros::Subscriber cloud_sub_ = node.subscribe(cloud_topic, 1, cloud_callback);

  geometry_msgs::Pose handPose; 

  // this node publish a message on currentThetaValues topic
  ros::Publisher graspPub = node.advertise<geometry_msgs::Pose>("selectedGrasp", 1);
  ros::Rate rate(0.1);
  
  while (ros::ok())
  {
    if (has_cloud_)
    {
      GraspDetector detector(node);

      // Preprocess the point cloud (voxelize, workspace, etc.).
      detector.preprocessPointCloud(*cloud_camera_);

      // Detect grasps in the point cloud.
      std::vector<Grasp> grasps = detector.detectGrasps(*cloud_camera_);

      Eigen::Vector3d bottom;
      Eigen::Vector3d angles; // -> the unit here is radian
      tf::Matrix3x3 matrix2;
      Eigen::Matrix3d matrix1;
      int bestGraspNumber = 0;
      double goodPitch = 0;
      double goodYaw = 0;
      double goodY = 0;
      int numberofgoodGrasp = 0;

      // the loop to find the best pose the Auto I5 robot
      for (int i = 0; i< grasps.size(); ++i)
      {
          angles(0) = 0;
          angles(1) = 0;
          angles(2) = 0;
          matrix1 = grasps[i].getFrame();
          //bottom = grasps[i].getGraspBottom();
          bottom = grasps[i].getGraspBottom();
          // handPose.position.x = bottom(0);
          // handPose.position.y = bottom(1);
          // handPose.position.z = bottom(2);


          matrix2.setValue(matrix1(0,0),matrix1(0,1),matrix1(0,2),
          matrix1(1,0),matrix1(1,1),matrix1(1,2),matrix1(2,0),matrix1(2,1),matrix1(2,2));
          matrix2.getRPY(angles(0), angles(1), angles(2), 1);

          std::cout << "\nPosition X Y Z and Angles Yaw Pitch Roll HandPose " << i << std::endl; // -> the unit here is radian
          std::cout << bottom(0) <<" " << bottom(1) <<" " << bottom(2) << std::endl;
          
          //std::cout << "Angles for each axis  " << i << std::endl; // -> the unit here is radian
          std::cout << angles(2) <<" " <<  angles(1) <<" " << angles(0) << std::endl;


          

          // if (i == 0)
          //   goodY = bottom(1);

          
          // object 1 and 2
          // goodPitch = angles(1);
          // goodYaw = angles(2);
          // if ((goodPitch < -0.3) and (goodPitch > -0.9))
          //     if (goodYaw < -2.0)
          //         if (angles(0) > 0)
          //             if (goodY >= bottom(1))
          //                 bestGraspNumber = i;
          //                 std::cout << "The Values of Y = " << bottom(1) << std::endl;
          //                 goodY = bottom(1);
          //             ++numberofgoodGrasp;  // check the heigh condition


          // object 3 and 4
          // if ((angles(0) < 0) and (angles(2) > 0) and (angles(1) < 0))
          //     if (goodY >= bottom(1))
          //         bestGraspNumber = i;
          //         std::cout << "The Values of Y = " << bottom(1) << std::endl;
          //         goodY = bottom(1);
          //     ++numberofgoodGrasp;  // check the heigh condition
      }

      //std::cout << "The number of good grasp: " << numberofgoodGrasp << std::endl;
      // reset to Zero
      bottom.setZero(); 
      angles.setZero(); // -> the unit here is radian
      //tf::Matrix3x3 matrix2;
      matrix1.setZero();

//       bottom = grasps[bestGraspNumber].getGraspBottom();
// //      bottom = grasps[0].getGraspBottom();
//       handPose.position.x = bottom(0);
//       handPose.position.y = bottom(1);
//       handPose.position.z = bottom(2);

      // matrix1 = grasps[bestGraspNumber].getFrame();
      // //matrix1 = grasps[0].getFrame();
      // matrix2.setValue(matrix1(0,0),matrix1(0,1),matrix1(0,2),
      // matrix1(1,0),matrix1(1,1),matrix1(1,2),matrix1(2,0),matrix1(2,1),matrix1(2,2));
      // matrix2.getRPY(angles(0), angles(1), angles(2), 1);
      

      //tf::transformEigenToTF(matrix1, matrix2);
      //std::cout << "\nGrasp Pose " << bestGraspNumber << " is the most appropriate one:" << std::endl;
      //std::cout << grasps[0].getFrame() << std::endl;
      //std::cout << matrix1 << std::endl;


      // matrix2.setValue(matrix1(0,0),matrix1(0,1),matrix1(0,2),
      //   matrix1(1,0),matrix1(1,1),matrix1(1,2),matrix1(2,0),matrix1(2,1),matrix1(2,2));
      
      // matrix2.getRPY(angles(0), angles(1), angles(2), 1);

      // std::cout << "Angles for each axis: \n"; // -> the unit here is radian
      // std::cout << angles << std::endl;

      // std::cout << "Position of the handPose: \n"; // -> the unit here is radian
      // std::cout << "Position X = " << handPose.position.x << std::endl;
      // std::cout << "Position Y = " << handPose.position.y << std::endl;
      // std::cout << "Position Z = " << handPose.position.z << std::endl;

      graspPub.publish(handPose);
      has_cloud_ = false;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
