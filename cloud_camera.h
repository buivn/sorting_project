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


#ifndef CLOUD_CAMERA_H_
#define CLOUD_CAMERA_H_

#include <algorithm>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include <Eigen/Dense>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include <stdio.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>

#include <string>
//#include <pcl_conversions/pcl_conversions.h>





#include <gpg/eigen_utils.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

//bui's adding
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;

/** CloudCamera class
 *
 * \brief A point cloud with camera sources and surface normals
 * 
 * This class stores a point cloud with camera sources and surface normals for each point in the point cloud and the
 * view point of the camera from which the cloud was observed.
 * 
*/
class CloudCamera
{
public:

  /**
   * \brief Comparator for checking uniqueness of two 3D-vectors.
  */
  struct UniqueVectorComparator
  {
    /**
     * \brief Compares two 3D-vectors for uniqueness.
     * \param a the first 3D-vector
     * \param b the second 3D-vector
     * \return true if they differ in at least one element, false if all elements are equal
    */
    bool operator ()(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
    {
      for (int i = 0; i < a.size(); i++)
      {
        if (a(i) != b(i))
        {
          return a(i) < b(i);
        }
      }

      return false;
    }
  };

  /**
   * \brief Comparator for checking uniqueness of two 4D-vectors.
  */
  struct UniqueVectorFirstThreeElementsComparator
  {
    /**
     * \brief Compares two 4D-vectors for uniqueness (ignores the last element).
     * \param a the first 4D-vector
     * \param b the second 4D-vector
     * \return true if they differ in at least one of the first three elements, false otherwise
    */
    bool operator ()(const Eigen::Vector4i& a, const Eigen::Vector4i& b)
    {
      for (int i = 0; i < a.size() - 1; i++)
      {
        if (a(i) != b(i))
        {
          return true;
        }
      }

      return false;
    }
  };

  /**
   * \brief Constructor.
   */
  CloudCamera();

  /**
   * \brief Constructor.
   * \param cloud the point cloud (of size n)
   * \param camera_source the camera source for each point in the cloud (size: k x n)
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const PointCloudRGB::Ptr& cloud, const Eigen::MatrixXi& camera_source,
    const Eigen::Matrix3Xd& view_points);

  /**
   * \brief Constructor.
   * \param cloud the point cloud with surface normals (of size n)
   * \param camera_source the camera source for each point in the cloud (size: k x n)
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const PointCloudPointNormal::Ptr& cloud, const Eigen::MatrixXi& camera_source,
    const Eigen::Matrix3Xd& view_points);

  /**
   * \brief Constructor for a two camera setup (left and right camera).
   * \param cloud the point cloud (of size n)
   * \param size_left_cloud the size of the point cloud from the left camera
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const PointCloudRGB::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points);

  /**
   * \brief Bui's constructor for a two cameras setup (left and right camera) = actually only one camera
   * \param cloud the point cloud (of size n)
   * \param size_left_cloud the size of the point cloud from the left camera
   * \param view_points the origins of the cameras (size: 3 x k)
   * \param c2Dpixel is the centroid in 2D receive from RGB image procesing
   */
  CloudCamera(const PointCloudRGB::Ptr& cloud, Eigen::Vector2i c2Dpixel, int size_left_cloud, const Eigen::Matrix3Xd& view_points, double PlanFiltPara); 

    /**
   * \brief Constructor for a two cameras setup (left and right camera).
   * \param cloud the point cloud with surface normals (of size n)
   * \param size_left_cloud the size of the point cloud from the left camera
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const PointCloudPointNormal::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points);

  /**
   * \brief Constructor for point cloud files (*.pcd).
   * \param filename the location of the point cloud file
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const std::string& filename, const Eigen::Matrix3Xd& view_points);

  /**
   * \brief bui's Constructor for point cloud files (*.pcd).
   * \param set the selected2Dpixel
   * \param filename the location of the point cloud file
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const std::string& filename, const Eigen::Matrix3Xd& view_points, Eigen::Vector2i s2Dpixel, double PlanFiltPara);
  
  /**
   * \brief Constructor for point cloud files (*.pcd) from a two cameras setup.
   * \param filename_left the location of the point cloud file from the first camera
   * \param filename_right the location of the point cloud file from the second camera
   * \param view_points the origins of the cameras (size: 3 x k)
   */
  CloudCamera(const std::string& filename_left, const std::string& filename_right,
    const Eigen::Matrix3Xd& view_points);

  /**
   * \brief Filter out points in the point cloud that lie outside the workspace dimensions.
   * \param[in] workspace a 6-D vector containing the workspace limits: [minX, maxX, minY, maxY, minZ, maxZ]
  */
  void filterWorkspace(const std::vector<double>& workspace);

  /**
   * \brief Filter out samples that lie outside the workspace dimensions.
   * \param[in] workspace a 6-D vector containing the workspace limits: [minX, maxX, minY, maxY, minZ, maxZ]
  */
  void filterSamples(const std::vector<double>& workspace);

  /**
   * \brief Voxelize the point cloud and keep track of the camera source for each voxel.
   * \param[in] cell_size the size of each voxel
  */
  void voxelizeCloud(double cell_size);

  /**
   * \brief Subsample the point cloud according to the uniform distribution.
   * \param[in] num_samples the number of samples to draw from the point cloud
  */
  void subsampleUniformly(int num_samples);

  /**
   * \brief Subsample the samples according to the uniform distribution.
   * \param[in] num_samples the number of samples to draw from the samples
  */
  void subsampleSamples(int num_samples);

  /**
   * \brief Calculate surface normals for the point cloud.
   * \param[in] num_threads the number of CPU threads to be used in the calculation
  */
  void calculateNormals(int num_threads);

  /**
   * \brief Calculate surface normals for an organized point cloud.
   */
  void calculateNormalsOrganized();

  /**
   * \brief Calculate surface normals for an unorganized point cloud.
   * \param[in] num_threads the number of CPU threads to be used in the calculation
  */
  void calculateNormalsOMP(int num_threads);

  /**
   * \brief Reverse direction of surface normals (if a normal does not point to at least one camera).
   */
  void reverseNormals();

  /**
   * \brief Set surface normals from a file.
   * \param[in] filename the location of the file
  */
  void setNormalsFromFile(const std::string& filename);

  /**
   * \brief Write surface normals to a file.
   * \param filename the location of the file
   * \param normals the surface normals
  */
  void writeNormalsToFile(const std::string& filename, const Eigen::Matrix3Xd& normals);

  /**
   * \brief Return the camera source matrix.
   * \return the camera source matrix (k x n)
  */
  const Eigen::MatrixXi& getCameraSource() const
  {
    return camera_source_;
  }

  /**
   * \brief Return the preprocessed point cloud.
   * \return the point cloud
  */
  const PointCloudRGB::Ptr& getCloudProcessed() const
  {
    return cloud_processed_;
  }

  /**
   * \brief Return the original point cloud.
   * \return the point cloud
  */
  const PointCloudRGB::Ptr& getCloudOriginal() const
  {
    return cloud_original_;
  }

  /**
   * \brief Return the original point cloud.
   * \return the point cloud
  */
  const std::vector<int>& getSampleIndices() const
  {
    return sample_indices_;
  }

  /**
   * \brief Return the surface normals.
   * \return the surface normals (size: 3 x n)
  */
  const Eigen::Matrix3Xd& getNormals() const
  {
    return normals_;
  }

  /**
   * \brief Return the samples.
   * \return the samples (size: 3 x n)
  */
  const Eigen::Matrix3Xd& getSamples() const
  {
    return samples_;
  }

  /**
   * \brief Return the sample indices.
   * \return the sample indices (size: n)
  */
  void setSampleIndices(const std::vector<int>& sampleIndices)
  {
    sample_indices_ = sampleIndices;
  }

  /**
   * \brief Set the samples.
   * \return the samples (size: 3 x n)
  */
  void setSamples(const Eigen::Matrix3Xd& samples);

  /**
   * \brief Set the surface normals.
   * \return the surface normals (size: 3 x n)
  */
  void setNormals (const Eigen::Matrix3Xd& normals)
  {
    normals_ = normals;
  }

  /**
   * \brief Get the view points of the cameras.
   * \return the origins of the cameras (size: 3 x k)
  */
  const Eigen::Matrix3Xd& getViewPoints() const
  {
    return view_points_;
  }

  /**
   * \brief Set the camera view points.
   * \return the origins of the cameras (size: 3 x k)
  */
  void setViewPoints(const Eigen::Matrix3Xd& view_points)
  {
    view_points_ = view_points;
  }

  // bui'adding
  //Eigen::MatrixXf object_centroid(CloudRGB::Ptr cloud) const;
  // get the pointindices for each separate point cloud
  std::vector<pcl::PointIndices> extractObjects(CloudRGB::Ptr cloud) const;
  // return the selected object which matched selected 2D pixel
  CloudRGB::Ptr selectObject(CloudRGB::Ptr cloud, std::vector<pcl::PointIndices> cluster_indices, Eigen::Vector2i grasp_pos);
  // convert centroid3D to centroid2D pixel
  Eigen::Vector2i point3Dto_pixel(Eigen::Vector4f pc_coord, Eigen::Matrix3f cmatrix);
  // calculate the centroid 2-3D of point cloud cluster
  Eigen::MatrixXi calculateCentroid2D(CloudRGB::Ptr cloud, std::vector<pcl::PointIndices> cluster_indices);


  // bui'adding
  Eigen::MatrixXf getCentroid3D() const
  {
  	return centroid3D_;
  }
  // bui'adding  
  int getnCluster() const
  {
	return numberOfCluster_;
  }
  void setCentroid3D(const Eigen::MatrixXf& centroid3D)
  {	
	 centroid3D_ = centroid3D;
	 numberOfCluster_ = centroid3D.size()/4;
  }
  int getNOCluster() const
  {
	return numberOfCluster_;
  }

  void setSelected2Dpixel(const Eigen::Vector2i selected2Dpixel)
  {	
	selected2Dpixel_ = selected2Dpixel;
  }
  
  Eigen::Vector2i getSelected2Dpixel() const
  {
	return selected2Dpixel_;
  }
  // manage the Planar filter parameter - based on object number
  void setPlanarFilterPara(const double objectNumber_filtPara)
  {
	basedObjectNumber_filtPara_ = objectNumber_filtPara;
  }

  double getPlanarFilterPara() const
  {
	return basedObjectNumber_filtPara_;
  }
  


private:

  /**
   * \brief Load a point cloud from a file.
   * \param filename the location of the file
   * \return the point cloud
  */
  PointCloudRGB::Ptr loadPointCloudFromFile(const std::string& filename);
  // bui's adding
  PointCloudRGB::Ptr buiLoadPClFromFile(const std::string& filename, Eigen::Vector2i s2Dpixel);
  // filter the pointcloud as an input  
  PointCloudRGB::Ptr buiFilterPClTopic(const PointCloudRGB::Ptr& cloud, Eigen::Vector2i c2Dpixel);
  CloudRGB::Ptr passthrough_filter(const CloudRGB::Ptr cloud) const;
  CloudRGB::Ptr downsampling(const CloudRGB::Ptr cloud) const;
  CloudRGB::Ptr planar_filter(CloudRGB::Ptr cloud) const;

  PointCloudRGB::Ptr cloud_processed_; ///< the (processed) point cloud
  PointCloudRGB::Ptr cloud_original_; ///< the original point cloud
  Eigen::MatrixXi camera_source_; ///< binary matrix: (i,j) = 1 if point j is seen by camera i
  Eigen::Matrix3Xd normals_; ///< the surface normal for each point in the point cloud
  std::vector<int> sample_indices_; ///< the indices into the point cloud used to sample grasp hypotheses
  Eigen::Matrix3Xd samples_; ///< the samples used for finding grasp hypotheses
  Eigen::Matrix3Xd view_points_; ///< the viewpoints of the camera on the cloud
  // bui's adding
  Eigen::MatrixXf centroid3D_;  // setd centroid3D of each cluster
  int numberOfCluster_;
  Eigen::Vector2i selected2Dpixel_;
  double basedObjectNumber_filtPara_;
  

};

#endif /* CLOUD_CAMERA_H_ */
