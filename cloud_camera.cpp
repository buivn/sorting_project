#include <gpg/cloud_camera.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree.h>


CloudCamera::CloudCamera()
: cloud_original_(new PointCloudRGB), cloud_processed_(new PointCloudRGB)
{
  view_points_.resize(3,1);
  view_points_ << 0.0, 0.0, 0.0;
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  //centroid3D_.resize(0,4);
}

// bui's comments; this could be the second constructor to use
CloudCamera::CloudCamera(const PointCloudRGB::Ptr& cloud, const Eigen::MatrixXi& camera_source,
  const Eigen::Matrix3Xd& view_points) : cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB),
    camera_source_(camera_source), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  //centroid3D_.resize(0,4);

  pcl::copyPointCloud(*cloud, *cloud_original_);
  *cloud_processed_ = *cloud_original_;
}


CloudCamera::CloudCamera(const PointCloudPointNormal::Ptr& cloud, const Eigen::MatrixXi& camera_source,
  const Eigen::Matrix3Xd& view_points) : cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB),
    camera_source_(camera_source), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  //centroid3D_.resize(0,4);

  pcl::copyPointCloud(*cloud, *cloud_original_);
  *cloud_processed_ = *cloud_original_;
}


CloudCamera::CloudCamera(const PointCloudPointNormal::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points)
: cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  numberOfCluster_ = 0;
  //centroid3D_.resize(0,4);

  pcl::copyPointCloud(*cloud, *cloud_original_);
  *cloud_processed_ = *cloud_original_;

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  if (size_left_cloud == 0) // one camera
  {
    camera_source_ = Eigen::MatrixXi::Ones(1, cloud->size());
  }
  else // two cameras
  {
    int size_right_cloud = cloud->size() - size_left_cloud;
    camera_source_ = Eigen::MatrixXi::Zero(2, cloud->size());
    camera_source_.block(0,0,1,size_left_cloud) = Eigen::MatrixXi::Ones(1, size_left_cloud);
    camera_source_.block(1,size_left_cloud,1,size_right_cloud) = Eigen::MatrixXi::Ones(1, size_right_cloud);
  }

  normals_.resize(3, cloud->size());
  for (int i = 0; i < cloud->size(); i++)
  {
    normals_.col(i) << cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z;
  }
}



// Bui's comment: this constructor can be used for the project  // just processing something here
CloudCamera::CloudCamera(const PointCloudRGB::Ptr& cloud, int size_left_cloud, const Eigen::Matrix3Xd& view_points)
: cloud_processed_(cloud), cloud_original_(cloud), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  //centroid3D_.resize(0,4);

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  if (size_left_cloud == 0) // one camera
  {
    camera_source_ = Eigen::MatrixXi::Ones(1, cloud->size());
  }
  else // two cameras
  {
    int size_right_cloud = cloud->size() - size_left_cloud;
    camera_source_ = Eigen::MatrixXi::Zero(2, cloud->size());
    camera_source_.block(0,0,1,size_left_cloud) = Eigen::MatrixXi::Ones(1, size_left_cloud);
    camera_source_.block(1,size_left_cloud,1,size_right_cloud) = Eigen::MatrixXi::Ones(1, size_right_cloud);
  }
}

// Bui's constructor: used to filter the wall and table to process the stream of Sensor topic
CloudCamera::CloudCamera(const PointCloudRGB::Ptr& cloud, Eigen::Vector2i c2Dpixel, int size_left_cloud, 
	const Eigen::Matrix3Xd& view_points, double PlanFiltPara): cloud_processed_(new PointCloudRGB), 
  cloud_original_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  setPlanarFilterPara(PlanFiltPara);
  //centroid3D_.resize(0,4);

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  if (size_left_cloud == 0) // one camera
  {
    camera_source_ = Eigen::MatrixXi::Ones(1, cloud->size());
    cloud_processed_ = buiFilterPClTopic(cloud, c2Dpixel);
    //cloud_processed_ = buiLoadPClFromFile(filename, s2Dpixel);
    cloud_original_ = cloud_processed_;

  }
  else // two cameras
  {
    int size_right_cloud = cloud->size() - size_left_cloud;
    camera_source_ = Eigen::MatrixXi::Zero(2, cloud->size());
    camera_source_.block(0,0,1,size_left_cloud) = Eigen::MatrixXi::Ones(1, size_left_cloud);
    camera_source_.block(1,size_left_cloud,1,size_right_cloud) = Eigen::MatrixXi::Ones(1, size_right_cloud);
  }
}


CloudCamera::CloudCamera(const std::string& filename, const Eigen::Matrix3Xd& view_points)
: cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  cloud_processed_ = loadPointCloudFromFile(filename);
  cloud_original_ = cloud_processed_;
  camera_source_ = Eigen::MatrixXi::Ones(1, cloud_processed_->size());
  std::cout << "Loaded point cloud with " << camera_source_.cols() << " points \n";
}

// bui's constructor for a pcd file
CloudCamera::CloudCamera(const std::string& filename, const Eigen::Matrix3Xd& view_points, Eigen::Vector2i s2Dpixel, 
  double PlanFiltPara): cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  setPlanarFilterPara(PlanFiltPara);
  //centroid3D_.resize(0,4);

  cloud_processed_ = buiLoadPClFromFile(filename, s2Dpixel);
  *cloud_original_ = *cloud_processed_;
  camera_source_ = Eigen::MatrixXi::Ones(1, cloud_processed_->size());
  std::cout << "Loaded point cloud with " << camera_source_.cols() << " points \n";
}



CloudCamera::CloudCamera(const std::string& filename_left, const std::string& filename_right,
  const Eigen::Matrix3Xd& view_points)
: cloud_processed_(new PointCloudRGB), cloud_original_(new PointCloudRGB), view_points_(view_points)
{
  sample_indices_.resize(0);
  samples_.resize(3,0);
  normals_.resize(3,0);
  numberOfCluster_ = 0;
  //centroid3D_.resize(0,4);

  // load and combine the two point clouds
  std::cout << "Loading point clouds ...\n";
  PointCloudRGB::Ptr cloud_left(new PointCloudRGB), cloud_right(new PointCloudRGB);
  cloud_left = loadPointCloudFromFile(filename_left);
  cloud_right = loadPointCloudFromFile(filename_right);

  std::cout << "Concatenating point clouds ...\n";
  *cloud_processed_ = *cloud_left + *cloud_right;
  cloud_original_ = cloud_processed_;

  std::cout << "Loaded left point cloud with " << cloud_left->size() << " points \n";
  std::cout << "Loaded right point cloud with " << cloud_right->size() << " points \n";

  // set the camera source matrix: (i,j) = 1 if point j is seen by camera i
  camera_source_ = Eigen::MatrixXi::Zero(2, cloud_processed_->size());
  camera_source_.block(0,0,1,cloud_left->size()) = Eigen::MatrixXi::Ones(1, cloud_left->size());
  camera_source_.block(1,cloud_left->size(),1,cloud_right->size()) = Eigen::MatrixXi::Ones(1, cloud_right->size());
}


void CloudCamera::filterWorkspace(const std::vector<double>& workspace)
{
  // Filter indices into the point cloud.
  if (sample_indices_.size() > 0)
  {
    std::vector<int> indices_to_keep;

    for (int i = 0; i < sample_indices_.size(); i++)
    {
      const pcl::PointXYZRGBA& p = cloud_processed_->points[sample_indices_[i]];
      if (p.x > workspace[0] && p.x < workspace[1] && p.y > workspace[2] && p.y < workspace[3]
          && p.z > workspace[4] && p.z < workspace[5])
      {
        indices_to_keep.push_back(i);
      }
    }

    sample_indices_ = indices_to_keep;
    std::cout << sample_indices_.size() << " sample indices left after workspace filtering \n";
  }

  // Filter (x,y,z)-samples.
  if (samples_.cols() > 0)
  {
    std::vector<int> indices_to_keep;

    for (int i = 0; i < samples_.cols(); i++)
    {
      if (samples_(0,i) > workspace[0] && samples_(0,i) < workspace[1]
          && samples_(1,i) > workspace[2] && samples_(1,i) < workspace[3]
          && samples_(2,i) > workspace[4] && samples_(2,i) < workspace[5])
      {
        indices_to_keep.push_back(i);
      }
    }

    samples_= EigenUtils::sliceMatrix(samples_, indices_to_keep);
    std::cout << samples_.cols() << " samples left after workspace filtering \n";
  }

  // Filter the point cloud.
  std::vector<int> indices;
  for (int i = 0; i < cloud_processed_->size(); i++)
  {
    const pcl::PointXYZRGBA& p = cloud_processed_->points[i];
    if (p.x > workspace[0] && p.x < workspace[1] && p.y > workspace[2] && p.y < workspace[3]
        && p.z > workspace[4] && p.z < workspace[5])
    {
      indices.push_back(i);
    }
  }

  Eigen::MatrixXi camera_source(camera_source_.rows(), indices.size());
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  cloud->points.resize(indices.size());
  for (int i = 0; i < indices.size(); i++)
  {
    camera_source.col(i) = camera_source_.col(indices[i]);
    cloud->points[i] = cloud_processed_->points[indices[i]];
  }
  if (normals_.cols() > 0)
  {
    Eigen::Matrix3Xd normals(3, indices.size());
    for (int i = 0; i < indices.size(); i++)
    {
      normals.col(i) = normals_.col(indices[i]);
    }
    normals_ = normals;
  }
  cloud_processed_ = cloud;
  camera_source_ = camera_source;
}


void CloudCamera::filterSamples(const std::vector<double>& workspace)
{
  std::vector<int> indices;
  for (int i = 0; i < samples_.size(); i++)
  {
    if (samples_(0,i) > workspace[0] && samples_(0,i) < workspace[1]
                                                                  && samples_(1,i) > workspace[2] && samples_(1,i) < workspace[3]
                                                                                                                               && samples_(2,i) > workspace[4] && samples_(2,i) < workspace[5])
    {
      indices.push_back(i);
    }
  }

  Eigen::Matrix3Xd filtered_samples(3, indices.size());
  for (int i = 0; i < indices.size(); i++)
  {
    filtered_samples.col(i) = samples_.col(i);
  }
  samples_ = filtered_samples;
}


void CloudCamera::voxelizeCloud(double cell_size)
{
  Eigen::MatrixXf pts = cloud_processed_->getMatrixXfMap();
  Eigen::Vector3f min_xyz;
  min_xyz << pts.row(0).minCoeff(), pts.row(1).minCoeff(), pts.row(2).minCoeff();

  // find the cell that each point falls into
  std::set< Eigen::Vector4i, CloudCamera::UniqueVectorFirstThreeElementsComparator> bins;
  std::vector<Eigen::Vector3d> avg_normals;
  avg_normals.resize(pts.cols());
  std::vector<int> counts;
  counts.resize(pts.cols());

  for (int i = 0; i < pts.cols(); i++)
  {
    Eigen::Vector3f p;
    p << pts.col(i)(0), pts.col(i)(1), pts.col(i)(2);
    Eigen::Vector3i v = EigenUtils::floorVector((p - min_xyz) / cell_size);
    Eigen::Vector4i v4;
    v4 << v(0), v(1), v(2), i;
    std::pair< std::set<Eigen::Vector4i, CloudCamera::UniqueVectorFirstThreeElementsComparator>::iterator, bool> res = bins.insert(v4);

    if (res.second && normals_.cols() > 0)
    {
      avg_normals[i] = normals_.col(i);
      counts[i] = 1;
    }
    else if (normals_.cols() > 0)
    {
      const int& idx = (*res.first)(3);
      avg_normals[idx] += normals_.col(i);
      counts[idx]++;
    }
  }

  // Calculate the point value and the average surface normal for each cell, and set the camera source for each point.
  Eigen::Matrix3Xf voxels(3, bins.size());
  Eigen::Matrix3Xd normals(3, bins.size());
  Eigen::MatrixXi camera_source(camera_source_.rows(), bins.size());
  int i = 0;
  std::set<Eigen::Vector4i, CloudCamera::UniqueVectorFirstThreeElementsComparator>::iterator it;

  for (it = bins.begin(); it != bins.end(); it++)
  {
    voxels.col(i) = (*it).block(0,0,3,1).cast<float>();
    const int& idx = (*it)(3);

    for (int j = 0; j < camera_source_.rows(); j++)
    {
      camera_source(j,i) = (camera_source_(j, idx) == 1) ? 1 : 0;
    }
    if (normals_.cols() > 0)
    {
      normals.col(i) = avg_normals[idx] / (double) counts[idx];
    }
    i++;
  }

  voxels.row(0) = voxels.row(0) * cell_size + Eigen::VectorXf::Ones(voxels.cols()) * min_xyz(0);
  voxels.row(1) = voxels.row(1) * cell_size + Eigen::VectorXf::Ones(voxels.cols()) * min_xyz(1);
  voxels.row(2) = voxels.row(2) * cell_size + Eigen::VectorXf::Ones(voxels.cols()) * min_xyz(2);

  // Copy the voxels into the point cloud.
  cloud_processed_->points.resize(voxels.cols());
  for(int i=0; i < voxels.cols(); i++)
  {
    cloud_processed_->points[i].getVector3fMap() = voxels.col(i).cast<float>();
  }

  camera_source_ = camera_source;

  if (normals_.cols() > 0)
    normals_ = normals;
}

void CloudCamera::subsampleUniformly(int num_samples)
{
  sample_indices_.resize(num_samples);
  pcl::RandomSample<pcl::PointXYZRGBA> random_sample;
  random_sample.setInputCloud(cloud_processed_);
  random_sample.setSample(num_samples);
  random_sample.filter(sample_indices_);
}

void CloudCamera::subsampleSamples(int num_samples)
{
  // use all incoming samples
  if (num_samples == 0 || num_samples >= samples_.cols())
  {
    std::cout << "Using all " << samples_.cols() << " samples.\n";
  }
  // subsample the incoming samples
  else
  {
    std::cout << "Using " << num_samples << " out of " << samples_.cols() << " available samples.\n"; 
    std::vector<int> seq(samples_.cols());
    for (int i = 0; i < seq.size(); i++)
    {
      seq[i] = i;
    }
    std::random_shuffle(seq.begin(), seq.end());

    Eigen::Matrix3Xd subsamples(3, num_samples);
    for (int i = 0; i < num_samples; i++)
    {
      subsamples.col(i) = samples_.col(seq[i]);
    }
    samples_ = subsamples;

    std::cout << "Subsampled " << samples_.cols() << " samples at random uniformly.\n";
  }
}

void CloudCamera::writeNormalsToFile(const std::string& filename, const Eigen::Matrix3Xd& normals)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());

  for (int i = 0; i < normals.cols(); i++)
  {
    myfile << boost::lexical_cast<std::string>(normals(0,i)) << "," << boost::lexical_cast<std::string>(normals(1,i)) << "," << boost::lexical_cast<std::string>(normals(2,i)) << "\n";
  }

  myfile.close();
}

void CloudCamera::calculateNormals(int num_threads)
{
  double t0 = omp_get_wtime();
  std::cout << "Calculating surface normals ...\n";

  if (cloud_processed_->isOrganized())
  {
    calculateNormalsOrganized();
  }
  else
  {
    calculateNormalsOMP(num_threads);
  }

  std::cout << " runtime (normals): " << omp_get_wtime() - t0 << "\n";

  // reverse direction of normals (if a normal does not point to at least one camera)
  std::cout << "Reversing direction of normals that do not point to at least one camera ...\n";
  reverseNormals();
}

void CloudCamera::calculateNormalsOrganized()
{
  if (!cloud_processed_->isOrganized())
  {
    std::cout << "Error: point cloud is not organized!\n";
    return;
  }

  std::cout << "Using integral images for surface normals estimation ...\n";
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud(cloud_processed_);
  ne.setViewPoint(view_points_(0,0), view_points_(1,0), view_points_(2,0));
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setNormalSmoothingSize(20.0f);
  ne.compute(*cloud_normals);
  normals_ = cloud_normals->getMatrixXfMap().cast<double>();
}

void CloudCamera::calculateNormalsOMP(int num_threads)
{
  std::vector< std::vector<int> > indices(view_points_.cols());

  for (int i = 0; i < camera_source_.cols(); i++)
  {
    for (int j = 0; j < view_points_.cols(); j++)
    {
      if (camera_source_(j,i) == 1) // point is seen by this camera
      {
        indices[j].push_back(i);
        break; // TODO: multiple cameras
      }
    }
  }

  // Calculate surface normals for each view point.
  std::vector<PointCloudNormal::Ptr> normals_list(view_points_.cols());
  pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> estimator(num_threads);
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
  estimator.setInputCloud(cloud_processed_);
  estimator.setSearchMethod(tree_ptr);
  estimator.setRadiusSearch(0.03);
  pcl::IndicesPtr indices_ptr(new std::vector<int>);

  for (int i = 0; i < view_points_.cols(); i++)
  {
    PointCloudNormal::Ptr normals_cloud(new PointCloudNormal);
    indices_ptr->assign(indices[i].begin(), indices[i].end());
    estimator.setIndices(indices_ptr);
    estimator.setViewPoint(view_points_(0,i), view_points_(1,i), view_points_(2,i));
    estimator.compute(*normals_cloud);
    normals_list[i] = normals_cloud;
    printf("camera: %d, #indices: %d, #normals: %d \n", i, (int) indices[i].size(), (int) normals_list[i]->size());
  }

  // Assign the surface normals to the points.
  normals_.resize(3, camera_source_.cols());

  for (int i = 0; i < normals_list.size(); i++)
  {
    for (int j = 0; j < normals_list[i]->size(); j++)
    {
      const pcl::Normal& normal = normals_list[i]->at(j);
      normals_.col(indices[i][j]) << normal.normal_x, normal.normal_y, normal.normal_z;
    }
  }
}

void CloudCamera::reverseNormals()
{
  double t1 = omp_get_wtime();
  int c = 0;

  for (int i = 0; i < normals_.cols(); i++)
  {
    bool needs_reverse = true;

    for (int j = 0; j < view_points_.cols(); j++)
    {
      if (camera_source_(j,i) == 1) // point is seen by this camera
      {
        Eigen::Vector3d cam_to_point = cloud_processed_->at(i).getVector3fMap().cast<double>() - view_points_.col(j);

        if (normals_.col(i).dot(cam_to_point) < 0) // normal points toward camera
        {
          needs_reverse = false;
          break;
        }
      }
    }

    if (needs_reverse)
    {
      normals_.col(i) *= -1.0;
      c++;
    }
  }

  std::cout << " reversed " << c << " normals\n";
  std::cout << " runtime (reverse normals): " << omp_get_wtime() - t1 << "\n";
}

void CloudCamera::setNormalsFromFile(const std::string& filename)
{
  std::ifstream in;
  in.open(filename.c_str());
  std::string line;
  normals_.resize(3, cloud_original_->size());
  int i = 0;

  while(std::getline(in, line))
  {
    std::stringstream lineStream(line);
    std::string cell;
    int j = 0;

    while(std::getline(lineStream, cell, ','))
    {
      normals_(i,j) = boost::lexical_cast<double>(cell);
      j++;
    }

    i++;
  }
}

PointCloudRGB::Ptr CloudCamera::loadPointCloudFromFile(const std::string& filename)
{
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, *cloud) == -1)
  {
    std::cout << "Couldn't read .pcd file: " << filename << "\n";
    cloud->points.resize(0);
  }
  return cloud;
}

// bui's addition a function to handle with pcl::PointXYZRGB file type
PointCloudRGB::Ptr CloudCamera::buiLoadPClFromFile(const std::string& filename, Eigen::Vector2i s2Dpixel)
{
  
  int count;
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  Eigen::MatrixXi centroid2D;

  CloudRGB::Ptr cloudp1 (new CloudRGB), cloudp2 (new CloudRGB), cloudp3 (new CloudRGB), cloudp4 (new CloudRGB);
  std::vector<pcl::PointIndices> clusterIndices;
  //Eigen::Vector2i s2Dpixel;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloudp1) == -1)
  {
    std::cout << "Couldn't read .pcd file: " << filename << "\n";
    cloudp1->points.resize(0);
  }
  else
  {
    setSelected2Dpixel(s2Dpixel);
    cloudp2 = downsampling(cloudp1);
    cloudp3 = planar_filter(cloudp2);
    clusterIndices = extractObjects(cloudp3);
    centroid2D = calculateCentroid2D(cloudp3, clusterIndices);
    //std::cout << "-----------------bui loadPCDFile is still ok-----------\n";
    cloudp4 = selectObject(cloudp3, clusterIndices, s2Dpixel);

    pcl::copyPointCloud(*cloudp4, *cloud);
  }
  return cloud;
}


// bui's adding a function to handle with a point cloud topic
PointCloudRGB::Ptr CloudCamera::buiFilterPClTopic(const PointCloudRGB::Ptr& cloudp, Eigen::Vector2i s2Dpixel)
{
  
  int count;
  PointCloudRGB::Ptr cloudpA (new PointCloudRGB);
  Eigen::MatrixXi centroid2D;

  CloudRGB::Ptr cloudp1 (new CloudRGB), cloudp2 (new CloudRGB), cloudp3 (new CloudRGB), cloudp4 (new CloudRGB);
  std::vector<pcl::PointIndices> clusterIndices;
  
  pcl::copyPointCloud(*cloudp, *cloudp1);

  setSelected2Dpixel(s2Dpixel);
  cloudp2 = downsampling(cloudp1);
  cloudp3 = planar_filter(cloudp2);
  clusterIndices = extractObjects(cloudp3);
  
  centroid2D = calculateCentroid2D(cloudp3, clusterIndices);
  //std::cout << "-----------------bui loadPCDFile is still ok-----------\n";
  cloudp4 = selectObject(cloudp3, clusterIndices, s2Dpixel);

  pcl::copyPointCloud(*cloudp4, *cloudpA);
  
  return cloudpA;
}

// bui's adding: passThrough filter
CloudRGB::Ptr CloudCamera::passthrough_filter(const CloudRGB::Ptr cloud) const
{
    
    CloudRGB::Ptr cloud_filtered (new CloudRGB);
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> distance_pass;
    distance_pass.setInputCloud (cloud);
    distance_pass.setFilterFieldName ("z");
    distance_pass.setFilterLimits (0.25, 1.4);
    //pass.setFilterLimitsNegative (true);
    distance_pass.filter (*cloud_filtered);

    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << "/home/bui/bui_ws/image_sample/afterPassthroughFilter.pcd";
    writer.write<pcl::PointXYZRGB> (ss.str(), *cloud_filtered, false); //*

    std::cerr << "Cloud after filtering: " << std::endl;

return cloud_filtered;
}
// bui's additing a function to downsampling the data
CloudRGB::Ptr CloudCamera::downsampling(const CloudRGB::Ptr cloudp) const
{
    // this code run with input as argument of point cloud data 
    CloudRGB::Ptr cloud_filtered (new CloudRGB), cloudp_inter (new CloudRGB);

    cloudp_inter = passthrough_filter(cloudp);

    std::cerr << "PointCloud before downsampling: "<< cloudp_inter->width*cloudp_inter->height <<" data points." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudp_inter);
    // define the size of the the leaf (in meter) = voxel size
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    // perform the downsampling filter then save the new data to cloud_filtered
    sor.filter (*cloud_filtered);

    //std::cerr << "PointCloud after downsampling: "<<cloud_filtered->width*cloud_filtered->height<<" data points." << std::endl;

    return cloud_filtered;
}

// bui's additing a function to filtering the planer
CloudRGB::Ptr CloudCamera::planar_filter(CloudRGB::Ptr cloud) const
{
    //std::cout << "Checking the planar_filter -------------------------\n";
    //CloudRGB::Ptr cloud_p (new CloudRGB);
    CloudRGB::Ptr cloud_f (new CloudRGB);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory - set parameter for the planar filter
    seg.setModelType (pcl::SACMODEL_PLANE); // -> segment the plane on the point cloud
    // -> RANSAC = Random Sample Consensus = estimator method
    seg.setMethodType (pcl::SAC_RANSAC);    //-> use 2 step to determine the outliers or inliers
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.018);    // -> tolerate range to define the inliers & outlier
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int) cloud->points.size ();
    
    // While 30% of the original cloud is still there
    double filterParameter;
    filterParameter = getPlanarFilterPara();
    //std::cout << "The value of the filter parameter is -------------" << filterParameter << std::endl;
    while (cloud->points.size() > filterParameter*nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        // seg.segment() result in the inliers point and coefficients
        seg.segment (*inliers, *coefficients); //inliers are all point belong the plane
        // outlier -> mau ngoai lai, out of observation range -> skew the estimation result
        // inlier -> sample lie inside the range -> fit with the model 
        if (inliers->indices.size () == 0) // -> if there is no inliers
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        // input a pointer to vector of indices (inliers)->represent the input data
        extract.setIndices (inliers);
        // Create the filtering object
        extract.setNegative (true); //true -> return all point of data except the input indices
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }
    return cloud;
}

Eigen::MatrixXi CloudCamera::calculateCentroid2D(CloudRGB::Ptr cloud, std::vector<pcl::PointIndices> c_indices)
{
    int nCluster = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = c_indices.begin (); it != c_indices.end (); ++it)
         ++nCluster;

    Eigen::MatrixXi centroid2D_vector(nCluster, 2);
    centroid2D_vector << Eigen::MatrixXi::Zero(nCluster,2);
    
    int count = 0;
    Eigen::Vector4f centroid3D;
    Eigen::Vector2i centroid2D1;
    Eigen::Matrix3f camera_matrix1;
    camera_matrix1 << 538.2075, 0.0, 318.3089, 0.0, 538.6995, 231.273, 0.0, 0.0, 1.0;
    // go to each point cloud cluster in the cluster vector - cluster_indices
    for (std::vector<pcl::PointIndices>::const_iterator it = c_indices.begin (); it != c_indices.end (); ++it)
    {     
        centroid3D << Eigen::Vector4f::Zero(1,4);
        centroid2D1 << Eigen::Vector4i::Zero(1,2);
        CloudRGB::Ptr cloud_cluster (new CloudRGB);
        // copy all the point cloud of one cluster to a new point cloud data
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            // copy each point cloud of the cluster to a new point cloud data
            cloud_cluster->points.push_back(cloud->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::compute3DCentroid(*cloud, *it, centroid3D);
        
        centroid2D1 = point3Dto_pixel(centroid3D, camera_matrix1);
        std::cout << "3D centroid of PCl Cluster " << count+1 << std::endl;
        std::cout << centroid3D << std::endl;

        std::cout << "2D centroid of PCl Cluster " << count+1 << std::endl;
        std::cout << centroid2D1 << std::endl;
        
        // save the centroid to the centroi_vector
        centroid2D_vector.row(count) = centroid2D1;

        ++count;
    }
    return centroid2D_vector;
}


std::vector<pcl::PointIndices> CloudCamera::extractObjects(CloudRGB::Ptr cloud) const
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    // create the indices vector for clusters in the point cloud
    std::vector<pcl::PointIndices> cluster_indices;
    // store the cluster extraction of point cloud
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // the radius of the sphere to determine the neighbor points
    ec.setClusterTolerance (0.02); // 2cm
    // minimum of number of point in a cluster
    ec.setMinClusterSize (70);
    // maximum of number of point in a cluster
    ec.setMaxClusterSize (25000);
    // the search method is tree
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    // perform then save the found clusters to cluster_indices
    ec.extract(cluster_indices); // cluster_indices = set of point cloud clusters

    return cluster_indices;
}

// if no object point cloud matched, this function will return the input point cloud
CloudRGB::Ptr CloudCamera::selectObject(CloudRGB::Ptr cloud, 
            std::vector<pcl::PointIndices> cluster_indices, Eigen::Vector2i grasp_pos)
{
    
    int distance, distance_x, distance_y, threshold, ob_grasp;
    threshold = 30;
    //ob_grasp = 20;
    //int clusternumber = ob_centroids.size()/2;
    
    int nCluster = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
         ++nCluster;
    
    int count = 0;
    Eigen::Vector4f centroid3D;
    Eigen::Vector2i centroid2D;
    Eigen::Matrix3f camera_matrix;
    camera_matrix << 538.2075, 0.0, 318.3089, 0.0, 538.6995, 231.273, 0.0, 0.0, 1.0;
    // go to each point cloud cluster in the cluster vector - cluster_indices
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {     
        CloudRGB::Ptr cloud_cluster (new CloudRGB);
        centroid3D << Eigen::Vector4f::Zero(1,4);
        //CloudRGB::Ptr cloud_cluster (new CloudRGB);
        // copy all the point cloud of one cluster to a new point cloud data
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            // copy each point cloud of the cluster to a new point cloud data
            cloud_cluster->points.push_back(cloud->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        pcl::compute3DCentroid(*cloud, *it, centroid3D);
        centroid2D = point3Dto_pixel(centroid3D, camera_matrix);

        // comparing this centroid2D with the selected2Dpixel
        std::cout <<  "Distance of PCl Cluster " << count+1 << " to the Selected Object: \n";
        distance_x = abs(grasp_pos(0) - centroid2D(0));
        std::cout <<  "The distance in x axis: " << distance_x << std::endl;
        distance_y = abs(grasp_pos(1) - centroid2D(1));
        std::cout <<  "The distance in y axis: " << distance_y << std::endl;
        distance = sqrt(pow(distance_x, 2) + pow(distance_y, 2));
        std::cout <<  "The aggregated distance: " << distance << std::endl;

        if (distance < threshold)

        {
            std::cout << "Object " << count+1 << " is selected (its distance smaller than threshold-" << threshold << ")\n";
            return cloud_cluster;
        }
        ++count;
    }

    return cloud;
}

Eigen::Vector2i CloudCamera::point3Dto_pixel(Eigen::Vector4f pc_coord, Eigen::Matrix3f cmatrix)
{
    Eigen::Vector2i pixel_pos;
    //std::cout << "inside the point3Dto_pixel" << std::endl;
    pixel_pos(0) = (int)(pc_coord(0)*cmatrix(0,0)/pc_coord(2) + cmatrix(0,2));
    pixel_pos(1) = (int)(pc_coord(1)*cmatrix(1,1)/pc_coord(2) + cmatrix(1,2));
    //std::cout << "inside the point3Dto_pixel" << std::endl;
    return pixel_pos;
}


void CloudCamera::setSamples(const Eigen::Matrix3Xd& samples)
{
  samples_ = samples;
}
