#include "grasp_affordances/featurecloud.h"

    FeatureCloud::FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

 // Process the given cloud
    void FeatureCloud::setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void FeatureCloud::loadInputCloud (CloudNormal::Ptr &xyzn) //pcl::PointCloud<pcl::PointNormal>::Ptr input_icr_;
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
      xyzn_ = xyzn;

      std::cerr << "Test: x, y, z: " << (*xyzn_).points[0].x << "  " << (*xyzn_).points[0].y << "  " << (*xyzn_).points[0].z << std::endl;

      std::cerr << "Normals: " << (*xyzn_).points[0].normal[0] << "  " << (*xyzn_).points[0].normal[1] << "  " << (*xyzn_).points[0].normal[2] << std::endl;

      std::cerr << "Size: " << (*xyzn_) << std::endl;

      //Giving the point clouds the correct size
      (*xyz_).width = (*normals_).width = (*xyzn_).width;
      (*xyz_).height = (*normals_).height = (*xyzn_).height;
      (*xyz_).points.resize((*xyzn_).points.size());
      (*normals_).points.resize((*xyzn_).points.size());

      std::cerr << "xyz.x: " << (*xyz_) << std::endl;
      std::cerr << "normal: " << (*normals_) << std::endl;


      //Splitting the obtained point cloud (PointNormal) into two point clouds (PointXYZ and Normal)
      for(size_t i = 0; i < (*xyzn_).points.size(); ++i)
      {
	(*xyz_).points[i].x = (*xyzn_).points[i].x;
	(*xyz_).points[i].y = (*xyzn_).points[i].y;
	(*xyz_).points[i].z = (*xyzn_).points[i].z;
	
	(*normals_).points[i].normal[0] = (*xyzn_).points[i].normal[0];
	(*normals_).points[i].normal[1] = (*xyzn_).points[i].normal[1];
	(*normals_).points[i].normal[2] = (*xyzn_).points[i].normal[2];
      }

      //pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Compute the surface normals and local features
    void FeatureCloud::processInput ()
    {
      //computeSurfaceNormals ();
      computeLocalFeatures ();
    }
/*
    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      //normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }
*/
    // Compute the local feature descriptors
    void FeatureCloud::computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }
