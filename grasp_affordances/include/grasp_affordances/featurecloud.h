

#ifndef featurecloud_h___
#define featurecloud_h___

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/kdtree/kdtree.h>


class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
    typedef pcl::PointCloud<pcl::PointNormal> CloudNormal;

    FeatureCloud();

    ~FeatureCloud(){};

    // Process the given cloud
    void setInputCloud (PointCloud::Ptr xyz);

    // Load and process the cloud in the given PCD file
    //void loadInputCloud (const std::string &pcd_file);
    void loadInputCloud (CloudNormal::Ptr &icr);

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    };

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    };



  protected:
    // Compute the surface normals and local features
    void processInput ();


    // Compute the surface normals
  //  void computeSurfaceNormals ();

    // Compute the local feature descriptors
    void computeLocalFeatures ();



  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    CloudNormal::Ptr xyzn_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};
#endif
