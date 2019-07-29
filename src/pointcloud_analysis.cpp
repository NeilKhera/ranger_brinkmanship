#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>

using namespace pcl;

visualization::PCLVisualizer::Ptr viewer;

visualization::PCLVisualizer::Ptr createViewer() {
  visualization::PCLVisualizer::Ptr viewer (new visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

void updateViewer (visualization::PCLVisualizer::Ptr viewer, PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals) {
  viewer->removePointCloud("cloud");
  viewer->addPointCloud<PointXYZRGB>(cloud, "cloud");
  viewer->removePointCloud("normals");
  viewer->addPointCloudNormals<PointXYZRGB, Normal> (cloud, normals, 25, 0.15, "normals");
  viewer->spinOnce();
}

PointCloud<PointXYZRGB> downSample(PointCloud<PointXYZRGB>::Ptr cloud, double xDim, double yDim, double zDim) {
  PointCloud<PointXYZRGB> cloud_filtered;

  VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(xDim, yDim, zDim);
  sor.filter(cloud_filtered);

  return cloud_filtered;
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::ConstPtr cloud, double radius) {
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud(cloud);

  search::KdTree<PointXYZRGB>::Ptr kdtree (new search::KdTree<PointXYZRGB> ());
  ne.setSearchMethod(kdtree);

  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

  ne.setRadiusSearch(radius);
  ne.compute(*cloud_normals);
  return cloud_normals;
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double depth, double smoothing) {
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

  IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(depth);
  ne.setNormalSmoothingSize(smoothing);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  return normals;
}

void pointcloudCallback(const PointCloud<PointXYZRGB>::ConstPtr& cloud) {
  PointCloud<PointXYZRGB>::Ptr cloud_conv (new PointCloud<PointXYZRGB> (*cloud));
  
  //PointCloud<PointXYZRGB>::Ptr downsampled_ptr (new PointCloud<PointXYZRGB>);
  //*downsampled_ptr = downSample(cloud_conv, 0.05, 0.05, 0.05);
  
  PointCloud<Normal>::Ptr cloud_normals = computeNormals(cloud_conv, 0.02, 10.0);

  updateViewer(viewer, cloud_conv, cloud_normals);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;
  viewer = createViewer();
  
  ros::Subscriber sub_cloud = n.subscribe<PointCloud<PointXYZRGB>>("/camera/depth_registered/points", 1, pointcloudCallback);

  ros::spin();
  return 0;
}
