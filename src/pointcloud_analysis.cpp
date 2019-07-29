#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
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

PCLPointCloud2 downSample(PCLPointCloud2ConstPtr cloud, double xDim, double yDim, double zDim) {
  PCLPointCloud2 cloud_filtered;

  VoxelGrid<PCLPointCloud2> sor;
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

/*PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double depth, double smoothing) {
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

  IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(depth);
  ne.setNormalSmoothingSize(smoothing);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  return normals;
}*/

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  PCLPointCloud2Ptr cloudPtr(new PCLPointCloud2);
  pcl_conversions::toPCL(*cloud, *(cloudPtr.get()));
  PCLPointCloud2 downsampled = downSample(cloudPtr, 0.05, 0.05, 0.05);

  pcl::PointCloud<pcl::PointXYZRGB> downsampled_b;
  pcl::fromPCLPointCloud2(downsampled, downsampled_b);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_b_ptr (new pcl::PointCloud<pcl::PointXYZRGB> (downsampled_b));

  PointCloud<Normal>::Ptr cloud_normals = computeNormals(downsampled_b_ptr, 0.05);

  updateViewer(viewer, downsampled_b_ptr, cloud_normals);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;
  viewer = createViewer();
  
  ros::Subscriber sub_cloud = n.subscribe("/camera/depth_registered/points", 1, pointcloudCallback);

  ros::spin();
  return 0;
}
