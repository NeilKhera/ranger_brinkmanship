#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>

ros::Publisher pub_cloud;
pcl::visualization::PCLVisualizer::Ptr viewer;

pcl::visualization::PCLVisualizer::Ptr createViewer() {
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

void updateViewer (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
  viewer->removePointCloud("cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
  viewer->removePointCloud("normals");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 25, 0.15, "normals");
  viewer->spinOnce();
}

pcl::PCLPointCloud2
downSample(pcl::PCLPointCloud2ConstPtr cloud, double xDim, double yDim, double zDim) {
  pcl::PCLPointCloud2 cloud_filtered;

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(xDim, yDim, zDim);
  sor.filter(cloud_filtered);

  return cloud_filtered;
}

pcl::PointCloud<pcl::Normal>::Ptr
computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, double radius) {
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod(kdtree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  ne.setRadiusSearch(radius);

  ne.compute(*cloud_normals);

  return cloud_normals;
}

/*pcl::PointCloud<pcl::Normal>::Ptr
computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double depth, double smoothing) {
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(depth);
  ne.setNormalSmoothingSize(smoothing);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  return normals;
}*/

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PCLPointCloud2Ptr cloudPtr(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*msg, *(cloudPtr.get()));

  pcl::PCLPointCloud2 downsampled = downSample(cloudPtr, 0.05, 0.05, 0.05);

  pcl::PointCloud<pcl::PointXYZRGB> downsampled_b;
  pcl::fromPCLPointCloud2(downsampled, downsampled_b);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_b_ptr (new pcl::PointCloud<pcl::PointXYZRGB> (downsampled_b));

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = computeNormals(downsampled_b_ptr, 0.05);

  updateViewer(viewer, downsampled_b_ptr, cloud_normals);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;

  viewer = createViewer();

  ros::Subscriber sub_cloud = n.subscribe("camera/depth/color/points", 1, pointcloudCallback);

  pub_cloud = n.advertise<sensor_msgs::PointCloud2>("camera/depth/color/points/processed", 1);

  ros::spin();
  return 0;
}
