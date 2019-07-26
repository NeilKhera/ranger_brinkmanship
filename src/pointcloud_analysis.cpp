#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
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

void updateViewer (pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->removePointCloud();
  viewer->addPointCloud (cloud, rgb);
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

/*pcl::PointCloud<pcl::Normal>::Ptr
computeNormals(pcl::PCLPointCloud2ConstPtr cloud, double radius) {
  using namespace pcl;
  NormalEstimation<PointXYZ, Normal> ne;
  ne.setInputCloud(cloud);

  search::KdTree<PointXYZ>::Ptr kdtree (new search::KdTree<PointXYZ> ());
  ne.setSearchMethod(kdtree);

  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);

  ne.setRadiusSearch(radius);

  ne.compute(*cloud_normals);

  return cloud_normals;
}*/

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PCLPointCloud2Ptr cloudPtr(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*msg, *(cloudPtr.get()));

  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 downsampled = downSample(cloudPtr, 0.1, 0.1, 0.1);
  pcl_conversions::fromPCL(downsampled, output);

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  pcl::fromPCLPointCloud2(*cloudPtr, point_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr (new pcl::PointCloud<pcl::PointXYZRGB> (point_cloud));
  updateViewer(viewer, pcptr);
  //pub_cloud.publish(output);
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
