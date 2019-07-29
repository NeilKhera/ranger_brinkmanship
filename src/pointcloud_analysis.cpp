#include "tic_toc.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

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
  viewer->addPointCloudNormals<PointXYZRGB, Normal> (cloud, normals, 1, 0.1, "normals");
  viewer->spinOnce();
}

void cutOff(PointCloud<PointXYZRGB>::Ptr cloud) {
  PassThrough<PointXYZRGB> pt;
  pt.setInputCloud(cloud);

  pt.setFilterFieldName("x");
  pt.setFilterLimits(-100, 100);
  pt.filter(*cloud);

  pt.setFilterFieldName("y");
  pt.setFilterLimits(-100, 100);
  pt.filter(*cloud);

  pt.setFilterFieldName("z");
  pt.setFilterLimits(-100, 100);
  pt.filter(*cloud);
}

void statOutRemoval(PointCloud<PointXYZRGB>::Ptr cloud) {
  StatisticalOutlierRemoval<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(10);
  sor.setStddevMulThresh(2.0);
  sor.setKeepOrganized(true);
  sor.filter(*cloud);
}

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double xDim, double yDim, double zDim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(xDim, yDim, zDim);
  vox.filter(*cloud);
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double radius) {
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
  search::KdTree<PointXYZRGB>::Ptr kdtree (new search::KdTree<PointXYZRGB> ());

  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);
  
  return normals;
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

void pointcloudCallback(const PointCloud<PointXYZRGB>::ConstPtr& cloud) {
  PointCloud<PointXYZRGB>::Ptr cloud_conv (new PointCloud<PointXYZRGB> (*cloud));
  ROS_ERROR("SIZE1: %lu", cloud_conv->points.size());

  tic(); 
  cutOff(cloud_conv);
  ROS_ERROR("CUTOFF: %f", toc());

  //tic();
  //statOutRemoval(cloud_conv);
  //ROS_ERROR("STAT: %f", toc());

  tic();
  downSample(cloud_conv, 0.05, 0.05, 0.05);
  ROS_ERROR("DOWNSAMPLE: %f", toc());

  ROS_ERROR("SIZE2: %lu", cloud_conv->points.size());

  tic();
  PointCloud<Normal>::Ptr cloud_normals = computeNormals(cloud_conv, 0.05);
  ROS_ERROR("NORMALS: %f", toc());

  ROS_ERROR("NORMAL SIZE: %lu", cloud_normals->points.size());

  updateViewer(viewer, cloud_conv, cloud_normals);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;
  viewer = createViewer();
  
  ros::Subscriber sub_cloud =
	  n.subscribe<PointCloud<PointXYZRGB>>("/camera/depth_registered/points", 1, pointcloudCallback);

  ros::spin();
  return 0;
}
