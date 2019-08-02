#include "tic_toc.h"

#include <math.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

using namespace pcl;

static const double CAMERA_ANGLE = M_PI / 4;
static const double CAMERA_HEIGHT = 0.10;

float roll = 0.0;
float pitch = 0.0;

ros::Publisher pub_cloud;
visualization::PCLVisualizer::Ptr viewer;

visualization::PCLVisualizer::Ptr createViewer() {
  visualization::PCLVisualizer::Ptr viewer(
      new visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return viewer;
}

void updateViewer(visualization::PCLVisualizer::Ptr viewer,
                  PointCloud<PointXYZRGB>::Ptr cloud,
                  PointCloud<Normal>::Ptr normals) {
  viewer->removePointCloud("cloud");
  viewer->addPointCloud<PointXYZRGB>(cloud, "cloud");
  viewer->removePointCloud("normals");
  viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud, normals, 1, 0.1,
                                                    "normals");
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

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double x_dim, double y_dim,
                double z_dim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  vox.filter(*cloud);
}

PointCloud<PointXYZRGB>::Ptr
transformCameraToRobot(PointCloud<PointXYZRGB>::ConstPtr cloud) {
  Eigen::Affine3f camera_transform_matrix = Eigen::Affine3f::Identity();

  camera_transform_matrix.translation() << 0.0, -CAMERA_HEIGHT, 0.0;
  camera_transform_matrix.rotate(Eigen::AngleAxisf(-CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZRGB>::Ptr cloud_angle_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_angle_corrected, camera_transform_matrix);

  return cloud_angle_corrected;
}

PointCloud<PointXYZRGB>::Ptr
rotateToAlignWithGravity(PointCloud<PointXYZRGB>::ConstPtr cloud) {
  //ROS_ERROR("ROLL: %f PITCH: %f", roll, pitch);
  //Eigen::Matrix4f gravity_transform_matrix = Eigen::Matrix4f::Identity();

  //gravity_transform_matrix (0,0) = std::cos(pitch);
  //gravity_transform_matrix (0,1) = std::sin(pitch) * std::sin(roll);
  //gravity_transform_matrix (0,2) = std::sin(pitch) * std::cos(roll);
  //gravity_transform_matrix (1,0) = 0;
  //gravity_transform_matrix (1,1) = std::cos(roll);
  //gravity_transform_matrix (1,2) = -std::sin(roll);
  //gravity_transform_matrix (2,0) = -std::sin(pitch);
  //gravity_transform_matrix (2,1) = std::cos(pitch) * std::sin(roll);
  //gravity_transform_matrix (2,2) = std::cos(pitch) * std::cos(roll);

  Eigen::Affine3f gravity_transform_matrix = Eigen::Affine3f::Identity();
  gravity_transform_matrix.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));
  gravity_transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ()));

  PointCloud<PointXYZRGB>::Ptr cloud_gravity_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_gravity_corrected, gravity_transform_matrix);

  return cloud_gravity_corrected;
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud,
                                       double radius) {
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  search::KdTree<PointXYZRGB>::Ptr kdtree(new search::KdTree<PointXYZRGB>());

  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);

  return normals;
}

/*PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud,
double depth, double smoothing) { PointCloud<Normal>::Ptr normals (new
PointCloud<Normal>);

  IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(depth);
  ne.setNormalSmoothingSize(smoothing);
  ne.setInputCloud(cloud);
  ne.compute(*normals);

  return normals;
}*/

PointCloud<PointXYZI> normalAnalysis(PointCloud<Normal> normals,
                                     PointCloud<PointXYZRGB> cloud) {
  PointCloud<PointXYZI> output_cloud;
  output_cloud.height = cloud.height;
  output_cloud.width = cloud.width;
  output_cloud.header.frame_id = cloud.header.frame_id;

  float vector_x = 0.0;
  float vector_y = -1.0;
  float vector_z = 0.0;

  float threshold = 0.3;

  for (int i = 0; i < normals.points.size(); i++) {
    Normal n = normals.points[i];
    PointXYZRGB point = cloud.points[i];

    float norm_x = n.normal_x;
    float norm_y = n.normal_y;
    float norm_z = n.normal_z;

    float dot = vector_x * norm_x + vector_y * norm_y + vector_z * norm_z;
    float theta = std::acos(dot);
    ROS_ERROR("%f", theta);

    PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    if (theta > threshold) {
      p.intensity = 1;
    } else {
      p.intensity = 0;
    }

    output_cloud.points.push_back(p);
  }

  return output_cloud;
}

void orientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr &vector_msg) {
  roll = vector_msg->vector.y;
  pitch = vector_msg->vector.x;
}

void pointcloudCallback(const PointCloud<PointXYZRGB>::ConstPtr &cloud) {
  PointCloud<PointXYZRGB>::Ptr cloud_converted(
      new PointCloud<PointXYZRGB>(*cloud));

  cutOff(cloud_converted);
  downSample(cloud_converted, 0.05, 0.05, 0.05);
  // statOutRemoval(cloud_converted);

  PointCloud<PointXYZRGB>::Ptr cloud_transformed_gravity =
      rotateToAlignWithGravity(cloud_converted);
  PointCloud<PointXYZRGB>::Ptr cloud_transformed_camera =
      transformCameraToRobot(cloud_transformed_gravity);

  PointCloud<Normal>::Ptr cloud_normals =
      computeNormals(cloud_transformed_camera, 0.05);
  PointCloud<PointXYZI> output =
      normalAnalysis(*cloud_normals, *cloud_transformed_camera);

  pub_cloud.publish(output);
  updateViewer(viewer, cloud_transformed_camera, cloud_normals);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;
  viewer = createViewer();

  ros::Subscriber sub_orientation = n.subscribe("/imu/rpy", 1, orientationCallback);

  ros::Subscriber sub_cloud = n.subscribe<PointCloud<PointXYZRGB>>(
      "/camera/depth_registered/points", 1, pointcloudCallback);

  pub_cloud = n.advertise<PointCloud<PointXYZI>>("/normal_processed/points", 1);

  ros::spin();
  return 0;
}
