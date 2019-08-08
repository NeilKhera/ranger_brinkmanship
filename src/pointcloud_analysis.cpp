#include "tic_toc.h"

#include <math.h>
#include <std_msgs/Bool.h>
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

float ROVER_WIDTH;
float CAMERA_ANGLE;
float CAMERA_HEIGHT;
float ANGLE_THRESHOLD;
float Y_MAX;
float Y_MIN;

float roll = 0.0;
float pitch = 0.0;

ros::Publisher pub_cloud;
ros::Publisher pub_go;
visualization::PCLVisualizer::Ptr viewer;

visualization::PCLVisualizer::Ptr createViewer() {
  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return viewer;
}

void updateViewer(visualization::PCLVisualizer::Ptr viewer, PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals) {
  viewer->removePointCloud("cloud");
  viewer->addPointCloud<PointXYZRGB>(cloud, "cloud");
  viewer->removePointCloud("normals");
  viewer->addPointCloudNormals<PointXYZRGB, Normal>(cloud, normals, 1, 0.1, "normals");
  viewer->spinOnce();
}

void cutOff(PointCloud<PointXYZRGB>::Ptr cloud) {
  PassThrough<PointXYZRGB> pt;
  pt.setInputCloud(cloud);

  pt.setFilterFieldName("x");
  pt.setFilterLimits(-ROVER_WIDTH / 2, ROVER_WIDTH / 2);
  pt.filter(*cloud);

  pt.setFilterFieldName("y");
  pt.setFilterLimits(Y_MIN, Y_MAX);
  pt.filter(*cloud);
}

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double x_dim, double y_dim, double z_dim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  vox.filter(*cloud);
}

PointCloud<PointXYZRGB>::Ptr frameTransform(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << 0.0, -CAMERA_HEIGHT, 0.0;
  transform_matrix.rotate(Eigen::AngleAxisf(-CAMERA_ANGLE, Eigen::Vector3f::UnitX()));
  transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);

  return cloud_corrected;
}

PointCloud<PointXYZRGB>::Ptr orientationCorrection(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));
  //transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);

  return cloud_corrected;
}

float findEdge(PointCloud<PointXYZRGB> cloud) {
  float z_min = -1.0;

  for (int i = 0; i < cloud.width; i += 10) {
    for (int j = 0; j < cloud.height; j += 10) {
      if (!isnan(cloud.at(i, j).z) && cloud.at(i, j).y > Y_MIN && cloud.at(i, j).y < Y_MAX) {
        if (cloud.at(i, j).z < z_min || z_min < 0) {
          z_min = cloud.at(i, j).z;
        }
        break;
      }
    }
  }

  return z_min;
}


PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double radius) {
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

PointCloud<PointXYZI> normalAnalysis(PointCloud<Normal> normals, PointCloud<PointXYZRGB> cloud) {
  PointCloud<PointXYZI> output_cloud;
  output_cloud.height = cloud.height;
  output_cloud.width = cloud.width;
  output_cloud.header.frame_id = cloud.header.frame_id;

  for (int i = 0; i < normals.points.size(); i++) {
    float dot = -normals.points[i].normal_y;
    float theta = std::acos(dot);

    PointXYZRGB point = cloud.points[i];
    PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    if (point.y > 0.15 or point.y < -0.2) {
      p.intensity = 1;
    } else if (theta > ANGLE_THRESHOLD && theta < M_PI - ANGLE_THRESHOLD) {
      p.intensity = 2;
    } else {
      p.intensity = 0;
    }

    output_cloud.points.push_back(p);
  }

  return output_cloud;
}

void goOrNoGo(PointCloud<PointXYZI> cloud, float distance) {
  std_msgs::Bool go;
  go.data = true;

  /*int count = 0;
  for (int i = 0; i < cloud.points.size(); i++) {
    PointXYZI point = cloud.points[i];
    if (point.intensity == 0) {
      count++;
    }
  }

  if (count < (0.85 * cloud.points.size())) {
    go.data = false;
  }*/

  if (distance < 0.25) {
    go.data = false;
  }
  pub_go.publish(go);
}

void orientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr &vector_msg) {
  roll = vector_msg->vector.y;
  pitch = vector_msg->vector.x;
}

void pointcloudCallback(const PointCloud<PointXYZRGB>::ConstPtr &cloud) {
  PointCloud<PointXYZRGB>::Ptr cloud_converted(
      new PointCloud<PointXYZRGB>(*cloud));

  PointCloud<PointXYZRGB>::Ptr cloud_transformed = frameTransform(cloud_converted);

  tic();
  float d = findEdge(*cloud_transformed);
  ROS_ERROR("Edges: %f, %f", toc(), d);

  downSample(cloud_transformed, 0.05, 0.05, 0.05);
  //cutOff(cloud_transformed);
  
  PointCloud<PointXYZRGB>::Ptr cloud_orientation_corrected = 
	  orientationCorrection(cloud_transformed);

  PointCloud<Normal>::Ptr cloud_normals =
      computeNormals(cloud_orientation_corrected, 0.05);
  PointCloud<PointXYZI> output =
      normalAnalysis(*cloud_normals, *cloud_orientation_corrected);

  goOrNoGo(output, d);

  pub_cloud.publish(output);
  //updateViewer(viewer, cloud_orientation_corrected, cloud_normals);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;

  n.getParam("/pointcloud_analysis/ROVER_WIDTH", ROVER_WIDTH);
  n.getParam("/pointcloud_analysis/CAMERA_ANGLE", CAMERA_ANGLE);
  n.getParam("/pointcloud_analysis/CAMERA_HEIGHT", CAMERA_HEIGHT);
  n.getParam("/pointcloud_analysis/ANGLE_THRESHOLD", ANGLE_THRESHOLD);
  n.getParam("/pointcloud_analysis/Y_MAX", Y_MAX);
  n.getParam("/pointcloud_analysis/Y_MIN", Y_MIN);

  //viewer = createViewer();

  ros::Subscriber sub_orientation = n.subscribe("/imu/rpy", 1, orientationCallback);
  ros::Subscriber sub_cloud = n.subscribe<PointCloud<PointXYZRGB>>("/camera/depth_registered/points", 1, pointcloudCallback);

  pub_cloud = n.advertise<PointCloud<PointXYZI>>("/pointcloud/analysis/points", 1);
  pub_go = n.advertise<std_msgs::Bool>("/pointcloud/analysis/go", 1);
  
  ros::spin();
  return 0;
}
