#include "tic_toc.h"

#include <math.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

using namespace pcl;

float ROVER_OUTER_WIDTH;
float ROVER_INNER_WIDTH;
float CAMERA_ANGLE;
float CAMERA_HEIGHT;
float ASCENDING_ANGLE_THRESHOLD;
float DESCENDING_ANGLE_THRESHOLD;
float EDGE_THRESHOLD;
float Y_MAX;
float Y_MIN;

float roll = 0.0;
float pitch = 0.0;

ros::Publisher pub_cloud;
ros::Publisher pub_go;

PointCloud<PointXYZRGB>::Ptr frameTransform(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << 0.0, -CAMERA_HEIGHT, 0.0;
  transform_matrix.rotate(Eigen::AngleAxisf(-CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

PointCloud<PointXYZRGB>::Ptr orientationCorrection(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));
  transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

float getPlaneFalloff(PointCloud<PointXYZRGB> cloud) {
  float prev_y[cloud.width] = {};
  float prev_z[cloud.width] = {};

  int spacing = 10;
  for (int i = cloud.height - 1; i >= 0; i -= spacing) {
    for (int j = 0; j < cloud.width; j += spacing) {
      PointXYZRGB point = cloud.at(j, i);

      if (!isnan(point.z)) {
        if (std::abs(point.y - prev_y[j]) > -Y_MIN && std::abs(point.x) > ROVER_INNER_WIDTH / 2 && std::abs(point.x) < ROVER_OUTER_WIDTH / 2) {
	  return prev_z[j];
        }
        prev_y[j] = point.y;
	prev_z[j] = point.z;
      }
    }
  }
  
  float z_min = prev_z[0];
  for (int i = 0; i < cloud.width; i += spacing) {
    if (prev_z[i] < z_min) {
      z_min = prev_z[i];
    }
  }
  return z_min;
}

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double x_dim, double y_dim, double z_dim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  vox.filter(*cloud);
}

void cutOff(PointCloud<PointXYZRGB>::Ptr cloud, float edgeDistance) {
  PassThrough<PointXYZRGB> pt;
  pt.setInputCloud(cloud);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(0.0, edgeDistance);
  pt.filter(*cloud);
  pt.setFilterFieldName("x");
  pt.setFilterLimits(-ROVER_OUTER_WIDTH / 2, ROVER_OUTER_WIDTH / 2);
  pt.filter(*cloud);
}

float getObstacleDistance(PointCloud<PointXYZRGB>::Ptr cloud) {
  float min_z = -1.0;
  for (int i = 0; i < cloud->points.size(); i++) {
    PointXYZRGB point = cloud->points[i];
    if (point.y > Y_MAX && (point.z < min_z || min_z < 0)) {
      min_z = point.z;
    }
  }
  return min_z;
}

void marking(int intensity, KdTreeFLANN<PointXYZI> kdtree, PointXYZI search_point, PointCloud<PointXYZI>::Ptr cloud, PointCloud<PointXYZI>::Ptr temp_cloud, PointIndices::Ptr removals) {
  (*temp_cloud).push_back(search_point);
	
  std::vector<int> pointIndices;
  std::vector<float> pointDistancesSq;
  float radius = 0.05;
  
  kdtree.radiusSearch(search_point, radius, pointIndices, pointDistancesSq);
  for (int i = 0; i < pointIndices.size(); i++) {
    PointXYZI point = cloud->points[pointIndices[i]];
    if (point.intensity == intensity) {
      cloud->points[pointIndices[i]].intensity = 0;
      removals->indices.push_back(pointIndices[i]);
      marking(intensity, kdtree, point, cloud, temp_cloud, removals);
    }
  }
}

PointCloud<PointXYZI>::Ptr markObstacles(PointCloud<PointXYZI>::Ptr cloud) {
  KdTreeFLANN<PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  PointCloud<PointXYZI>::Ptr pointholder_cloud(new PointCloud<PointXYZI>());
  PointIndices::Ptr removals(new PointIndices());
  
  int obstacle_num = 1;
  for (int i = 0; i < cloud->points.size(); i++) {
    PointXYZI point = cloud->points[i];
    if (point.intensity == 1 || point.intensity == 2) {
      PointCloud<PointXYZI>::Ptr temp_cloud(new PointCloud<PointXYZI>());

      cloud->points[i].intensity = 0;
      removals->indices.push_back(i);
      marking(point.intensity, kdtree, point, cloud, temp_cloud, removals);

      if (temp_cloud->points.size() >= 15) {
        for (int j = 0; j < temp_cloud->points.size(); j++) {
	  temp_cloud->points[j].intensity = obstacle_num;
          (*pointholder_cloud).push_back(temp_cloud->points[j]);
	}
	obstacle_num++;
      }
    }
  }

  ExtractIndices<PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(removals);
  extract.setNegative(true);
  extract.filter(*cloud);

  for (int i = 0; i < pointholder_cloud->points.size(); i++) {
    (*cloud).push_back(pointholder_cloud->points[i]);
  }

  return pointholder_cloud;
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double radius) {
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  search::KdTree<PointXYZRGB>::Ptr kdtree(new search::KdTree<PointXYZRGB>());

  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud(cloud);
  ne.setViewPoint(0.0, CAMERA_HEIGHT, 0.0);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);

  return normals;
}

PointCloud<PointXYZI>::Ptr normalAnalysis(PointCloud<Normal>::Ptr normals, PointCloud<PointXYZRGB>::Ptr cloud) {
  PointCloud<PointXYZI>::Ptr output_cloud(new PointCloud<PointXYZI>());
  output_cloud->header.frame_id = cloud->header.frame_id;

  for (int i = 0; i < normals->points.size(); i++) {
    float dot = -normals->points[i].normal_y;
    float theta = std::acos(dot);

    PointXYZRGB point = cloud->points[i];
    PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    if (normals->points[i].normal_z < 0 && theta > ASCENDING_ANGLE_THRESHOLD && theta < M_PI - ASCENDING_ANGLE_THRESHOLD) {
      p.intensity = 1;
    } else if (normals->points[i].normal_z >= 0 && theta > DESCENDING_ANGLE_THRESHOLD && theta < M_PI - DESCENDING_ANGLE_THRESHOLD) {
      p.intensity = 2;
    } else {
      p.intensity = 0;
    }
    output_cloud->points.push_back(p);
  }

  return output_cloud;
}

void goOrNoGo(float distance) {
  std_msgs::Bool go;
  go.data = true;

  if (distance < EDGE_THRESHOLD) {
    go.data = false;
  }
  pub_go.publish(go);
}

void orientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr &vector_msg) {
  roll = vector_msg->vector.y;
  pitch = vector_msg->vector.x;
}

void pointcloudCallback(const PointCloud<PointXYZRGB>::ConstPtr &cloud) {
  PointCloud<PointXYZRGB>::Ptr cloud_converted(new PointCloud<PointXYZRGB>(*cloud));
  PointCloud<PointXYZRGB>::Ptr cloud_transformed = frameTransform(cloud_converted);

  float edgeDistance = getPlaneFalloff(*cloud_transformed);
  ROS_ERROR("Edge: %f", edgeDistance);

  downSample(cloud_transformed, 0.05, 0.05, 0.05);
  cutOff(cloud_transformed, edgeDistance);

  float obstacleDistance = getObstacleDistance(cloud_transformed);
  ROS_ERROR("Obstacle: %f", obstacleDistance);

  PointCloud<PointXYZRGB>::Ptr cloud_oriented = orientationCorrection(cloud_transformed);
  PointCloud<Normal>::Ptr cloud_normals = computeNormals(cloud_oriented, 0.05);
  PointCloud<PointXYZI>::Ptr cloud_analysed = normalAnalysis(cloud_normals, cloud_oriented);

  PointCloud<PointXYZI>::Ptr cloud_obstacles = markObstacles(cloud_analysed);
  
  goOrNoGo(edgeDistance);
  pub_cloud.publish(*cloud_analysed);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_analysis");
  ros::NodeHandle n;

  n.getParam("/pointcloud_analysis/ROVER_OUTER_WIDTH", ROVER_OUTER_WIDTH);
  n.getParam("/pointcloud_analysis/ROVER_INNER_WIDTH", ROVER_INNER_WIDTH);
  n.getParam("/pointcloud_analysis/CAMERA_ANGLE", CAMERA_ANGLE);
  n.getParam("/pointcloud_analysis/CAMERA_HEIGHT", CAMERA_HEIGHT);
  n.getParam("/pointcloud_analysis/ASCENDING_ANGLE_THRESHOLD", ASCENDING_ANGLE_THRESHOLD);
  n.getParam("/pointcloud_analysis/DESCENDING_ANGLE_THRESHOLD", DESCENDING_ANGLE_THRESHOLD);
  n.getParam("/pointcloud_analysis/EDGE_THRESHOLD", EDGE_THRESHOLD);
  n.getParam("/pointcloud_analysis/Y_MAX", Y_MAX);
  n.getParam("/pointcloud_analysis/Y_MIN", Y_MIN);

  ros::Subscriber sub_orientation = n.subscribe("/ak2/imu/rpy", 1, orientationCallback);
  ros::Subscriber sub_cloud = n.subscribe<PointCloud<PointXYZRGB>>("/camera/depth_registered/points", 1, pointcloudCallback);

  pub_cloud = n.advertise<PointCloud<PointXYZI>>("/pointcloud/analysis/points", 1);
  pub_go = n.advertise<std_msgs::Bool>("/pointcloud/analysis/go", 1);
  
  ros::spin();
  return 0;
}
