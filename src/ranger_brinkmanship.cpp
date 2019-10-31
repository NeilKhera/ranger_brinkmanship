#include <math.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ranger_brinkmanship/Distances.h>
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

std::string POINTCLOUD_ROSTOPIC;

float ROVER_OUTER_WIDTH;
float ROVER_INNER_WIDTH;
float CAMERA_ANGLE;
float CAMERA_HEIGHT;
float CAMERA_X_LIMIT;
float ROLL_THRESHOLD;
float ASCENDING_ANGLE_THRESHOLD;
float DESCENDING_ANGLE_THRESHOLD;
float EDGE_THRESHOLD;
float Z_MAX;
float Z_MIN;

float roll = 0.0;
float pitch = 0.0;

ros::Publisher pub_cloud;
ros::Publisher pub_diag;
ros::Publisher pub_go;

PointCloud<PointXYZRGB>::Ptr frameTransform(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.translation() << 0.0, 0.0, CAMERA_HEIGHT;
  transform_matrix.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));
  transform_matrix.rotate(Eigen::AngleAxisf(-(M_PI / 2) - CAMERA_ANGLE, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

PointCloud<PointXYZRGB>::Ptr orientationCorrection(PointCloud<PointXYZRGB>::Ptr cloud) {
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  transform_matrix.rotate(Eigen::AngleAxisf(-pitch, Eigen::Vector3f::UnitY()));
  transform_matrix.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

  PointCloud<PointXYZRGB>::Ptr cloud_corrected (new PointCloud<PointXYZRGB>());
  transformPointCloud(*cloud, *cloud_corrected, transform_matrix);
  return cloud_corrected;
}

float getPlaneFalloff(PointCloud<PointXYZRGB> cloud) {
  float prev_z[cloud.width] = {};
  float prev_x[cloud.width] = {};

  int spacing = 10;
  int roverBodyPointRemoval = 20;
  for (int i = cloud.height - 1 - roverBodyPointRemoval; i >= 0; i -= spacing) {
    for (int j = 0; j < cloud.width; j += spacing) {
      PointXYZRGB point = cloud.at(j, i);

      if (!isnan(point.x)) {
	float deltaZ = point.z - prev_z[j];
	float dispY = std::abs(point.y);
        if ((deltaZ > Z_MAX || deltaZ < Z_MIN) && dispY > ROVER_INNER_WIDTH / 2 && dispY < ROVER_OUTER_WIDTH / 2) {
	  return prev_x[j];
        }
        prev_z[j] = point.z;
	prev_x[j] = point.x;
      }
    }
  }
  
  float x_min = prev_x[0];
  for (int i = 0; i < cloud.width; i += spacing) {
    if (prev_x[i] < x_min) {
      x_min = prev_x[i];
    }
  }
  return x_min;
}

void downSample(PointCloud<PointXYZRGB>::Ptr cloud, double x_dim, double y_dim, double z_dim) {
  VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(x_dim, y_dim, z_dim);
  vox.setFilterFieldName("x");
  vox.setFilterLimits(0.1, CAMERA_X_LIMIT);
  vox.filter(*cloud);
}

void cutOff(PointCloud<PointXYZRGB>::Ptr cloud, float edgeDistance) {
  PassThrough<PointXYZRGB> pt;
  pt.setInputCloud(cloud);
  pt.setFilterFieldName("y");
  pt.setFilterLimits(-ROVER_OUTER_WIDTH / 2, ROVER_OUTER_WIDTH / 2);
  pt.filter(*cloud);
}

float getObstructionDistance(PointCloud<PointXYZRGB>::Ptr cloud) {
  float min_x = CAMERA_X_LIMIT;
  for (int i = 0; i < cloud->points.size(); i++) {
    PointXYZRGB point = cloud->points[i];
    if (point.z > Z_MAX && point.x < min_x) {
      min_x = point.x;
    }
  }
  return min_x;
}

void marking(float* z_min, float* z_max, int intensity, KdTreeFLANN<PointXYZI> kdtree, PointXYZI search_point, PointCloud<PointXYZI>::Ptr cloud, PointCloud<PointXYZI>::Ptr temp_cloud, PointIndices::Ptr removals) {
  (*temp_cloud).push_back(search_point);
  
  if (search_point.z < *z_min) {
    *z_min = search_point.z;
  }

  if (search_point.z > *z_max) {
    *z_max = search_point.z;
  }
	
  std::vector<int> pointIndices;
  std::vector<float> pointDistancesSq;
  float radius = 0.06;
  
  kdtree.radiusSearch(search_point, radius, pointIndices, pointDistancesSq);
  for (int i = 0; i < pointIndices.size(); i++) {
    PointXYZI point = cloud->points[pointIndices[i]];
    if (point.intensity == intensity) {
      cloud->points[pointIndices[i]].intensity = 0;
      removals->indices.push_back(pointIndices[i]);
      marking(z_min, z_max, intensity, kdtree, point, cloud, temp_cloud, removals);
    }
  }
}

float distance(PointXYZI point) {
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

float markObstacles(PointCloud<PointXYZI>::Ptr cloud) {
  KdTreeFLANN<PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  PointCloud<PointXYZI>::Ptr pointholder_cloud(new PointCloud<PointXYZI>());
  PointIndices::Ptr removals(new PointIndices());
 
  int obstacle_num = 1;
  float d_min = CAMERA_X_LIMIT;
  for (int i = 0; i < cloud->points.size(); i++) {
    PointXYZI point = cloud->points[i];
    if (point.intensity == 1 || point.intensity == 2) {
      PointCloud<PointXYZI>::Ptr temp_cloud(new PointCloud<PointXYZI>());

      float* z_min = new float(point.z);
      float* z_max = new float(point.z);

      cloud->points[i].intensity = 0;
      removals->indices.push_back(i);
      marking(z_min, z_max, point.intensity, kdtree, point, cloud, temp_cloud, removals);

      if (*z_max - *z_min > Z_MAX) {
	float d_curr = distance(temp_cloud->points[0]);
        for (int j = 0; j < temp_cloud->points.size(); j++) {
	  if (distance(temp_cloud->points[j]) < d_curr) {
            d_curr = distance(temp_cloud->points[j]);
	  }
	  temp_cloud->points[j].intensity = obstacle_num;
          (*pointholder_cloud).push_back(temp_cloud->points[j]);
	}
	if (d_curr < d_min) {
	  d_min = d_curr;
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

  return d_min;
}

PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, double radius) {
  PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
  search::KdTree<PointXYZRGB>::Ptr kdtree(new search::KdTree<PointXYZRGB>());

  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setInputCloud(cloud);
  ne.setViewPoint(CAMERA_HEIGHT, 0.0, 0.0);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);

  return normals;
}

PointCloud<PointXYZI>::Ptr normalAnalysis(PointCloud<Normal>::Ptr normals, PointCloud<PointXYZRGB>::Ptr cloud) {
  PointCloud<PointXYZI>::Ptr output_cloud(new PointCloud<PointXYZI>());
  output_cloud->header.frame_id = cloud->header.frame_id;

  for (int i = 0; i < normals->points.size(); i++) {
    float dot = normals->points[i].normal_z;
    float theta = std::acos(dot);

    PointXYZRGB point = cloud->points[i];
    PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;

    if (normals->points[i].normal_x < 0 && theta > ASCENDING_ANGLE_THRESHOLD && theta < M_PI - ASCENDING_ANGLE_THRESHOLD) {
      p.intensity = 1;
    } else if (normals->points[i].normal_x >= 0 && theta > DESCENDING_ANGLE_THRESHOLD && theta < M_PI - DESCENDING_ANGLE_THRESHOLD) {
      p.intensity = 2;
    } else {
      p.intensity = 0;
    }
    output_cloud->points.push_back(p);
  }

  return output_cloud;
}

void goOrNoGo(float edgeDistance, float obstructionDistance, float obstacleDistance) {
  std_msgs::Bool go;
  go.data = true;

  if (roll > ROLL_THRESHOLD || roll < -ROLL_THRESHOLD || 
      pitch > ASCENDING_ANGLE_THRESHOLD || pitch < -DESCENDING_ANGLE_THRESHOLD ||
      edgeDistance < EDGE_THRESHOLD || obstructionDistance < EDGE_THRESHOLD || obstacleDistance < EDGE_THRESHOLD) {
    go.data = false;
  }
  pub_go.publish(go);
}

void orientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr &vector_msg) {
  roll = vector_msg->vector.y;
  pitch = vector_msg->vector.x;
}

void pointcloudCallback(const PointCloud<PointXYZRGB>::ConstPtr &cloud) {
  ranger_brinkmanship::Distances dists;

  PointCloud<PointXYZRGB>::Ptr cloud_converted(new PointCloud<PointXYZRGB>(*cloud));
  PointCloud<PointXYZRGB>::Ptr cloud_transformed = frameTransform(cloud_converted);

  float edgeDistance = getPlaneFalloff(*cloud_transformed);
  dists.edge = edgeDistance;

  downSample(cloud_transformed, 0.025, 0.025, 0.025);
  cutOff(cloud_transformed, edgeDistance);

  float obstructionDistance = getObstructionDistance(cloud_transformed);
  dists.obstruction = obstructionDistance;

  PointCloud<PointXYZRGB>::Ptr cloud_oriented = orientationCorrection(cloud_transformed);
  PointCloud<Normal>::Ptr cloud_normals = computeNormals(cloud_oriented, 0.08);
  PointCloud<PointXYZI>::Ptr cloud_analysed = normalAnalysis(cloud_normals, cloud_oriented);

  float obstacleDistance = markObstacles(cloud_analysed);
  dists.obstacle = obstacleDistance;

  pub_diag.publish(dists);
  goOrNoGo(edgeDistance, obstructionDistance, obstacleDistance);
  pub_cloud.publish(*cloud_analysed);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ranger_brinkmanship");
  ros::NodeHandle n;

  n.getParam("/ranger_brinkmanship/POINTCLOUD_ROSTOPIC", POINTCLOUD_ROSTOPIC);
  n.getParam("/ranger_brinkmanship/ROVER_OUTER_WIDTH", ROVER_OUTER_WIDTH);
  n.getParam("/ranger_brinkmanship/ROVER_INNER_WIDTH", ROVER_INNER_WIDTH);
  n.getParam("/ranger_brinkmanship/CAMERA_ANGLE", CAMERA_ANGLE);
  n.getParam("/ranger_brinkmanship/CAMERA_HEIGHT", CAMERA_HEIGHT);
  n.getParam("/ranger_brinkmanship/CAMERA_X_LIMIT", CAMERA_X_LIMIT);
  n.getParam("/ranger_brinkmanship/ROLL_THRESHOLD", ROLL_THRESHOLD);
  n.getParam("/ranger_brinkmanship/ASCENDING_ANGLE_THRESHOLD", ASCENDING_ANGLE_THRESHOLD);
  n.getParam("/ranger_brinkmanship/DESCENDING_ANGLE_THRESHOLD", DESCENDING_ANGLE_THRESHOLD);
  n.getParam("/ranger_brinkmanship/EDGE_THRESHOLD", EDGE_THRESHOLD);
  n.getParam("/ranger_brinkmanship/Z_MAX", Z_MAX);
  n.getParam("/ranger_brinkmanship/Z_MIN", Z_MIN);

  ros::Subscriber sub_orientation = n.subscribe("/ak2/imu/rpy", 1, orientationCallback);
  ros::Subscriber sub_cloud = n.subscribe<PointCloud<PointXYZRGB>>(POINTCLOUD_ROSTOPIC, 1, pointcloudCallback);

  pub_cloud = n.advertise<PointCloud<PointXYZI>>("/ranger/brinkmanship/points", 1);
  pub_diag = n.advertise<ranger_brinkmanship::Distances>("/ranger/brinkmanship/diagnostics", 1);
  pub_go = n.advertise<std_msgs::Bool>("/ranger/brinkmanship/go", 1);
  
  ros::spin();
  return 0;
}
