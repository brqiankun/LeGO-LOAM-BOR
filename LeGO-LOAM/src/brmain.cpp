#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include <chrono>
#include <typeinfo>

#include <rosgraph_msgs/Clock.h>



int main(int argc, char** argv) {
  ros::init(argc, argv, "br_lego_loam_bor");

  ros::NodeHandle nh("~");
  std::string lidar_topic;

  nh.getParam("lidar_topic", lidar_topic);

  // one writer and one reader
  Channel<ProjectionOut> projection_out_channel(true);
  Channel<AssociationOut> association_out_channel(true);

  ImageProjection IP(nh, projection_out_channel);

  FeatureAssociation FA(nh, projection_out_channel, association_out_channel);

  MapOptimization MO(nh, association_out_channel);

  TransformFusion TF(nh);

  ROS_INFO("\033[1;32m---->\033[0m LeGO-LOAM Started.");

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, &ImageProjection::cloudHandler, &IP);
  ROS_INFO("------------------------ImageProjection::cloudHandler algorithm register done-----------------------");

  ros::spin();

  ros::shutdown();
  return 0;

} 