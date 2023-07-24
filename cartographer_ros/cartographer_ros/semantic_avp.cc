#include "segmentation_ros_msgs/FreespaceInfo.h"
#include "segmentation_ros_msgs/FreespaceSet.h"
#include "segmentation_ros_msgs/Point2f.h"

#include <tf/tf.h>
#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <string.h>
#include <fstream>

#include "gflags/gflags.h"

#include "node_constants.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include <unistd.h>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace carto = ::cartographer;
using carto::transform::Rigid3d;

DEFINE_bool(task_mapping, true,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
// DEFINE_string(rtk_trj_filename, "default.gps",
//               "If non-empty, filename of a .gps file to save/load, containing "
//               "a rtk trajectory.");
DEFINE_string(save_rtk_trj_filename, "abc",
              "If non-empty, filename of a .gps file to save, containing "
              "a rtk trajectory.");
DEFINE_string(load_rtk_trj_filename, "efg",
              "If non-empty, filename of a .gps file to load, containing "
              "a rtk trajectory.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

ros::Publisher pointcloudPub;

ros::Time timesamp2rostime(int64_t timesamp){
    std::string suanz = std::to_string(timesamp);
    std::string sec_string = suanz.substr(0, 10);
    std::string nsec_string = suanz.substr(10, 9);
    while(nsec_string.length() < 9){
        nsec_string += "0";
    }
    return ros::Time(std::stoi(sec_string), std::stoi(nsec_string));
}

void freespaceCallback(const segmentation_ros_msgs::FreespaceSet::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& cloud = *ptr;
    auto fsi = msg->freespace_set;
    int time = msg->time_stamp;
    
   
    for(auto f:fsi){
        for(auto p:f.vehicle_points){
            pcl::PointXYZ point = {p.x,p.y,0};
            ptr->push_back(point);
        }
    }
    sensor_msgs::PointCloud2 pointcloudRos;
    pcl::toROSMsg(cloud, pointcloudRos);
    pointcloudRos.header.frame_id = "base_link";
    pointcloudRos.header.stamp = timesamp2rostime(msg->time_stamp);

    pointcloudPub.publish(pointcloudRos);
}

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "semantic_avp");
    ros::NodeHandle nh;
    ros::Subscriber freespaceSub = nh.subscribe("vision_rmd_freespace", 1000, freespaceCallback);
    pointcloudPub = nh.advertise<sensor_msgs::PointCloud2>("lidar_point_cloud", 10);
    ros::ServiceClient clientStartTrajectory = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>(cartographer_ros::kStartTrajectoryServiceName);
    ros::service::waitForService(cartographer_ros::kStartTrajectoryServiceName);
    
    if (FLAGS_task_mapping) {
        std::cout << "Starting mapping TASK!" << std::endl;
        cartographer_ros_msgs::StartTrajectory srvMapping;
        srvMapping.request.configuration_directory = FLAGS_configuration_directory;
        srvMapping.request.configuration_basename = FLAGS_configuration_basename;

        if (clientStartTrajectory.call(srvMapping)) {
            ROS_DEBUG("StartTrajectory OK!");
            std::cout << srvMapping.response.status.message << std::endl;
            std::cout << srvMapping.response.trajectory_id << std::endl;
            std::cout << srvMapping.response.status.code << std::endl;
        } else {
            ROS_ERROR("fail to call service StartTrajectory!");
            return 1;
        }
    } else {
        std::cout << "Starting localization TASK!" << std::endl;


        //get the history map trajectory data
        ros::ServiceClient client = nh.serviceClient<cartographer_ros_msgs::TrajectoryQuery>(cartographer_ros::kTrajectoryQueryServiceName);
        ros::service::waitForService(cartographer_ros::kTrajectoryQueryServiceName);
        cartographer_ros_msgs::TrajectoryQuery srvTrajectoryQuery;
        srvTrajectoryQuery.request.trajectory_id = 0;        
        if (client.call(srvTrajectoryQuery)) {
            ROS_INFO("TrajectoryQuery OK");
            std::cout << srvTrajectoryQuery.response.status.message << std::endl;
            std::cout << srvTrajectoryQuery.response.trajectory.size() << std::endl;
            std::cout << srvTrajectoryQuery.response.trajectory[0] << std::endl;
        }
        else {
            ROS_ERROR("Failed to call service TrajectoryQuery");
            return 1;
        }
        
        int i_Nearest = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.position.x = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.position.y = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.position.z = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.orientation.x = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.orientation.y = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.orientation.z = 0;
        srvTrajectoryQuery.response.trajectory[i_Nearest].pose.orientation.w = 1;

        std::cout << "Initial pose is: " << std::endl << srvTrajectoryQuery.response.trajectory[i_Nearest].pose << std::endl;

        cartographer_ros_msgs::StartTrajectory srvLocalization;
        srvLocalization.request.configuration_directory = FLAGS_configuration_directory;
        srvLocalization.request.configuration_basename = FLAGS_configuration_basename;
        srvLocalization.request.use_initial_pose = false;
        // srvLocalization.request.use_initial_pose = true;
        // srvLocalization.request.initial_pose = srvTrajectoryQuery.response.trajectory[i_Nearest].pose;
        srvLocalization.request.relative_to_trajectory_id = 0;

        bool startTrjCall = true;
        startTrjCall = clientStartTrajectory.call(srvLocalization);
        if (startTrjCall) {
            ROS_INFO("StartTrajectory OK");
            std::cout << srvLocalization.response.status.message << std::endl;
            std::cout << srvLocalization.response.trajectory_id << std::endl;
            std::cout << srvLocalization.response.status.code << std::endl;
        } else {
            ROS_INFO("Failed to call service StartTrajectory");
            return 1;
        }
    } 


    ros::spin();

    return 0;
}
