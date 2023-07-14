include "map_builder.lua" --后端
include "trajectory_builder_semantic.lua" --前端的配置参数

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,--false
  publish_frame_projected_to_2d = false,
  publish_tracked_pose = true,--added by DYC
  publish_frame_projected_to_2d = true, --added by DYC
  use_pose_extrapolator = true,--true
  use_odometry = true,--false
  use_nav_sat = false,--false --only for backend, making the global more consistent, but local map worse
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1., --three
  odometry_sampling_ratio = 1.,  --one 1.
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,  --two
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options

