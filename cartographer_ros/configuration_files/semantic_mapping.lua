include "semantic.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 5,--4
  }
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1 -- 0.2

POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 1. 
POSE_GRAPH.constraint_builder.max_constraint_distance = 15. 
POSE_GRAPH.constraint_builder.min_score = 0.45 

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6 --0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)--30.

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5 --1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5--1e5

POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2--1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4--1e5

POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 0.5e1--1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e4--1e2

return options