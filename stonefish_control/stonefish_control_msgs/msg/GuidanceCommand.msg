# GuidanceCommand.msg
# Guidance layer output command for control layer
# Contains desired pose and speed reference

std_msgs/Header header

# Desired pose (4DOF for marine vehicles: x, y, z, yaw)
# Roll and pitch are typically set to 0 for underwater vehicles
geometry_msgs/Pose desired_pose

# Desired forward speed (m/s)
# Control layer should track this speed along the path
float64 desired_speed

# Cross-track error (m) - for monitoring and logging
# Positive: vehicle is to the right of the path
# Negative: vehicle is to the left of the path
float64 cross_track_error

# Along-track error (m) - for monitoring and logging
# Positive: vehicle is ahead of the virtual target
# Negative: vehicle is behind the virtual target
float64 along_track_error

# Path progress parameter (0.0 to 1.0)
# 0.0 = start of path, 1.0 = end of path
float64 path_progress
