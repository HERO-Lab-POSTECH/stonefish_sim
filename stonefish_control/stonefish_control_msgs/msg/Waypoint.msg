# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Waypoint definition for trajectory control
# Coordinate: World frame NED (North-East-Down)
# Control: 4DOF (XYZ + Yaw, Roll=0, Pitch=0)

std_msgs/Header header
geometry_msgs/Point point          # Position [X, Y, Z] in world frame
float64 heading                    # Yaw angle in radians (4DOF)
float64 max_forward_speed          # Speed limit to this waypoint (m/s)
bool use_fixed_heading             # True: use specified heading, False: auto-calculate
float64 radius_of_acceptance       # Acceptance radius (meters)
