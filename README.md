# Topics

## Published topics for external use

  - `geometry_msgs::Twist` on **/rb_obstcale/rb_obstcale/twist_cmd**

# Nodes


## actor_obstacle

Take input from all obstacle nodes and decide a velocity command to send based
on obstacle priority. Send a large velocity if no obstacles are detected.

  - Publish:
    - **Twist on "~/twist_cmd"**: `linear.z` will be used as a urgency rate 0.0 = no urgency, 1.0 = high urgency
    - **Uint32 on "/turn_flag/flag"**: Sends flags to the turn state machine
  - Subscribe:
    - Point on "/actor_obstacle/front_obstacle/obstacle_loc"
    - Point on "/actor_obstacle/road_obstacle/obstacle_loc"
    - UInt8 on "actor_obstacle/stop_sign_detection/stop_sign"


## road_obstacle

Look for obstructions in the car's path using the long-range top lidar

  - Publish:
    - **Point on "~/obstcale_loc"**: Represents approximate distance from nearest obstacle
    - **LaserScan on "~/debug_scan"**: Scan message displaying only objects that have been detected
  - Subscribe:
    - LaserScan on "/actor_input/lidar_top/scan"


## front_obstacle

Look for obstructions in the car's path using the front lidar.

  - Publish:
    - **Point on "~/obstacle_loc"**: Represents approximate distance from nearest obstacle
    - **LaserScan on "~/debug_scan"**: Scan showing only points detected as an obstacle (use in rviz)
  - Subscribe:
    - LaserScan on "/actor_input/lidar_front/scan"

## stop_sign_detection

Detects if there is a stop sign in front of the car.

  - Publish:
    - **UInt8 on "~/stop_sign"**: Boolean value, true if sign detected, false otherwise
    - **UInt32 on "~/sign_size"**: Size of sign in pixels
  - Subscribe:
    - <params/front_cam_in> *(loaded from config/params.yaml)*