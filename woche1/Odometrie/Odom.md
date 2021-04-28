# Odometrie Datenformat

## Husky Implementierung

* drei Topics bereits implementiert

|Topic                         | Sources               | Description                               |
|------------------------------|-----------------------|-------------------------------------------|
|husky_velocity_controller/odom| husky_node            | Raw odometry as read from Husky encoders  |
|imu/data                      |	mu_filter_madgwick | Orientation estimate from the IMU         |
| odometry/filtered            | ekf_localization_node | Fused odometry estimate (encoders and IMU)|

## nav_msgs/Odometry.msg
```
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

<u><b>Beispiel:</b></u>

```
//publishing via ROS msgs
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
//set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
   
//set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
  
//publish the message
    odom_pub.publish(odom);
```
