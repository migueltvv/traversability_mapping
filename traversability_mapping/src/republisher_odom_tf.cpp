#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>


std::string frame_id, child_frame_id, input_posEndEffector;
tf::TransformBroadcaster *odom_broadcaster;

void EndEffector_cb (const nav_msgs::Odometry& msg)
{

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = msg.pose.pose.position.x;
    odom_trans.transform.translation.y = msg.pose.pose.position.y;
    odom_trans.transform.translation.z = msg.pose.pose.position.z;
    odom_trans.transform.rotation = msg.pose.pose.orientation;

    odom_broadcaster->sendTransform(odom_trans);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lowlevel_republisher");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");
  
  pn.param<std::string>("input_posEndEffector", input_posEndEffector, "endEffector_odom");
  pn.param<std::string>("child_frame_id", child_frame_id, "end_effector_link");
  pn.param<std::string>("frame_id", frame_id, "base_link");
  
  // Create a ROS subscriber for the input posEndEffector from low_level
  ros::Subscriber sub = nh.subscribe (input_posEndEffector, 1, EndEffector_cb);

  // tf broadcaster
  odom_broadcaster = new tf::TransformBroadcaster();

  ROS_INFO("input_topic: %s",input_posEndEffector.c_str());
  ROS_INFO("child_frame_id: %s",child_frame_id.c_str());
  ROS_INFO("frame_id: %s",frame_id.c_str());

  // Spinv
  ros::spin ();
}
