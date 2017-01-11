#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
//#include <cstdio>

//std::string static_turtle_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "contact_points_tf_broadcaster");

  ros::NodeHandle nh("~");
  std::string path;
  nh.param("contact_points_file", path, std::string(""));
//  static_turtle_name = argv[1];
//  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//  geometry_msgs::TransformStamped static_transformStamped;

//  static_transformStamped.header.stamp = ros::Time::now();
//  static_transformStamped.header.frame_id = "world";
//  static_transformStamped.child_frame_id = static_turtle_name;
//  static_transformStamped.transform.translation.x = atof(argv[2]);
//  static_transformStamped.transform.translation.y = atof(argv[3]);
//  static_transformStamped.transform.translation.z = atof(argv[4]);
//  tf::Quaternion quat;
//  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
//  static_transformStamped.transform.rotation.x = quat.x();
//  static_transformStamped.transform.rotation.y = quat.y();
//  static_transformStamped.transform.rotation.z = quat.z();
//  static_transformStamped.transform.rotation.w = quat.w();
//  static_broadcaster.sendTransform(static_transformStamped);
//  ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());
//  ros::spin();
  return 0;
}
