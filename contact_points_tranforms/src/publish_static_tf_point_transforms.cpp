#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <jaco2_kin_dyn_lib/yaml_to_kdl_tranform.h>
#include <kdl/kdl.hpp>
//#include <cstdio>

//std::string static_turtle_name;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "contact_points_tf_broadcaster");

    ros::NodeHandle nh("~");
    std::string path;
    std::string tf_prefix;
    double rate = 20;

    nh.param("contact_points_file", path, std::string(""));
    nh.param("node_rate", rate, 20.0);
    nh.param("tf_prefix", tf_prefix, std::string(""));
    std::vector<Jaco2KinDynLib::KDLTransformation> points;
    if(path != ""){
        Jaco2KinDynLib::load(path, points);
    }

    std::vector<tf2_ros::StaticTransformBroadcaster> static_broadcasters;
    for(const Jaco2KinDynLib::KDLTransformation& point : points)
    {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transformStamped;

        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = tf_prefix +  point.parent;
        static_transformStamped.child_frame_id  = tf_prefix + point.name;
        static_transformStamped.transform.translation.x = point.frame.p(0);
        static_transformStamped.transform.translation.y = point.frame.p(1);
        static_transformStamped.transform.translation.z = point.frame.p(2);

        double& qx = static_transformStamped.transform.rotation.x;
        double& qy = static_transformStamped.transform.rotation.y;
        double& qz = static_transformStamped.transform.rotation.z;
        double& qw = static_transformStamped.transform.rotation.w;
        point.frame.M.GetQuaternion(qx, qy, qz, qw);
        static_broadcaster.sendTransform(static_transformStamped);
        static_broadcasters.push_back(static_broadcaster);

    }
    //  static_turtle_name = argv[1];
    //  ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());
    ros::Rate r(rate);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
