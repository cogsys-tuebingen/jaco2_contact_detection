#include <jaco2_kin_dyn_lib/jaco2_kinematic_model.h>
#include <jaco2_kin_dyn_lib/yaml_to_kdl_tranform.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

sensor_msgs::JointState state;

void cb(const sensor_msgs::JointStateConstPtr& msg){
    state = *msg;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n("~");
    ros::Rate r(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker2", 1);

    ros::Subscriber sub_joint_states = n.subscribe("/joint_states", 1, cb);

    std::string path;
    n.param("point_path", path, std::string("/home/zwiener/collision_data/configs/collision_points.yaml"));
    std::vector<Jaco2Yaml2KDLTransform::KDLTransformation> points;


    Jaco2Yaml2KDLTransform::load(path, points);

    Jaco2KinematicModel model("robot_description","jaco_link_base","jaco_link_hand");

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::ARROW;

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/jaco_link_5";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 1.3 * 0.0254;
        marker.pose.position.y = 0;
        marker.pose.position.z = -0.9 * 0.0254;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = sin(-15.0/180*M_PI);
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = cos(-15.0/180*M_PI);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.2;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        visualization_msgs::Marker marker2;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker2.header.frame_id = "/jaco_link_base";
        marker2.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker2.ns = "basic_shapes";
        marker2.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker2.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker2.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        if(state.position.size() > 6){
//            ROS_INFO_STREAM("pos" << state.position);
            std::vector<double> angles;
            angles.insert(angles.end(), state.position.begin(), state.position.begin() + 6 );
//            tf::Pose transform;
//            model.getFKPose(angles,transform, "jaco_link_5");
//            tf::Pose p511;
//            tf::Quaternion q = tf::Quaternion(0,0,sin(M_PI_2),cos(M_PI_2)) * tf::Quaternion(0,sin(15.0/180*M_PI),0,cos(15.0/180*M_PI));
//            tf::Vector3 v(-0.7 * 0.0254, 0, -2 * 0.0254);
//            tf::Vector3 end_p = tf::quatRotate(q,tf::Vector3(0.2,0,0));
//            p511.setOrigin(v+end_p);
//            p511.setRotation(q);
//            tf::Pose marker_pose = transform * p511;
//            tf::Pose p521;
//            tf::Quaternion q(0,sin(-15.0/180*M_PI),0,cos(-15.0/180*M_PI));
//            tf::Vector3 v(1.3 * 0.0254, 0, -0.9 * 0.0254);
//            tf::Vector3 end_p = tf::quatRotate(q,tf::Vector3(0.2,0,0));
//            p521.setOrigin(v+end_p);
//            p521.setRotation(q);
//            tf::Pose marker_pose = transform * p521;
//            tf::Pose p531;
//            tf::Vector3 v(0, 1.4 * 0.0254, -1.5 *0.0254);
//            tf::Quaternion q(0,0,sin(-M_PI_4),cos(-M_PI_4));
//            tf::Vector3 end_p = tf::quatRotate(q,tf::Vector3(0.2,0,0));
//            geometry_msgs::Vector3 vmsg;
//            geometry_msgs::Quaternion qmsg;
//            tf::vector3TFToMsg(v, vmsg);
//            tf::quaternionTFToMsg(q, qmsg);
//            ROS_INFO_STREAM("position: x " << vmsg   );
//            ROS_INFO_STREAM("orientation: "<< qmsg );
////            p531.setOrigin(v+end_p);
//            p531.setRotation(q);
//            tf::Pose marker_pose = transform * p531;


//            marker2.pose.position.x = marker_pose.getOrigin().getX();
//            marker2.pose.position.y = marker_pose.getOrigin().getY();
//            marker2.pose.position.z = marker_pose.getOrigin().getZ();
//            marker2.pose.orientation.x = marker_pose.getRotation().getX();
//            marker2.pose.orientation.y = marker_pose.getRotation().getY();
//            marker2.pose.orientation.z = marker_pose.getRotation().getZ();
//            marker2.pose.orientation.w = marker_pose.getRotation().getW();


            KDL::Frame transform;
            model.getFKPose(angles,transform, points[0].parent);
            std::cout << points[0].name << std::endl;
            KDL::Frame pframe = points[0].frame;

            KDL::Vector force = pframe.M * KDL::Vector(0.2,0,0);

            KDL::Frame force_frame(pframe.M,pframe.p + force);
            KDL::Frame marker_frame = transform * force_frame;


            marker2.pose.position.x = marker_frame.p(0);
            marker2.pose.position.y = marker_frame.p(1);
            marker2.pose.position.z = marker_frame.p(2);
            marker_frame.M.GetQuaternion(marker2.pose.orientation.x, marker2.pose.orientation.y, marker2.pose.orientation.z, marker2.pose.orientation.w );



            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker2.scale.x = -0.2;
            marker2.scale.y = 0.01;
            marker2.scale.z = 0.01;

            // Set the color -- be sure to set alpha to something non-zero!
            marker2.color.r = 0.0f;
            marker2.color.g = 1.0f;
            marker2.color.b = 1.0f;
            marker2.color.a = 1.0;

            marker2.lifetime = ros::Duration();
            marker_pub2.publish(marker2);
        }

        // Publish the marker
//        while (marker_pub.getNumSubscribers() < 1)
//        {
//            if (!ros::ok())
//            {
//                return 0;
//            }
//            ROS_WARN_ONCE("Please create a subscriber to the marker");
//            sleep(1);
//        }
        marker_pub.publish(marker);

        // Cycle between different shapes
        //    switch (shape)
        //    {
        //    case visualization_msgs::Marker::CUBE:
        //      shape = visualization_msgs::Marker::SPHERE;
        //      break;
        //    case visualization_msgs::Marker::SPHERE:
        //      shape = visualization_msgs::Marker::ARROW;
        //      break;
        //    case visualization_msgs::Marker::ARROW:
        //      shape = visualization_msgs::Marker::CYLINDER;
        //      break;
        //    case visualization_msgs::Marker::CYLINDER:
        //      shape = visualization_msgs::Marker::CUBE;
        //      break;
        //    }
        ros::spinOnce();

        r.sleep();
    }
}
