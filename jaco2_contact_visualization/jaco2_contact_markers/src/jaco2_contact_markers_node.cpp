#include <jaco2_kin_dyn_lib/jaco2_kinematic_model.h>
#include <jaco2_kin_dyn_lib/yaml_to_kdl_tranform.h>

#include <jaco2_contact_msgs/Jaco2ContactMsgs.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

sensor_msgs::JointState state;
jaco2_contact_msgs::Jaco2ContactMsgs cmsg;

void cb(const sensor_msgs::JointStateConstPtr& msg){
    state = *msg;
}

void cb_contact(const jaco2_contact_msgs::Jaco2ContactMsgsConstPtr& msg){
    cmsg = *msg;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "contact_markers");
    ros::NodeHandle n("~");
    ros::Rate r(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker2", 1);

    ros::Subscriber sub_joint_states = n.subscribe("/jaco_arm_driver/out/joint_states", 1, cb);
    ros::Subscriber sub_contacts = n.subscribe("/csapex/contact", 1, cb_contact);

    std::string path;
    n.param("point_path", path, std::string("/home/zwiener/collision_data/configs/collision_points.yaml"));
    std::vector<Jaco2KinDynLib::KDLTransformation> points;

    int no_contact_label = 0;
    n.param("no_contact_label", no_contact_label, no_contact_label);

    Jaco2KinDynLib::load(path, points);

    std::map<int,Jaco2KinDynLib::KDLTransformation> transform_map;

    for(const Jaco2KinDynLib::KDLTransformation& point : points){
        std::string name = point.name;
        name.erase(0,1);
        int label = std::stoi(name);
        transform_map[label] = point;

    }

    Jaco2KinDynLib::Jaco2KinematicModel model("robot_description","jaco_link_base","jaco_link_hand");

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::ARROW;
    tf::TransformListener listener;
    while (ros::ok())
    {
//        visualization_msgs::Marker marker;
//        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
//        marker.header.frame_id = "/jaco_link_5";
//        marker.header.stamp = ros::Time::now();

//        // Set the namespace and id for this marker.  This serves to create a unique ID
//        // Any marker sent with the same namespace and id will overwrite the old one
//        marker.ns = "basic_shapes";
//        marker.id = 0;

//        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
//        marker.type = shape;

//        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
//        marker.action = visualization_msgs::Marker::ADD;

//        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//        marker.pose.position.x = 1.3 * 0.0254;
//        marker.pose.position.y = 0;
//        marker.pose.position.z = -0.9 * 0.0254;
//        marker.pose.orientation.x = 0.0;
//        marker.pose.orientation.y = sin(-15.0/180*M_PI);
//        marker.pose.orientation.z = 0.0;
//        marker.pose.orientation.w = cos(-15.0/180*M_PI);

//        // Set the scale of the marker -- 1x1x1 here means 1m on a side
//        marker.scale.x = 0.2;
//        marker.scale.y = 0.01;
//        marker.scale.z = 0.01;

//        // Set the color -- be sure to set alpha to something non-zero!
//        marker.color.r = 1.0f;
//        marker.color.g = 1.0f;
//        marker.color.b = 0.0f;
//        marker.color.a = 1.0;

//        marker.lifetime = ros::Duration();

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


        std::vector<double> angles;

        if(state.position.size() > 6){
            angles.insert(angles.end(), state.position.begin(), state.position.begin() + 6 );
        }
        std::cout << cmsg.label << std::endl;

        if(cmsg.label != no_contact_label){
//            const Jaco2KinDynLib::KDLTransformation& trans = transform_map[cmsg.label];
//            std::string parent = trans.parent;

//            KDL::Frame transform;
//            model.getFKPose(angles,transform, parent);

            tf::StampedTransform transform;
              try{
                listener.lookupTransform(marker2.header.frame_id, cmsg.header.frame_id,
                                         ros::Time(0), transform);
              }
              catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                r.sleep();
              }

            double fx = std::abs(0.1*cmsg.force);
//            std::cout << trans.name << std::endl;
            std::cout << cmsg.label << std::endl;
//            KDL::Frame pframe = trans.frame;

//            KDL::Vector force = pframe.M * KDL::Vector(fx,0,0);
//            KDL::Frame force_frame(pframe.M,pframe.p + force);
//            KDL::Frame marker_frame = transform * force_frame;

            tf::Pose pframe;
            tf::poseMsgToTF(cmsg.transform,pframe);
            tf::Vector3 force = tf::quatRotate(pframe.getRotation(), tf::Vector3(fx,0,0));
            tf::Pose force_frame;
            force_frame.setOrigin(force + pframe.getOrigin());
            force_frame.setRotation(pframe.getRotation());
            tf::Pose marker_frame = transform * force_frame;

            tf::poseTFToMsg(marker_frame,marker2.pose);

//            marker2.pose.position.x = marker_frame.p(0);
//            marker2.pose.position.y = marker_frame.p(1);
//            marker2.pose.position.z = marker_frame.p(2);
//            marker_frame.M.GetQuaternion(marker2.pose.orientation.x, marker2.pose.orientation.y, marker2.pose.orientation.z, marker2.pose.orientation.w );



            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker2.scale.x = -fx;
            marker2.scale.y = 0.01;
            marker2.scale.z = 0.01;

            // Set the color -- be sure to set alpha to something non-zero!
            marker2.color.r = 165.0f/255.0f;
            marker2.color.g = 3.0f/255.0f;
            marker2.color.b = 55.0f/255.0f;
            marker2.color.a = 1.0;

            marker2.lifetime = ros::Duration();
        }
        else{
            marker2.scale.x = 0.0001;
            marker2.scale.y = 0.0001;
            marker2.scale.z = 0.0001;
        }
        marker_pub2.publish(marker2);
        ros::spinOnce();
        r.sleep();
    }

//    marker_pub.publish(marker);


}

