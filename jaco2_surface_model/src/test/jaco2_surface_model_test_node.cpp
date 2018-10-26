//#include <jaco2_surface_model/parse_stl.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
//#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jaco2_surface_test_node");
  ros::NodeHandle n("~");

  ros::Publisher pub = n.advertise<visualization_msgs::Marker>("point_on_surface",1);

  std::string obj_filename= "/home/zwiener/workspace/development/src/jaco2/jaco2_contact_detection/jaco2_surface_model/jaco2_surface_meshes/jaco2_link_2.obj";
  MyMesh mesh;
  mesh.request_vertex_normals();
  // assure we have vertex normals
  if (!mesh.has_vertex_normals())
  {
    std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    return 1;
  }
  OpenMesh::IO::Options opt;
  if (!OpenMesh::IO::read_mesh(mesh, obj_filename, opt)){
    std::cerr << "read error\n";
    exit(1);
  }
  // If the file did not provide vertex normals, then calculate them
  if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) )
  {
    // we need face normals to update the vertex normals
    mesh.request_face_normals();
    // let the mesh update the normals
    mesh.update_normals();
    // dispose the face normals, as we don't need them anymore
    mesh.release_face_normals();
  }



  MyMesh::VertexIter v_it = mesh.vertices_begin()+200;
  // circulate around the current vertex
  MyMesh::Point p = mesh.point(*v_it);
  MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it);
  MyMesh::Point pi = mesh.point(*vv_it);
  MyMesh::Point v = pi -p;
  MyMesh::Normal ni = mesh.normal(*vv_it);
  double s = 0.1;
  MyMesh::Point pn = pi + s * v;
  visualization_msgs::Marker msg;
  msg.type = visualization_msgs::Marker::ARROW;
  msg.action = visualization_msgs::Marker::MODIFY;
  msg.type = visualization_msgs::Marker::ARROW;
  msg.scale.x = 0.005;
  msg.scale.y = 0.01;
  msg.scale.z = 0.01;
  msg.header.frame_id = "jaco_link_2";
  msg.header.stamp = ros::Time::now();
  msg.pose.orientation.w = 1.0;
  msg.color.a = 0.8;
  msg.points.resize(2);


  msg.color.a = 0.8;
  msg.color.r = 0.5;
  msg.color.g = 0;
  msg.color.b = 0.5;
  msg.points[0].x = pn[0];
  msg.points[0].y = pn[1];
  msg.points[0].z = pn[2];
  msg.points[1].x = pn[0] + .1 * ni[0];
  msg.points[1].y = pn[1]+ .1 * ni[1];
  msg.points[1].z = pn[2]+ .1 * ni[2];


//    double yaw  = std::atan2(ni[1],ni[0]);
//    double pitch = std::atan2(hypot(ni[0],ni[1]),ni[2]);
//    double roll =0;
//    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

//  for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
//  {
//    MyMesh::Point pi = mesh.point(*vv_it);
//    MyMesh::Normal ni = mesh.normal(*vv_it);
    // do something with e.g. mesh.point(*vv_it)
//    mesh.point()
//  }
//  msg.poses.push_back(pose);
  ros::Rate r(10);
  while(ros::ok()){
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
