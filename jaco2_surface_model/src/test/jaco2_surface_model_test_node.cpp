//#include <jaco2_surface_model/parse_stl.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
//#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

MyMesh::VertexIter getVertex(const MyMesh& mesh, const MyMesh::Point& p)
{
    double min_dist = std::numeric_limits<double>::infinity();
    auto it_min = mesh.vertices_begin();
    std::size_t n_points = 0;
    for(auto it = mesh.vertices_begin(); it!=mesh.vertices_end(); ++it){
        MyMesh::Point pi =  mesh.point(*it);
        MyMesh::Point diff = pi - p;
        double d = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]+diff[2];
        if(d < min_dist){
            min_dist = d;
            it_min = it;
        }
        ++n_points;
    }
    std::cout << "min_dist: " << std::sqrt(min_dist) << " | #points: " << n_points << std::endl;
    return it_min;
}

MyMesh::Point getPoint(const MyMesh& mesh, const MyMesh::Point& p)
{
    double min_dist = std::numeric_limits<double>::infinity();
    MyMesh::Point p_min;
    std::size_t n_points = 0;
    auto it_min = mesh.faces_begin();
    for(auto it = mesh.faces_begin(); it!=mesh.faces_end(); ++it){
        MyMesh::Point pi =  mesh.calc_face_centroid(*it);
        MyMesh::Point diff = pi - p;
        double d = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]+diff[2];
        if(d < min_dist){
            min_dist = d;
            p_min=  pi;
            it_min = it;
        }
        ++n_points;
    }
    //    MyMesh::Face f = mesh.face(it_min);
    std::cout << "min_dist: " << std::sqrt(min_dist) << " | #points: " << n_points << std::endl;
    return p_min;
}

void visualizeVertices(const MyMesh& mesh, visualization_msgs::Marker& msg)
{
    for(auto it = mesh.vertices_begin(); it!=mesh.vertices_end(); ++it){
        MyMesh::Point piv =  mesh.point(*it);
        geometry_msgs::Point point;
        point.x = piv[0];
        point.y = piv[1];
        point.z = piv[2];
        msg.points.push_back(point);

    }
    msg.color.a = 0.8;
    msg.color.r = 0.5;
    msg.color.g = 0;
    msg.color.b = 0.5;
    msg.ns = "vertices";

    msg.scale.x = 0.005;
    msg.scale.y = 0.005;
    msg.scale.z = 0.005;
}

void visualizeOrigin(visualization_msgs::Marker& msg, visualization_msgs::MarkerArray& marray)
{
    msg.ns = "origin";
    msg.points.resize(2);
    msg.points[0].x = 0;
    msg.points[0].y = 0;
    msg.points[0].z = 0;
    msg.points[1].x = 0.2;
    msg.points[1].y = 0;
    msg.points[1].z = 0;
    ++msg.id;
    msg.type = visualization_msgs::Marker::ARROW;
    msg.scale.x = 0.001;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
    marray.markers.push_back(msg);
    msg.points[1].x = 0.0;
    msg.points[1].y = 0.2;
    msg.points[1].z = 0;
    msg.color.r = 0;
    msg.color.g = 1;
    msg.color.b = 0;
    ++msg.id;
    marray.markers.push_back(msg);
    msg.points[1].x = 0.0;
    msg.points[1].y = 0;
    msg.points[1].z = 0.2;
    msg.color.r = 0;
    msg.color.g = 0;
    msg.color.b = 1;
    ++msg.id;
    marray.markers.push_back(msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_surface_test_node");
    ros::NodeHandle n("~");

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("point_on_surface",1);

    std::string obj_filename = argv[1];
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

    tf::Vector3 vtest(0.255,0.0,-0.039);
    vtest = tf::Matrix3x3(tf::createQuaternionFromRPY(-M_PI_2,0,0))* vtest;
    MyMesh::Point ptest;
    //    ptest[0] = 0.255;
    //    ptest[1] = 0.0;
    //    ptest[2] = -0.039;
    ptest[0] = vtest.x();
    ptest[1] = vtest.y();
    ptest[2] = vtest.z();

    MyMesh::VertexIter v_it = getVertex(mesh, ptest);
    MyMesh::Point pclose = getPoint(mesh, ptest);

    //    MyMesh::VertexIter v_it = mesh.vertices_begin()+500;
    // circulate around the current vertex


    visualization_msgs::Marker msg;
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.id = 2;
    msg.header.frame_id = "/jaco_link_2";
    msg.header.stamp = ros::Time::now();
//        msg.pose.orientation.w = 1.0;
//        msg.pose.orientation.x = 0.0;
//        msg.pose.orientation.y = 0.0;
//        msg.pose.orientation.z = 0.0;
    msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2,0,0);
    msg.color.a = 0.8;

    visualization_msgs::MarkerArray marray;
    visualizeVertices(mesh,msg);
    marray.markers.push_back(msg);
    msg.type = visualization_msgs::Marker::ARROW;
//    for(auto v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it){
        msg.color.a = 0.8;
        msg.color.r = 1.0;
        msg.color.g = 0;
        msg.color.b = 0;
//        ++msg.id;
        msg.ns = "test_p";
        MyMesh::Point p = mesh.point(*v_it);
        MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it);
        std::cout << p[0] << ", " << p[1] << ", " <<p[2] << std::endl;
        MyMesh::Normal ni = mesh.normal(*v_it);
        MyMesh::Point pi = mesh.point(*vv_it);
            MyMesh::Point v = pi - p;
        double s = 0.1;
        MyMesh::Point pn = pi + s * v;
//        MyMesh::Point pn = p ;
        msg.points.resize(2);
        msg.points[0].x = pn[0];
        msg.points[0].y = pn[1];
        msg.points[0].z = pn[2];
        msg.points[1].x = pn[0] + .1 * (ni[0] /*- pn[0]*/);
        msg.points[1].y = pn[1] + .1 * (ni[1] /*- pn[1])*/);
        msg.points[1].z = pn[2] + .1 * (ni[2] /*- pn[2]*/);
        ++msg.id;
        marray.markers.push_back(msg);
        visualizeOrigin(msg, marray);
//    }

    // use this for random walk: v = vertex
    //    for(PolyMesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(v); voh_it; ++voh_it) {
    //            // Iterate over all outgoing halfedges...
    //    }

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
        for(visualization_msgs::Marker& m : marray.markers){
            m.header.stamp = ros::Time::now();
        }
        pub.publish(marray);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
