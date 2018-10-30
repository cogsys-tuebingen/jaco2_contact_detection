//#include <jaco2_surface_model/parse_stl.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Mesh/TriMeshT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <OpenMesh/Core/Mesh/ArrayKernel.hh>
//#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <memory>
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

MyMesh::VertexIter getVertex(const MyMesh& mesh, const MyMesh::Point& p)
{
    double min_dist = std::numeric_limits<double>::infinity();
    auto it_min = mesh.vertices_begin();
    std::size_t n_points = 0;
    for(auto it = mesh.vertices_begin(); it!=mesh.vertices_end(); ++it){
        MyMesh::Point pi =  mesh.point(*it);
        MyMesh::Point diff = pi - p;
        double d = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];
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
        double d = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2];
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
    msg.points.clear();
    msg.type = visualization_msgs::Marker::POINTS;
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

    msg.scale.x = 0.005;
    msg.scale.y = 0.005;
    msg.scale.z = 0.005;
    ++msg.id;
}
void visualizeNormal(const::MyMesh::Point& point, const::MyMesh::Point& normal,
                     visualization_msgs::Marker& msg)
{
    msg.points.clear();
    msg.type = visualization_msgs::Marker::ARROW;
    msg.color.a = 0.8;
    msg.color.r = 1.0;
    msg.color.g = 0;
    msg.color.b = 0;
    msg.scale.x = 0.005;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
    msg.ns = "normal";
    msg.points.resize(2);
    msg.points[0].x = point[0];
    msg.points[0].y = point[1];
    msg.points[0].z = point[2];
    msg.points[1].x = point[0] + .1 * normal[0];
    msg.points[1].y = point[1] + .1 * normal[1];
    msg.points[1].z = point[2] + .1 * normal[2];
    ++msg.id;
}
void visualizeOrigin(visualization_msgs::Marker& msg, visualization_msgs::MarkerArray& marray)
{
    msg.points.clear();
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

bool loadMeshWithNormals(const std::string& file, MyMesh & mesh)
{
    mesh.request_vertex_normals();
    // assure we have vertex normals
    if (!mesh.has_vertex_normals())
    {
        std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
        return false;
    }
    OpenMesh::IO::Options opt;
    if (!OpenMesh::IO::read_mesh(mesh, file, opt)){
        std::cerr << "read error\n";
        return false;
    }
    // If the file did not provide vertex normals, then calculate them
    if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) ){
        // we need face normals to update the vertex normals
        mesh.request_face_normals();
        // let the mesh update the normals
        mesh.update_normals();
        // dispose the face normals, as we don't need them anymore
        mesh.release_face_normals();
    }
    return true;
}

bool writeMesh(const MyMesh& mesh, const std::string& file)
{
    OpenMesh::IO::Options opt(OpenMesh::IO::Options::Flag::VertexNormal);
    try
    {
        if ( !OpenMesh::IO::write_mesh(mesh, file, opt) )
        {
            std::cerr << "Cannot write mesh to file " << file << std::endl;
            return true;
        }
    }
    catch( std::exception& x )
    {
        std::cerr << x.what() << std::endl;
        return false;
    }
}

MyMesh::Point rotPoint(const tf::Matrix3x3& rot, const MyMesh::Point& p )
{
    MyMesh::Point res;
    tf::Vector3 v(p[0],p[1],p[2]);
    v = rot * v;
    res[0] = v.x();
    res[1] = v.y();
    res[2] = v.z();
    return res;
}
void rotateMesh( const tf::Matrix3x3& rot, MyMesh& mesh)
{
    for(MyMesh::VertexIter it = mesh.vertices_begin(); it!=mesh.vertices_end(); ++it){
        MyMesh::Point p =  mesh.point(*it);
        MyMesh::Point n = mesh.normal(*it);
        MyMesh::Point pp = rotPoint(rot, p);
        MyMesh::Point nn = rotPoint(rot, n);
        mesh.set_point(*it, pp);
        mesh.set_normal(*it, nn);
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_surface_test_node");
    ros::NodeHandle n("~");

//    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("point_on_surface",1);

//    visualization_msgs::MarkerArray marray;
    std::cout << "# arguments: "<< argc << std::endl;
    std::vector<std::string> obj_filenames;
    for(std::size_t in = 1; in < argc; ++ in){
        obj_filenames.push_back(argv[in]);
    }
    std::vector<std::string> frames = {"/jaco_link_2","/jaco_link_3", "/jaco_link_4", "/jaco_link_5", "/jaco_link_hand" ,
                                       "/jaco_link_finger_1", "/jaco_link_finger_2", "/jaco_link_finger_3"};
    std::vector<MyMesh> meshes;
    std::size_t i = 0;

    visualization_msgs::Marker msg;
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.lifetime = ros::Duration(0.2);
    msg.id = 0;
    //        tf::Quaternion static_rot = tf::createQuaternionFromRPY(M_PI_2,0,0);
    tf::Quaternion static_rot2 = tf::createQuaternionFromRPY(M_PI_2,0,0);
    bool write = true;

    tf::Matrix3x3 rot(static_rot2);
    tf::Quaternion static_rot(0,0,0,1);
    for(auto obj_filename : obj_filenames){

        MyMesh mesh;
        bool loaded = loadMeshWithNormals(obj_filename, mesh);
        if(!loaded){
            return 42;
        }
        rotateMesh(rot, mesh);
        if(write){
            std::string outfile = frames[i];
            outfile.erase(0,1);
            outfile += "_r.obj";
            writeMesh(mesh, outfile);
        }
        meshes.push_back(mesh);

    }

    return 0;
}
