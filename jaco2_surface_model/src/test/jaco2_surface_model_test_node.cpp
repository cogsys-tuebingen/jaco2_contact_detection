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
#include <random>
#include <algorithm>
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

namespace MeshToTF{

tf::Vector3 getPoint(const MyMesh& mesh, const MyMesh::VertexHandle& it)
{
    MyMesh::Point p = mesh.point(it);
    return tf::Vector3(p[0],p[1],p[2]);
}

tf::Vector3 getNormal(const MyMesh& mesh, const MyMesh::VertexHandle& it)
{
    MyMesh::Point p = mesh.normal(it);
    return tf::Vector3(p[0],p[1],p[2]);
}
}

struct Particle
{
    Particle():
        s(0)
    {
    }
    void setVertices(MyMesh& mesh,
                     MyMesh::VertexIter active_vertex,
                     MyMesh::VOHIter goal_vertex)
    {
        this->active_vertex = mesh.handle(mesh.vertex(*active_vertex));
        this->goal_vertex = mesh.to_vertex_handle(*goal_vertex);
    }

    void setVertices(MyMesh& mesh,
                     std::size_t active_vertex,
                     MyMesh::VOHIter goal_vertex)
    {
        this->active_vertex = mesh.vertex_handle(active_vertex);
        this->goal_vertex = mesh.to_vertex_handle(*goal_vertex);
    }


    void setVertices(MyMesh& mesh,
                     std::size_t active_vertex)
    {
        this->active_vertex = mesh.vertex_handle(active_vertex);
        MyMesh::VOHIter vg = mesh.voh_iter(this->active_vertex );
        this->goal_vertex = mesh.to_vertex_handle(*vg);
    }

    tf::Vector3 getPosition(const MyMesh& mesh) const
    {
        tf::Vector3 p0 = MeshToTF::getPoint(mesh, active_vertex);
        tf::Vector3 p1 = MeshToTF::getPoint(mesh, goal_vertex);
        tf::Vector3 pos = p0 + s * (p1 - p0);
        return pos;
    }

    tf::Vector3 getActiveVertex(const MyMesh& mesh) const
    {
       return MeshToTF::getPoint(mesh, active_vertex);
    }

    tf::Vector3 getGoalVertex(const MyMesh& mesh) const
    {
       return MeshToTF::getPoint(mesh, goal_vertex);
    }

    void updateEdgeLength(const MyMesh& mesh)
    {
        e = (getGoalVertex(mesh) - getGoalVertex(mesh)).length();
    }

    double getDistanceToGoal()
    {
        return (1-s) * e;
    }

    tf::Vector3 getNormal(const MyMesh &mesh) const
    {
        return MeshToTF::getNormal(mesh, active_vertex);
    }

    MyMesh::VertexHandle active_vertex;
    MyMesh::VertexHandle goal_vertex;
    double s;
    double e;
};

void visualizeParticle(const Particle& p,
                       const MyMesh& mesh,
                       visualization_msgs::Marker& msg)
{
    msg.points.clear();
    msg.type = visualization_msgs::Marker::ARROW;
    msg.color.a = 0.8;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0;
    msg.scale.x = 0.005;
    msg.scale.y = 0.01;
    msg.scale.z = 0.01;
    msg.points.resize(2);
    tf::Vector3 pos = p.getPosition(mesh);
    tf::Vector3 n = p.getNormal(mesh);
    tf::Vector3 p2 = pos + 0.1 * n;
    msg.points[0].x = pos.x();
    msg.points[0].y = pos.y();
    msg.points[0].z = pos.z();
    msg.points[1].x = p2.x();
    msg.points[1].y = p2.y();
    msg.points[1].z = p2.z();
}

//tf::Vector3 particleGetPosition(const Particle & p, const MyMesh& mesh)
//{
//     tf::Vector3 p0 = getPoint(mesh, p.active_vertex);
//     tf::Vector3 p1 = getPoint(mesh, p.goal_vertex);
//     tf::Vector3 pos = p0 + p.s * (p1 - p0);
//     return pos;
//}

struct RandomWalk
{
    RandomWalk(double distance = 0.025):
        generator_(rd_()),
        momentum_(0,distance)
    {

    }

    void update(Particle & p, MyMesh& mesh)
    {
        double delta_p = momentum_(generator_);
        double distance = 0;
        while(distance < delta_p){
            double d = p.getDistanceToGoal();
            if(d < delta_p){
                p.s += delta_p/p.e;
                break;
            }  else{
                distance += d;
                p.active_vertex = p.goal_vertex;
                std::size_t n_edges = 0;
                MyMesh::VOHIter vhs =mesh.voh_iter(p.active_vertex);
                for(MyMesh::VOHIter vohit =vhs; vohit.is_valid(); ++vohit) {
                    ++n_edges;
                }

                std::uniform_int_distribution<std::size_t> neighbors(0,n_edges);
                std::size_t index = neighbors(generator_);
                for(std::size_t i = 0; i < index; ++i){
                    ++vhs;
                }
                p.goal_vertex = mesh.to_vertex_handle(*vhs);
                p.updateEdgeLength(mesh);
                p.s = (delta_p - d)/p.e;

            }
        }



    }

    std::random_device rd_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> momentum_;
    //    std::uniform_int_distribution<int> neighbor_;

};


struct GaussianWalk
{
    GaussianWalk(double mean, double std):
        generator_(rd_()),
        distribution_(mean, std)
    {

    }
    void update(Particle & p, const MyMesh& mesh)
    {

    }
    std::random_device rd_;
    std::mt19937 generator_;
    std::normal_distribution<double> distribution_;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaco2_surface_test_node");
    ros::NodeHandle n("~");

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("point_on_surface",1);
    ros::Publisher pub2 = n.advertise<visualization_msgs::MarkerArray>("random_walk",1);

    visualization_msgs::MarkerArray marray;
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
    //    tf::Quaternion static_rot2 = tf::createQuaternionFromRPY(M_PI_2,0,0);
    //    bool write = true;
    tf::Quaternion static_rot2(0,0,0,1);
    bool write = false;
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

        tf::Vector3 vtest(0.255,0.0,-0.039);
        vtest = tf::Matrix3x3(static_rot.inverse()) * vtest;
        MyMesh::Point ptest;
        //    ptest[0] = 0.255;
        //    ptest[1] = 0.0;
        //    ptest[2] = -0.039;
        ptest[0] = vtest.x();
        ptest[1] = vtest.y();
        ptest[2] = vtest.z();

        MyMesh::VertexIter v_it = getVertex(meshes[i], ptest);
        //        MyMesh::Point pclose = getPoint(meshes[i], ptest);


        //    MyMesh::VertexIter v_it = mesh.vertices_begin()+500;
        //        msg.pose.orientation.w = 1.0;
        //        msg.pose.orientation.x = 0.0;
        //        msg.pose.orientation.y = 0.0;
        //        msg.pose.orientation.z = 0.0;

        tf::quaternionTFToMsg(static_rot, msg.pose.orientation);

        msg.header.frame_id = frames[i];
        msg.header.stamp = ros::Time::now();
        msg.ns = "vertices_" + std::to_string(i);
        visualizeVertices(meshes[i], msg);
        marray.markers.push_back(msg);


        //    for(auto v_it = mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it){

        MyMesh::Point p = mesh.point(*v_it);
        MyMesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it);

        std::cout << p[0] << ", " << p[1] << ", " <<p[2] << std::endl;
        MyMesh::Normal ni = mesh.normal(*v_it);
        MyMesh::Point pi = mesh.point(*vv_it);
        MyMesh::Point v = pi - p;
        double s = 0.0;
        MyMesh::Point pn = p + s * v;
        visualizeNormal(pn, ni, msg);
        marray.markers.push_back(msg);

        visualizeOrigin(msg, marray);
        ++i;
        //    }

    }

    //create moving particles
    std::vector<Particle> particles;
    std::vector<std::size_t> selected;
    visualization_msgs::MarkerArray vparticles;
    vparticles.markers.clear();

    RandomWalk rand;
    MyMesh& mesh = meshes.front();
    std::size_t n_vertices = mesh.n_vertices();
    visualization_msgs::Marker mpart;
    mpart.header.frame_id = frames.front();
    mpart.ns = "random_walk";
    mpart.id = 0;
    std::uniform_int_distribution<std::size_t> dist_vert(0,n_vertices-1);
    for(std::size_t n = 0; n < 100; ++n){
        std::size_t index = dist_vert(rand.generator_);
        Particle p;
        p.setVertices(mesh, index);
        visualizeParticle(p, mesh, mpart);
        particles.push_back(p);
        vparticles.markers.push_back(mpart);
        ++mpart.id;

    }

    ros::Rate r(20);
    ros::Duration update_time(0.1);
    ros::Time last_update = ros::Time::now();
    while(ros::ok()){

        ros::Time current = ros::Time::now();
        for(visualization_msgs::Marker& m : marray.markers){
            m.header.stamp = current;
        }
        bool updated = (current - last_update) > update_time;
        auto it = particles.begin();
        for(visualization_msgs::Marker& m : vparticles.markers){
            m.header.stamp = current;
            if(updated){
                Particle& p = *it;
                rand.update(p, mesh);
                visualizeParticle(p, mesh, m);
            }
            ++it;
        }
        if(updated){
            last_update = current;
        }
        pub.publish(marray);
        pub2.publish(vparticles);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
