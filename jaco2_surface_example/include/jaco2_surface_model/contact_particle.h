#ifndef CONTACTPARTICLE_H
#define CONTACTPARTICLE_H
#include <tf/tf.h>
#include <memory>

class  ContactParticle
{
    ContactParticle();
    void setMesh();

protected:

protected:
    std::shared_ptr<MyMesh> mesh_;
    double dist_between_vertices_;
    MyMesh::VertexIter parent_vertex_;
    tf::Vector3 vertex_start_;
    tf::Vector3 vertex_end_;
    tf::Vector3 positon_;
    tf::Vector3 normal_;
    std::size_t link_id_;
};
#endif // CONTACTPARTICLE_H
