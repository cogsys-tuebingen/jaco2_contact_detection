#ifndef LINK_SURFACE_H
#define LINK_SURFACE_H
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/ArrayKernel.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
class LinkSurface
{
public:
    typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;
public:
    LinkSurface() {}

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
        if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) )
        {
            // we need face normals to update the vertex normals
            mesh.request_face_normals();
            // let the mesh update the normals
            mesh.update_normals();
            // dispose the face normals, as we don't need them anymore
            mesh.release_face_normals();
        }
        return true;
    }
protected:
    Mesh mesh_;
}
#endif // LINK_SURFACE_H
