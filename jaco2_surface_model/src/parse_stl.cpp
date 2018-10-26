#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>

#include <jaco2_surface_model/parse_stl.h>

using namespace surface;
using namespace stl;

std::ostream& operator<<(std::ostream& out, const Point p) {
  out << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Triangle& t) {
  out << "---- TRIANGLE ----" << std::endl;
  out << t.normal << std::endl;
  out << t.v1 << std::endl;
  out << t.v2 << std::endl;
  out << t.v3 << std::endl;
  return out;
}

float parseFloat(std::ifstream& s) {
  char f_buf[sizeof(float)];
  s.read(f_buf, 4);
  float* fptr = (float*) f_buf;
  return *fptr;
}

Point parsePoint(std::ifstream& s) {
  float x = parseFloat(s);
  float y = parseFloat(s);
  float z = parseFloat(s);
  return Point(x, y, z);
}

StlData parseStl(const std::string& stl_path) {
  std::ifstream stl_file(stl_path.c_str(), std::ios::in | std::ios::binary);
  if (!stl_file) {
    std::cout << "ERROR: COULD NOT READ FILE" << std::endl;
    assert(false);
  }

  char header_info[80] = "";
  char n_triangles[4];
  stl_file.read(header_info, 80);
  stl_file.read(n_triangles, 4);
  std::string h(header_info);
  StlData info(h);
  unsigned int* r = (unsigned int*) n_triangles;
  unsigned int num_triangles = *r;
  for (unsigned int i = 0; i < num_triangles; i++) {
    Point normal = parsePoint(stl_file);
    Point v1 = parsePoint(stl_file);
    Point v2 = parsePoint(stl_file);
    Point v3 = parsePoint(stl_file);
    info.triangles.push_back(Triangle(normal, v1, v2, v3));
    char dummy[2];
    stl_file.read(dummy, 2);
  }
  return info;
}

Eigen::Vector3d Point::toEigen() const{
  return Eigen::Vector3d(x,y,z);
}

