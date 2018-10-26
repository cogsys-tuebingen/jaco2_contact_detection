#ifndef PARSE_STL_H
#define PARSE_STL_H
#include <string>
#include <vector>
#include <Eigen/Core>
namespace surface {

namespace stl {

  struct Point {
    float x;
    float y;
    float z;

    Point() : x(0), y(0), z(0) {}
    Point(float xp, float yp, float zp) : x(xp), y(yp), z(zp) {}
    Eigen::Vector3d toEigen() const;
  };

  struct Triangle {
    Point normal;
    Point v1;
    Point v2;
    Point v3;
    Triangle(Point normalp, Point v1p, Point v2p, Point v3p) :
      normal(normalp), v1(v1p), v2(v2p), v3(v3p) {}

  };

  std::ostream& operator<<(std::ostream& out, const Triangle& t);

  struct StlData {
    std::string name;
    std::vector<Triangle> triangles;

    StlData(std::string namep) : name(namep) {}
  };

  StlData parseStl(const std::string& stl_path);



}
}
#endif // PARSE_STL_H
