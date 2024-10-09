#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

using Matrix34f = Matrix<float, 3,4>;

Matrix3f Rx(float rot_x) {
  float c=cos(rot_x);
  float s=sin(rot_x);
  Matrix3f R;
  R <<
    1,  0,  0,
    0,  c,  -s,
    0,  s,  c;
  return R;
}

Matrix3f Ry(float rot_y) {
  float c=cos(rot_y);
  float s=sin(rot_y);
  Matrix3f R;
  R <<
    c,  0,  s,
    0,  1,  0,
    -s,  0, c;
  return R;
}

Matrix3f Rz(float rot_z) {
  float c=cos(rot_z);
  float s=sin(rot_z);
  Matrix3f R;
  R <<
    c,  -s,  0,
    s,  c,  0,
    0,  0, 1;
  return R;
}

Matrix3f Rxyz(const Eigen::Matrix<float, 3, 1> angles) {
  return Rx(angles(0,0)) * Ry(angles(1,0)) *Rz(angles(2,0));
}


struct EigenUser {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Matrix34f m34;
  Matrix3f m33;
  Isometry3f iso3;
  
};
  
int main() {
  Matrix34f m34;
  
  cout << "m34: " << endl << m34 << endl;

  m34.setZero();
  cout << "m34: " << endl << m34 << endl;

  m34(0,0)=10;
  cout << "m34: " << endl << m34 << endl;

  cerr << "rows: " << Matrix34f::RowsAtCompileTime << endl; 

  Matrix34f::Scalar f = m34(0,0);

  m34.setRandom();

  cerr << "after random: " << endl << m34 <<  endl; 
 
  Eigen::Matrix<float, 3, 3>  result = m34*m34.transpose();
  
  cerr << "result: " << endl << result <<  endl; 

  cerr << "inv_result: " << result.inverse() << endl;

  cerr  << "stuff by inverse: " << endl << result * result.inverse() << endl;

  using Isometry3f = Eigen::Transform<float, 3, Eigen::Isometry>;
  
  Isometry3f iso;
  iso.linear() = Rz(0.1);
  iso.translation().setZero();

  Vector3f point;
  point.setRandom();

  Isometry3f rotation;
  rotation.setIdentity();

  ofstream os("out.txt");
  cerr << "point: " << point.transpose() << endl;
  for (int i=0; i<100; ++i) {
    cerr << "transformed point: " << (rotation*point).transpose() << endl;
    rotation=rotation*iso;
    os << (rotation*point).transpose() << endl;
  }

 
  
}
