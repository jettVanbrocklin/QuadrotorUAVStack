#include "structurecomputer.h"

#include <Eigen/LU>

void pr(Eigen::MatrixXd m) { std::cout << m << std::endl; }
void pr(Eigen::VectorXd m) { std::cout << m << std::endl; }
void pr(Eigen::Matrix3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector2d m) { std::cout << m << std::endl; }

Eigen::Vector2d backProject(const Eigen::Matrix3d& RCI,
                            const Eigen::Vector3d& rc_I,
                            const Eigen::Vector3d& X3d) {
  using namespace Eigen;
  Vector3d t = -RCI * rc_I;
  MatrixXd Pextrinsic(3, 4);
  Pextrinsic << RCI, t;
  SensorParams sp;
  MatrixXd Pc = sp.K() * Pextrinsic;
  VectorXd X(4, 1);
  X.head(3) = X3d;
  X(3) = 1;
  Vector3d x = Pc * X;
  Vector2d xc_pixels = (x.head(2) / x(2)) / sp.pixelSize();
  return xc_pixels;
}

Eigen::Vector3d pixelsToUnitVector_C(const Eigen::Vector2d& rPixels) {
  using namespace Eigen;
  SensorParams sp;
  // Convert input vector to meters
  Vector2d rMeters = rPixels * sp.pixelSize();
  // Write as a homogeneous vector, with a 1 in 3rd element
  Vector3d rHomogeneous;
  rHomogeneous.head(2) = rMeters;
  rHomogeneous(2) = 1;
  // Invert the projection operation through the camera intrinsic matrix K to
  // yield a vector rC in the camera coordinate frame that has a Z value of 1
  Vector3d rC = sp.K().lu().solve(rHomogeneous);
  // Normalize rC so that output is a unit vector
  return rC.normalized();
}

void StructureComputer::clear() {
  // Zero out contents of point_
  point_.rXIHat.fill(0);
  point_.Px.fill(0);
  // Clear bundleVec_
  bundleVec_.clear();
}

void StructureComputer::push(std::shared_ptr<const CameraBundle> bundle) {
  bundleVec_.push_back(bundle);
}

// This function is where the computation is performed to estimate the
// contents of point_.  The function returns a copy of point_.
//
Point StructureComputer::computeStructure() {
  // Throw an error if there are fewer than 2 CameraBundles in bundleVec_,
  // since in this case structure computation is not possible.
  if (bundleVec_.size() < 2) {
    throw std::runtime_error(
        "At least 2 CameraBundle objects are "
        "needed for structure computation.");
  }

  // *********************************************************************
  // Fill in here the required steps to calculate the 3D position of the
  // feature point and its covariance.  Put these respectively in
  // point_.rXIHat and point_.Px
  // *********************************************************************

  // Go through each item bundleVec_ (pointer to cameraBundle Objects)

  int N = bundleVec_.size();


  Eigen::MatrixXd H_prime;
  H_prime.resize(2*N, 4);

  int h_prime_i = 0;

  Eigen::Vector3d t_C;
  Eigen::Matrix<double, 3, 4> RCI_tC;

  Eigen::Matrix<double,3,4> P;
  Eigen::RowVector4d p1;
  Eigen::RowVector4d p2;
  Eigen::RowVector4d p3;

  for (int i = 0; i < N; i++){
    t_C = bundleVec_[i]->RCI * bundleVec_[i]->rc_I;
    
    RCI_tC.block<3,3>(0,0) = bundleVec_[i]->RCI;  // left 3x3
    RCI_tC.col(3) = -t_C;           // 4th column


    P = sensorParams_.K() * RCI_tC;
    // Eigen::Matrix3d K_fixed;
    // K_fixed << sensorParams_.f(), 0, 0, 0, sensorParams_.f(), 0, 0, 0, 1;
    // P = K_fixed * RCI_tC;


    p1 = P.row(0);
    p2 = P.row(1);
    p3 = P.row(2);

    double x_delta = sensorParams_.pixelSize() * bundleVec_[i]->rx[0];
    double y_delta = sensorParams_.pixelSize() * bundleVec_[i]->rx[1];

    H_prime.row(h_prime_i) = (x_delta * p3) - p1;
    H_prime.row(h_prime_i+1) = (y_delta * p3) - p2;
    h_prime_i+=2;
    // std::cout << "H_Prime = " << H_prime << std::endl;
  }



  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2*N, 2*N);

  for (int i = 0; i < N; ++i) {
      m.block<2,2>(2*i, 2*i) = sensorParams_.Rc();
  }

  Eigen::MatrixXd R = (sensorParams_.pixelSize() * sensorParams_.pixelSize()) * m;

  Eigen::MatrixXd H;
  H.resize(2*N, 3);
  H.col(0) = H_prime.col(0);
  H.col(1) = H_prime.col(1);
  H.col(2) = H_prime.col(2);

  Eigen::VectorXd z;
  z.resize(2*N);
  z = -H_prime.col(3);


  // std::cout << "H_Prime = " << H_prime << std::endl;
  // std::cout<<std::endl;
  // std::cout << "H = " << H << std::endl;
  // std::cout<<std::endl;
  // std::cout << "z = " << z << std::endl;
  // std::cout<<std::endl;
  // std::cout << "H_Prime = " << H_prime << std::endl;
  // std::cout << "H_Prime = " << H_prime << std::endl;
  // std::cout << "H_Prime = " << H_prime << std::endl;
  // std::cout << "H_Prime = " << H_prime << std::endl;
  // std::cout << "H_Prime = " << H_prime << std::endl;
  // std::cout << "H_Prime = " << H_prime << std::endl;



// % Find rXIHat
// R_inv = inv(R);
// rXIHat = inv(H'*R_inv*H) * H'*R_inv*z; % Directly from Main.pdf page 82
// % Find Covariance Matrix
// Px = inv(H'*inv(R)*H);

  point_.rXIHat = ((H.transpose() * R.inverse() * H).inverse()) * H.transpose() * R.inverse() * z;
  point_.Px = ((H.transpose() * R.inverse() * H).inverse());

  return point_;
}
