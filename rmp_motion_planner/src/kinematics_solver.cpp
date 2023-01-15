#include "riemmanian_motion_policies/kinematics_solver.h"


bool
KinematicsSolver::forward_ik(Eigen::Isometry3d &x, const Eigen::VectorXd &q)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i = 5; i >= 0; i--)
  {
    double di       = dh[i].d;
    double thetai   = dh[i].theta + q[i];
    double ai_1     = (i > 0) ? dh[i-1].a : 0;
    double alphai_1 = (i > 0) ? dh[i-1].alpha : 0;

    Eigen::Matrix4d Ri;
    Ri << std::cos(thetai), -std::sin(thetai), 0,  0,
          std::sin(thetai),  std::cos(thetai), 0,  0,
                 0,                 0,         1, di,
                 0,                 0,         0,  1;

    Eigen::Matrix4d Qi_1;
    Qi_1 << 1,         0,                   0,          ai_1,
            0, std::cos(alphai_1), -std::sin(alphai_1),   0,
            0, std::sin(alphai_1),  std::cos(alphai_1),   0,
            0,         0,                   0,            1;

    Eigen::Matrix4d Ti = Qi_1 * Ri;

    T = Ti * T;

    std::cout << "A" << i+1 << std::endl;
    std::cout << Ti << std::endl;
    std::cout << "---" << std::endl;
  }

  x = T;
  return true;
}


bool
KinematicsSolver::jacobian(Eigen::MatrixXd &J, const Eigen::VectorXd &q)
{
  std::vector<Eigen::Matrix4d> A(dh.size()+1);
  A[0] = Eigen::Matrix4d::Identity();

  for (int i = 0; i < dh.size(); i++)
  {
    double di     = dh[i].d;
    double thetai = dh[i].theta + q[i];
    double ai     = dh[i].a;
    double alphai = dh[i].alpha;

    Eigen::Matrix4d Ri;
    Ri << std::cos(thetai), -std::sin(thetai), 0,  0,
          std::sin(thetai),  std::cos(thetai), 0,  0,
          0,                 0,         1, di,
          0,                 0,         0,  1;

    Eigen::Matrix4d Qi;
    Qi << 1,         0,                   0,      ai,
          0, std::cos(alphai), -std::sin(alphai),  0,
          0, std::sin(alphai),  std::cos(alphai),  0,
          0,         0,                   0,       1;

    A[i+1] = A[i] * Ri * Qi;
  }

  std::vector<Eigen::Vector3d> z(A.size());
  std::vector<Eigen::Vector3d> t(A.size());
  for (int i = 0; i < A.size(); i++)
  {
    z[i] = A[i].block<3,1>(0,2);
    t[i] = A[i].block<3,1>(0,3);

    std::cout << "A" << i << std::endl;
    std::cout << A[i] << std::endl;

    std::cout << "z" << i << std::endl;
    std::cout << z[i] << std::endl;

    std::cout << "t" << i << std::endl;
    std::cout << t[i] << std::endl;

    std::cout << "---" << std::endl;
  }

  for (int i = 0; i < A.size()-1; i++)
  {
    J.block<3,1>(0,i) << z[i].cross(t[6] - t[i]);
    J.block<3,1>(3,i) << z[i];
  }

  std::cout << "J" << std::endl;
  std::cout << J << std::endl;
  std::cout << "---" << std::endl;

  return true;
}
